// Unit tests of the manual driving pipeline, run on an emulated Cortex-M7 by run_tests.py (no host compiler):
// vehicle_dynamics.c (the layer) and throttle_control.c (the VCU glue: limits, BSPD, inverter commands), built with
// the firmware toolchain. Bare metal: results stay in RAM (failures[], check_count, done) for run_tests.py.
// REPLAY_HEADER->count != 0: instead of the tests, replays recorded inputs through the layer (replay_logs.py).
#include <math.h>
#include <stdint.h>
#include <string.h>

#include <autonomous_t26.h>

#include "throttle_control.h"
#include "traction_control_torque_vectoring.h"
#include "vehicle_dynamics.h"

#define DT 0.005f
#define ERPM_PERIOD_STEPS 7   // the inverters send ERPM every ~33 ms: the rear wheel speeds move in steps

/* ============================== results for run_tests.py ============================== */

typedef struct {
    const char *test;
    const char *expression;
    uint32_t line;
    float value;
} failure_t;

#define MAX_FAILURES 64
volatile failure_t failures[MAX_FAILURES];
volatile uint32_t failure_count, check_count, test_count, done;
static const char *current_test;

static void record_failure(const char *expression, uint32_t line, float value) {
    if (failure_count < MAX_FAILURES) {
        failures[failure_count].test = current_test;
        failures[failure_count].expression = expression;
        failures[failure_count].line = line;
        failures[failure_count].value = value;
    }
    failure_count++;
}

// CHECK(condition, value): value is reported when the condition fails
#define CHECK(condition, value) \
    do { check_count++; if (!(condition)) record_failure(#condition, __LINE__, (float)(value)); } while (0)
#define TEST(name) static void name(void)
#define RUN(name) do { current_test = #name; test_count++; name(); } while (0)

/* ============================== layer helpers ============================== */

static vehicle_dynamics_t vd;

// VCU today: TC / TV from the switches (both 0), no IMU, no front wheels
static void layer_init(bool traction_control, float friction) {
    vehicle_dynamics_init(&vd, &VEHICLE_DYNAMICS_T26_CONFIG);
    vd.config.traction_control_enable = traction_control;
    vd.config.torque_vectoring_enable = false;
    vd.config.tyre_LMUX *= friction;
    vd.config.tyre_LMUY *= friction;
}

static float rad_s(float speed_mps) { return speed_mps / VEHICLE_DYNAMICS_T26_CONFIG.wheel_radius_m; }

static vehicle_dynamics_torque_t step(float rear_left_mps, float rear_right_mps, float throttle, float brake) {
    vehicle_dynamics_input_t input = {
        .wheel_speed_rad_s = {0.0f, 0.0f, rad_s(rear_left_mps), rad_s(rear_right_mps)},
        .rear_wheel_speeds_valid = true,
        .steering_valid = true,
        .throttle_percent = throttle,
        .throttle_valid = true,
        .brake_pressure_bar = brake,
        .brake_pressure_valid = true,
    };
    return vehicle_dynamics_compute(&vd, &input, DT);
}

// Constant speed for a while: the estimate settles on the wheels
static vehicle_dynamics_torque_t hold(float speed_mps, float throttle, float brake, int steps) {
    vehicle_dynamics_torque_t torque = {0};
    for (int i = 0; i < steps; i++) torque = step(speed_mps, speed_mps, throttle, brake);
    return torque;
}

static float minimum_torque_factor(void) {
    return fminf(vd.traction_control_torque_factor[WHEEL_REAR_LEFT], vd.traction_control_torque_factor[WHEEL_REAR_RIGHT]);
}

/* ============================== layer: pedals ============================== */

TEST(test_regen_is_linear_from_2_to_10_bar) {
    layer_init(false, 1.0f);
    const float bars[] = {0.0f, 1.9f, 2.0f, 4.0f, 6.0f, 8.0f, 10.0f, 20.0f};
    float previous_current = 1.0f;
    for (unsigned i = 0; i < sizeof bars / sizeof bars[0]; i++) {
        vehicle_dynamics_torque_t torque = hold(10.0f, 0.0f, bars[i], 100);
        float share = fminf(fmaxf((bars[i] - 2.0f) / 8.0f, 0.0f), 1.0f);
        CHECK(fabsf(vd.regen_fraction - share * vd.config.regen_maximum_fraction) < 1e-5f, vd.regen_fraction);
        CHECK(torque.rear_left.motor_current_a <= 0.0f, torque.rear_left.motor_current_a);
        CHECK(torque.rear_left.motor_current_a == torque.rear_right.motor_current_a, torque.rear_right.motor_current_a);
        if (bars[i] <= 2.0f) CHECK(torque.rear_left.motor_current_a == 0.0f, torque.rear_left.motor_current_a);
        if (bars[i] > 2.0f && bars[i] <= 10.0f) CHECK(torque.rear_left.motor_current_a < previous_current, torque.rear_left.motor_current_a);
        if (bars[i] > 10.0f) CHECK(fabsf(torque.rear_left.motor_current_a - previous_current) < 0.5f, torque.rear_left.motor_current_a);
        previous_current = torque.rear_left.motor_current_a;
    }
}

TEST(test_pedal_gives_a_share_of_what_the_wheels_can_take) {
    layer_init(false, 1.0f);
    hold(10.0f, 0.0f, 0.0f, 400);                                            // vx settles at 10 m/s
    float half = hold(10.0f, 52.5f, 0.0f, 100).rear_left.motor_current_a;   // (52.5 - 5) / 95 = 0.5
    float full = hold(10.0f, 100.0f, 0.0f, 100).rear_left.motor_current_a;
    CHECK(half > 0.0f, half);
    CHECK(fabsf(half / full - 0.5f) < 0.02f, half / full);
    CHECK(full <= vd.config.motor_maximum_drive_current_a, full);
    CHECK(hold(10.0f, 4.0f, 0.0f, 20).rear_left.motor_current_a == 0.0f, 0);       // below throttle_start_percent
}

TEST(test_brake_has_priority) {
    layer_init(false, 1.0f);
    CHECK(hold(10.0f, 100.0f, 5.9f, 50).rear_left.motor_current_a > 0.0f, 0);       // light brake: still drives
    vehicle_dynamics_torque_t torque = hold(10.0f, 20.0f, 6.0f, 50);                // brake pressed: no drive, regen
    CHECK(torque.rear_left.motor_current_a < 0.0f, torque.rear_left.motor_current_a);
}

TEST(test_apps_brake_plausibility_latches_until_throttle_released) {
    layer_init(false, 1.0f);
    hold(10.0f, 30.0f, 8.0f, 10);                                                   // brake + 30 % throttle
    CHECK(vd.pedal_plausibility_cut, 0);
    CHECK(hold(10.0f, 30.0f, 0.0f, 50).rear_left.motor_current_a <= 0.0f, 0);       // brake off: still cut
    hold(10.0f, 4.0f, 0.0f, 10);                                                    // throttle under 5 %: reset
    CHECK(!vd.pedal_plausibility_cut, 0);
    CHECK(hold(10.0f, 30.0f, 0.0f, 50).rear_left.motor_current_a > 0.0f, 0);
}

TEST(test_bad_sensors_give_no_drive) {
    layer_init(false, 1.0f);
    hold(10.0f, 0.0f, 0.0f, 50);
    vehicle_dynamics_input_t input = {
        .wheel_speed_rad_s = {0.0f, 0.0f, rad_s(10.0f), rad_s(10.0f)},
        .rear_wheel_speeds_valid = true, .throttle_percent = 80.0f, .throttle_valid = false,
        .brake_pressure_bar = 0.0f, .brake_pressure_valid = true,
    };
    CHECK(vehicle_dynamics_compute(&vd, &input, DT).rear_left.motor_current_a <= 0.0f, 0);   // APPS error
    input.throttle_valid = true;
    input.brake_pressure_valid = false;
    CHECK(vehicle_dynamics_compute(&vd, &input, DT).rear_left.motor_current_a <= 0.0f, 0);   // no brake frame
    input.brake_pressure_valid = true;
    input.throttle_percent = NAN;
    input.wheel_speed_rad_s[WHEEL_REAR_LEFT] = INFINITY;
    vehicle_dynamics_torque_t torque = vehicle_dynamics_compute(&vd, &input, NAN);
    CHECK(isfinite(torque.rear_left.motor_current_a) && torque.rear_left.motor_current_a <= 0.0f, torque.rear_left.motor_current_a);
    CHECK(isfinite(torque.rear_right.motor_current_a), torque.rear_right.motor_current_a);
    CHECK(isfinite(vd.vehicle_speed_mps), vd.vehicle_speed_mps);
}

/* ============================== layer: wet track ============================== */

typedef struct { float available_n, drive_a, regen_a, maximum_acceleration_ms2; } grip_t;

static grip_t grip_at_10_mps(float friction) {
    grip_t g;
    layer_init(false, friction);
    hold(10.0f, 0.0f, 0.0f, 400);                                   // vx settles, no load transfer
    g.drive_a = hold(10.0f, 100.0f, 0.0f, 50).rear_left.motor_current_a;
    g.available_n = vd.available_drive_force_n[WHEEL_REAR_LEFT];
    g.maximum_acceleration_ms2 = vd.maximum_acceleration_ms2;
    g.regen_a = hold(10.0f, 0.0f, 10.0f, 50).rear_left.motor_current_a;
    return g;
}

TEST(test_wet_track_scales_the_grip) {
    grip_t dry = grip_at_10_mps(1.0f), wet = grip_at_10_mps(0.75f);
    // tyre limited at 10 m/s: force, drive and regen current and the no-IMU acceleration limit all x 0.75
    CHECK(fabsf(wet.available_n / dry.available_n - 0.75f) < 0.02f, wet.available_n / dry.available_n);
    CHECK(fabsf(wet.drive_a / dry.drive_a - 0.75f) < 0.02f, wet.drive_a / dry.drive_a);
    CHECK(fabsf(wet.regen_a / dry.regen_a - 0.75f) < 0.02f, wet.regen_a / dry.regen_a);
    CHECK(fabsf(wet.maximum_acceleration_ms2 / dry.maximum_acceleration_ms2 - 0.75f) < 0.02f, wet.maximum_acceleration_ms2 / dry.maximum_acceleration_ms2);
}

/* ============================== layer: vehicle speed, rear wheels only, no IMU ============================== */

// Car speeds up at acceleration_ms2 from rest, both rear wheels at car + spin (from spin_from_s, at spin_ms2, at most
// spin_max_mps over the car), ERPM held ERPM_PERIOD_STEPS
typedef struct { float vehicle_error_max_mps, torque_factor_min, slip_max, wheel_over_estimate_mps; } run_result_t;

static run_result_t accelerate(float acceleration_ms2, float seconds, float spin_from_s, float spin_ms2, float spin_max_mps) {
    run_result_t result = {0.0f, 1.0f, 0.0f, 0.0f};
    float wheel = 0.0f;
    for (int i = 0; i < (int)(seconds / DT); i++) {
        float t = i * DT, car = acceleration_ms2 * t;
        if (i % ERPM_PERIOD_STEPS == 0) wheel = car + (t > spin_from_s ? fminf(spin_ms2 * (t - spin_from_s), spin_max_mps) : 0.0f);
        step(wheel, wheel, 100.0f, 0.0f);
        if (t > 0.5f) result.vehicle_error_max_mps = fmaxf(result.vehicle_error_max_mps, fabsf(vd.vehicle_speed_mps - car));
        result.torque_factor_min = fminf(result.torque_factor_min, minimum_torque_factor());
        result.slip_max = fmaxf(result.slip_max, vd.slip_ratio[WHEEL_REAR_LEFT]);
        result.wheel_over_estimate_mps = wheel - vd.vehicle_speed_mps;
    }
    return result;
}

TEST(test_no_imu_speed_follows_gripping_rear_wheels) {
    layer_init(true, 1.0f);
    run_result_t r = accelerate(7.0f, 4.0f, 99.0f, 0.0f, 0.0f);   // hard launch, no spin, 0 -> 28 m/s
    CHECK(r.vehicle_error_max_mps < 0.5f, r.vehicle_error_max_mps);
    CHECK(r.torque_factor_min > 0.999f, r.torque_factor_min);      // traction control never cuts a gripping wheel
    CHECK(fabsf(vd.speed_derivative_ms2 - 7.0f) < 1.5f, vd.speed_derivative_ms2);   // ax without IMU
    CHECK(vd.wheel_load_n[WHEEL_REAR_LEFT] > vd.static_load_n[WHEEL_REAR_LEFT] + 100.0f, vd.wheel_load_n[WHEEL_REAR_LEFT]);
}

TEST(test_no_imu_spin_does_not_drag_the_speed) {
    layer_init(true, 1.0f);
    // open loop (the wheels do not answer the cut): from 1 s both rear wheels spin up at 40 m/s2 to +6 m/s for 0.4 s
    run_result_t r = accelerate(5.0f, 1.4f, 1.0f, 40.0f, 6.0f);
    CHECK(r.wheel_over_estimate_mps > 3.0f, r.wheel_over_estimate_mps);   // vx rises at the car limit, not with the spin
    CHECK(r.slip_max > 0.3f, r.slip_max);
    CHECK(r.torque_factor_min < 0.8f, r.torque_factor_min);              // traction control cuts
}

TEST(test_no_imu_lock_does_not_drag_the_speed) {
    layer_init(true, 1.0f);
    hold(20.0f, 0.0f, 0.0f, 400);
    float car = 20.0f, wheel = 20.0f, speed_at_lock = 0.0f, limit_sum = 0.0f;
    for (int i = 0; i < 120; i++) {                                // 8 m/s2 braking, rear wheels lock after 0.3 s
        car -= 8.0f * DT;
        if (i % ERPM_PERIOD_STEPS == 0) wheel = i * DT < 0.3f ? car : 0.0f;
        if (i * DT >= 0.3f && speed_at_lock == 0.0f) speed_at_lock = vd.vehicle_speed_mps;
        step(wheel, wheel, 0.0f, 30.0f);
        if (speed_at_lock > 0.0f) limit_sum += vd.maximum_deceleration_ms2 * DT;
    }
    CHECK(speed_at_lock - vd.vehicle_speed_mps <= limit_sum + 0.3f, speed_at_lock - vd.vehicle_speed_mps - limit_sum);
    CHECK(vd.vehicle_speed_mps > 5.0f, vd.vehicle_speed_mps);     // far above the locked wheels
    CHECK(vd.slip_ratio[WHEEL_REAR_LEFT] < -0.5f, vd.slip_ratio[WHEEL_REAR_LEFT]);
    CHECK(vd.maximum_deceleration_ms2 > 10.0f && vd.maximum_deceleration_ms2 < 25.0f, vd.maximum_deceleration_ms2);
}

TEST(test_no_imu_cornering_keeps_the_speed) {
    // 15 m/s, 8 m/s2 left turn: r = 0.533 rad/s, road wheel 3.16 deg = steering wheel 0.305 rad; inner (left) rear
    // wheel slower by r * track / 2. Without gyro the free two-track model drifted here (13-10 replay: r -6 rad/s, vx -> 0)
    layer_init(true, 1.0f);
    hold(15.0f, 0.0f, 0.0f, 400);
    const float r = 8.0f / 15.0f, half_track = VEHICLE_DYNAMICS_T26_CONFIG.track_rear_m / 2.0f;
    vehicle_dynamics_input_t input = {
        .wheel_speed_rad_s = {0.0f, 0.0f, rad_s(15.0f - r * half_track), rad_s(15.0f + r * half_track)},
        .rear_wheel_speeds_valid = true, .steering_wheel_angle_rad = 0.305f, .steering_valid = true,
        .throttle_percent = 50.0f, .throttle_valid = true, .brake_pressure_valid = true,
    };
    float factor = 1.0f;
    for (int i = 0; i < 600; i++) {
        vehicle_dynamics_compute(&vd, &input, DT);
        factor = fminf(factor, minimum_torque_factor());
    }
    CHECK(fabsf(vd.vehicle_speed_mps - 15.0f) < 0.3f, vd.vehicle_speed_mps);
    CHECK(fabsf(vd.yaw_rate_rad_s - r) < 0.05f, vd.yaw_rate_rad_s);
    CHECK(fabsf(vd.lateral_velocity_mps) < 0.2f, vd.lateral_velocity_mps);
    CHECK(factor > 0.999f, factor);                                // the faster outer wheel is not a spin
    input.steering_wheel_angle_rad = 2.0f;                         // impossible steer for 15 m/s: r stays at the grip limit
    for (int i = 0; i < 200; i++) vehicle_dynamics_compute(&vd, &input, DT);
    CHECK(fabsf(vd.yaw_rate_rad_s) * vd.vehicle_speed_mps < 1.05f * vd.total_lateral_grip_n / vd.config.mass_kg, vd.yaw_rate_rad_s);
    CHECK(vd.vehicle_speed_mps > 13.0f, vd.vehicle_speed_mps);
}

TEST(test_starts_from_the_wheels_when_rolling) {
    // ready to drive with the car already rolling: vx starts at the wheels, not at 0 (from 0, without the IMU, the
    // wheels read as spinning until vx caught up at the acceleration limit: traction control cut everything meanwhile)
    layer_init(true, 1.0f);
    step(10.0f, 10.0f, 50.0f, 0.0f);
    CHECK(fabsf(vd.vehicle_speed_mps - 10.0f) < 0.1f, vd.vehicle_speed_mps);
    hold(10.0f, 50.0f, 0.0f, 40);
    CHECK(minimum_torque_factor() > 0.999f, minimum_torque_factor());
}

static float rear_available_force(void) {
    return vd.available_drive_force_n[WHEEL_REAR_LEFT] + vd.available_drive_force_n[WHEEL_REAR_RIGHT];
}

TEST(test_straight_line_boost_ignores_the_steering_without_gyro) {
    // LSR 21-11: the steering read a curve on the straight and, without gyro, the layer kept grip for it: 10 A at full
    // pedal. Now ay comes from the rear wheels (equal = straight) and full pedal on a straight asks the boost over the model
    // 10 m/s: the motors (power limit 26 kW) still have room for the boost
    layer_init(false, 1.0f);
    vehicle_dynamics_input_t input = {
        .wheel_speed_rad_s = {0.0f, 0.0f, rad_s(10.0f), rad_s(10.0f)}, .rear_wheel_speeds_valid = true,
        .steering_wheel_angle_rad = 0.3f, .steering_valid = true,     // steering sensor off centre: 3 deg of road wheel
        .throttle_percent = 100.0f, .throttle_valid = true, .brake_pressure_valid = true,
    };
    for (int i = 0; i < 200; i++) vehicle_dynamics_compute(&vd, &input, DT);
    CHECK(fabsf(vd.lateral_acceleration_ms2) < 0.1f, vd.lateral_acceleration_ms2);
    CHECK(vd.straight_line_share > 0.99f, vd.straight_line_share);
    CHECK(vd.driver_force_request_n > 1.45f * rear_available_force(), vd.driver_force_request_n / rear_available_force());
    // a real left turn, 8 m/s2 (outer wheel faster): the model is the limit again
    const float r = 8.0f / 10.0f, half_track = VEHICLE_DYNAMICS_T26_CONFIG.track_rear_m / 2.0f;
    input.wheel_speed_rad_s[WHEEL_REAR_LEFT] = rad_s(10.0f - r * half_track);
    input.wheel_speed_rad_s[WHEEL_REAR_RIGHT] = rad_s(10.0f + r * half_track);
    for (int i = 0; i < 200; i++) vehicle_dynamics_compute(&vd, &input, DT);
    CHECK(fabsf(vd.lateral_acceleration_ms2 - 8.0f) < 0.5f, vd.lateral_acceleration_ms2);
    CHECK(vd.straight_line_share < 0.01f, vd.straight_line_share);
    CHECK(vd.driver_force_request_n <= 1.001f * rear_available_force(), vd.driver_force_request_n / rear_available_force());
}

TEST(test_no_imu_speed_follows_coasting_and_braking) {
    layer_init(true, 1.0f);
    hold(15.0f, 0.0f, 0.0f, 400);
    CHECK(fabsf(vd.vehicle_speed_mps - 15.0f) < 0.1f, vd.vehicle_speed_mps);
    float car = 15.0f;
    for (int i = 0; i < 200; i++) { car -= 6.0f * DT; step(car, car, 0.0f, 15.0f); }   // braking with grip
    CHECK(fabsf(vd.vehicle_speed_mps - car) < 0.3f, vd.vehicle_speed_mps - car);
}

TEST(test_imu_sideslip_is_bounded) {
    // 13-10 oversteer with the IMU on: rear left locked, countersteer, gyro +1 rad/s, ay +10 m/s2, no pedal (brake not
    // logged). The ay correction through the saturated tyre model took vy to -10 m/s at 10 m/s, then vx' = ax + r vy
    // took vx to 2 m/s with the right rear wheel at 11
    layer_init(false, 1.0f);
    vehicle_dynamics_input_t input = {
        .wheel_speed_rad_s = {0.0f, 0.0f, rad_s(11.5f), rad_s(11.5f)}, .rear_wheel_speeds_valid = true,
        .steering_valid = true, .imu_valid = true, .yaw_rate_valid = true, .throttle_valid = true, .brake_pressure_valid = true,
    };
    for (int i = 0; i < 400; i++) vehicle_dynamics_compute(&vd, &input, DT);
    input.wheel_speed_rad_s[WHEEL_REAR_LEFT] = rad_s(1.4f);
    input.wheel_speed_rad_s[WHEEL_REAR_RIGHT] = rad_s(11.0f);
    input.steering_wheel_angle_rad = -0.25f;
    input.imu_longitudinal_acceleration_ms2 = -3.0f;
    input.imu_lateral_acceleration_ms2 = 10.0f;
    input.yaw_rate_rad_s = 1.0f;
    float sideslip_max = 0.0f;
    for (int i = 0; i < 120; i++) {
        vehicle_dynamics_compute(&vd, &input, DT);
        sideslip_max = fmaxf(sideslip_max, fabsf(vd.lateral_velocity_mps) / fmaxf(vd.vehicle_speed_mps, 1.0f));
    }
    CHECK(sideslip_max <= 0.1501f, sideslip_max);                  // without the bound: 0.24
}

/* ============================== layer: torque vectoring allocation ============================== */

// TV on, 15 m/s left turn: steering 0.305 rad = reference yaw rate ~0.47 rad/s. The gyro reads gyro_rad_s, the rear
// wheels roll with it. 2 s at coast first.
static vehicle_dynamics_input_t tv_corner(bool traction_control, float gyro_rad_s) {
    layer_init(traction_control, 1.0f);
    vd.config.torque_vectoring_enable = true;
    const float half_track = VEHICLE_DYNAMICS_T26_CONFIG.track_rear_m / 2.0f;
    vehicle_dynamics_input_t input = {
        .wheel_speed_rad_s = {0.0f, 0.0f, rad_s(15.0f - gyro_rad_s * half_track), rad_s(15.0f + gyro_rad_s * half_track)},
        .rear_wheel_speeds_valid = true, .steering_wheel_angle_rad = 0.305f, .steering_valid = true,
        .yaw_rate_rad_s = gyro_rad_s, .yaw_rate_valid = true, .throttle_valid = true, .brake_pressure_valid = true,
    };
    for (int i = 0; i < 400; i++) vehicle_dynamics_compute(&vd, &input, DT);
    return input;
}

static float rear_force_sum(void) { return vd.wheel_force_n[WHEEL_REAR_LEFT] + vd.wheel_force_n[WHEEL_REAR_RIGHT]; }

TEST(test_tv_never_drives_more_than_the_pedal) {
    // Light pedal, TV asks for more yaw: the inner wheel stops at 0, the outer one gets at most what the driver asked
    // (was: request / 2 + Mz / track, several times the request)
    vehicle_dynamics_input_t input = tv_corner(false, 0.3f);
    input.throttle_percent = 10.0f;
    float excess = -1e9f;
    for (int i = 0; i < 200; i++) {
        vehicle_dynamics_compute(&vd, &input, DT);
        excess = fmaxf(excess, rear_force_sum() - vd.driver_force_request_n);
    }
    CHECK(vd.torque_vectoring_active && vd.yaw_moment_request_nm > 50.0f, vd.yaw_moment_request_nm);
    CHECK(excess <= 0.5f, excess);
    CHECK(vd.yaw_moment_delivered_nm > 0.0f && vd.yaw_moment_delivered_nm < vd.yaw_moment_request_nm, vd.yaw_moment_delivered_nm);
    CHECK(vd.yaw_moment_integral_term_nm < 0.5f * vd.config.maximum_yaw_moment_nm, vd.yaw_moment_integral_term_nm);   // anti-windup
}

TEST(test_tv_keeps_the_yaw_moment_at_full_pedal) {
    // Full pedal: the outer wheel at its limit, the inner one gives the whole Mz (was: about half of it)
    vehicle_dynamics_input_t input = tv_corner(false, 0.3f);
    input.throttle_percent = 100.0f;
    for (int i = 0; i < 400; i++) vehicle_dynamics_compute(&vd, &input, DT);
    CHECK(vd.yaw_moment_request_nm > 100.0f, vd.yaw_moment_request_nm);
    CHECK(fabsf(vd.yaw_moment_delivered_nm - vd.yaw_moment_request_nm) < 1.0f, vd.yaw_moment_delivered_nm - vd.yaw_moment_request_nm);
    CHECK(rear_force_sum() <= vd.driver_force_request_n + 0.5f, rear_force_sum() - vd.driver_force_request_n);
}

TEST(test_tv_traction_control_cut_never_turns_the_yaw_moment) {
    // Oversteer (gyro 0.8 rad/s, reference ~0.47): TV asks Mz < 0 = more drive on the inner (left) wheel, which spins.
    // Traction control caps it BEFORE the split: the outer wheel gives way too, never more drive than the inner one
    // (was: cut after TV, the inner wheel below the outer = a yaw moment the WRONG way, into the spin)
    vehicle_dynamics_input_t input = tv_corner(true, 0.8f);
    input.throttle_percent = 100.0f;
    for (int i = 0; i < 100; i++) vehicle_dynamics_compute(&vd, &input, DT);
    input.wheel_speed_rad_s[WHEEL_REAR_LEFT] *= 1.4f;
    float wrong_way = -1e9f, factor = 1.0f;
    for (int i = 0; i < 200; i++) {
        vehicle_dynamics_compute(&vd, &input, DT);
        wrong_way = fmaxf(wrong_way, vd.yaw_moment_delivered_nm);
        factor = fminf(factor, vd.traction_control_torque_factor[WHEEL_REAR_LEFT]);
    }
    CHECK(vd.yaw_moment_request_nm < -100.0f, vd.yaw_moment_request_nm);
    CHECK(factor < 0.5f, factor);                                  // traction control did cut the inner wheel
    CHECK(wrong_way <= 0.5f, wrong_way);
}

/* ============================== VCU glue: throttle_control.c ============================== */

// Stubs for the CAN senders and the globals the glue reads
VCU_Signals_t vcu;
static int16_t sent_ac[3], sent_brake[3];
static uint8_t sent_mode[3];   // 1 = SetAcCurrent, 2 = SetBrakeCurrent, 3 = SetRelCurrent
void can_bus_send_FSIC_SetAcCurrent(uint8_t id, int16_t value, CAN_HandleTypeDef *hcan) { (void)hcan; sent_ac[id] = value; sent_mode[id] = 1; }
void can_bus_send_FSIC_SetBrakeCurrent(uint8_t id, int16_t value, CAN_HandleTypeDef *hcan) { (void)hcan; sent_brake[id] = value; sent_mode[id] = 2; }
void can_bus_send_FSIC_SetRelCurrent(uint8_t id, int16_t value, CAN_HandleTypeDef *hcan) { (void)hcan; (void)value; sent_mode[id] = 3; }

static FSIC_t inverter_1, inverter_2;
static APPS_Result_t apps;
static uint32_t now_ms;

static void glue_reset(void) {
    memset(&inverter_1, 0, sizeof inverter_1);
    memset(&inverter_2, 0, sizeof inverter_2);
    memset(&apps, 0, sizeof apps);
    memset(&vcu, 0, sizeof vcu);
    memset(&vehicle_sensors, 0, sizeof vehicle_sensors);
    now_ms = 1000;
    throttle_control_reset();
}

static int32_t erpm(float speed_mps) {
    return (int32_t)(rad_s(speed_mps) * 60.0f / 6.2831853f * VEHICLE_DYNAMICS_T26_CONFIG.gear_ratio * MOTOR_POLE_PAIRS);
}

static void glue_step(float speed_mps, uint16_t pedal_1000, float brake_bar, bool bspd) {
    now_ms += 5;
    inverter_1.Actual_ERPM = erpm(speed_mps);
    inverter_2.Actual_ERPM = -erpm(speed_mps);   // mirrored motor: the sign is ignored
    apps.percentage_1000 = pedal_1000;
    vcu.brake_raw = (uint16_t)(brake_bar * 10.0f);
    vcu.brake_last_rx_time = now_ms;
    vehicle_sensors.steering_frame_time_ms = now_ms;
    throttle_control_update(&apps, bspd, &inverter_1, &inverter_2, 0, now_ms);
    throttle_control_send(NULL, now_ms);
}

TEST(test_glue_rise_limit_then_full_request) {
    glue_reset();
    float previous = 0.0f, maximum_step = THROTTLE_TORQUE_RISE_MAX_PCT_S * 0.01f * vehicle_dynamics.config.motor_maximum_drive_current_a * 0.005f;
    for (int i = 0; i < 200; i++) {
        glue_step(10.0f, 1000, 0.0f, false);
        CHECK(throttle.current_left_a - previous <= maximum_step + 1e-3f, throttle.current_left_a - previous);
        previous = throttle.current_left_a;
    }
    CHECK(throttle.current_left_a > 20.0f && throttle.current_left_a == throttle.current_right_a, throttle.current_left_a);
    CHECK(throttle.status == THROTTLE_STATUS_DRIVE, throttle.status);
    CHECK(sent_mode[1] == 1 && sent_mode[2] == 1, sent_mode[1]);                      // SetAcCurrent, 0.1 A per bit
    CHECK(sent_ac[INVERTER_ID_REAR_LEFT] == (int16_t)(throttle.current_left_a * 10.0f), sent_ac[INVERTER_ID_REAR_LEFT]);
}

TEST(test_glue_regen_goes_as_brake_current) {
    glue_reset();
    for (int i = 0; i < 100; i++) glue_step(10.0f, 0, 10.0f, false);
    if (THROTTLE_REGEN_ENABLE) {
        CHECK(throttle.current_left_a < 0.0f, throttle.current_left_a);
        CHECK(throttle.status == THROTTLE_STATUS_REGEN, throttle.status);
        CHECK(sent_mode[1] == 2 && sent_mode[2] == 2, sent_mode[1]);                  // SetBrakeCurrent, positive
        CHECK(sent_brake[INVERTER_ID_REAR_LEFT] == (int16_t)(-throttle.current_left_a * 10.0f), sent_brake[INVERTER_ID_REAR_LEFT]);
    } else {                                                                           // regen switched off: no motor current
        CHECK(throttle.current_left_a == 0.0f && throttle.current_right_a == 0.0f, throttle.current_left_a);
        CHECK(throttle.status == THROTTLE_STATUS_COAST, throttle.status);
    }
    inverter_1.Actual_FaultCode = 3;                                                   // inverter fault: no regen
    glue_step(10.0f, 0, 10.0f, false);
    CHECK(throttle.current_left_a == 0.0f && throttle.current_right_a == 0.0f, throttle.current_left_a);
    CHECK(throttle.status == THROTTLE_STATUS_INVERTER_FAULT, throttle.status);
}

TEST(test_glue_digital_bspd_is_last) {
    glue_reset();
    for (int i = 0; i < 100; i++) glue_step(10.0f, 1000, 0.0f, false);
    glue_step(10.0f, 1000, 0.0f, true);
    CHECK(throttle.current_left_a == 0.0f && throttle.current_right_a == 0.0f, throttle.current_left_a);
    CHECK(sent_ac[1] == 0 && sent_ac[2] == 0 && sent_mode[1] == 1, sent_ac[1]);
    CHECK(throttle.status == THROTTLE_STATUS_BSPD_CUT, throttle.status);
    glue_step(10.0f, 1000, 0.0f, false);                                               // released: rises again from 0
    CHECK(throttle.current_left_a > 0.0f && throttle.current_left_a <= 3.01f, throttle.current_left_a);
}

TEST(test_glue_stale_brake_frame_gives_no_drive) {
    glue_reset();
    for (int i = 0; i < 100; i++) glue_step(10.0f, 1000, 0.0f, false);
    vcu.brake_last_rx_time = now_ms - SENSOR_TIMEOUT_MS - 10;
    now_ms += 5;
    throttle_control_update(&apps, false, &inverter_1, &inverter_2, 0, now_ms);
    CHECK(!throttle.brake_valid && throttle.current_left_a <= 0.0f, throttle.current_left_a);
}

TEST(test_glue_track_surface_and_front_wheels) {
    glue_reset();
    float friction = THROTTLE_WET_TRACK ? THROTTLE_FRICTION_WET : THROTTLE_FRICTION_DRY;
    CHECK(vehicle_dynamics.config.tyre_LMUX == VEHICLE_DYNAMICS_T26_CONFIG.tyre_LMUX * friction, vehicle_dynamics.config.tyre_LMUX);
    CHECK(vehicle_dynamics.config.tyre_LMUY == VEHICLE_DYNAMICS_T26_CONFIG.tyre_LMUY * friction, vehicle_dynamics.config.tyre_LMUY);
    CHECK(vehicle_dynamics.config.traction_control_enable == TRACTION_CONTROL_ENABLE, 0);
    // AQT2 fresh but reading 0 (seen 08-10 / 10-10): with THROTTLE_FRONT_WHEEL_SPEEDS 0 the speed follows the rear
    for (int i = 0; i < 400; i++) {
        vehicle_sensors.front_wheel_frame_time_ms = now_ms + 5;
        glue_step(i * 0.025f, 0, 0.0f, false);                                        // 5 m/s2 coasting up to 10 m/s
    }
    if (!THROTTLE_FRONT_WHEEL_SPEEDS) CHECK(fabsf(vehicle_dynamics.vehicle_speed_mps - 10.0f) < 0.3f, vehicle_dynamics.vehicle_speed_mps);
}

static void put_int16(uint8_t *data, int16_t value) {
    data[0] = (uint8_t)(value & 0xFF);
    data[1] = (uint8_t)((uint16_t)value >> 8);
}

TEST(test_glue_imu_frames_into_the_layer) {
    glue_reset();
    // IMU x backwards, y right, z up: x -2.00 m/s2 = speeding up at 2, y -1.00 = 1 m/s2 to the left, z +0.300 rad/s = left turn
    can_msg_t acceleration = {.id = AUTONOMOUS_T26_IMU_ACCELERATION_FRAME_ID, .dlc = 6, .bus = CAN_BUS_3};
    can_msg_t angular_velocity = {.id = AUTONOMOUS_T26_IMU_ANGULAR_VELOCITY_FRAME_ID, .dlc = 6, .bus = CAN_BUS_3};
    put_int16(&acceleration.data[0], -200);
    put_int16(&acceleration.data[2], -100);
    put_int16(&angular_velocity.data[4], 300);
    for (int i = 0; i < 200; i++) {
        acceleration.timestamp = angular_velocity.timestamp = now_ms + 5;
        vehicle_sensors_can_receive(&acceleration);
        vehicle_sensors_can_receive(&angular_velocity);
        glue_step(10.0f, 0, 0.0f, false);
    }
    CHECK(fabsf(vehicle_sensors.imu_longitudinal_acceleration_ms2 - 2.0f) < 1e-4f, vehicle_sensors.imu_longitudinal_acceleration_ms2);
    CHECK(fabsf(vehicle_sensors.imu_lateral_acceleration_ms2 - 1.0f) < 1e-4f, vehicle_sensors.imu_lateral_acceleration_ms2);
    CHECK(fabsf(vehicle_sensors.imu_yaw_rate_rad_s - 0.3f) < 1e-4f, vehicle_sensors.imu_yaw_rate_rad_s);
    CHECK(vehicle_dynamics.yaw_rate_source == (THROTTLE_IMU_GYRO ? YAW_RATE_SOURCE_GYRO : YAW_RATE_SOURCE_NONE), vehicle_dynamics.yaw_rate_source);
    if (THROTTLE_IMU_GYRO) CHECK(fabsf(vehicle_dynamics.yaw_rate_rad_s - 0.3f) < 0.02f, vehicle_dynamics.yaw_rate_rad_s);
    for (int i = 0; i < (IMU_TIMEOUT_MS + 50) / 5; i++) glue_step(10.0f, 0, 0.0f, false);   // frames stop: layer without IMU
    CHECK(!vehicle_sensors.imu_acceleration_valid && !vehicle_sensors.imu_yaw_rate_valid, 0);
    CHECK(vehicle_dynamics.yaw_rate_source != YAW_RATE_SOURCE_GYRO, vehicle_dynamics.yaw_rate_source);
    CHECK(fabsf(vehicle_dynamics.vehicle_speed_mps - 10.0f) < 0.5f, vehicle_dynamics.vehicle_speed_mps);
}

/* ============================== released pedal: APPS.c (calibration from APPS.h) -> glue ============================== */

uint32_t HAL_GetTick(void) { return now_ms; }   // APPS.c error timeout
int printf(const char *format, ...) { (void)format; return 0; }   // APPS_PrintStatus: no newlib stdio

static uint16_t apps_window[APPS_MA_WINDOW_SIZE];
static uint32_t apps_window_position;

static void apps_reset(uint16_t throttle_bits) {
    APPS_Init();
    for (int i = 0; i < APPS_MA_WINDOW_SIZE; i++) apps_window[i] = throttle_bits;
    apps_window_position = 0;
}

// main.c: 100 Hz timer -> moving average -> APPS_Process(apps2_avg, apps1_avg) (the throttle sensor is the 2nd CAN
// value, APPS1 in APPS.h); the control loop runs twice per APPS sample. The other sensor follows the APPS2 scale.
static void apps_glue_step(float speed_mps, uint16_t throttle_bits, float brake_bar) {
    apps_window[apps_window_position++ % APPS_MA_WINDOW_SIZE] = throttle_bits;
    uint32_t sum = 0;
    for (int i = 0; i < APPS_MA_WINDOW_SIZE; i++) sum += apps_window[i];
    int32_t other = (int32_t)throttle_bits * (int32_t)APPS2_GAIN_X1000 / 1000 + APPS2_OFFSET;
    apps = APPS_Process((uint16_t)(sum / APPS_MA_WINDOW_SIZE), (uint16_t)(other < 0 ? 0 : other));
    glue_step(speed_mps, apps.percentage_1000, brake_bar, false);
    glue_step(speed_mps, apps.percentage_1000, brake_bar, false);
}

static float sent_current_a(uint8_t id) { return sent_mode[id] == 2 ? -0.1f * sent_brake[id] : 0.1f * sent_ac[id]; }

// Worst case for a stray request: TC and TV on, steering 30 deg with the gyro at 0 (TV wants a yaw moment)
static void worst_case_controls(void) {
    vehicle_dynamics.config.traction_control_enable = true;
    vehicle_dynamics.config.torque_vectoring_enable = true;
    vehicle_sensors.steering_angle_raw_deg = 30.0f + STEERING_OFFSET_DEG;
}

static void gyro_frame(void) {
    can_msg_t angular_velocity = {.id = AUTONOMOUS_T26_IMU_ANGULAR_VELOCITY_FRAME_ID, .dlc = 6, .bus = CAN_BUS_3, .timestamp = now_ms + 5};
    vehicle_sensors_can_receive(&angular_velocity);
}

TEST(test_released_pedal_requests_no_current) {
    // 0 = APPS CAN timeout (main.c), 30 = open sensor, below the 0 % point, the 0 % point, the hysteresis dead zone
    const uint16_t throttle_bits[] = {0, 30, 1000, APPS_MIN_BITS - 1, APPS_MIN_BITS, APPS_MIN_BITS + APPS_HYSTERESIS_BITS};
    const float speeds_mps[] = {0.0f, 2.0f, 10.0f, 30.0f};
    const float brakes_bar[] = {0.0f, VEHICLE_DYNAMICS_T26_CONFIG.regen_start_bar - 0.1f};   // no regen below regen_start_bar
    for (unsigned b = 0; b < sizeof throttle_bits / sizeof throttle_bits[0]; b++) {
        for (unsigned s = 0; s < sizeof speeds_mps / sizeof speeds_mps[0]; s++) {
            for (unsigned p = 0; p < sizeof brakes_bar / sizeof brakes_bar[0]; p++) {
                glue_reset();
                apps_reset(throttle_bits[b]);
                worst_case_controls();
                float worst = 0.0f, worst_pedal = 0.0f, worst_sent = 0.0f;
                bool vectoring = false;
                for (int i = 0; i < 100; i++) {                        // 1 s, past the 600 ms APPS error timeout
                    gyro_frame();
                    apps_glue_step(speeds_mps[s], throttle_bits[b], brakes_bar[p]);
                    worst = fmaxf(worst, fmaxf(fabsf(throttle.current_left_a), fabsf(throttle.current_right_a)));
                    worst_sent = fmaxf(worst_sent, fmaxf(fabsf(sent_current_a(1)), fabsf(sent_current_a(2))));
                    worst_pedal = fmaxf(worst_pedal, throttle.pedal_1000);
                    vectoring |= vehicle_dynamics.torque_vectoring_active && vehicle_dynamics.yaw_moment_request_nm != 0.0f;
                }
                CHECK(worst_pedal == 0.0f, throttle_bits[b]);
                CHECK(worst == 0.0f, worst);
                CHECK(worst_sent == 0.0f, worst_sent);
                CHECK(throttle.status == THROTTLE_STATUS_COAST || throttle.status == THROTTLE_STATUS_APPS_ERROR, throttle.status);
                if (THROTTLE_IMU_GYRO && speeds_mps[s] > 5.0f) CHECK(vectoring, speeds_mps[s]);   // TV was asking, got nothing
            }
        }
    }
}

TEST(test_pedal_release_cuts_current_and_dead_zone_edge) {
    glue_reset();
    apps_reset(APPS_MIN_BITS);
    worst_case_controls();
    for (int i = 0; i < 100; i++) { gyro_frame(); apps_glue_step(10.0f, APPS_MAX_BITS, 0.0f); }
    CHECK(throttle.current_left_a > 5.0f, throttle.current_left_a);       // full pedal, 30 deg steering: grip reserve
    int steps_to_zero = -1;                                    // foot resting inside the dead zone
    for (int i = 0; i < 100; i++) {
        gyro_frame();
        apps_glue_step(10.0f, APPS_MIN_BITS + APPS_HYSTERESIS_BITS, 0.0f);
        bool zero = throttle.current_left_a == 0.0f && throttle.current_right_a == 0.0f && sent_ac[1] == 0 && sent_ac[2] == 0;
        if (zero && steps_to_zero < 0) steps_to_zero = i;
        if (steps_to_zero >= 0) CHECK(zero, throttle.current_left_a);  // and stays 0
    }
    CHECK(steps_to_zero >= 0 && steps_to_zero <= APPS_MA_WINDOW_SIZE, steps_to_zero);   // moving average lag only
    for (int i = 0; i < 20; i++) { gyro_frame(); apps_glue_step(10.0f, APPS_MIN_BITS + APPS_HYSTERESIS_BITS + 2, 0.0f); }
    CHECK(throttle.pedal_1000 > 0 && throttle.current_left_a > 0.0f, throttle.current_left_a);   // just past the dead zone: drive
}

/* ============================== replay of recorded inputs (replay_logs.py) ============================== */

typedef struct {
    float rear_left_rad_s, rear_right_rad_s, throttle_percent, brake_bar, steering_rad;
    float imu_longitudinal_ms2, imu_lateral_ms2, yaw_rate_rad_s;   // car axes
    uint32_t flags;   // 1 throttle, 2 brake, 4 steering, 16 IMU accelerations, 32 gyro valid; 8 reset the layer first
} replay_input_t;
typedef struct {
    float vehicle_speed_mps, speed_derivative_ms2, maximum_acceleration_ms2, slip_left, slip_right;
    float torque_factor_left, torque_factor_right, current_left_a, current_right_a, available_drive_force_n;
    float maximum_deceleration_ms2, driver_force_request_n, lateral_velocity_mps, yaw_rate_rad_s, peak_slip_drive;
} replay_output_t;

// Outside RAM (the startup code would clear it): run_tests.py maps this region and writes it before the start
typedef struct {
    uint32_t count;                     // 0 = run the unit tests
    float friction;                     // x tyre_LMUX / tyre_LMUY
    uint32_t traction_control;
    float no_imu_acceleration_margin;   // 0 = config value
} replay_header_t;
#define REPLAY_HEADER ((const volatile replay_header_t *)0x60000000u)
#define REPLAY_INPUT ((const replay_input_t *)0x60000010u)
#define REPLAY_OUTPUT ((replay_output_t *)0x64000000u)

static void replay(void) {
    for (uint32_t i = 0; i < REPLAY_HEADER->count; i++) {
        const replay_input_t *in = &REPLAY_INPUT[i];
        if (in->flags & 8) {
            layer_init(REPLAY_HEADER->traction_control, REPLAY_HEADER->friction);
            if (REPLAY_HEADER->no_imu_acceleration_margin > 0.0f) vd.config.no_imu_acceleration_margin = REPLAY_HEADER->no_imu_acceleration_margin;
        }
        vehicle_dynamics_input_t input = {
            .wheel_speed_rad_s = {0.0f, 0.0f, in->rear_left_rad_s, in->rear_right_rad_s},
            .rear_wheel_speeds_valid = true,
            .steering_wheel_angle_rad = in->steering_rad, .steering_valid = in->flags & 4,
            .throttle_percent = in->throttle_percent, .throttle_valid = in->flags & 1,
            .brake_pressure_bar = in->brake_bar, .brake_pressure_valid = in->flags & 2,
            .imu_longitudinal_acceleration_ms2 = in->imu_longitudinal_ms2, .imu_lateral_acceleration_ms2 = in->imu_lateral_ms2,
            .imu_valid = in->flags & 16, .yaw_rate_rad_s = in->yaw_rate_rad_s, .yaw_rate_valid = in->flags & 32,
        };
        vehicle_dynamics_torque_t torque = vehicle_dynamics_compute(&vd, &input, DT);
        replay_output_t out = {
            vd.vehicle_speed_mps, vd.speed_derivative_ms2, vd.maximum_acceleration_ms2,
            vd.slip_ratio[WHEEL_REAR_LEFT], vd.slip_ratio[WHEEL_REAR_RIGHT],
            vd.traction_control_torque_factor[WHEEL_REAR_LEFT], vd.traction_control_torque_factor[WHEEL_REAR_RIGHT],
            torque.rear_left.motor_current_a, torque.rear_right.motor_current_a, vd.available_drive_force_n[WHEEL_REAR_LEFT],
            vd.maximum_deceleration_ms2, vd.driver_force_request_n, vd.lateral_velocity_mps, vd.yaw_rate_rad_s,
            vd.peak_slip_drive[WHEEL_REAR_LEFT],
        };
        REPLAY_OUTPUT[i] = out;
    }
}

/* ============================== main and bare-metal startup ============================== */

static void run(void) {
    if (REPLAY_HEADER->count) {
        replay();
    } else {
        RUN(test_regen_is_linear_from_2_to_10_bar);
        RUN(test_pedal_gives_a_share_of_what_the_wheels_can_take);
        RUN(test_brake_has_priority);
        RUN(test_apps_brake_plausibility_latches_until_throttle_released);
        RUN(test_bad_sensors_give_no_drive);
        RUN(test_wet_track_scales_the_grip);
        RUN(test_no_imu_speed_follows_gripping_rear_wheels);
        RUN(test_no_imu_spin_does_not_drag_the_speed);
        RUN(test_no_imu_lock_does_not_drag_the_speed);
        RUN(test_no_imu_cornering_keeps_the_speed);
        RUN(test_starts_from_the_wheels_when_rolling);
        RUN(test_straight_line_boost_ignores_the_steering_without_gyro);
        RUN(test_no_imu_speed_follows_coasting_and_braking);
        RUN(test_imu_sideslip_is_bounded);
        RUN(test_tv_never_drives_more_than_the_pedal);
        RUN(test_tv_keeps_the_yaw_moment_at_full_pedal);
        RUN(test_tv_traction_control_cut_never_turns_the_yaw_moment);
        RUN(test_glue_rise_limit_then_full_request);
        RUN(test_glue_regen_goes_as_brake_current);
        RUN(test_glue_digital_bspd_is_last);
        RUN(test_glue_stale_brake_frame_gives_no_drive);
        RUN(test_glue_track_surface_and_front_wheels);
        RUN(test_glue_imu_frames_into_the_layer);
        RUN(test_released_pedal_requests_no_current);
        RUN(test_pedal_release_cuts_current_and_dead_zone_edge);
    }
    done = 1;
}

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;
extern void _estack(void);   // linker symbol (top of RAM), declared as a function for the vector table

void Fault_Handler(void) {
    for (;;) __asm volatile("bkpt #1");
}

void Reset_Handler(void) {
    *(volatile uint32_t *)0xE000ED88u |= 0xFu << 20;   // CPACR: CP10 + CP11 full access = FPU on
    __asm volatile("dsb\n\tisb");
    for (uint32_t *source = &_sidata, *target = &_sdata; target < &_edata;) *target++ = *source++;
    for (uint32_t *target = &_sbss; target < &_ebss;) *target++ = 0;
    run();
    for (;;) __asm volatile("bkpt #0");
}

__attribute__((section(".isr_vector"), used)) void (*const vector_table[16])(void) = {
    _estack, Reset_Handler, Fault_Handler, Fault_Handler, Fault_Handler, Fault_Handler, Fault_Handler, 0,
    0, 0, 0, Fault_Handler, Fault_Handler, 0, Fault_Handler, Fault_Handler,
};
