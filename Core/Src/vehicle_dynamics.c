/**
 * @file vehicle_dynamics.c
 * @brief Vehicle dynamics layer. vehicle_dynamics_compute() at the bottom calls one function per step, in order;
 *        vehicle_dynamics.h describes each step and how to send the result to the DTI inverters.
 *
 * VCU copy of Powertrain V1 - Simulator/vcu_vehicle_dynamics, NOT in the Python twin yet: regen 2 -> 10 bar, without the
 * IMU vx follows the driven wheels per ERPM frame within maximum_acceleration / _deceleration (no_imu_acceleration_margin)
 * and ax = dvx/dt, kinematic vy / r when nothing measures them, |vy| <= MAXIMUM_SIDESLIP x vx (the EKF ran away with the
 * real IMU), torque vectoring allocation that keeps Mz and never exceeds the request (split_axle), traction control as a
 * cap before that allocation, yaw PI anti-windup. Tests: tests/vehicle_dynamics (run_tests.py, replay_logs.py).
 */

#include "vehicle_dynamics.h"

#include <math.h>
#include <string.h>

#define GRAVITY_MS2                     9.81f
#define AIR_DENSITY_KG_M3               1.225f
#define PI_F                            3.14159265f
#define MINIMUM_LOAD_N                  20.0f   // a lifted wheel still gets this load in the tyre model [N]
#define MAXIMUM_CURVATURE               0.98f   // MF curvature E for the peak solver (E = 1 has no peak)
#define MAXIMUM_TYRE_USE                0.95f   // friction ellipse always leaves sqrt(1 - 0.95^2) = 31 % for the other direction
#define BRAKING_ACCELERATION_MS2       -3.0f    // below this even free wheels may lock: they only bound vx
#define REQUEST_DEAD_BAND_N            50.0f    // |driver force request| below this = coasting: driven wheels roll with the car
#define MAXIMUM_DT_S                    0.05f   // a longer gap (debugger, missed periods) integrates only this [s]
#define YAW_MOMENT_TOLERANCE_NM         1.0f    // delivered within this of the request = delivered (yaw PI anti-windup)
// Plausible sensor ranges: beyond them, or NaN / infinity, the sensor counts as not valid for that step
#define MAXIMUM_WHEEL_SPEED_RAD_S     600.0f    // 120 m/s at 0.2 m
#define MAXIMUM_STEERING_WHEEL_ANGLE_RAD  6.3f  // one full turn
#define MAXIMUM_ACCELERATION_MS2      100.0f    // 10 g
#define MAXIMUM_YAW_RATE_RAD_S         10.0f
#define MAXIMUM_THROTTLE_PERCENT      150.0f    // a calibration a bit off still reads up to ~110 %
#define MAXIMUM_BRAKE_PRESSURE_BAR    300.0f
// FS rules, APPS / brake pedal plausibility check: brake + throttle above this -> no drive until below the reset
#define PEDAL_PLAUSIBILITY_CUT_PERCENT     25.0f
#define PEDAL_PLAUSIBILITY_RESET_PERCENT    5.0f
// Kalman filter
#define LOW_SPEED_TIME_CONSTANT_S       0.05f   // below kalman_minimum_speed_mps: vy -> 0, r -> vx tan(steer) / L
#define KALMAN_IMU_MISSING_NOISE_MS2   20.0f    // vx process noise without the IMU: the wheels decide
#define MAXIMUM_SIDESLIP                0.15f   // |vy| <= this x vx (8.5 deg): past it the estimate is the model, not the car
#define SPEED_DERIVATIVE_TIME_CONSTANT_S 0.1f   // dvx/dt low-pass (ERPM frames every ~33 ms: vx moves in steps)
#define MAXIMUM_WHEEL_FRAME_AGE_S       0.1f    // no IMU: vx moves at most the limit x this per new ERPM frame
enum { STATE_VX = 0, STATE_VY, STATE_YAW_RATE, STATE_COUNT };
static const float KALMAN_INITIAL_VARIANCE[STATE_COUNT] = {1.0f, 0.25f, 0.25f};   // vx [m/s]^2, vy [m/s]^2, r [rad/s]^2
static const float KALMAN_JACOBIAN_STEP[STATE_COUNT] = {0.01f, 0.01f, 0.001f};     // finite differences of the model

/* ============================== T26 VALUES ==============================
 * Car, aero, motor: Powertrain V1 - Simulator/loadParams.m. Tyre: Hoosier 43075 16x7.5-10 R20 7in 12 psi
 * MF 5.2 .tir (tyre data/). Steering: firmware STEERING_*_GAIN.
 * One value per line: vehicle_dynamics.py reads this block.
 */
const vehicle_dynamics_config_t VEHICLE_DYNAMICS_T26_CONFIG = {
    .mass_kg = 290.0f,                                  // car + driver
    .yaw_inertia_kg_m2 = 100.0f,                        // PLACEHOLDER: typical FS car + driver
    .wheelbase_m = 1.55f,
    .center_of_gravity_to_front_axle_m = 0.853f,        // CAD sketch 852.5 mm
    .center_of_gravity_height_m = 0.35f,                // PLACEHOLDER: suspension CAD sketch says 0.25
    .track_front_m = 1.20f,
    .track_rear_m = 1.19f,
    .roll_stiffness_front_share = 0.5f,                 // PLACEHOLDER: same springs front and rear, no anti-roll bar data
    .imu_ahead_of_center_of_gravity_m = 0.078f,         // CHECK: IMU at mid wheelbase (0.775 m behind the front axle), CG at 0.853 m
    .aero_lift_coefficient_area_m2 = 4.72f,             // Cl 4.33 * A 1.09, normal wing
    .aero_front_share = 0.40f,                          // PLACEHOLDER
    .wheel_radius_m = 0.200f,                           // rolling radius, TTC runs (NOTES.md "Radius")
    .gear_ratio = 14.73f,                               // CHECK: firmware THROTTLE_GEAR_RATIO; loadParams.m uses 12.75
    .gear_efficiency = 0.90f,                           // PLACEHOLDER
    .wheel_driven = {false, false, true, true},         // T26: two rear motors
    .motor_pole_pairs = 4.0f,                           // MOTOR_POLE_PAIRS (ERPM = rpm * pole pairs)
    .motor_flux_linkage_v_s = 0.0504f,                  // fitted on the dyno 24-09 (loadParams psi): Kt = 0.3024 N m/A
    .motor_maximum_drive_current_a = 120.0f,            // DTI MaxAcCurrent (log 02-10)
    .motor_maximum_brake_current_a = 80.0f,             // CHECK: DTI brake current limit (loadParams I_ac_regen)
    .motor_maximum_power_w = 26000.0f,                  // PLACEHOLDER: DTI MaxDcCurrent 55 A x ~480 V per motor (shaft power, losses not counted)
    .throttle_start_percent = 5.0f,                     // TUNE with the driver
    .throttle_full_percent = 100.0f,
    .throttle_curve_exponent = 1.0f,                    // linear, like the 4WD car of the onboard video
    .brake_pressed_bar = 6.0f,                          // firmware BRAKE_PRESSURE_THRESHOLD (rest reads ~3.1 bar)
    .regen_start_bar = 2.0f,                            // VCU: no regen up to 2 bar (CHECK the brake at rest reads below this)
    .regen_full_bar = 10.0f,                            // VCU: linear up to full regen at 10 bar
    .regen_maximum_fraction = 0.5f,                     // TUNE: half of what the rear tyres can take, the hydraulic brakes need the rest
    .pedal_relative_to_available = true,                // 100 % pedal = what the wheels can take now (false: = the motor limits)
    .pedal_lateral_grip_reserve = 0.8f,                 // the pedal reference keeps 20 % of the lateral grip free for steering
    .steering_linear_gain = 0.180952f,                  // firmware STEERING_LINEAR_GAIN [rad/rad]
    .steering_cubic_gain = 0.003328f,                   // firmware STEERING_CUBIC_GAIN [rad/rad^3]
    .ackermann_fraction = 1.0f,                         // PLACEHOLDER: measure both road wheel angles at full lock
    .tyre_FNOMIN = 1000.0f,
    .tyre_LMUX = 0.6f,                                  // track / TTC belt friction: 0.6 (team 03-11, cold tyres untested: 0.5-0.6)
    .tyre_LMUY = 0.6f,
    .tyre_PCX1 = 1.4262f,
    .tyre_PDX1 = 2.204f,
    .tyre_PDX2 = -0.0461f,
    .tyre_PEX1 = -0.2237f,
    .tyre_PEX2 = -1.0f,
    .tyre_PEX3 = 0.0f,
    .tyre_PEX4 = 0.455f,
    .tyre_PKX1 = 38.4778f,
    .tyre_PKX2 = 0.0f,
    .tyre_PKX3 = -0.57f,
    .tyre_PCY1 = 1.4530291f,
    .tyre_PDY1 = 2.3796794f,
    .tyre_PDY2 = -0.26099519f,
    .tyre_PEY1 = 0.16141392f,
    .tyre_PEY2 = -0.48291836f,
    .tyre_PEY3 = 0.31089406f,
    .tyre_PKY1 = -34.450276f,
    .tyre_PKY2 = 1.3093368f,
    .acceleration_filter_time_constant_s = 0.02f,
    .slip_minimum_reference_speed_mps = 2.0f,
    .kalman_minimum_speed_mps = 3.0f,
    .kalman_process_noise_longitudinal_ms2 = 0.5f,
    .kalman_process_noise_lateral_ms2 = 1.5f,
    .kalman_process_noise_yaw_rad_s2 = 3.0f,
    .kalman_wheel_speed_noise_mps = 0.1f,
    .kalman_gyro_noise_rad_s = 0.01f,
    .kalman_lateral_acceleration_noise_ms2 = 1.0f,
    .no_imu_acceleration_margin = 1.2f,                 // 13-10 replay (replay_logs.py): IMU ax over the limit 0.3 % (1.0: 1.4 %)
    .torque_vectoring_enable = true,
    .torque_vectoring_allows_opposite_torque = false,
    .torque_vectoring_trusts_front_wheel_yaw_rate = false,  // T26: AQT2 at ~5 Hz -> torque vectoring needs the gyro
    .torque_vectoring_minimum_speed_mps = 3.0f,
    .reference_understeer_gradient_rad_s2_per_m = 0.001f,   // TUNE on the skidpad (0.001 = 0.56 deg of extra steer per g)
    .reference_grip_scale = 0.95f,
    .yaw_rate_dead_band_rad_s = 0.03f,
    .yaw_rate_proportional_gain_nm_s_per_rad = 1500.0f,
    .yaw_rate_integral_gain_nm_per_rad = 3000.0f,
    .maximum_yaw_moment_nm = 500.0f,
    .friction_limit_enable = true,
    .friction_limit_scale = 1.1f,
    .traction_control_enable = true,
    .slip_target_scale = 1.0f,
    .slip_proportional_gain = 4.0f,
    .slip_integral_gain_per_s = 30.0f,
    .slip_recovery_per_s = 2.0f,
    .minimum_torque_factor = 0.0f,
};

/* Inputs after the guards */
typedef struct {
    bool wheel_speed_valid[WHEEL_COUNT];
    bool front_wheel_speeds_valid;
    bool steering_valid;
    bool imu_valid;
    bool yaw_rate_valid;
    bool throttle_valid;
    bool brake_pressure_valid;
    float steering_wheel_angle_rad;                   // 0 when not valid
} checked_input_t;

/* What the two-track model needs besides the state; fixed during one step */
typedef struct {
    bool low_speed;                                   // kinematic model (no tyre slip angles)
    bool kinematic;                                   // low speed, or nothing measures vy / r (no IMU, gyro, front wheels)
    bool imu_valid;
    float longitudinal_acceleration_ms2;              // IMU at the centre of gravity, not filtered
    float mean_road_wheel_angle_rad;
    float road_wheel_angle_rad[WHEEL_COUNT];
    float longitudinal_force_n[WHEEL_COUNT];          // previous step command, along the wheel
    float peak_lateral_force_n[WHEEL_COUNT];          // MF D at this step's load
    float lateral_stiffness_factor[WHEEL_COUNT];      // MF B
    float lateral_curvature[WHEEL_COUNT];             // MF E before the sign term
    float lateral_grip_share[WHEEL_COUNT];            // sqrt(1 - (Fx / Dx)^2): what the longitudinal force leaves
} two_track_inputs_t;

/* ============================== small helpers ============================== */

static float clamp(float value, float minimum, float maximum) {
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}

static bool plausible(float value, float maximum) {
    return fabsf(value) <= maximum;     // false for NaN and infinity too
}

static float low_pass(float state, float value, float dt_s, float time_constant_s) {
    return state + (value - state) * dt_s / (time_constant_s + dt_s);
}

/* ============================== Magic Formula 5.2 ============================== */

/* Peak of D sin(C atan(B k - E (B k - atan(B k)))): C atan(phi) = pi / 2, phi = tan(pi / (2 C)).
 * x = B k solves x (1 - E) + E atan(x) = phi; the left side always rises (E < 1), Newton from x = phi. */
static float magic_formula_peak_slip(float stiffness_factor, float curvature, float shape_factor) {
    const float phi = tanf(PI_F / (2.0f * shape_factor));
    const float e = fminf(curvature, MAXIMUM_CURVATURE);
    float x = phi;
    for (int iteration = 0; iteration < 6; iteration++) {
        x -= (x * (1.0f - e) + e * atanf(x) - phi) / ((1.0f - e) + e / (1.0f + x * x));
    }
    return x / stiffness_factor;
}

void magic_formula_longitudinal_peak(const vehicle_dynamics_config_t *config, float load_n, float *peak_force_n,
                                     float *peak_slip_drive, float *peak_slip_brake) {
    const vehicle_dynamics_config_t *c = config;
    const float load = fmaxf(load_n, MINIMUM_LOAD_N);
    const float load_change = (load - c->tyre_FNOMIN) / c->tyre_FNOMIN;                            // dfz [-]
    const float peak_force = (c->tyre_PDX1 + c->tyre_PDX2 * load_change) * c->tyre_LMUX * load;    // D [N]
    const float slip_stiffness = load * (c->tyre_PKX1 + c->tyre_PKX2 * load_change) * expf(c->tyre_PKX3 * load_change);  // K [N]
    const float stiffness_factor = slip_stiffness / (c->tyre_PCX1 * peak_force);                  // B = K / (C D)
    const float curvature = c->tyre_PEX1 + c->tyre_PEX2 * load_change + c->tyre_PEX3 * load_change * load_change;
    *peak_force_n = peak_force;
    *peak_slip_drive = magic_formula_peak_slip(stiffness_factor, curvature * (1.0f - c->tyre_PEX4), c->tyre_PCX1);
    *peak_slip_brake = -magic_formula_peak_slip(stiffness_factor, curvature * (1.0f + c->tyre_PEX4), c->tyre_PCX1);
}

float magic_formula_lateral_peak_force(const vehicle_dynamics_config_t *config, float load_n) {
    const float load = fmaxf(load_n, MINIMUM_LOAD_N);
    const float load_change = (load - config->tyre_FNOMIN) / config->tyre_FNOMIN;
    return (config->tyre_PDY1 + config->tyre_PDY2 * load_change) * config->tyre_LMUY * load;       // camber 0
}

float magic_formula_cornering_stiffness(const vehicle_dynamics_config_t *config, float load_n) {
    // Kya = PKY1 Fz0 sin(2 atan(Fz / (PKY2 Fz0))), camber 0; magnitude (the .tir sign is the ISO side convention)
    return fabsf(config->tyre_PKY1) * config->tyre_FNOMIN
           * sinf(2.0f * atanf(fmaxf(load_n, MINIMUM_LOAD_N) / (config->tyre_PKY2 * config->tyre_FNOMIN)));  // [N/rad]
}

/* Pure lateral force, camber 0, shifts 0: slip angle > 0 -> force > 0 (to the left of the wheel) */
static float magic_formula_lateral_force(const vehicle_dynamics_config_t *config, float slip_angle_rad, float peak_force_n,
                                         float stiffness_factor, float curvature) {
    const float e = fminf(curvature * (1.0f - config->tyre_PEY3 * (slip_angle_rad >= 0.0f ? 1.0f : -1.0f)), 1.0f);
    const float x = stiffness_factor * slip_angle_rad;
    return peak_force_n * sinf(config->tyre_PCY1 * atanf(x - e * (x - atanf(x))));
}

/* ============================== init ============================== */

static void kalman_reset(vehicle_dynamics_t *vehicle) {
    memset(vehicle->kalman_covariance, 0, sizeof(vehicle->kalman_covariance));
    for (int i = 0; i < STATE_COUNT; i++) vehicle->kalman_covariance[i][i] = KALMAN_INITIAL_VARIANCE[i];
}

void vehicle_dynamics_init(vehicle_dynamics_t *vehicle, const vehicle_dynamics_config_t *config) {
    memset(vehicle, 0, sizeof(*vehicle));
    vehicle->config = *config;
    const vehicle_dynamics_config_t *c = &vehicle->config;
    const float front = c->center_of_gravity_to_front_axle_m;           // a
    const float rear = c->wheelbase_m - front;                          // b
    const float x[WHEEL_COUNT] = {front, front, -rear, -rear};
    const float y[WHEEL_COUNT] = {c->track_front_m / 2.0f, -c->track_front_m / 2.0f, c->track_rear_m / 2.0f,
                                  -c->track_rear_m / 2.0f};

    vehicle->motor_torque_constant_nm_per_a = 1.5f * c->motor_pole_pairs * c->motor_flux_linkage_v_s;
    const float maximum_drive_motor_torque = vehicle->motor_torque_constant_nm_per_a * c->motor_maximum_drive_current_a;
    const float maximum_brake_motor_torque = vehicle->motor_torque_constant_nm_per_a * c->motor_maximum_brake_current_a;
    for (int wheel = 0; wheel < WHEEL_COUNT; wheel++) {
        vehicle->wheel_x_from_center_of_gravity_m[wheel] = x[wheel];
        vehicle->wheel_y_from_center_of_gravity_m[wheel] = y[wheel];
        vehicle->static_load_n[wheel] =
            c->mass_kg * GRAVITY_MS2 * (wheel < WHEEL_REAR_LEFT ? rear : front) / (2.0f * c->wheelbase_m);
        if (c->wheel_driven[wheel]) {
            vehicle->maximum_drive_force_n[wheel] =
                maximum_drive_motor_torque * c->gear_ratio * c->gear_efficiency / c->wheel_radius_m;
            vehicle->maximum_brake_force_n[wheel] =
                maximum_brake_motor_torque * c->gear_ratio / (c->gear_efficiency * c->wheel_radius_m);
        }
        vehicle->traction_control_torque_factor[wheel] = 1.0f;
    }
    kalman_reset(vehicle);

    // Linear bicycle model: K = m / L (b / C_front - a / C_rear), axle cornering stiffness at the static loads
    const float front_axle_stiffness = 2.0f * magic_formula_cornering_stiffness(c, vehicle->static_load_n[WHEEL_FRONT_LEFT]);
    const float rear_axle_stiffness = 2.0f * magic_formula_cornering_stiffness(c, vehicle->static_load_n[WHEEL_REAR_LEFT]);
    vehicle->natural_understeer_gradient_rad_s2_per_m =
        c->mass_kg / c->wheelbase_m * (rear / front_axle_stiffness - front / rear_axle_stiffness);
}

/* ============================== 0. guards ============================== */

static checked_input_t check_inputs(const vehicle_dynamics_input_t *input) {
    checked_input_t checked;
    const float *speed = input->wheel_speed_rad_s;
    const bool front = input->front_wheel_speeds_valid && plausible(speed[WHEEL_FRONT_LEFT], MAXIMUM_WHEEL_SPEED_RAD_S)
                       && plausible(speed[WHEEL_FRONT_RIGHT], MAXIMUM_WHEEL_SPEED_RAD_S);
    const bool rear = input->rear_wheel_speeds_valid && plausible(speed[WHEEL_REAR_LEFT], MAXIMUM_WHEEL_SPEED_RAD_S)
                      && plausible(speed[WHEEL_REAR_RIGHT], MAXIMUM_WHEEL_SPEED_RAD_S);
    checked.front_wheel_speeds_valid = front;
    checked.wheel_speed_valid[WHEEL_FRONT_LEFT] = front;
    checked.wheel_speed_valid[WHEEL_FRONT_RIGHT] = front;
    checked.wheel_speed_valid[WHEEL_REAR_LEFT] = rear;
    checked.wheel_speed_valid[WHEEL_REAR_RIGHT] = rear;
    checked.steering_valid = input->steering_valid && plausible(input->steering_wheel_angle_rad, MAXIMUM_STEERING_WHEEL_ANGLE_RAD);
    checked.imu_valid = input->imu_valid && plausible(input->imu_longitudinal_acceleration_ms2, MAXIMUM_ACCELERATION_MS2)
                        && plausible(input->imu_lateral_acceleration_ms2, MAXIMUM_ACCELERATION_MS2);
    checked.yaw_rate_valid = input->yaw_rate_valid && plausible(input->yaw_rate_rad_s, MAXIMUM_YAW_RATE_RAD_S);
    checked.throttle_valid = input->throttle_valid && plausible(input->throttle_percent, MAXIMUM_THROTTLE_PERCENT);
    checked.brake_pressure_valid = input->brake_pressure_valid && plausible(input->brake_pressure_bar, MAXIMUM_BRAKE_PRESSURE_BAR);
    checked.steering_wheel_angle_rad = checked.steering_valid ? input->steering_wheel_angle_rad : 0.0f;
    return checked;
}

/* ============================== 1. road wheel angles ============================== */

static void compute_road_wheel_angles(vehicle_dynamics_t *v, float steering_wheel_angle_rad) {
    const vehicle_dynamics_config_t *c = &v->config;
    const float x = steering_wheel_angle_rad;
    const float mean_angle = c->steering_linear_gain * x + c->steering_cubic_gain * x * x * x;
    const float size = fabsf(mean_angle);
    float inner_angle = size;
    float outer_angle = size;
    if (size > 1e-4f) {
        const float turn_radius = c->wheelbase_m / tanf(size);             // rear axle centre to turn centre [m]
        const float half_track = c->track_front_m / 2.0f;
        inner_angle += c->ackermann_fraction * (atanf(c->wheelbase_m / fmaxf(turn_radius - half_track, 0.1f)) - size);
        outer_angle += c->ackermann_fraction * (atanf(c->wheelbase_m / (turn_radius + half_track)) - size);
    }
    const bool left_turn = mean_angle >= 0.0f;                              // the inner wheel turns more
    const float sign = left_turn ? 1.0f : -1.0f;
    v->mean_road_wheel_angle_rad = mean_angle;
    v->road_wheel_angle_rad[WHEEL_FRONT_LEFT] = sign * (left_turn ? inner_angle : outer_angle);
    v->road_wheel_angle_rad[WHEEL_FRONT_RIGHT] = sign * (left_turn ? outer_angle : inner_angle);
    v->road_wheel_angle_rad[WHEEL_REAR_LEFT] = 0.0f;
    v->road_wheel_angle_rad[WHEEL_REAR_RIGHT] = 0.0f;
}

/* ============================== 2. IMU at the centre of gravity ============================== */

static void move_imu_to_center_of_gravity(vehicle_dynamics_t *v, const vehicle_dynamics_input_t *input, bool imu_valid, float dt) {
    const vehicle_dynamics_config_t *c = &v->config;
    float longitudinal = v->speed_derivative_ms2;                       // without IMU: ax = dvx/dt, ay = vx * r
    float lateral = v->vehicle_speed_mps * v->yaw_rate_rad_s;
    if (imu_valid) {                                                    // yaw rate / acceleration of the last step
        const float imu_x = c->imu_ahead_of_center_of_gravity_m;
        longitudinal = input->imu_longitudinal_acceleration_ms2 + v->yaw_rate_rad_s * v->yaw_rate_rad_s * imu_x;
        lateral = input->imu_lateral_acceleration_ms2 - v->yaw_acceleration_rad_s2 * imu_x;
    }
    v->longitudinal_acceleration_unfiltered_ms2 = longitudinal;
    v->lateral_acceleration_unfiltered_ms2 = lateral;
    v->longitudinal_acceleration_ms2 = low_pass(v->longitudinal_acceleration_ms2, longitudinal, dt, c->acceleration_filter_time_constant_s);
    v->lateral_acceleration_ms2 = low_pass(v->lateral_acceleration_ms2, lateral, dt, c->acceleration_filter_time_constant_s);
}

/* ============================== 3. wheel loads ============================== */

static void compute_wheel_loads(vehicle_dynamics_t *v) {
    const vehicle_dynamics_config_t *c = &v->config;
    const float speed = v->vehicle_speed_mps;                          // of the last step
    const float downforce = 0.5f * AIR_DENSITY_KG_M3 * c->aero_lift_coefficient_area_m2 * speed * speed;
    const float front_downforce = downforce * c->aero_front_share / 2.0f;              // per wheel
    const float rear_downforce = downforce * (1.0f - c->aero_front_share) / 2.0f;
    const float longitudinal_transfer =                                                // per wheel, front -> rear
        c->mass_kg * v->longitudinal_acceleration_ms2 * c->center_of_gravity_height_m / (2.0f * c->wheelbase_m);
    const float lateral_transfer_front =                                               // left -> right in a left turn
        c->mass_kg * v->lateral_acceleration_ms2 * c->center_of_gravity_height_m * c->roll_stiffness_front_share / c->track_front_m;
    const float lateral_transfer_rear =
        c->mass_kg * v->lateral_acceleration_ms2 * c->center_of_gravity_height_m * (1.0f - c->roll_stiffness_front_share) / c->track_rear_m;
    const float load[WHEEL_COUNT] = {
        v->static_load_n[WHEEL_FRONT_LEFT] + front_downforce - longitudinal_transfer - lateral_transfer_front,
        v->static_load_n[WHEEL_FRONT_RIGHT] + front_downforce - longitudinal_transfer + lateral_transfer_front,
        v->static_load_n[WHEEL_REAR_LEFT] + rear_downforce + longitudinal_transfer - lateral_transfer_rear,
        v->static_load_n[WHEEL_REAR_RIGHT] + rear_downforce + longitudinal_transfer + lateral_transfer_rear,
    };
    for (int wheel = 0; wheel < WHEEL_COUNT; wheel++) v->wheel_load_n[wheel] = fmaxf(load[wheel], MINIMUM_LOAD_N);
}

/* ============================== 4. tyre limits ============================== */

static two_track_inputs_t compute_tyre_limits(vehicle_dynamics_t *v, bool imu_valid, bool lateral_measured) {
    const vehicle_dynamics_config_t *c = &v->config;
    const float rear = c->wheelbase_m - c->center_of_gravity_to_front_axle_m;
    const float axle_lateral_force[2] = {                              // steady state: moments about the CG
        c->mass_kg * v->lateral_acceleration_ms2 * rear / c->wheelbase_m,
        c->mass_kg * v->lateral_acceleration_ms2 * c->center_of_gravity_to_front_axle_m / c->wheelbase_m,
    };
    two_track_inputs_t model;
    v->total_lateral_grip_n = 0.0f;
    for (int wheel = 0; wheel < WHEEL_COUNT; wheel++) {
        const float load = v->wheel_load_n[wheel];
        magic_formula_longitudinal_peak(c, load, &v->peak_longitudinal_force_n[wheel], &v->peak_slip_drive[wheel],
                                        &v->peak_slip_brake[wheel]);
        v->peak_lateral_force_n[wheel] = magic_formula_lateral_peak_force(c, load);
        v->total_lateral_grip_n += v->peak_lateral_force_n[wheel];

        // friction ellipse: the lateral force (axle force shared by load) leaves this much for Fx
        const float axle_load = load + v->wheel_load_n[wheel ^ 1];      // wheel ^ 1 = other wheel of the same axle
        v->lateral_force_n[wheel] = axle_lateral_force[wheel / 2] * load / axle_load;
        const float lateral_use = fminf(fabsf(v->lateral_force_n[wheel]) / v->peak_lateral_force_n[wheel], MAXIMUM_TYRE_USE);
        v->friction_limit_n[wheel] = v->peak_longitudinal_force_n[wheel] * sqrtf(1.0f - lateral_use * lateral_use) * c->friction_limit_scale;

        // what this wheel can take NOW: its motor (torque, and power at this speed) and its tyre, keeping a lateral
        // reserve (the lateral force counted as if the tyre had only pedal_lateral_grip_reserve of its lateral grip)
        const float power_limit = c->motor_maximum_power_w * c->gear_efficiency / fmaxf(v->vehicle_speed_mps, 1.0f);
        v->motor_drive_limit_n[wheel] = fminf(v->maximum_drive_force_n[wheel], power_limit);
        const float reserved_use = fminf(fabsf(v->lateral_force_n[wheel]) / (v->peak_lateral_force_n[wheel] * c->pedal_lateral_grip_reserve),
                                         MAXIMUM_TYRE_USE);
        const float tyre_room = v->peak_longitudinal_force_n[wheel] * sqrtf(1.0f - reserved_use * reserved_use);
        v->available_drive_force_n[wheel] = c->friction_limit_enable ? fminf(v->motor_drive_limit_n[wheel], tyre_room) : v->motor_drive_limit_n[wheel];
        v->available_regen_force_n[wheel] = c->friction_limit_enable ? fminf(v->maximum_brake_force_n[wheel], tyre_room) : v->maximum_brake_force_n[wheel];

        // what the Kalman model needs: MF lateral B, E and the room the drive force (last step) leaves for Fy
        const float load_change = (load - c->tyre_FNOMIN) / c->tyre_FNOMIN;
        const float longitudinal_use = fminf(fabsf(v->wheel_force_n[wheel]) / v->peak_longitudinal_force_n[wheel], MAXIMUM_TYRE_USE);
        model.road_wheel_angle_rad[wheel] = v->road_wheel_angle_rad[wheel];
        model.longitudinal_force_n[wheel] = v->wheel_force_n[wheel];
        model.peak_lateral_force_n[wheel] = v->peak_lateral_force_n[wheel];
        model.lateral_stiffness_factor[wheel] = magic_formula_cornering_stiffness(c, load) / (c->tyre_PCY1 * v->peak_lateral_force_n[wheel]);
        model.lateral_curvature[wheel] = c->tyre_PEY1 + c->tyre_PEY2 * load_change;
        model.lateral_grip_share[wheel] = sqrtf(1.0f - longitudinal_use * longitudinal_use);
    }
    // how fast vx can change, for the Kalman filter without the IMU: driven wheels (tyre, motor and power) x margin,
    // and the tyre grip of all four wheels (hydraulic brakes)
    float driven_force = 0.0f, total_grip = 0.0f;
    for (int wheel = 0; wheel < WHEEL_COUNT; wheel++) {
        total_grip += v->peak_longitudinal_force_n[wheel];
        if (c->wheel_driven[wheel]) driven_force += fminf(v->peak_longitudinal_force_n[wheel], v->motor_drive_limit_n[wheel]);
    }
    v->maximum_acceleration_ms2 = c->no_imu_acceleration_margin * driven_force / c->mass_kg;
    v->maximum_deceleration_ms2 = total_grip / c->mass_kg;

    model.low_speed = v->vehicle_speed_mps < c->kalman_minimum_speed_mps;
    model.kinematic = model.low_speed || !lateral_measured;     // the free two-track model drifts with nothing to correct it
    model.imu_valid = imu_valid;
    model.longitudinal_acceleration_ms2 = v->longitudinal_acceleration_unfiltered_ms2;
    model.mean_road_wheel_angle_rad = v->mean_road_wheel_angle_rad;
    return model;
}

/* ============================== 4b. driver request from the pedals ============================== */

/* Pedals -> shares; share x the force the driven wheels can take NOW (pedal_relative_to_available) or x the motor
 * limits -> total longitudinal force the driver asks for [N] */
static void compute_driver_request(vehicle_dynamics_t *v, const vehicle_dynamics_input_t *input, const checked_input_t *checked) {
    const vehicle_dynamics_config_t *c = &v->config;
    const float throttle = checked->throttle_valid ? input->throttle_percent : 0.0f;
    const float brake = checked->brake_pressure_valid ? input->brake_pressure_bar : 0.0f;
    const bool brake_pressed = brake >= c->brake_pressed_bar;

    // FS rules APPS / brake plausibility: latched until the throttle is released
    if (brake_pressed && throttle > PEDAL_PLAUSIBILITY_CUT_PERCENT) v->pedal_plausibility_cut = true;
    if (throttle < PEDAL_PLAUSIBILITY_RESET_PERCENT) v->pedal_plausibility_cut = false;

    // throttle -> share of the drive force (pedal curve); the brake has priority
    const float pedal = clamp((throttle - c->throttle_start_percent) / (c->throttle_full_percent - c->throttle_start_percent), 0.0f, 1.0f);
    const bool drive_allowed = checked->throttle_valid && checked->brake_pressure_valid && !brake_pressed && !v->pedal_plausibility_cut;
    v->throttle_fraction = drive_allowed ? powf(pedal, c->throttle_curve_exponent) : 0.0f;

    // brake pressure -> share of the regen force (regen only from the brake pedal)
    const float brake_share = clamp((brake - c->regen_start_bar) / (c->regen_full_bar - c->regen_start_bar), 0.0f, 1.0f);
    v->regen_fraction = brake_share * c->regen_maximum_fraction;

    // 100 % = what the driven wheels can take now (or the motors' limits)
    float total_drive = 0.0f, total_regen = 0.0f;
    for (int wheel = 0; wheel < WHEEL_COUNT; wheel++) {
        total_drive += c->pedal_relative_to_available ? v->available_drive_force_n[wheel] : v->maximum_drive_force_n[wheel];
        total_regen += c->pedal_relative_to_available ? v->available_regen_force_n[wheel] : v->maximum_brake_force_n[wheel];
    }
    v->driver_force_request_n = v->throttle_fraction * total_drive - v->regen_fraction * total_regen;
}

/* ============================== 5. Kalman filter ============================== */

/* Two-track model, state (vx, vy, r) -> derivative, and the lateral acceleration (at the CG) it predicts.
 *   vx' = ax_imu + r vy                     (the IMU measures the longitudinal dynamics: drag, brakes, slope)
 *   vy' = sum(Fy) / m - r vx               (MF lateral force per wheel at its slip angle and load)
 *   r'  = sum(x Fy - y Fx) / Iz            (includes the yaw moment of the left / right drive difference)
 * Low speed, or no IMU / gyro / front wheels (nothing would correct vy and r: the free model drifts, test 13-10 r -6 rad/s):
 * kinematic, vy -> 0 and r -> vx tan(steer) / L, within the grip limit reference_grip_scale * lateral grip / m / vx. */
static void two_track_model(const vehicle_dynamics_t *vehicle, const two_track_inputs_t *in, const float state[STATE_COUNT],
                            float derivative[STATE_COUNT], float *lateral_acceleration) {
    const vehicle_dynamics_config_t *c = &vehicle->config;
    const float vx = state[STATE_VX], vy = state[STATE_VY], r = state[STATE_YAW_RATE];
    derivative[STATE_VX] = (in->imu_valid ? in->longitudinal_acceleration_ms2 : 0.0f) + r * vy;
    if (in->kinematic) {
        const float maximum_yaw_rate = c->reference_grip_scale * vehicle->total_lateral_grip_n / c->mass_kg / fmaxf(vx, 0.5f);
        const float yaw_rate = clamp(vx * tanf(in->mean_road_wheel_angle_rad) / c->wheelbase_m, -maximum_yaw_rate, maximum_yaw_rate);
        derivative[STATE_VY] = -vy / LOW_SPEED_TIME_CONSTANT_S;
        derivative[STATE_YAW_RATE] = (yaw_rate - r) / LOW_SPEED_TIME_CONSTANT_S;
        *lateral_acceleration = vx * r;
        return;
    }
    float force_y = 0.0f, moment_z = 0.0f;
    for (int wheel = 0; wheel < WHEEL_COUNT; wheel++) {
        const float x = vehicle->wheel_x_from_center_of_gravity_m[wheel];
        const float y = vehicle->wheel_y_from_center_of_gravity_m[wheel];
        const float angle = in->road_wheel_angle_rad[wheel];
        const float slip_angle = angle - atan2f(vy + r * x, vx - r * y);
        const float wheel_y = in->lateral_grip_share[wheel] * magic_formula_lateral_force(c, slip_angle, in->peak_lateral_force_n[wheel],
                                                                                         in->lateral_stiffness_factor[wheel], in->lateral_curvature[wheel]);
        const float wheel_x = in->longitudinal_force_n[wheel];
        const float car_x = wheel_x * cosf(angle) - wheel_y * sinf(angle);    // wheel frame -> car frame
        const float car_y = wheel_x * sinf(angle) + wheel_y * cosf(angle);
        force_y += car_y;
        moment_z += x * car_y - y * car_x;
    }
    derivative[STATE_VY] = force_y / c->mass_kg - r * vx;
    derivative[STATE_YAW_RATE] = moment_z / c->yaw_inertia_kg_m2;
    *lateral_acceleration = force_y / c->mass_kg;
}

/* One scalar measurement z = h x + noise: x += K innovation, P -= K h P, K = P h' / (h P h' + R) */
static void kalman_update(float state[STATE_COUNT], float covariance[STATE_COUNT][STATE_COUNT], const float measurement_row[STATE_COUNT],
                          float innovation, float noise_variance) {
    float covariance_h[STATE_COUNT];
    float innovation_variance = noise_variance;
    for (int i = 0; i < STATE_COUNT; i++) {
        covariance_h[i] = covariance[i][0] * measurement_row[0] + covariance[i][1] * measurement_row[1]
                        + covariance[i][2] * measurement_row[2];
        innovation_variance += measurement_row[i] * covariance_h[i];
    }
    if (!(innovation_variance > 0.0f)) return;
    for (int i = 0; i < STATE_COUNT; i++) {
        state[i] += covariance_h[i] / innovation_variance * innovation;
        for (int j = 0; j < STATE_COUNT; j++) covariance[i][j] -= covariance_h[i] * covariance_h[j] / innovation_variance;
    }
}

typedef struct {                                      // what the lateral acceleration correction needs from the prediction
    float linearisation_state[STATE_COUNT];           // state where the model was evaluated
    float lateral_acceleration_ms2;                   // ay the model predicts there
    float lateral_acceleration_gradient[STATE_COUNT]; // d(ay) / d(vx, vy, r)
} kalman_prediction_t;

/* Predict: x += f(x) dt, P = F P F' + Q dt with F = I + J dt, J = df/dx by finite differences */
static kalman_prediction_t kalman_predict(vehicle_dynamics_t *v, const two_track_inputs_t *model, float state[STATE_COUNT],
                                          bool imu_valid, float dt) {
    const vehicle_dynamics_config_t *c = &v->config;
    float (*covariance)[STATE_COUNT] = v->kalman_covariance;
    kalman_prediction_t prediction;
    float derivative[STATE_COUNT], jacobian[STATE_COUNT][STATE_COUNT];
    two_track_model(v, model, state, derivative, &prediction.lateral_acceleration_ms2);
    for (int j = 0; j < STATE_COUNT; j++) {
        float shifted[STATE_COUNT] = {state[0], state[1], state[2]};
        float shifted_derivative[STATE_COUNT], shifted_lateral;
        shifted[j] += KALMAN_JACOBIAN_STEP[j];
        two_track_model(v, model, shifted, shifted_derivative, &shifted_lateral);
        for (int i = 0; i < STATE_COUNT; i++) jacobian[i][j] = (shifted_derivative[i] - derivative[i]) / KALMAN_JACOBIAN_STEP[j];
        prediction.lateral_acceleration_gradient[j] = (shifted_lateral - prediction.lateral_acceleration_ms2) / KALMAN_JACOBIAN_STEP[j];
    }

    float transition[STATE_COUNT][STATE_COUNT], product[STATE_COUNT][STATE_COUNT];
    for (int i = 0; i < STATE_COUNT; i++) {
        for (int j = 0; j < STATE_COUNT; j++) transition[i][j] = (i == j ? 1.0f : 0.0f) + jacobian[i][j] * dt;
    }
    for (int i = 0; i < STATE_COUNT; i++) {                            // product = F P
        for (int j = 0; j < STATE_COUNT; j++) {
            product[i][j] = transition[i][0] * covariance[0][j] + transition[i][1] * covariance[1][j] + transition[i][2] * covariance[2][j];
        }
    }
    for (int i = 0; i < STATE_COUNT; i++) {                            // P = F P F'
        for (int j = 0; j < STATE_COUNT; j++) {
            covariance[i][j] = product[i][0] * transition[j][0] + product[i][1] * transition[j][1] + product[i][2] * transition[j][2];
        }
    }
    const float process_noise[STATE_COUNT] = {imu_valid ? c->kalman_process_noise_longitudinal_ms2 : KALMAN_IMU_MISSING_NOISE_MS2,
                                              c->kalman_process_noise_lateral_ms2, c->kalman_process_noise_yaw_rad_s2};
    for (int i = 0; i < STATE_COUNT; i++) {
        prediction.linearisation_state[i] = state[i];
        covariance[i][i] += process_noise[i] * process_noise[i] * dt;
        state[i] += derivative[i] * dt;
    }
    v->yaw_acceleration_rad_s2 = derivative[STATE_YAW_RATE];
    v->kalman_lateral_acceleration_ms2 = prediction.lateral_acceleration_ms2;
    return prediction;
}

/* Wheels. A free wheel (no motor) rolls with the ground: wheel speed * R = (vx - r y) cos(steer) + (vy + r x) sin(steer),
 * that measures vx AND r. Driven wheels (or any wheel braking hard) may slip: they only bound vx (driving: the slowest
 * >= vx, braking: the fastest <= vx) or measure it when coasting. Returns true if a free wheel was used. */
static bool kalman_correct_with_wheels(vehicle_dynamics_t *v, const vehicle_dynamics_input_t *input, const checked_input_t *checked,
                                       float state[STATE_COUNT], float dt) {
    const vehicle_dynamics_config_t *c = &v->config;
    const bool braking = v->longitudinal_acceleration_ms2 < BRAKING_ACCELERATION_MS2;
    const float noise = c->kalman_wheel_speed_noise_mps * c->kalman_wheel_speed_noise_mps;
    // new ERPM frame = a driven wheel speed changed: the wheels moved over the whole time since the last one
    bool new_frame = false;
    for (int wheel = 0; wheel < WHEEL_COUNT; wheel++) {
        if (c->wheel_driven[wheel] && input->wheel_speed_rad_s[wheel] != v->previous_wheel_speed_rad_s[wheel]) new_frame = true;
        v->previous_wheel_speed_rad_s[wheel] = input->wheel_speed_rad_s[wheel];
    }
    v->wheel_speed_age_s = fminf(v->wheel_speed_age_s + dt, MAXIMUM_WHEEL_FRAME_AGE_S);
    if (v->wheel_speed_age_s >= MAXIMUM_WHEEL_FRAME_AGE_S) new_frame = true;  // same value again (standing, repeat)
    const float frame_time_s = new_frame ? v->wheel_speed_age_s : 0.0f;
    if (new_frame) v->wheel_speed_age_s = 0.0f;
    bool free_wheel_used = false;
    float slowest = 1e9f, fastest = -1e9f, sum = 0.0f;
    int bound_count = 0;
    for (int wheel = 0; wheel < WHEEL_COUNT; wheel++) {
        if (!checked->wheel_speed_valid[wheel]) continue;
        const float x = v->wheel_x_from_center_of_gravity_m[wheel];
        const float y = v->wheel_y_from_center_of_gravity_m[wheel];
        const float cosine = cosf(v->road_wheel_angle_rad[wheel]);
        const float sine = sinf(v->road_wheel_angle_rad[wheel]);
        const float surface_speed = input->wheel_speed_rad_s[wheel] * c->wheel_radius_m;
        if (!c->wheel_driven[wheel] && !braking) {
            const float row[STATE_COUNT] = {cosine, sine, -y * cosine + x * sine};
            const float predicted = (state[STATE_VX] - state[STATE_YAW_RATE] * y) * cosine + (state[STATE_VY] + state[STATE_YAW_RATE] * x) * sine;
            kalman_update(state, v->kalman_covariance, row, surface_speed - predicted, noise);
            free_wheel_used = true;
        } else {                                                        // vx this wheel says, if it rolled
            const float speed = (surface_speed - (state[STATE_VY] + state[STATE_YAW_RATE] * x) * sine) / cosine + state[STATE_YAW_RATE] * y;
            slowest = fminf(slowest, speed);
            fastest = fmaxf(fastest, speed);
            sum += speed;
            bound_count++;
        }
    }
    if (!free_wheel_used && bound_count > 0) {
        const float request = v->driver_force_request_n;
        float measured = sum / (float)bound_count;                    // coasting: they roll with the car
        bool apply = true;
        if (braking || request < -REQUEST_DEAD_BAND_N) {
            measured = fastest;                                       // lower bound
            apply = measured > state[STATE_VX];
        } else if (request > REQUEST_DEAD_BAND_N) {
            measured = slowest;                                       // upper bound
            apply = measured < state[STATE_VX];
        }
        if (!checked->imu_valid) {                                    // nothing else moves vx: the wheels always do, per frame,
            measured = clamp(measured, state[STATE_VX] - v->maximum_deceleration_ms2 * frame_time_s,   // at most as fast
                             state[STATE_VX] + v->maximum_acceleration_ms2 * frame_time_s);            // as the car can
            apply = true;
        }
        if (apply) {
            const float row[STATE_COUNT] = {1.0f, 0.0f, 0.0f};
            kalman_update(state, v->kalman_covariance, row, measured - state[STATE_VX], noise);
        }
    }
    v->vehicle_speed_from_wheels = free_wheel_used || bound_count > 0;
    return free_wheel_used;
}

static void run_kalman_filter(vehicle_dynamics_t *v, const vehicle_dynamics_input_t *input, const checked_input_t *checked,
                              const two_track_inputs_t *model, float dt) {
    const vehicle_dynamics_config_t *c = &v->config;
    float (*covariance)[STATE_COUNT] = v->kalman_covariance;
    // first step with a wheel speed: vx starts at the slowest wheel, not at 0. From 0 (ready to drive while rolling) the
    // driven wheels read as spinning until vx catches up at the acceleration limit: seconds of traction control cut
    if (!v->speed_started) {
        float slowest = 1e9f;
        for (int wheel = 0; wheel < WHEEL_COUNT; wheel++) {
            if (checked->wheel_speed_valid[wheel]) slowest = fminf(slowest, input->wheel_speed_rad_s[wheel] * c->wheel_radius_m);
        }
        if (slowest < 1e9f) {
            v->vehicle_speed_mps = slowest;
            v->speed_started = true;
        }
    }
    float state[STATE_COUNT] = {v->vehicle_speed_mps, v->lateral_velocity_mps, v->yaw_rate_rad_s};
    const kalman_prediction_t prediction = kalman_predict(v, model, state, checked->imu_valid, dt);

    v->yaw_rate_source = YAW_RATE_SOURCE_NONE;
    if (checked->yaw_rate_valid) {                                    // gyro -> r
        const float row[STATE_COUNT] = {0.0f, 0.0f, 1.0f};
        kalman_update(state, covariance, row, input->yaw_rate_rad_s - state[STATE_YAW_RATE], c->kalman_gyro_noise_rad_s * c->kalman_gyro_noise_rad_s);
        v->yaw_rate_source = YAW_RATE_SOURCE_GYRO;
    }
    if (kalman_correct_with_wheels(v, input, checked, state, dt) && v->yaw_rate_source == YAW_RATE_SOURCE_NONE) {
        v->yaw_rate_source = YAW_RATE_SOURCE_FRONT_WHEELS;
    }
    if (checked->imu_valid && !model->low_speed) {                    // ay against the tyre model -> vy
        float predicted = prediction.lateral_acceleration_ms2;
        for (int j = 0; j < STATE_COUNT; j++) predicted += prediction.lateral_acceleration_gradient[j] * (state[j] - prediction.linearisation_state[j]);
        const float noise = c->kalman_lateral_acceleration_noise_ms2;
        kalman_update(state, covariance, prediction.lateral_acceleration_gradient, v->lateral_acceleration_unfiltered_ms2 - predicted, noise * noise);
        if (v->yaw_rate_source == YAW_RATE_SOURCE_NONE) v->yaw_rate_source = YAW_RATE_SOURCE_LATERAL_ACCELERATION;
    }
    // a sideslip beyond this is the tyre model past its peak, not the car: the ay correction runs away (13-10 replay
    // with the IMU, rear wheels locking in a corner: vy -9.8 m/s at 10 m/s, then vx' = ax + r vy took vx to 3 m/s)
    const float maximum_lateral_velocity = MAXIMUM_SIDESLIP * fmaxf(state[STATE_VX], 1.0f);
    state[STATE_VY] = clamp(state[STATE_VY], -maximum_lateral_velocity, maximum_lateral_velocity);

    // keep P symmetric and positive; an impossible state (should not happen) restarts the filter
    for (int i = 0; i < STATE_COUNT; i++) {
        covariance[i][i] = fmaxf(covariance[i][i], 1e-6f);
        for (int j = i + 1; j < STATE_COUNT; j++) covariance[i][j] = covariance[j][i] = 0.5f * (covariance[i][j] + covariance[j][i]);
    }
    if (!(plausible(state[STATE_VX], 200.0f) && plausible(state[STATE_VY], 100.0f) && plausible(state[STATE_YAW_RATE], 20.0f))) {
        state[STATE_VX] = plausible(v->vehicle_speed_mps, 200.0f) ? v->vehicle_speed_mps : 0.0f;
        state[STATE_VY] = 0.0f;
        state[STATE_YAW_RATE] = 0.0f;
        kalman_reset(v);
    }
    const float previous_speed = v->vehicle_speed_mps;
    v->vehicle_speed_mps = fmaxf(state[STATE_VX], 0.0f);
    v->speed_derivative_ms2 = low_pass(v->speed_derivative_ms2, (v->vehicle_speed_mps - previous_speed) / dt, dt,
                                       SPEED_DERIVATIVE_TIME_CONSTANT_S);
    v->lateral_velocity_mps = state[STATE_VY];
    v->yaw_rate_rad_s = state[STATE_YAW_RATE];
    v->sideslip_angle_rad = atan2f(v->lateral_velocity_mps, fmaxf(v->vehicle_speed_mps, 0.5f));
}

/* ============================== 6. slip ============================== */

static void compute_slip(vehicle_dynamics_t *v, const vehicle_dynamics_input_t *input, const checked_input_t *checked, bool low_speed) {
    const vehicle_dynamics_config_t *c = &v->config;
    const float vx = v->vehicle_speed_mps, vy = v->lateral_velocity_mps, r = v->yaw_rate_rad_s;
    for (int wheel = 0; wheel < WHEEL_COUNT; wheel++) {
        const float x = v->wheel_x_from_center_of_gravity_m[wheel];
        const float y = v->wheel_y_from_center_of_gravity_m[wheel];
        const float angle = v->road_wheel_angle_rad[wheel];
        const float centre_speed = (vx - r * y) * cosf(angle) + (vy + r * x) * sinf(angle);    // along the wheel
        const float surface_speed = input->wheel_speed_rad_s[wheel] * c->wheel_radius_m;
        v->slip_ratio[wheel] = checked->wheel_speed_valid[wheel]
            ? (surface_speed - centre_speed) / fmaxf(fabsf(centre_speed), c->slip_minimum_reference_speed_mps) : 0.0f;
        v->slip_angle_rad[wheel] = low_speed ? 0.0f : angle - atan2f(vy + r * x, vx - r * y);
    }
}

/* ============================== 7. torque vectoring ============================== */

/* PI controller on the yaw rate. Needs a MEASURED, FRESH yaw rate: from ay alone the filter cannot tell vy from r
 * at the grip limit (test: yaw rate error 0.17 rad/s, spin); front wheel speeds only if they arrive fast enough
 * (T26 AQT2 ~5 Hz: 200 ms old, the slalom test spins with torque vectoring and not without), else only the gyro. */
static void run_torque_vectoring(vehicle_dynamics_t *v, bool steering_valid, float dt) {
    const vehicle_dynamics_config_t *c = &v->config;
    const float vx = v->vehicle_speed_mps;
    const bool reference_available = steering_valid && vx > c->torque_vectoring_minimum_speed_mps;
    const bool yaw_rate_measured = v->yaw_rate_source == YAW_RATE_SOURCE_GYRO
        || (v->yaw_rate_source == YAW_RATE_SOURCE_FRONT_WHEELS && c->torque_vectoring_trusts_front_wheel_yaw_rate);
    v->torque_vectoring_active = c->torque_vectoring_enable && reference_available && yaw_rate_measured;

    // reference (also shown with torque vectoring off): linear bicycle model, capped by the grip
    v->yaw_rate_reference_rad_s = 0.0f;
    v->yaw_rate_error_rad_s = 0.0f;
    if (reference_available) {
        const float maximum_yaw_rate = c->reference_grip_scale * v->total_lateral_grip_n / c->mass_kg / vx;
        const float bicycle = vx * v->mean_road_wheel_angle_rad / (c->wheelbase_m + c->reference_understeer_gradient_rad_s2_per_m * vx * vx);
        v->yaw_rate_reference_rad_s = clamp(bicycle, -maximum_yaw_rate, maximum_yaw_rate);
        const float error = v->yaw_rate_reference_rad_s - v->yaw_rate_rad_s;
        const float band = c->yaw_rate_dead_band_rad_s;
        v->yaw_rate_error_rad_s = error > band ? error - band : (error < -band ? error + band : 0.0f);
    }

    const float limit = c->maximum_yaw_moment_nm;
    if (v->torque_vectoring_active) {
        // anti-windup: the wheels fell short of the last request (range, traction control): no growth that way
        const float growth = c->yaw_rate_integral_gain_nm_per_rad * v->yaw_rate_error_rad_s * dt;
        const float shortfall = v->yaw_moment_request_nm - v->yaw_moment_delivered_nm;     // of the last step
        if (!(growth * shortfall > 0.0f && fabsf(shortfall) > YAW_MOMENT_TOLERANCE_NM)) {
            v->yaw_moment_integral_term_nm = clamp(v->yaw_moment_integral_term_nm + growth, -limit, limit);
        }
        v->yaw_moment_request_nm = clamp(c->yaw_rate_proportional_gain_nm_s_per_rad * v->yaw_rate_error_rad_s + v->yaw_moment_integral_term_nm,
                                         -limit, limit);
    } else {
        v->yaw_moment_integral_term_nm = 0.0f;
        v->yaw_moment_request_nm = 0.0f;
    }
}

/* ============================== 8. allocation ============================== */

/* Range of one driven wheel's force: motor (torque and power), friction ellipse, and never against the driver's
 * direction (unless torque_vectoring_allows_opposite_torque). Always low <= 0 <= high. */
static void wheel_force_range(const vehicle_dynamics_t *v, int wheel, float total_force, float *low, float *high) {
    const vehicle_dynamics_config_t *c = &v->config;
    float minimum = -v->maximum_brake_force_n[wheel];
    float maximum = v->motor_drive_limit_n[wheel];
    if (!c->torque_vectoring_allows_opposite_torque) {
        if (total_force >= 0.0f) minimum = 0.0f;
        if (total_force <= 0.0f) maximum = 0.0f;
    }
    if (c->friction_limit_enable) {
        minimum = fmaxf(minimum, -v->friction_limit_n[wheel]);
        maximum = fminf(maximum, v->friction_limit_n[wheel]);
    }
    *low = fminf(minimum, maximum);
    *high = maximum;
}

/* Left [0] and right [1] wheel of one axle, each within [low, high], with right - left = difference (the yaw moment)
 * and left + right = sum between 0 and the most the axle can make towards the request: never more than the driver asks.
 * The difference comes first (the sum gives way, down to 0), then the sum closest to the request; the difference gives
 * way only when no sum allows it. = the rule-based LLC of Colucci 2025, appendix A, for two wheels; closed form
 * (eliminate the sum from the 2 unknowns), no QP solver.
 * ponytail: no floor on the sum (the thesis QP keeps >= 80 % of the request): with one wheel capped by traction
 * control, a floor forces a yaw moment of the WRONG sign (inner wheel spinning under a stabilising Mz). If the drivers
 * miss the drive, add a floor for agility moments only (Mz with the yaw rate). */
static void split_axle(float sum_request, float difference, const float low[2], const float high[2], float force[2]) {
    const float sum_free = clamp(sum_request, low[0] + low[1], high[0] + high[1]);    // the sum with no yaw moment
    const float sum_minimum = fminf(sum_free, 0.0f), sum_maximum = fmaxf(sum_free, 0.0f);
    // differences that some sum in [sum_minimum, sum_maximum] allows, both wheels in range (never empty: low <= 0 <= high)
    const float difference_minimum = fmaxf(fmaxf(low[1] - high[0], 2.0f * low[1] - sum_maximum), sum_minimum - 2.0f * high[0]);
    const float difference_maximum = fminf(fminf(high[1] - low[0], sum_maximum - 2.0f * low[0]), 2.0f * high[1] - sum_minimum);
    const float d = clamp(difference, difference_minimum, difference_maximum);
    // the sum closest to the request that this difference allows
    const float sum_low = fmaxf(fmaxf(2.0f * low[0] + d, 2.0f * low[1] - d), sum_minimum);
    const float sum_high = fminf(fminf(2.0f * high[0] + d, 2.0f * high[1] - d), sum_maximum);
    const float sum = clamp(sum_request, sum_low, sum_high);
    force[0] = clamp(0.5f * (sum - d), low[0], high[0]);                // rounding guard
    force[1] = clamp(0.5f * (sum + d), low[1], high[1]);
}

/* Driver force -> driven axles by axle load, each wheel within its range (motor, power, friction ellipse).
 * Torque vectoring off: each wheel of an axle EQUAL shares (the pedal never makes a yaw moment by itself), a wheel that
 * cannot take its share leaves it (traction control cuts only the spinning wheel).
 * Torque vectoring on: split_axle, right - left = 2 Mz / track on each driven axle (Mz > 0 = turn left): when one wheel
 * cannot take its part (range, traction control) the other wheel gives way, Mz is kept, the total drops.
 * factor: NULL = first pass; else traction control: a wheel at most factor x its first-pass force (reference), where
 * factor < 1. Also sets yaw_moment_delivered_nm of this allocation. */
static void allocate_wheel_forces(vehicle_dynamics_t *v, const float *factor, const float *reference, float force[WHEEL_COUNT]) {
    const vehicle_dynamics_config_t *c = &v->config;
    const float total_force = v->driver_force_request_n;
    int driven_wheels_on_axle[2] = {0, 0};
    for (int wheel = 0; wheel < WHEEL_COUNT; wheel++) {
        if (c->wheel_driven[wheel]) driven_wheels_on_axle[wheel / 2]++;
    }
    const float front_load = v->wheel_load_n[WHEEL_FRONT_LEFT] + v->wheel_load_n[WHEEL_FRONT_RIGHT];
    const float rear_load = v->wheel_load_n[WHEEL_REAR_LEFT] + v->wheel_load_n[WHEEL_REAR_RIGHT];
    float front_share = 0.0f;                                           // of the total force and of Mz
    if (driven_wheels_on_axle[0] > 0) front_share = driven_wheels_on_axle[1] > 0 ? front_load / (front_load + rear_load) : 1.0f;
    const float axle_share[2] = {front_share, 1.0f - front_share};
    const float axle_track[2] = {c->track_front_m, c->track_rear_m};

    float low[WHEEL_COUNT] = {0}, high[WHEEL_COUNT] = {0};
    for (int wheel = 0; wheel < WHEEL_COUNT; wheel++) {
        force[wheel] = 0.0f;
        if (!c->wheel_driven[wheel]) continue;
        wheel_force_range(v, wheel, total_force, &low[wheel], &high[wheel]);
        if (factor != NULL && factor[wheel] < 1.0f) {                   // traction control: less than it got
            if (reference[wheel] > 0.0f) high[wheel] = fminf(high[wheel], factor[wheel] * reference[wheel]);
            if (reference[wheel] < 0.0f) low[wheel] = fmaxf(low[wheel], factor[wheel] * reference[wheel]);
        }
    }
    v->yaw_moment_delivered_nm = 0.0f;
    for (int axle = 0; axle < 2; axle++) {
        const int left = 2 * axle, right = left + 1;
        const float axle_force = total_force * axle_share[axle];
        if (driven_wheels_on_axle[axle] == 2 && v->torque_vectoring_active) {
            // (F_right - F_left) * track / 2 = this axle's part of Mz
            split_axle(axle_force, 2.0f * v->yaw_moment_request_nm * axle_share[axle] / axle_track[axle], &low[left], &high[left],
                       &force[left]);
        } else {
            for (int wheel = left; wheel <= right; wheel++) {
                if (c->wheel_driven[wheel]) force[wheel] = clamp(axle_force / (float)driven_wheels_on_axle[axle], low[wheel], high[wheel]);
            }
        }
        v->yaw_moment_delivered_nm += (force[right] - force[left]) * axle_track[axle] / 2.0f;
    }
}

/* ============================== 9. traction control ============================== */

/* PI controller per wheel on the slip ratio beyond the MF peak (driving and regen) -> torque factor. Only ever reduces
 * the force: the factor caps the wheel in the second allocation, so torque vectoring keeps Mz with the other wheel. */
static void run_traction_control(vehicle_dynamics_t *v, const checked_input_t *checked, float dt) {
    const vehicle_dynamics_config_t *c = &v->config;
    for (int wheel = 0; wheel < WHEEL_COUNT; wheel++) {
        const float force = v->wheel_force_allocated_n[wheel];
        const bool active = c->traction_control_enable && c->wheel_driven[wheel] && checked->wheel_speed_valid[wheel]
                            && v->vehicle_speed_from_wheels;
        if (!active) {
            v->traction_control_integral_term[wheel] = 0.0f;
            v->traction_control_torque_factor[wheel] = 1.0f;
            continue;
        }
        float slip_excess = -1.0f;                                    // no force: recover
        if (force > 0.0f) slip_excess = v->slip_ratio[wheel] - c->slip_target_scale * v->peak_slip_drive[wheel];
        if (force < 0.0f) slip_excess = c->slip_target_scale * v->peak_slip_brake[wheel] - v->slip_ratio[wheel];
        const float proportional_term = c->slip_proportional_gain * fmaxf(slip_excess, 0.0f);
        const float integral_change = slip_excess > 0.0f ? c->slip_integral_gain_per_s * slip_excess : -c->slip_recovery_per_s;
        v->traction_control_integral_term[wheel] = clamp(v->traction_control_integral_term[wheel] + integral_change * dt, 0.0f,
                                                         1.0f - c->minimum_torque_factor);
        v->traction_control_torque_factor[wheel] = clamp(1.0f - proportional_term - v->traction_control_integral_term[wheel],
                                                         c->minimum_torque_factor, 1.0f);
    }
}

/* ============================== 10. torque and current ============================== */

/* Wheel force -> wheel torque -> motor torque (gear ratio; losses: driving the motor gives more, braking it gets less)
 * -> Iq by the DTI equation torque = 1.5 * pole pairs * flux linkage * Iq, within the inverter current limits */
static vehicle_dynamics_torque_t convert_to_torque(const vehicle_dynamics_t *v) {
    const vehicle_dynamics_config_t *c = &v->config;
    vehicle_dynamics_torque_t torque;
    vehicle_dynamics_wheel_torque_t *const by_wheel[WHEEL_COUNT] = {&torque.front_left, &torque.front_right, &torque.rear_left,
                                                                    &torque.rear_right};
    for (int wheel = 0; wheel < WHEEL_COUNT; wheel++) {
        const float wheel_torque = v->wheel_force_n[wheel] * c->wheel_radius_m;
        const bool driving = wheel_torque >= 0.0f;
        const float motor_torque = c->wheel_driven[wheel]
            ? (driving ? wheel_torque / (c->gear_ratio * c->gear_efficiency) : wheel_torque * c->gear_efficiency / c->gear_ratio)
            : 0.0f;
        float current = clamp(motor_torque / v->motor_torque_constant_nm_per_a,
                              -c->motor_maximum_brake_current_a, c->motor_maximum_drive_current_a);
        if (!plausible(current, 1.0e4f)) current = 0.0f;              // last guard before the inverter: never NaN
        by_wheel[wheel]->wheel_torque_nm = wheel_torque;
        by_wheel[wheel]->motor_torque_nm = motor_torque;
        by_wheel[wheel]->motor_current_a = current;
        by_wheel[wheel]->maximum_drive_wheel_torque_nm = v->available_drive_force_n[wheel] * c->wheel_radius_m;
        by_wheel[wheel]->maximum_regen_wheel_torque_nm = v->available_regen_force_n[wheel] * c->wheel_radius_m;
    }
    return torque;
}

/* ============================== the layer, one call per control period ============================== */

vehicle_dynamics_torque_t vehicle_dynamics_compute(vehicle_dynamics_t *vehicle, const vehicle_dynamics_input_t *input, float dt_s) {
    const float dt = dt_s > 0.0f ? fminf(dt_s, MAXIMUM_DT_S) : 0.001f;     // NaN or <= 0 -> 1 ms
    const checked_input_t checked = check_inputs(input);                 // 0
    compute_road_wheel_angles(vehicle, checked.steering_wheel_angle_rad);  // 1
    move_imu_to_center_of_gravity(vehicle, input, checked.imu_valid, dt);  // 2
    compute_wheel_loads(vehicle);                                        // 3
    const two_track_inputs_t model = compute_tyre_limits(vehicle, checked.imu_valid,    // 4
                                                         checked.imu_valid || checked.yaw_rate_valid || checked.front_wheel_speeds_valid);
    compute_driver_request(vehicle, input, &checked);                    // 4b
    run_kalman_filter(vehicle, input, &checked, &model, dt);             // 5
    compute_slip(vehicle, input, &checked, model.low_speed);             // 6
    run_torque_vectoring(vehicle, checked.steering_valid, dt);           // 7
    allocate_wheel_forces(vehicle, NULL, NULL, vehicle->wheel_force_allocated_n);   // 8
    run_traction_control(vehicle, &checked, dt);                         // 9
    allocate_wheel_forces(vehicle, vehicle->traction_control_torque_factor, vehicle->wheel_force_allocated_n,
                          vehicle->wheel_force_n);                       // 9b: again, traction control as caps
    return convert_to_torque(vehicle);                                   // 10
}
