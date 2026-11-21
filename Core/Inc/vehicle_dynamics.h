/**
 * @file vehicle_dynamics.h
 * @brief Vehicle dynamics layer: pedals, wheel speeds, steering and IMU in -> torque and motor current per wheel out.
 *
 * HOW TO USE (hardware free: no HAL, no CAN, no globals)
 *   static vehicle_dynamics_t vehicle_dynamics;                                    // Live Expressions
 *   vehicle_dynamics_init(&vehicle_dynamics, &VEHICLE_DYNAMICS_T26_CONFIG);        // once
 *   every control period:
 *     vehicle_dynamics_torque_t torque = vehicle_dynamics_compute(&vehicle_dynamics, &input, 0.005f);
 *     current = torque.rear_left.motor_current_a;          (and torque.rear_right, front_left, front_right)
 *     current >= 0 -> SetAcCurrent    (INV*_CMD_TargetAcCurrent,    0.1 A, signed)   = target Iq
 *     current <  0 -> SetBrakeCurrent (INV*_CMD_TargetBrakeCurrent, 0.1 A, -current)  brakes towards 0 rpm,
 *                    never drives the motor backwards like a negative SetAcCurrent could at standstill
 *   DTI torque equation (surface PMSM, Id = 0): motor torque = 1.5 * pole pairs * flux linkage * Iq.
 *   Feedback to compare with: INV*_Actual_FOC_iq / INV*_TargetIq (A peak) in powertrain_t26.dbc.
 *
 * Conventions: inside, arrays in the order WHEEL_FRONT_LEFT, WHEEL_FRONT_RIGHT, WHEEL_REAR_LEFT, WHEEL_REAR_RIGHT.
 * Axes x forward, y left, z up (ISO 8855): yaw rate > 0, steering > 0, lateral acceleration > 0 = turning LEFT.
 *
 * WHAT vehicle_dynamics_compute() DOES, one function per step (every result is a field of vehicle_dynamics_t):
 *  0. check_inputs: a NaN, infinite or out-of-range sensor value = that sensor not valid this step; dt capped at 50 ms.
 *  1. compute_road_wheel_angles: steering wheel angle x -> mean road wheel angle = linear * x + cubic * x^3, then
 *     ACKERMANN: turn centre on the rear axle line at R = L / tan(mean); inner wheel atan(L / (R - t/2)),
 *     outer atan(L / (R + t/2)), blended with the mean by ackermann_fraction (1 = full Ackermann, 0 = parallel).
 *  2. move_imu_to_center_of_gravity: ax_cg = ax_imu + r^2 x_imu, ay_cg = ay_imu - yaw acceleration x_imu;
 *     a first-order low-pass copy feeds the wheel loads.
 *  3. compute_wheel_loads (quasi-static): static + aero downforce (front share) + longitudinal transfer
 *     m ax h / L + lateral transfer m ay h * roll stiffness share / track.
 *  4. compute_tyre_limits, MAGIC FORMULA 5.2 (coefficients of the .tir) per wheel at its load: peak longitudinal
 *     force, slip ratio of that peak (driving and braking), peak lateral force; friction ellipse = longitudinal
 *     room left by the lateral force (from ay, shared by load); tyre data for the Kalman model.
 *     TORQUE AVAILABLE NOW per wheel = min(motor torque, motor power / speed, tyre room with a lateral reserve):
 *     the tyre room counts the lateral force as if the tyre had only pedal_lateral_grip_reserve of its lateral grip,
 *     so full pedal in a corner still leaves grip to steer. Returned as maximum_drive / maximum_regen torque.
 *  4b. compute_driver_request, the force the driver asks for, from the PEDALS:
 *     throttle share = ((throttle % - start) / (full - start)) ^ curve exponent, 0..1; regen share = (brake bar -
 *     regen start) / (regen full - regen start) * regen maximum fraction (no regen from lifting the throttle).
 *     Driver force = throttle share x sum of the available drive forces - regen share x sum of the available regen
 *     forces (pedal_relative_to_available; false: x the motor limits). The brake has priority: brake pressed -> no
 *     drive. FS rules APPS / brake plausibility: brake pressed with more than 25 % throttle -> no drive until the
 *     throttle is back under 5 %. Throttle or brake sensor not valid -> no drive.
 *  5. run_kalman_filter, EXTENDED KALMAN FILTER, state vx, vy, r on the 4-WHEEL (two-track) MODEL with the MF lateral
 *     force of each wheel at its slip angle and load:  vx' = ax_imu + r vy,  vy' = sum(Fy) / m - r vx,
 *     r' = sum(x Fy - y Fx) / Iz  (Jacobian by finite differences). One scalar correction per sensor, no matrix
 *     inverse: gyro -> r; each free-rolling wheel -> its speed along the wheel = (vx - r y) cos(steer)
 *     + (vy + r x) sin(steer); driven wheels only bound vx (driving: slowest >= vx, braking: fastest <= vx,
 *     coasting: their mean); IMU ay against the model's sum(Fy) / m (this makes vy, the sideslip, observable).
 *     Below kalman_minimum_speed_mps: kinematic model.
 *  6. compute_slip: slip ratio (wheel speed * R - wheel centre speed along the wheel) / max(|centre|, minimum)
 *     and slip angle per wheel, with the estimated vy.
 *  7. run_torque_vectoring, PROPORTIONAL-INTEGRAL (PI) controller on the yaw rate. Reference from the linear
 *     BICYCLE MODEL: r_ref = vx * steer / (L + K vx^2), capped at the grip limit / vx. Yaw moment Mz = PI(r_ref - r)
 *     outside a dead band. Only with a MEASURED, FRESH yaw rate (gyro, or front wheels if trusted). Anti-windup: the
 *     integral does not grow while the wheels fall short of the last request (yaw_moment_delivered_nm).
 *  8. allocate_wheel_forces over the 4 WHEELS: driver force -> driven axles by axle load, each wheel within its friction
 *     ellipse and its motor (torque and power). Torque vectoring off: EQUAL per wheel (the pedal alone never makes a
 *     yaw moment). On: right - left = 2 Mz / track on each driven axle (Mz > 0 = turn left), the sum never above the
 *     driver's request; when a wheel cannot take its part the OTHER wheel gives way, Mz is kept and the total drops
 *     (rule-based LLC of Colucci 2025, KTH / PoliTO, closed form for two wheels).
 *  9. run_traction_control, PI controller per wheel on the slip ratio above the MF peak slip (driving and regen):
 *     torque factor = 1 - proportional term - integral term; the integral fades at recovery_per_s in grip.
 *     9b. allocate_wheel_forces AGAIN with each wheel capped at factor x its step 8 force: a cut wheel no longer
 *     breaks (or turns round) the yaw moment, the other wheel follows it.
 * 10. convert_to_torque: wheel force -> wheel torque -> motor torque (gear ratio, gear efficiency) -> Iq [A] by the
 *     DTI equation, within the inverter current limits.
 *
 * FALLBACKS: steering not valid -> no torque vectoring. No measured, fresh yaw rate -> no torque vectoring (ay
 * alone through the tyre model cannot tell vy from r at the grip limit; the T26 front wheel speeds, ~5 Hz, count
 * only with torque_vectoring_trusts_front_wheel_yaw_rate). IMU not valid -> ax = low-pass of dvx/dt, lateral from
 * vx * r, vx from the wheels. IMU and front wheels not valid -> vx follows the driven wheels (driving: the slowest,
 * braking: the fastest, coasting: their mean) but rises at most maximum_acceleration_ms2 and falls at most
 * maximum_deceleration_ms2 over the time since the last ERPM frame (at most 0.1 s): a spinning or locking wheel
 * cannot drag vx (the reference speed of the T26 rpm-derivative traction control). No valid wheel speed -> no
 * traction control. Everything off (and
 * pedal_relative_to_available false) -> each driven motor gets the pedal share of its current limit.
 *
 * Python twin: vehicle_dynamics.py (reads the config of vehicle_dynamics.c). Tests: test_vehicle_dynamics.py,
 * arm_test/run_arm.py (this C on an emulated Cortex-M7 against the twin). GUI: gui_vehicle_dynamics.py.
 */

#ifndef VEHICLE_DYNAMICS_H
#define VEHICLE_DYNAMICS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { WHEEL_FRONT_LEFT = 0, WHEEL_FRONT_RIGHT, WHEEL_REAR_LEFT, WHEEL_REAR_RIGHT, WHEEL_COUNT };

enum {                              // best yaw rate information the Kalman filter had this step
    YAW_RATE_SOURCE_NONE = 0,
    YAW_RATE_SOURCE_GYRO,           // IMU gyro
    YAW_RATE_SOURCE_FRONT_WHEELS,   // free-rolling (front) wheel speeds
    YAW_RATE_SOURCE_LATERAL_ACCELERATION,  // only ay through the tyre model: estimate kept, torque vectoring off
};

/* What the VCU measured this period, in SI units */
typedef struct {
    float wheel_speed_rad_s[WHEEL_COUNT];  // wheel angular speed (motor speed / gear ratio for driven wheels) [rad/s]
    bool front_wheel_speeds_valid;         // both front wheels (AQT2) fresh
    bool rear_wheel_speeds_valid;          // both rear wheels (inverter ERPM) fresh
    float steering_wheel_angle_rad;        // steering wheel angle, offset removed, > 0 = turning left [rad]
    bool steering_valid;
    float imu_longitudinal_acceleration_ms2;  // > 0 = speeding up [m/s2]
    float imu_lateral_acceleration_ms2;       // > 0 = towards the left [m/s2]
    bool imu_valid;
    float yaw_rate_rad_s;                  // gyro z, > 0 = turning left [rad/s] (optional: yaw_rate_valid = false)
    bool yaw_rate_valid;
    float throttle_percent;                // accelerator pedal (APPS), 0 = released, 100 = floored [%]
    bool throttle_valid;                   // APPS fresh and plausible (both sensors agree)
    float brake_pressure_bar;              // hydraulic brake pressure [bar] (the circuit the BSPD reads)
    bool brake_pressure_valid;
} vehicle_dynamics_input_t;

/* One wheel of the result (all 0 for a wheel without motor) */
typedef struct {
    float wheel_torque_nm;                 // at the wheel, > 0 = driving [N m]
    float motor_torque_nm;                 // at the motor shaft [N m]
    float motor_current_a;                 // Iq the inverter must make [A peak]: >= 0 SetAcCurrent, < 0 SetBrakeCurrent(-value)
    float maximum_drive_wheel_torque_nm;   // most this wheel can take now while driving: motor, power, tyre with reserve [N m]
    float maximum_regen_wheel_torque_nm;   // most it can take now in regen (>= 0) [N m]
} vehicle_dynamics_wheel_torque_t;

/* What vehicle_dynamics_compute() returns */
typedef struct {
    vehicle_dynamics_wheel_torque_t front_left;
    vehicle_dynamics_wheel_torque_t front_right;
    vehicle_dynamics_wheel_torque_t rear_left;
    vehicle_dynamics_wheel_torque_t rear_right;
} vehicle_dynamics_torque_t;

/* Every value of the layer. One number per line in vehicle_dynamics.c (the Python twin parses them). */
typedef struct {
    // Car
    float mass_kg;                                // car + driver
    float yaw_inertia_kg_m2;                      // Iz about the centre of gravity
    float wheelbase_m;
    float center_of_gravity_to_front_axle_m;
    float center_of_gravity_height_m;
    float track_front_m;
    float track_rear_m;
    float roll_stiffness_front_share;             // share of the lateral load transfer taken by the front axle
    float imu_ahead_of_center_of_gravity_m;       // IMU x position - centre of gravity x position
    float aero_lift_coefficient_area_m2;          // Cl * A (downforce > 0)
    float aero_front_share;                       // share of the downforce on the front axle
    // Wheels, gearbox, motors, inverters
    float wheel_radius_m;                         // rolling radius: speed = wheel speed * radius
    float gear_ratio;                             // motor speed / wheel speed
    float gear_efficiency;
    bool wheel_driven[WHEEL_COUNT];               // wheel has a motor
    float motor_pole_pairs;                       // DTI torque = 1.5 * pole pairs * flux linkage * Iq
    float motor_flux_linkage_v_s;
    float motor_maximum_drive_current_a;          // inverter MaxAcCurrent [A peak]
    float motor_maximum_brake_current_a;          // inverter brake current limit [A peak, > 0]
    float motor_maximum_power_w;                  // per motor: drive force <= this / speed [W]
    // Driver pedals -> torque request
    float throttle_start_percent;                 // pedal travel below this = no torque
    float throttle_full_percent;                  // pedal travel from this = full torque
    float throttle_curve_exponent;                // 1 = torque proportional to the pedal, > 1 = finer at small pedal
    float brake_pressed_bar;                      // brake actuated: no drive torque, pedal plausibility check
    float regen_start_bar;                        // regen starts at this brake pressure
    float regen_full_bar;                         // and is at its maximum from this one
    float regen_maximum_fraction;                 // regen at full brake pressure, share of the available regen force
    bool pedal_relative_to_available;             // true: 100 % pedal = what the wheels can take now; false: = motor limits
    float pedal_lateral_grip_reserve;             // tyre room for the pedal counts the lateral force / this (< 1 = reserve)
    // Steering (road wheel angle = linear * x + cubic * x^3, x = steering wheel angle)
    float steering_linear_gain;
    float steering_cubic_gain;
    float ackermann_fraction;                     // 1 = full Ackermann, 0 = parallel steer
    // Tyre: Magic Formula 5.2, names as in the .tir (scaling factors LFZO, LCX, LEX, LKX, LKY = 1; shifts 0; camber 0)
    float tyre_FNOMIN;
    float tyre_LMUX;
    float tyre_LMUY;
    float tyre_PCX1;
    float tyre_PDX1;
    float tyre_PDX2;
    float tyre_PEX1;
    float tyre_PEX2;
    float tyre_PEX3;
    float tyre_PEX4;
    float tyre_PKX1;
    float tyre_PKX2;
    float tyre_PKX3;
    float tyre_PCY1;
    float tyre_PDY1;
    float tyre_PDY2;
    float tyre_PEY1;
    float tyre_PEY2;
    float tyre_PEY3;
    float tyre_PKY1;
    float tyre_PKY2;
    // Estimation
    float acceleration_filter_time_constant_s;    // low-pass of ax, ay for the wheel loads
    float slip_minimum_reference_speed_mps;       // slip ratio denominator never below this
    // Extended Kalman filter (standard deviations)
    float kalman_minimum_speed_mps;               // below: kinematic model, no ay correction
    float kalman_process_noise_longitudinal_ms2;  // how much vx' may differ from ax_imu + r vy
    float kalman_process_noise_lateral_ms2;       // vy' model error
    float kalman_process_noise_yaw_rad_s2;        // r' model error
    float kalman_wheel_speed_noise_mps;
    float kalman_gyro_noise_rad_s;
    float kalman_lateral_acceleration_noise_ms2;  // IMU noise + tyre model error
    float no_imu_acceleration_margin;             // no IMU: vx rises at most this x what the driven wheels can push / m
    // Torque vectoring (PI controller on the yaw rate)
    bool torque_vectoring_enable;
    bool torque_vectoring_allows_opposite_torque; // one wheel may regen while the other drives
    bool torque_vectoring_trusts_front_wheel_yaw_rate;  // true only if the front wheel speeds arrive at >= ~50 Hz
    float torque_vectoring_minimum_speed_mps;
    float reference_understeer_gradient_rad_s2_per_m;  // K of the bicycle model reference
    float reference_grip_scale;                   // reference yaw rate <= this * grip limit / vx
    float yaw_rate_dead_band_rad_s;
    float yaw_rate_proportional_gain_nm_s_per_rad;
    float yaw_rate_integral_gain_nm_per_rad;
    float maximum_yaw_moment_nm;
    // Friction limit (feedforward, friction ellipse from the MF peaks)
    bool friction_limit_enable;
    float friction_limit_scale;                   // > 1: only caps beyond the model grip
    // Traction control (PI controller on the slip ratio, per wheel)
    bool traction_control_enable;
    float slip_target_scale;                      // slip target = this * MF peak slip
    float slip_proportional_gain;                 // torque cut per unit of slip over target
    float slip_integral_gain_per_s;               // learned cut grows this per unit of slip over target, per second
    float slip_recovery_per_s;                    // learned cut fades this per second once back under target
    float minimum_torque_factor;                  // never less than this x the allocated force
} vehicle_dynamics_config_t;

/* Everything the layer remembers and every intermediate result, by step */
typedef struct {
    vehicle_dynamics_config_t config;             // own copy: gains can be changed live

    // Fixed at init
    float static_load_n[WHEEL_COUNT];
    float wheel_x_from_center_of_gravity_m[WHEEL_COUNT];
    float wheel_y_from_center_of_gravity_m[WHEEL_COUNT];
    float motor_torque_constant_nm_per_a;         // 1.5 * pole pairs * flux linkage
    float maximum_drive_force_n[WHEEL_COUNT];     // motor limit at the tyre (0 without motor)
    float maximum_brake_force_n[WHEEL_COUNT];     // motor brake limit at the tyre
    float natural_understeer_gradient_rad_s2_per_m;  // linear bicycle model at static loads (information only)

    // 1. Road wheel angles
    float mean_road_wheel_angle_rad;
    float road_wheel_angle_rad[WHEEL_COUNT];
    // 2. IMU at the centre of gravity
    float longitudinal_acceleration_unfiltered_ms2;   // Kalman filter input
    float lateral_acceleration_unfiltered_ms2;
    float longitudinal_acceleration_ms2;          // low-pass, for the loads
    float lateral_acceleration_ms2;
    // 3-4. Loads and tyre limits
    float wheel_load_n[WHEEL_COUNT];
    float peak_longitudinal_force_n[WHEEL_COUNT];
    float peak_lateral_force_n[WHEEL_COUNT];
    float total_lateral_grip_n;                   // sum of the lateral peaks: grip limit = this / mass
    float peak_slip_drive[WHEEL_COUNT];           // > 0
    float peak_slip_brake[WHEEL_COUNT];           // < 0
    float lateral_force_n[WHEEL_COUNT];
    float friction_limit_n[WHEEL_COUNT];          // allocation cap: friction ellipse x friction_limit_scale
    float motor_drive_limit_n[WHEEL_COUNT];       // motor torque and power at this speed, at the tyre
    float available_drive_force_n[WHEEL_COUNT];   // what the wheel can take now: motor, power, tyre with lateral reserve
    float available_regen_force_n[WHEEL_COUNT];
    float maximum_acceleration_ms2;               // most the car can speed up now: driven wheels (grip, motor) x margin / m
    float maximum_deceleration_ms2;               // most it can slow down now: grip of all four wheels / m (> 0)
    // 4b. Driver request from the pedals
    float throttle_fraction;                      // after the pedal curve, 0..1
    float regen_fraction;                         // from the brake pressure, 0..regen_maximum_fraction
    bool pedal_plausibility_cut;                  // brake + > 25 % throttle seen: no drive until throttle < 5 %
    float driver_force_request_n;                 // total longitudinal force the driver asks for, > 0 = drive [N]
    // 5. Kalman filter
    bool speed_started;                           // vx started from the wheels (first step with a wheel speed)
    float vehicle_speed_mps;                      // vx
    float lateral_velocity_mps;                   // vy
    float yaw_rate_rad_s;                         // r
    float yaw_acceleration_rad_s2;                // r' of the model
    float sideslip_angle_rad;                     // atan(vy / vx)
    float kalman_covariance[3][3];                // P of (vx, vy, r)
    float kalman_lateral_acceleration_ms2;        // ay the tyre model predicts
    uint8_t yaw_rate_source;
    bool vehicle_speed_from_wheels;               // a wheel speed corrected vx this step
    float speed_derivative_ms2;                   // low-pass of dvx/dt: the longitudinal acceleration without the IMU
    float previous_wheel_speed_rad_s[WHEEL_COUNT];   // a change = a new ERPM frame (every ~33 ms)
    float wheel_speed_age_s;                      // time since the driven wheel speeds last changed
    // 6. Slip
    float slip_ratio[WHEEL_COUNT];
    float slip_angle_rad[WHEEL_COUNT];
    // 7. Torque vectoring
    bool torque_vectoring_active;
    float yaw_rate_reference_rad_s;
    float yaw_rate_error_rad_s;
    float yaw_moment_integral_term_nm;
    float yaw_moment_request_nm;
    float yaw_moment_delivered_nm;                // what the final wheel forces make (< request: range or traction control)
    // 8-9. Allocation and traction control
    float wheel_force_allocated_n[WHEEL_COUNT];   // before traction control
    float traction_control_integral_term[WHEEL_COUNT];
    float traction_control_torque_factor[WHEEL_COUNT];
    float wheel_force_n[WHEEL_COUNT];             // final
} vehicle_dynamics_t;

extern const vehicle_dynamics_config_t VEHICLE_DYNAMICS_T26_CONFIG;

void vehicle_dynamics_init(vehicle_dynamics_t *vehicle, const vehicle_dynamics_config_t *config);
vehicle_dynamics_torque_t vehicle_dynamics_compute(vehicle_dynamics_t *vehicle, const vehicle_dynamics_input_t *input, float dt_s);

/* Magic Formula 5.2 helpers (also used by the tests) */
void magic_formula_longitudinal_peak(const vehicle_dynamics_config_t *config, float load_n, float *peak_force_n,
                                     float *peak_slip_drive, float *peak_slip_brake);
float magic_formula_lateral_peak_force(const vehicle_dynamics_config_t *config, float load_n);
float magic_formula_cornering_stiffness(const vehicle_dynamics_config_t *config, float load_n);

#ifdef __cplusplus
}
#endif

#endif /* VEHICLE_DYNAMICS_H */
