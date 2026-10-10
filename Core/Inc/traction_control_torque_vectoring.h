/**
 * @file traction_control_torque_vectoring.h
 * @brief Traction control and torque vectoring on the two rear motors - first, light version
 *
 * Runs at the end of throttle_control_update() (step 8, after the digital BSPD). Input: the drive
 * command (per mille, one value for both motors). Output: one drive command per motor.
 * Regen does not go through here (both motors keep the same brake current).
 *
 * Three parts, in this order, each with its own global for Live Expressions:
 *
 * 1. VEHICLE SENSORS ("vehicle_sensors") - no IMU on the car:
 *      front wheel speed  = wheel rpm * 2 pi R * 60 / 1000                       [km/h], per front wheel
 *      rear wheel speed   = ERPM / pole pairs / gear ratio * 2 pi R * 60 / 1000   [km/h], per rear motor
 *      road wheel angle   = linear * x + cubic * x^3, x = (steering angle - offset) * sign  [rad], > 0 = left
 *      vehicle speed      = mean front wheel speed * cos(road wheel angle)
 *      lateral accel.     = vehicle speed^2 * tan(road wheel angle) / wheelbase  (kinematic, steady state)
 *
 * 2. TORQUE VECTORING ("torque_vectoring") - FEEDFORWARD, open loop: no PI or PID controller,
 *    there is no yaw rate sensor to close the loop with.
 *      shift = TORQUE_VECTORING_GAIN_PER_G * lateral acceleration [g],
 *              within +-TORQUE_VECTORING_MAXIMUM_SHIFT, moves at most TORQUE_VECTORING_SHIFT_RATE_PER_S
 *      left  = drive * (1 - shift), right = drive * (1 + shift)   left turn: the outer (right) gets more
 *      Each capped at 1000 (full pedal: the inner wheel loses, the outer cannot gain).
 *      Off below TORQUE_VECTORING_MINIMUM_SPEED_KMH.
 *
 * 3. TRACTION CONTROL ("traction_control") - one PI CONTROLLER (proportional + integral) per rear
 *    wheel. Not a PID: no derivative term, the derivative of a wheel speed error is mostly sensor
 *    noise. The reference is the front wheel on the SAME side: same lateral position -> same
 *    longitudinal speed in a corner, so the inner/outer speed difference is already in it.
 *      allowed speed     = front wheel speed * cos(road wheel angle) * (1 + slip target) + speed margin
 *      error             = rear wheel speed - allowed speed                  [km/h], > 0 = wheel spinning
 *      proportional term = TRACTION_CONTROL_PROPORTIONAL_GAIN * max(error, 0)
 *      integral term     = integral term + TRACTION_CONTROL_INTEGRAL_GAIN * error * dt,
 *                          within [0, 1 - minimum torque factor]  (anti-windup)
 *      torque factor     = 1 - proportional term - integral term, within [minimum torque factor, 1]
 *      Only ever reduces torque. Off below TRACTION_CONTROL_MINIMUM_SPEED_KMH of front wheel speed
 *      (launch: the throttle map limits the torque).
 *
 * FALLBACK: no front wheel speed frame for SENSOR_TIMEOUT_MS -> traction control and torque
 * vectoring off; no steering frame -> torque vectoring off, traction control with a 0 deg road wheel
 * angle. Off = both motors get the same command, the car as before.
 *
 * Simulator twin: Powertrain V1 - Simulator/model/vcu/TractionControlTorqueVectoring.m
 * (values in loadParams.m section 10).
 *
 * BEFORE THE FIRST RUN (Live Expressions: vehicle_sensors, torque_vectoring, traction_control)
 *   1. Car on stands, spin the LEFT rear wheel by hand: vehicle_sensors.rear_left_speed_kmh moves,
 *      otherwise swap INVERTER_ID_REAR_LEFT.
 *   2. Wheels straight: vehicle_sensors.road_wheel_angle_deg = 0 (STEERING_OFFSET_DEG). Turn LEFT:
 *      > 0 (STEERING_SIGN). Road wheel angle vs steering angle: STEERING_LINEAR_GAIN / STEERING_CUBIC_GAIN.
 *   3. Straight, constant speed: front wheel speeds = rear wheel speeds (FRONT_WHEEL_RADIUS_M).
 *   4. TRACTION_CONTROL_ENABLE 1 alone first. Then TORQUE_VECTORING_ENABLE 1: skidpad both ways.
 *
 * CAN inputs (raw decode, IDs from the generated T26_DBC code):
 *   AQT2  FRONT_LEFT_WHEEL_RPM bytes 0-1, FRONT_RIGHT_WHEEL_RPM bytes 2-3, uint16 LE, 1 rpm
 *         data bus, DATA_T26_AQT2_FRAME_ID
 *   AQT4  ST_ANGLE bytes 0-1, int16 LE, 0.1 deg - autonomous bus AUTONOMOUS_T26_AQT4_FRAME_ID AND
 *         powertrain bus POWERTRAIN_T26_AQT4_FRAME_ID, newest of either (redundancy)
 */

#ifndef TRACTION_CONTROL_TORQUE_VECTORING_H
#define TRACTION_CONTROL_TORQUE_VECTORING_H

#include <stdbool.h>
#include <stdint.h>

#include "can_queue.h"

/* ============================== SWITCHES ============================= */

#define TRACTION_CONTROL_ENABLE  1  // 1 = traction control on
#define TORQUE_VECTORING_ENABLE  1  // 1 = torque vectoring on

/* ================================ CAR ================================ */

#define INVERTER_ID_REAR_LEFT   1                            // Inverter on the rear LEFT wheel (check, step 1)
#define INVERTER_ID_REAR_RIGHT  (3 - INVERTER_ID_REAR_LEFT)
#define VEHICLE_WHEELBASE_M     1.55f
// Road wheel angle from the steering wheel angle (sensor on the column, 1:1 with the wheel).
// Measured map for ONE front wheel, wheel angle a [rad] -> steering wheel [deg]:
//   steering = -49.3021 a^3 + 90.5065 a^2 + 312.5504 a
// The vehicle model uses the MEAN of both front wheels: inverted, averaged left/right (the a^2 term
// is Ackermann, the inner wheel steers more, and cancels) and fitted as wheel = linear x + cubic x^3,
// x = steering wheel [rad], max error 0.3 deg up to +-150 deg. Mean ratio 5.45 at the centre, 5.14 at 120 deg.
#define STEERING_LINEAR_GAIN    0.180952f                    // x term [rad/rad]
#define STEERING_CUBIC_GAIN     0.003328f                    // x^3 term [rad/rad^3]
#define STEERING_SIGN           1.0f                         // -1.0f if turning left gives a negative steering angle
#define STEERING_OFFSET_DEG     5.0f                         // Steering angle with the wheels straight
#define FRONT_WHEEL_RADIUS_M    THROTTLE_WHEEL_RADIUS_M      // Front wheel speed calibration (step 3)
#define SENSOR_TIMEOUT_MS       300                          // Frame older than this -> sensor missing

/* ========================== TRACTION CONTROL ========================= */
// Simulation (loadParams.m section 10): tyre peak at slip 0.097, 97 % of the peak force at 0.15.

#define TRACTION_CONTROL_SLIP_TARGET             0.08f  // Slip allowed before cutting [-]
#define TRACTION_CONTROL_SPEED_MARGIN_KMH        4.0f   // + this speed [km/h] (sensor noise, radius error)
#define TRACTION_CONTROL_MINIMUM_SPEED_KMH       2.0f   // Off below this front wheel speed [km/h]
#define TRACTION_CONTROL_PROPORTIONAL_GAIN       0.08f  // PI controller, proportional: torque cut per km/h [-/(km/h)]
#define TRACTION_CONTROL_INTEGRAL_GAIN           0.6f   // PI controller, integral: torque cut per km/h per second
#define TRACTION_CONTROL_MINIMUM_TORQUE_FACTOR   0.3f   // Never less than this x the request (light: at most -70 %)

/* ========================== TORQUE VECTORING ========================= */

#define TORQUE_VECTORING_GAIN_PER_G         0.10f  // Torque shift per g of lateral acceleration [-/g]
#define TORQUE_VECTORING_MAXIMUM_SHIFT      0.15f  // Maximum shift: +-15 % of the drive command
#define TORQUE_VECTORING_MINIMUM_SPEED_KMH  5.0f  // Off below this vehicle speed [km/h]
#define TORQUE_VECTORING_SHIFT_RATE_PER_S   1.0f   // Shift changes at most this per second (0 -> 0.15 in 0.15 s)

/* =============================== TYPES =============================== */

// 1. Vehicle sensors
typedef struct {
    // CAN inputs
    uint16_t front_left_wheel_rpm;      // AQT2
    uint16_t front_right_wheel_rpm;
    float steering_angle_raw_deg;       // AQT4
    uint32_t front_wheel_frame_time_ms; // HAL tick of the last frame
    uint32_t steering_frame_time_ms;

    // As used by the controllers
    bool front_wheel_speed_valid;       // Frame newer than SENSOR_TIMEOUT_MS
    bool steering_angle_valid;
    float front_left_speed_kmh;
    float front_right_speed_kmh;
    float rear_left_speed_kmh;          // From the motor ERPM
    float rear_right_speed_kmh;
    float road_wheel_angle_deg;         // > 0 = left
    float vehicle_speed_kmh;            // Mean front wheel speed along the car
    float lateral_acceleration_g;       // Kinematic, > 0 = left turn
} vehicle_sensors_t;

// 2. Torque vectoring (feedforward)
typedef enum {  // Sent in VCU_states.tv_state (powertrain DBC) - keep its value table in sync
    TORQUE_VECTORING_STATE_DISABLED = 0,          // TORQUE_VECTORING_ENABLE 0
    TORQUE_VECTORING_STATE_NO_FRONT_WHEEL_SPEED,  // Off: AQT2 missing
    TORQUE_VECTORING_STATE_NO_STEERING_ANGLE,     // Off: AQT4 missing
    TORQUE_VECTORING_STATE_BELOW_MIN_SPEED,       // Off: below TORQUE_VECTORING_MINIMUM_SPEED_KMH
    TORQUE_VECTORING_STATE_STRAIGHT,              // On, no shift
    TORQUE_VECTORING_STATE_MORE_TORQUE_RIGHT,     // On, shift > 0 (left turn)
    TORQUE_VECTORING_STATE_MORE_TORQUE_LEFT,      // On, shift < 0 (right turn)
    TORQUE_VECTORING_STATE_AT_MAX_SHIFT,          // On, shift at +-TORQUE_VECTORING_MAXIMUM_SHIFT
} torque_vectoring_state_t;

typedef struct {
    float shift;                        // > 0: right wheel x (1 + shift), left wheel x (1 - shift)
    torque_vectoring_state_t state;
} torque_vectoring_t;

// 3. Traction control: PI controller of one rear wheel
typedef struct {
    float speed_error_kmh;              // Wheel speed - allowed speed (> 0 = spinning)
    float proportional_term;
    float integral_term;
    float torque_factor;                // 1 - proportional - integral, 1 = no cut
} traction_control_wheel_t;

typedef enum {  // Sent in VCU_states.tc_state (powertrain DBC) - keep its value table in sync
    TRACTION_CONTROL_STATE_DISABLED = 0,          // TRACTION_CONTROL_ENABLE 0
    TRACTION_CONTROL_STATE_NO_FRONT_WHEEL_SPEED,  // Off: AQT2 missing
    TRACTION_CONTROL_STATE_NO_DRIVE,              // No drive command (pedal released, regen, BSPD, ...)
    TRACTION_CONTROL_STATE_BELOW_MIN_SPEED,       // Off: front wheels below TRACTION_CONTROL_MINIMUM_SPEED_KMH
    TRACTION_CONTROL_STATE_GRIP,                  // On, not cutting
    TRACTION_CONTROL_STATE_CUT_REAR_LEFT,         // Cutting the rear left motor
    TRACTION_CONTROL_STATE_CUT_REAR_RIGHT,        // Cutting the rear right motor
    TRACTION_CONTROL_STATE_CUT_BOTH,              // Cutting both
} traction_control_state_t;

typedef struct {
    traction_control_wheel_t rear_left;
    traction_control_wheel_t rear_right;
    traction_control_state_t state;
} traction_control_t;

extern vehicle_sensors_t vehicle_sensors;
extern torque_vectoring_t torque_vectoring;
extern traction_control_t traction_control;

/* ============================= PUBLIC API ============================ */

/** @brief Clear both controllers (keeps the sensor values). Called by throttle_control_reset(). */
void traction_control_torque_vectoring_reset(void);

/** @brief Read the front wheel speed (CAN1) and steering (CAN2, CAN3) frames. Call for every received frame. */
void vehicle_sensors_can_receive(const can_msg_t *message);

/**
 * @brief Recompute vehicle_sensors from the last frames and the motor ERPM (no state kept: safe to
 *        call twice). Called by the control step, and every 10 ms in all states for VCU_states and
 *        Live Expressions (the control step only runs in READY_MANUAL).
 */
void vehicle_sensors_update(int32_t rear_left_erpm, int32_t rear_right_erpm, uint32_t now_ms);

/**
 * @brief One control step: sensors -> torque vectoring -> traction control -> one drive command per motor
 * @param drive_command_1000        drive command after the BSPD (same for both motors), 0..1000
 * @param rear_left_erpm            ERPM of the motor on the rear left wheel (sign ignored)
 * @param rear_right_erpm           ERPM of the motor on the rear right wheel
 * @param now_ms                    HAL tick
 * @param time_step_ms              time since the last step
 * @param drive_command_left_1000   [out] drive command of the rear left motor
 * @param drive_command_right_1000  [out] drive command of the rear right motor
 */
void traction_control_torque_vectoring_update(uint16_t drive_command_1000, int32_t rear_left_erpm,
                                              int32_t rear_right_erpm, uint32_t now_ms, uint32_t time_step_ms,
                                              uint16_t *drive_command_left_1000, uint16_t *drive_command_right_1000);

#endif  // TRACTION_CONTROL_TORQUE_VECTORING_H
