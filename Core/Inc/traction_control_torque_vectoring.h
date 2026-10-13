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
 * 3. TRACTION CONTROL ("traction_control") - per rear wheel, from the DERIVATIVE of its motor rpm:
 *    a wheel that rolls with the car cannot speed up faster than the car can accelerate, so a wheel
 *    that does is spinning. Needs only the motor ERPM: no front wheel speed (AQT2 reads 0 above
 *    ~4 km/h, tests 08-10 and 10-10), no IMU.
 *      wheel acceleration = (wheel speed - wheel speed of the newest ERPM frame at least
 *                           TRACTION_CONTROL_DERIVATIVE_WINDOW_MS older) / that time        [m/s2]
 *                           per ERPM frame (a new frame = the ERPM value changed)
 *      car limit          = min(TRACTION_CONTROL_MAXIMUM_ACCELERATION_MS2,                  [m/s2]
 *                               TRACTION_CONTROL_MAXIMUM_POWER_W_PER_KG / reference speed)
 *                           what the car can do: tyre grip at low speed, power at high speed
 *      reference speed    = the wheel speed, but rising at most the car limit and falling at most
 *                           TRACTION_CONTROL_MAXIMUM_DECELERATION_MS2 since the last frame: the speed
 *                           the wheel would have rolling with the car. = wheel speed while it grips.
 *      acceleration excess = wheel acceleration - car limit - TRACTION_CONTROL_ACCELERATION_MARGIN_MS2
 *                           [m/s2], > 0 = spinning up. 0 for one window after the wheel was below its
 *                           reference (it is catching up with the car after a lock, not spinning).
 *      speed error        = wheel speed - (reference * (1 + slip target) + speed margin)  [km/h], > 0 = spun up
 *      integral term      = + TRACTION_CONTROL_INTEGRAL_GAIN * speed error * dt while speed error > 0,
 *                           - TRACTION_CONTROL_RECOVERY_PER_S * dt otherwise, within [0, 1 - minimum]
 *      cut                = TRACTION_CONTROL_ACCELERATION_GAIN * max(acceleration excess, 0)
 *                         + TRACTION_CONTROL_PROPORTIONAL_GAIN * max(speed error, 0) + integral term
 *      torque factor      = 1 - cut, within [minimum torque factor, 1]
 *    The acceleration term cuts as soon as the wheel starts to spin, before the slip builds up; the
 *    proportional term until the wheel is back down with the car; the integral term remembers how
 *    much this surface takes off the request and gives it back slowly. Each wheel has its OWN
 *    reference (only its own past): in a corner the outer wheel runs faster than the inner one, and
 *    a reference shared with the slower wheel cut the outer wheel in every corner (version E, 08-10).
 *    Only ever reduces torque. Off below TRACTION_CONTROL_MINIMUM_SPEED_KMH of reference speed.
 *
 * FALLBACK: no front wheel speed frame for SENSOR_TIMEOUT_MS -> torque vectoring off; no steering
 * frame -> torque vectoring off. Traction control does not use either. Off = both motors get the
 * same command, the car as before.
 *
 * Simulator twin: Powertrain V1 - Simulator/model/vcu/TractionControlTorqueVectoring.m
 * (values in loadParams.m section 10) - still the OLD traction control (PI against the front wheels).
 *
 * BEFORE THE FIRST RUN (Live Expressions: vehicle_sensors, torque_vectoring, traction_control)
 *   1. Car on stands, spin the LEFT rear wheel by hand: vehicle_sensors.rear_left_speed_kmh moves,
 *      otherwise swap INVERTER_ID_REAR_LEFT.
 *   2. Wheels straight: vehicle_sensors.road_wheel_angle_deg = 0 (STEERING_OFFSET_DEG). Turn LEFT:
 *      > 0 (STEERING_SIGN). Road wheel angle vs steering angle: STEERING_LINEAR_GAIN / STEERING_CUBIC_GAIN.
 *   3. Straight, constant speed: front wheel speeds = rear wheel speeds (FRONT_WHEEL_RADIUS_M).
 *   4. Traction control, car on stands (wheels free = spinning): press the pedal. The rear wheels
 *      must speed up in steps, on average at about the car limit (~TRACTION_CONTROL_MAXIMUM_ACCELERATION_MS2
 *      * 3.6 km/h per second), not at once, with traction_control.state CUT_BOTH and torque_factor < 1.
 *   5. TRACTION_CONTROL_ENABLE 1 alone first. Then TORQUE_VECTORING_ENABLE 1: skidpad both ways.
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
#define STEERING_SIGN           -1.0f                        // Sensor reads > 0 turning RIGHT -> -1, so road wheel angle > 0 = left
#define STEERING_OFFSET_DEG     5.0f                         // Steering angle with the wheels straight
#define FRONT_WHEEL_RADIUS_M    THROTTLE_WHEEL_RADIUS_M      // Front wheel speed calibration (step 3)
#define SENSOR_TIMEOUT_MS       300                          // Frame older than this -> sensor missing

/* ========================== TRACTION CONTROL ========================= */
// What the car can do, 08-10 ESTG log (31 min moving, both rears gripping, 150 ms average): at most
// 7.8 m/s2 (p99.9), 8.8 max, at any speed up to 80 km/h (10-10 Maceira: 6.3 p99); above ~60 km/h
// accel x speed reaches ~160 W/kg (DTI 2 x 32 kW, 76 % to the wheels, 290 kg: 168 W/kg).
// A spinning rear wheel speeds up at 20-60 m/s2. ERPM comes every 33 ms; over 2 frames a gripping
// wheel reads at most 11.3 m/s2 (driveline ripple), so limit + margin = 13 never cut one in that log.

// Car limit: the reference speed rises at most this, and a wheel speeding up faster is spinning
#define TRACTION_CONTROL_MAXIMUM_ACCELERATION_MS2   10.0f  // Most the car can accelerate, tyre grip [m/s2] (~1 g)
#define TRACTION_CONTROL_MAXIMUM_POWER_W_PER_KG    200.0f  // Power at the wheels / mass: limit = this / speed [W/kg] (< 10 above 72 km/h)
#define TRACTION_CONTROL_MAXIMUM_DECELERATION_MS2   15.0f  // Most the car can brake: the reference falls at most this [m/s2]
// Derivative: wheel acceleration = speed change since the newest ERPM frame at least this old
#define TRACTION_CONTROL_DERIVATIVE_WINDOW_MS       55     // [ms] = 2 frames at 33 ms (1 frame: ripple up to 17 m/s2)
#define TRACTION_CONTROL_ACCELERATION_MARGIN_MS2     3.0f  // Over the car limit by more than this = spinning up [m/s2]
#define TRACTION_CONTROL_ACCELERATION_GAIN           0.02f // Torque cut per m/s2 of acceleration excess [-/(m/s2)]
// Speed terms: cut starts at slip = SLIP_TARGET + SPEED_MARGIN / reference speed over the reference
// Gains: straight line model of the car (ERPM every 33 ms, tyre mu 0.8-1.3) - 10x less slip than
// without TC, ~0.9 of the distance of a perfect pedal (no TC: 0.84). Integral 1.0 / fade 0.5 beat a
// plain "cut and give back at a rate" on all of: distance, mean slip, worst case. Tune on track.
#define TRACTION_CONTROL_SLIP_TARGET                 0.10f // Slip allowed before cutting [-] (tyre peak 0.097)
#define TRACTION_CONTROL_SPEED_MARGIN_KMH            1.5f  // + this speed [km/h]
#define TRACTION_CONTROL_PROPORTIONAL_GAIN           0.12f // Torque cut per km/h over the allowed speed [-/(km/h)]
#define TRACTION_CONTROL_INTEGRAL_GAIN               1.0f  // Learned cut grows this per km/h over, per second
#define TRACTION_CONTROL_RECOVERY_PER_S              0.5f  // Learned cut fades this per second once the wheel is back
// Cut
#define TRACTION_CONTROL_MINIMUM_SPEED_KMH           1.0f  // Off below this reference speed [km/h]
#define TRACTION_CONTROL_MINIMUM_TORQUE_FACTOR       0.2f  // Never less than this x the request (at most -80 %)
#define TRACTION_CONTROL_FRAME_HISTORY_LENGTH       16     // ERPM frames kept for the derivative

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

// 3. Traction control of one rear wheel
typedef struct {
    float acceleration_ms2;             // Wheel acceleration, derivative of the motor rpm
    float car_limit_ms2;                // Most the car can accelerate at the reference speed
    float reference_speed_kmh;          // Speed the wheel would have rolling with the car
    float acceleration_excess_ms2;      // Acceleration - car limit - margin (> 0 = spinning up)
    float speed_error_kmh;              // Wheel speed - allowed speed (> 0 = spun up)
    float integral_term;                // Learned cut: grows while spun up, fades once back
    float torque_factor;                // 1 - cut, 1 = no cut

    // Internal
    bool reference_valid;               // false after a reset: the next step starts from the wheel speed
    uint32_t below_reference_ms;        // Last time the wheel was below its reference (after a lock)
    int32_t frame_erpm;                 // ERPM of the newest frame (a different value = a new frame)
    uint8_t frame_index;                // Newest ERPM frame in the history
    uint8_t frame_count;
    float frame_speed_kmh[TRACTION_CONTROL_FRAME_HISTORY_LENGTH];
    uint32_t frame_time_ms[TRACTION_CONTROL_FRAME_HISTORY_LENGTH];  // HAL tick of the step that saw it
} traction_control_wheel_t;

typedef enum {  // Sent in VCU_states.tc_state (powertrain DBC) - keep its value table in sync
    TRACTION_CONTROL_STATE_DISABLED = 0,          // TRACTION_CONTROL_ENABLE 0
    TRACTION_CONTROL_STATE_NO_FRONT_WHEEL_SPEED,  // Not used: traction control no longer needs the front wheels
    TRACTION_CONTROL_STATE_NO_DRIVE,              // No drive command (pedal released, regen, BSPD, ...)
    TRACTION_CONTROL_STATE_BELOW_MIN_SPEED,       // Off: both references below TRACTION_CONTROL_MINIMUM_SPEED_KMH
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
