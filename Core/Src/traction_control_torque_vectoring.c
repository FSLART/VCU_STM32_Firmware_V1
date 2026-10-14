/**
 * @file traction_control_torque_vectoring.c
 * @brief Traction control (motor rpm derivative) and torque vectoring (feedforward) on the rear motors,
 *        see traction_control_torque_vectoring.h
 */

#include "traction_control_torque_vectoring.h"

#include <math.h>

#include <autonomous_t26.h>
#include <data_t26.h>
#include <powertrain_t26.h>

#include "CAN_utils.h"         // MOTOR_POLE_PAIRS
#include "throttle_control.h"  // THROTTLE_GEAR_RATIO, THROTTLE_WHEEL_RADIUS_M

// Frame IDs come from the generated DBC code: they follow the DBCs when CAN priorities change.
// AQT4 has a different ID on each bus (autonomous / powertrain).

#define DEGREES_TO_RADIANS 0.017453293f
#define GRAVITY_MS2        9.81f

vehicle_sensors_t vehicle_sensors;
torque_vectoring_t torque_vectoring;
traction_control_t traction_control;

/* ============================== HELPERS ============================== */

static float clamp(float value, float minimum, float maximum) {
    return (value < minimum) ? minimum : ((value > maximum) ? maximum : value);
}

/** @brief Wheel rpm -> km/h */
static float speed_kmh_from_wheel_rpm(float wheel_rpm, float radius_m) {
    return wheel_rpm * 6.2831853f * radius_m * 60.0f / 1000.0f;
}

/* ========================= 1. VEHICLE SENSORS ======================== */

void vehicle_sensors_update(int32_t rear_left_erpm, int32_t rear_right_erpm, uint32_t now_ms) {
    vehicle_sensors_t *s = &vehicle_sensors;

    s->front_wheel_speed_valid = (now_ms - s->front_wheel_frame_time_ms) <= SENSOR_TIMEOUT_MS;
    s->steering_angle_valid = (now_ms - s->steering_frame_time_ms) <= SENSOR_TIMEOUT_MS;

    s->front_left_speed_kmh = speed_kmh_from_wheel_rpm(s->front_left_wheel_rpm, FRONT_WHEEL_RADIUS_M);
    s->front_right_speed_kmh = speed_kmh_from_wheel_rpm(s->front_right_wheel_rpm, FRONT_WHEEL_RADIUS_M);
    s->rear_left_speed_kmh = speed_kmh_from_wheel_rpm(
        fabsf((float)rear_left_erpm) / MOTOR_POLE_PAIRS / THROTTLE_GEAR_RATIO, THROTTLE_WHEEL_RADIUS_M);
    s->rear_right_speed_kmh = speed_kmh_from_wheel_rpm(
        fabsf((float)rear_right_erpm) / MOTOR_POLE_PAIRS / THROTTLE_GEAR_RATIO, THROTTLE_WHEEL_RADIUS_M);

    // Mean road wheel angle of both front wheels from the steering angle (fit in the .h, rad -> rad)
    float steering_rad = s->steering_angle_valid
                             ? (s->steering_angle_raw_deg - STEERING_OFFSET_DEG) * STEERING_SIGN * DEGREES_TO_RADIANS
                             : 0.0f;
    float road_wheel_angle_rad =
        (STEERING_LINEAR_GAIN + STEERING_CUBIC_GAIN * steering_rad * steering_rad) * steering_rad;
    s->road_wheel_angle_deg = road_wheel_angle_rad / DEGREES_TO_RADIANS;
    s->vehicle_speed_kmh = 0.5f * (s->front_left_speed_kmh + s->front_right_speed_kmh) * cosf(road_wheel_angle_rad);

    float vehicle_speed_ms = s->vehicle_speed_kmh / 3.6f;
    s->lateral_acceleration_g =
        vehicle_speed_ms * vehicle_speed_ms * tanf(road_wheel_angle_rad) / VEHICLE_WHEELBASE_M / GRAVITY_MS2;
}

/* ===================== 2. TORQUE VECTORING (feedforward) ============== */

/**
 * @brief Torque shift towards the outer wheel from the kinematic lateral acceleration
 * @details Feedforward (open loop), no PI or PID controller: there is no yaw rate sensor.
 */
static void torque_vectoring_update(float time_step_s) {
    const vehicle_sensors_t *s = &vehicle_sensors;
    torque_vectoring_t *tv = &torque_vectoring;

    bool active = false;
    if (!TORQUE_VECTORING_ENABLE) {
        tv->state = TORQUE_VECTORING_STATE_DISABLED;
    } else if (!s->front_wheel_speed_valid) {
        tv->state = TORQUE_VECTORING_STATE_NO_FRONT_WHEEL_SPEED;
    } else if (!s->steering_angle_valid) {
        tv->state = TORQUE_VECTORING_STATE_NO_STEERING_ANGLE;
    } else if (s->vehicle_speed_kmh < TORQUE_VECTORING_MINIMUM_SPEED_KMH) {
        tv->state = TORQUE_VECTORING_STATE_BELOW_MIN_SPEED;
    } else {
        active = true;
    }

    float target_shift = 0.0f;
    if (active) {
        target_shift = clamp(TORQUE_VECTORING_GAIN_PER_G * s->lateral_acceleration_g, -TORQUE_VECTORING_MAXIMUM_SHIFT,
                             TORQUE_VECTORING_MAXIMUM_SHIFT);
    }

    // Rate limit: the shift moves at most TORQUE_VECTORING_SHIFT_RATE_PER_S
    float maximum_change = TORQUE_VECTORING_SHIFT_RATE_PER_S * time_step_s;
    tv->shift += clamp(target_shift - tv->shift, -maximum_change, maximum_change);

    if (active) {
        if (TORQUE_VECTORING_MAXIMUM_SHIFT > 0.0f && fabsf(tv->shift) >= 0.999f * TORQUE_VECTORING_MAXIMUM_SHIFT) {
            tv->state = TORQUE_VECTORING_STATE_AT_MAX_SHIFT;
        } else if (tv->shift > 0.005f) {
            tv->state = TORQUE_VECTORING_STATE_MORE_TORQUE_RIGHT;
        } else if (tv->shift < -0.005f) {
            tv->state = TORQUE_VECTORING_STATE_MORE_TORQUE_LEFT;
        } else {
            tv->state = TORQUE_VECTORING_STATE_STRAIGHT;
        }
    }
}

/* ================ 3. TRACTION CONTROL (motor rpm derivative) ========== */

#if TRACTION_CONTROL_DERIVATIVE_WINDOW_MS > (TRACTION_CONTROL_FRAME_HISTORY_LENGTH - 2) * 5
#error "traction_control_torque_vectoring.h: TRACTION_CONTROL_DERIVATIVE_WINDOW_MS longer than the frame history"
#endif

#define FRAME_TIMEOUT_MS 100  // No new ERPM value for this long: the wheel acceleration is stale -> 0

/** @brief What the car can do: most it can accelerate at this speed, tyre grip then power [m/s2] */
static float traction_control_car_limit_ms2(float speed_kmh) {
    float speed_ms = fmaxf(speed_kmh / 3.6f, 0.1f);
    return fminf(TRACTION_CONTROL_MAXIMUM_ACCELERATION_MS2, TRACTION_CONTROL_MAXIMUM_POWER_W_PER_KG / speed_ms);
}

/**
 * @brief New ERPM frame: move the reference by the time since the last frame, store the frame and
 *        recompute the wheel acceleration
 * @details The inverters send ERPM only every ~33 ms (08-10 log), so both work per frame, with the
 *          time between frames: per 5 ms step the speed stands still and then jumps. The acceleration
 *          is the speed change since the newest frame at least TRACTION_CONTROL_DERIVATIVE_WINDOW_MS
 *          older (2 frames): over 1 frame, the driveline ripple of a gripping wheel reads up to 17 m/s2.
 *          floor_kmh: the reference stops falling here (0 = no floor), never pushed up by it.
 */
static void traction_control_wheel_new_frame(traction_control_wheel_t *wheel, float speed_kmh, float floor_kmh,
                                             uint32_t now_ms) {
    float elapsed_s = (float)(now_ms - wheel->frame_time_ms[wheel->frame_index]) * 0.001f;

    // Reference: the wheel speed, but rising at most the car limit and falling at most the car's braking
    // (and not below the floor)
    wheel->car_limit_ms2 = traction_control_car_limit_ms2(wheel->reference_speed_kmh);
    float lowest_kmh = fmaxf(wheel->reference_speed_kmh - TRACTION_CONTROL_MAXIMUM_DECELERATION_MS2 * 3.6f * elapsed_s,
                             fminf(floor_kmh, wheel->reference_speed_kmh));
    wheel->reference_speed_kmh =
        clamp(speed_kmh, lowest_kmh, wheel->reference_speed_kmh + wheel->car_limit_ms2 * 3.6f * elapsed_s);
    if (speed_kmh < wheel->reference_speed_kmh) {
        wheel->below_reference_ms = now_ms;  // Fell faster than the car can brake: locked, will catch up
    }

    wheel->frame_index = (uint8_t)((wheel->frame_index + 1u) % TRACTION_CONTROL_FRAME_HISTORY_LENGTH);
    wheel->frame_speed_kmh[wheel->frame_index] = speed_kmh;
    wheel->frame_time_ms[wheel->frame_index] = now_ms;
    if (wheel->frame_count < TRACTION_CONTROL_FRAME_HISTORY_LENGTH) wheel->frame_count++;

    wheel->acceleration_ms2 = 0.0f;  // Until the history covers the window
    for (uint8_t age = 1; age < wheel->frame_count; age++) {
        uint8_t i = (uint8_t)((wheel->frame_index + TRACTION_CONTROL_FRAME_HISTORY_LENGTH - age) %
                              TRACTION_CONTROL_FRAME_HISTORY_LENGTH);
        uint32_t window_ms = now_ms - wheel->frame_time_ms[i];
        if (window_ms >= TRACTION_CONTROL_DERIVATIVE_WINDOW_MS) {
            wheel->acceleration_ms2 = (speed_kmh - wheel->frame_speed_kmh[i]) / 3.6f / ((float)window_ms * 0.001f);
            break;
        }
    }
}

/**
 * @brief Traction control of one rear wheel: acceleration and speed against what the car can do
 * @param wheel            controller state; result in wheel->torque_factor (1 = no cut)
 * @param active           traction control on and driving
 * @param erpm             motor ERPM as last received (a change = a new frame)
 * @param wheel_speed_kmh  rear wheel speed from it
 * @param floor_kmh        the reference does not fall below this (0 = no floor)
 */
static void traction_control_wheel_update(traction_control_wheel_t *wheel, bool active, int32_t erpm,
                                          float wheel_speed_kmh, float floor_kmh, uint32_t now_ms,
                                          float time_step_s) {
    // New ERPM frame = the value changed (while moving 0.2 % of the frames repeat the last value: then
    // the next one spans both, still right). First step after a reset: start from the wheel speed.
    if (!wheel->reference_valid) {
        wheel->reference_speed_kmh = wheel_speed_kmh;
        wheel->acceleration_ms2 = 0.0f;
        wheel->below_reference_ms = now_ms - TRACTION_CONTROL_DERIVATIVE_WINDOW_MS;
        wheel->frame_erpm = erpm;
        wheel->frame_index = 0;
        wheel->frame_count = 1;
        wheel->frame_speed_kmh[0] = wheel_speed_kmh;
        wheel->frame_time_ms[0] = now_ms;
        wheel->reference_valid = true;
    } else if (erpm != wheel->frame_erpm) {
        wheel->frame_erpm = erpm;
        traction_control_wheel_new_frame(wheel, wheel_speed_kmh, floor_kmh, now_ms);
    } else if (now_ms - wheel->frame_time_ms[wheel->frame_index] > FRAME_TIMEOUT_MS) {
        wheel->acceleration_ms2 = 0.0f;  // Standing still, or the frames stopped
    }

    // Acceleration term: only once the window holds no catching up after a lock
    bool rolled_with_car = (now_ms - wheel->below_reference_ms) >= TRACTION_CONTROL_DERIVATIVE_WINDOW_MS;
    wheel->acceleration_excess_ms2 =
        rolled_with_car ? wheel->acceleration_ms2 - wheel->car_limit_ms2 - TRACTION_CONTROL_ACCELERATION_MARGIN_MS2
                        : 0.0f;
    wheel->speed_error_kmh = wheel_speed_kmh - (wheel->reference_speed_kmh * (1.0f + TRACTION_CONTROL_SLIP_TARGET) +
                                                TRACTION_CONTROL_SPEED_MARGIN_KMH);

    if (!active || wheel->reference_speed_kmh < TRACTION_CONTROL_MINIMUM_SPEED_KMH) {
        wheel->integral_term = 0.0f;
        wheel->torque_factor = 1.0f;
        return;
    }

    // Integral term: grows while the wheel is over the allowed speed, fades at TRACTION_CONTROL_RECOVERY_PER_S
    // once it is back - remembers how much of the request this surface takes
    if (wheel->speed_error_kmh > 0.0f) {
        wheel->integral_term += TRACTION_CONTROL_INTEGRAL_GAIN * wheel->speed_error_kmh * time_step_s;
    } else {
        wheel->integral_term -= TRACTION_CONTROL_RECOVERY_PER_S * time_step_s;
    }
    wheel->integral_term = clamp(wheel->integral_term, 0.0f, 1.0f - TRACTION_CONTROL_MINIMUM_TORQUE_FACTOR);

    // Acceleration and proportional terms: cut at once, let go as soon as the wheel is back with the car
    float cut = TRACTION_CONTROL_ACCELERATION_GAIN * fmaxf(wheel->acceleration_excess_ms2, 0.0f) +
                TRACTION_CONTROL_PROPORTIONAL_GAIN * fmaxf(wheel->speed_error_kmh, 0.0f) + wheel->integral_term;
    wheel->torque_factor = clamp(1.0f - cut, TRACTION_CONTROL_MINIMUM_TORQUE_FACTOR, 1.0f);
}

/* ============================= PUBLIC API ============================ */

void traction_control_torque_vectoring_reset(void) {
    torque_vectoring.shift = 0.0f;
    traction_control_wheel_t released = {.torque_factor = 1.0f};
    traction_control.rear_left = released;
    traction_control.rear_right = released;
}

void vehicle_sensors_can_receive(const can_msg_t *message) {
    if (message->is_extended) return;
    // Front wheel speed only from the data bus; steering from the autonomous OR the powertrain bus
    // (redundancy: the newest frame from either is used, valid while either keeps arriving).
    // Raw decode: AQT2 FRONT_LEFT/RIGHT_WHEEL_RPM bytes 0-1 / 2-3, AQT4 ST_ANGLE bytes 0-1 (same on both buses)
    bool steering_frame = (message->bus == CAN_BUS_3 && message->id == AUTONOMOUS_T26_AQT4_FRAME_ID) ||
                          (message->bus == CAN_BUS_2 && message->id == POWERTRAIN_T26_AQT4_FRAME_ID);
    if (message->bus == CAN_BUS_1 && message->id == DATA_T26_AQT2_FRAME_ID && message->dlc >= 4) {
        vehicle_sensors.front_left_wheel_rpm = (uint16_t)(message->data[0] | (message->data[1] << 8));
        vehicle_sensors.front_right_wheel_rpm = (uint16_t)(message->data[2] | (message->data[3] << 8));
        vehicle_sensors.front_wheel_frame_time_ms = message->timestamp;
    } else if (steering_frame && message->dlc >= 2) {
        vehicle_sensors.steering_angle_raw_deg = (int16_t)(message->data[0] | (message->data[1] << 8)) * 0.1f;
        vehicle_sensors.steering_frame_time_ms = message->timestamp;
    }
}

void traction_control_torque_vectoring_update(uint16_t drive_command_1000, int32_t rear_left_erpm,
                                              int32_t rear_right_erpm, uint32_t now_ms, uint32_t time_step_ms,
                                              uint16_t *drive_command_left_1000, uint16_t *drive_command_right_1000) {
    float time_step_s = (float)time_step_ms * 0.001f;
    const vehicle_sensors_t *s = &vehicle_sensors;

    /* 1. Vehicle sensors */
    vehicle_sensors_update(rear_left_erpm, rear_right_erpm, now_ms);

    /* 2. Torque vectoring */
    torque_vectoring_update(time_step_s);

    /* 3. Traction control: each rear wheel against what the car can do (its own rpm derivative) */
    // With no drive command (regen, coast) each reference stops falling at a fraction of the other wheel:
    // a wheel locked under regen does not drag its reference below the car
    bool traction_control_active = TRACTION_CONTROL_ENABLE && drive_command_1000 > 0;
    float floor_factor = (drive_command_1000 == 0) ? TRACTION_CONTROL_OTHER_WHEEL_FLOOR : 0.0f;
    traction_control_wheel_update(&traction_control.rear_left, traction_control_active, rear_left_erpm,
                                  s->rear_left_speed_kmh, floor_factor * s->rear_right_speed_kmh, now_ms, time_step_s);
    traction_control_wheel_update(&traction_control.rear_right, traction_control_active, rear_right_erpm,
                                  s->rear_right_speed_kmh, floor_factor * s->rear_left_speed_kmh, now_ms, time_step_s);

    // State for VCU_states / Live Expressions: off and why, or which wheel is cut
    bool cut_left = traction_control.rear_left.torque_factor < 1.0f;
    bool cut_right = traction_control.rear_right.torque_factor < 1.0f;
    if (!TRACTION_CONTROL_ENABLE) {
        traction_control.state = TRACTION_CONTROL_STATE_DISABLED;
    } else if (drive_command_1000 == 0) {
        traction_control.state = TRACTION_CONTROL_STATE_NO_DRIVE;
    } else if (traction_control.rear_left.reference_speed_kmh < TRACTION_CONTROL_MINIMUM_SPEED_KMH &&
               traction_control.rear_right.reference_speed_kmh < TRACTION_CONTROL_MINIMUM_SPEED_KMH) {
        traction_control.state = TRACTION_CONTROL_STATE_BELOW_MIN_SPEED;
    } else if (cut_left && cut_right) {
        traction_control.state = TRACTION_CONTROL_STATE_CUT_BOTH;
    } else if (cut_left) {
        traction_control.state = TRACTION_CONTROL_STATE_CUT_REAR_LEFT;
    } else if (cut_right) {
        traction_control.state = TRACTION_CONTROL_STATE_CUT_REAR_RIGHT;
    } else {
        traction_control.state = TRACTION_CONTROL_STATE_GRIP;
    }

    /* Outputs: torque vectoring split (capped at 1000), then the traction control cut */
    *drive_command_left_1000 = (uint16_t)(fminf(drive_command_1000 * (1.0f - torque_vectoring.shift), 1000.0f) *
                                          traction_control.rear_left.torque_factor);
    *drive_command_right_1000 = (uint16_t)(fminf(drive_command_1000 * (1.0f + torque_vectoring.shift), 1000.0f) *
                                           traction_control.rear_right.torque_factor);
}
