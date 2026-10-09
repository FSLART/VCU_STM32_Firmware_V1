/**
 * @file traction_control_torque_vectoring.c
 * @brief Traction control (PI controller) and torque vectoring (feedforward) on the rear motors,
 *        see traction_control_torque_vectoring.h
 */

#include "traction_control_torque_vectoring.h"

#include <math.h>

#include "CAN_utils.h"         // MOTOR_POLE_PAIRS
#include "throttle_control.h"  // THROTTLE_GEAR_RATIO, THROTTLE_WHEEL_RADIUS_M

#define FRONT_WHEEL_SPEED_FRAME_ID 0x720u  // AQT2
#define STEERING_ANGLE_FRAME_ID    0x740u  // AQT4

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

    s->road_wheel_angle_deg =
        s->steering_angle_valid
            ? (s->steering_angle_raw_deg - STEERING_OFFSET_DEG) * STEERING_SIGN / STEERING_RATIO
            : 0.0f;
    float road_wheel_angle_rad = s->road_wheel_angle_deg * DEGREES_TO_RADIANS;
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

/* ===================== 3. TRACTION CONTROL (PI controller) ============ */

/**
 * @brief PI controller (proportional + integral, no derivative term) of one rear wheel
 * @param wheel                controller state; result in wheel->torque_factor (1 = no cut)
 * @param active               traction control on and driving
 * @param wheel_speed_kmh      rear wheel speed (motor)
 * @param reference_speed_kmh  front wheel on the same side, along the car
 */
static void traction_control_proportional_integral_controller(traction_control_wheel_t *wheel, bool active,
                                                              float wheel_speed_kmh, float reference_speed_kmh,
                                                              float time_step_s) {
    float allowed_speed_kmh =
        reference_speed_kmh * (1.0f + TRACTION_CONTROL_SLIP_TARGET) + TRACTION_CONTROL_SPEED_MARGIN_KMH;
    wheel->speed_error_kmh = wheel_speed_kmh - allowed_speed_kmh;

    if (!active || reference_speed_kmh < TRACTION_CONTROL_MINIMUM_SPEED_KMH) {
        wheel->proportional_term = 0.0f;
        wheel->integral_term = 0.0f;
        wheel->torque_factor = 1.0f;
        return;
    }

    // Proportional term: only when spinning (a negative error must not add torque)
    wheel->proportional_term = TRACTION_CONTROL_PROPORTIONAL_GAIN * fmaxf(wheel->speed_error_kmh, 0.0f);

    // Integral term: grows while spinning, unwinds back to 0 once the wheel grips (anti-windup limits)
    wheel->integral_term = clamp(wheel->integral_term + TRACTION_CONTROL_INTEGRAL_GAIN * wheel->speed_error_kmh * time_step_s,
                                 0.0f, 1.0f - TRACTION_CONTROL_MINIMUM_TORQUE_FACTOR);

    wheel->torque_factor = clamp(1.0f - wheel->proportional_term - wheel->integral_term,
                                 TRACTION_CONTROL_MINIMUM_TORQUE_FACTOR, 1.0f);
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
    if (message->id == FRONT_WHEEL_SPEED_FRAME_ID && message->dlc >= 4) {
        vehicle_sensors.front_left_wheel_rpm = (uint16_t)(message->data[0] | (message->data[1] << 8));
        vehicle_sensors.front_right_wheel_rpm = (uint16_t)(message->data[2] | (message->data[3] << 8));
        vehicle_sensors.front_wheel_frame_time_ms = message->timestamp;
    } else if (message->id == STEERING_ANGLE_FRAME_ID && message->dlc >= 2) {
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

    /* 3. Traction control: each rear wheel against the front wheel on its side */
    bool traction_control_active = TRACTION_CONTROL_ENABLE && s->front_wheel_speed_valid && drive_command_1000 > 0;
    float cos_road_wheel_angle = cosf(s->road_wheel_angle_deg * DEGREES_TO_RADIANS);
    float reference_left_kmh = s->front_left_speed_kmh * cos_road_wheel_angle;
    float reference_right_kmh = s->front_right_speed_kmh * cos_road_wheel_angle;
    traction_control_proportional_integral_controller(&traction_control.rear_left, traction_control_active,
                                                      s->rear_left_speed_kmh, reference_left_kmh, time_step_s);
    traction_control_proportional_integral_controller(&traction_control.rear_right, traction_control_active,
                                                      s->rear_right_speed_kmh, reference_right_kmh, time_step_s);

    // State for VCU_states / Live Expressions: off and why, or which wheel is cut
    bool cut_left = traction_control.rear_left.torque_factor < 1.0f;
    bool cut_right = traction_control.rear_right.torque_factor < 1.0f;
    if (!TRACTION_CONTROL_ENABLE) {
        traction_control.state = TRACTION_CONTROL_STATE_DISABLED;
    } else if (!s->front_wheel_speed_valid) {
        traction_control.state = TRACTION_CONTROL_STATE_NO_FRONT_WHEEL_SPEED;
    } else if (drive_command_1000 == 0) {
        traction_control.state = TRACTION_CONTROL_STATE_NO_DRIVE;
    } else if (reference_left_kmh < TRACTION_CONTROL_MINIMUM_SPEED_KMH &&
               reference_right_kmh < TRACTION_CONTROL_MINIMUM_SPEED_KMH) {
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
