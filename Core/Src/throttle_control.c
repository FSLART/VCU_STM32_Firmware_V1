/**
 * @file throttle_control.c
 * @brief Manual driving: pedals -> vehicle dynamics layer -> motor current (see throttle_control.h)
 */

#include "throttle_control.h"

#include <math.h>
#include <string.h>

#include "traction_control_torque_vectoring.h"  // vehicle_sensors, INVERTER_ID_*, STEERING_*, switches

#if MOTOR_POLE_PAIRS < 1
#error "CAN_utils.h: MOTOR_POLE_PAIRS must be at least 1 (divides by it)"
#endif
#if (THROTTLE_REGEN_CUTOFF_V > 0) && (THROTTLE_REGEN_FADE_START_V >= THROTTLE_REGEN_CUTOFF_V)
#error "throttle_control.h: THROTTLE_REGEN_FADE_START_V must be below THROTTLE_REGEN_CUTOFF_V"
#endif

#define MAX_DT_MS 50                // Longest time step the torque rise limit will integrate
#define BRAKE_LIMITS_PERIOD_MS 100  // How often the brake-limit frames are resent
#define RPM_TO_RAD_S 0.10471976f    // 2 pi / 60
#define DEGREES_TO_RADIANS 0.017453293f

throttle_t throttle;
vehicle_dynamics_t vehicle_dynamics;

/* ============================== HELPERS ============================== */

/**
 * @brief Speed of the slower rear motor, mechanical rpm
 * @details Absolute value: the motors may turn in opposite electrical directions depending on
 *          how they are mounted.
 */
static uint16_t slowest_motor_rpm(int32_t erpm_1, int32_t erpm_2) {
    uint32_t rpm_1 = (uint32_t)(erpm_1 < 0 ? -erpm_1 : erpm_1) / MOTOR_POLE_PAIRS;
    uint32_t rpm_2 = (uint32_t)(erpm_2 < 0 ? -erpm_2 : erpm_2) / MOTOR_POLE_PAIRS;
    uint32_t slowest = (rpm_1 < rpm_2) ? rpm_1 : rpm_2;
    return (slowest > UINT16_MAX) ? UINT16_MAX : (uint16_t)slowest;
}

static float speed_kmh_from_motor_rpm(uint16_t motor_rpm) {
    const float two_pi = 6.2831853f;
    return motor_rpm / THROTTLE_GEAR_RATIO * two_pi * THROTTLE_WHEEL_RADIUS_M * 60.0f / 1000.0f;
}

/**
 * @brief Fade regen out as the pack approaches full voltage
 * @return 1000 = no limit, 0 = no regen allowed
 */
static uint16_t dc_voltage_factor(uint16_t dc_voltage_v) {
#if THROTTLE_REGEN_CUTOFF_V > 0
    if (dc_voltage_v <= THROTTLE_REGEN_FADE_START_V) return 1000;
    if (dc_voltage_v >= THROTTLE_REGEN_CUTOFF_V) return 0;
    return (uint16_t)(1000u - (uint32_t)(dc_voltage_v - THROTTLE_REGEN_FADE_START_V) * 1000u /
                                  (THROTTLE_REGEN_CUTOFF_V - THROTTLE_REGEN_FADE_START_V));
#else
    (void)dc_voltage_v;
    return 1000;  // Limit disabled until the accumulator voltages are set
#endif
}

/**
 * @brief Steps 3-5 on one motor current: regen limits, torque rise limit, digital BSPD (last)
 * @param last_a current sent to this motor in the previous cycle (the rise starts from it)
 */
static float limit_current(const throttle_t *t, float current_a, float last_a, float dt_s) {
    if (current_a < 0.0f) {
        current_a = (!THROTTLE_REGEN_ENABLE || t->inverter_fault) ? 0.0f : current_a * t->voltage_factor_1000 / 1000.0f;
    }
    if (THROTTLE_TORQUE_RISE_MAX_PCT_S > 0) {
        float maximum_rise = THROTTLE_TORQUE_RISE_MAX_PCT_S * 0.01f * vehicle_dynamics.config.motor_maximum_drive_current_a * dt_s;
        current_a = fminf(current_a, fmaxf(last_a, 0.0f) + maximum_rise);
    }
    return t->bspd_active ? 0.0f : current_a;
}

/** @brief 0.1 A per bit: >= 0 drive (SetAcCurrent), < 0 brake towards 0 rpm (SetBrakeCurrent, positive value) */
static void send_motor_current(uint8_t inverter_id, float current_a, CAN_HandleTypeDef *hcan) {
    if (current_a >= 0.0f) {
        can_bus_send_FSIC_SetAcCurrent(inverter_id, (int16_t)(current_a * 10.0f), hcan);
    } else {
        can_bus_send_FSIC_SetBrakeCurrent(inverter_id, (int16_t)(-current_a * 10.0f), hcan);
    }
}

/* ============================= PUBLIC API ============================ */

void throttle_control_reset(void) {
    memset(&throttle, 0, sizeof(throttle));
    throttle.first_update = true;
    vehicle_dynamics_init(&vehicle_dynamics, &VEHICLE_DYNAMICS_T26_CONFIG);
    // The VCU switches (editor) decide, not the layer config
    vehicle_dynamics.config.traction_control_enable = TRACTION_CONTROL_ENABLE;
    vehicle_dynamics.config.torque_vectoring_enable = TORQUE_VECTORING_ENABLE;
    vehicle_dynamics.config.straight_line_grip_scale = THROTTLE_STRAIGHT_LINE_BOOST;
    const float friction = THROTTLE_WET_TRACK ? THROTTLE_FRICTION_WET : THROTTLE_FRICTION_DRY;
    vehicle_dynamics.config.tyre_LMUX *= friction;
    vehicle_dynamics.config.tyre_LMUY *= friction;
}

void throttle_control_update(const APPS_Result_t *apps, bool bspd_active, const FSIC_t *inv1,
                             const FSIC_t *inv2, int32_t power_w, uint32_t now_ms) {
    throttle_t *t = &throttle;
    (void)power_w;

    uint32_t dt_ms = t->first_update ? 0 : (now_ms - t->last_update_ms);
    if (dt_ms > MAX_DT_MS) dt_ms = MAX_DT_MS;
    float dt_s = (float)dt_ms * 0.001f;
    t->last_update_ms = now_ms;
    t->first_update = false;

    /* --- 1. Inputs --- */
    const FSIC_t *inverter_rear_left = (INVERTER_ID_REAR_LEFT == 1) ? inv1 : inv2;
    const FSIC_t *inverter_rear_right = (INVERTER_ID_REAR_LEFT == 1) ? inv2 : inv1;
    vehicle_sensors_update(inverter_rear_left->Actual_ERPM, inverter_rear_right->Actual_ERPM, now_ms);
    const vehicle_sensors_t *s = &vehicle_sensors;

    t->pedal_1000 = apps->percentage_1000;
    t->apps_error = apps->error;
    t->brake_bar = vcu.brake_raw * 0.1f;  // 0.1 bar raw (vcu.brake_pressure is truncated to whole bar)
    t->brake_valid = (now_ms - vcu.brake_last_rx_time) <= SENSOR_TIMEOUT_MS;
    t->inverter_fault = (inv1->Actual_FaultCode != 0) || (inv2->Actual_FaultCode != 0);
    t->bspd_active = bspd_active;
    t->motor_rpm = slowest_motor_rpm(inv1->Actual_ERPM, inv2->Actual_ERPM);
    t->speed_kmh = speed_kmh_from_motor_rpm(t->motor_rpm);
    t->dc_voltage_v = (inv1->Actual_InputVoltage > inv2->Actual_InputVoltage) ? inv1->Actual_InputVoltage
                                                                              : inv2->Actual_InputVoltage;
    t->voltage_factor_1000 = dc_voltage_factor(t->dc_voltage_v);

    /* --- 2. Vehicle dynamics: pedal + brake -> Iq per wheel --- */
    const float motor_erpm_to_wheel_rad_s = RPM_TO_RAD_S / MOTOR_POLE_PAIRS / vehicle_dynamics.config.gear_ratio;
    const vehicle_dynamics_input_t input = {
        .wheel_speed_rad_s = {
            s->front_left_wheel_rpm * RPM_TO_RAD_S,
            s->front_right_wheel_rpm * RPM_TO_RAD_S,
            fabsf((float)inverter_rear_left->Actual_ERPM) * motor_erpm_to_wheel_rad_s,
            fabsf((float)inverter_rear_right->Actual_ERPM) * motor_erpm_to_wheel_rad_s,
        },
        .front_wheel_speeds_valid = THROTTLE_FRONT_WHEEL_SPEEDS && s->front_wheel_speed_valid,
        // ponytail: no inverter frame timestamp, ERPM always trusted; add one if a silent inverter must stop the TC
        .rear_wheel_speeds_valid = true,
        .steering_wheel_angle_rad = (s->steering_angle_raw_deg - STEERING_OFFSET_DEG) * STEERING_SIGN * DEGREES_TO_RADIANS,
        .steering_valid = s->steering_angle_valid,
        .imu_longitudinal_acceleration_ms2 = s->imu_longitudinal_acceleration_ms2,
        .imu_lateral_acceleration_ms2 = s->imu_lateral_acceleration_ms2,
        .imu_valid = THROTTLE_IMU_ACCELERATIONS && s->imu_acceleration_valid,
        .yaw_rate_rad_s = s->imu_yaw_rate_rad_s,
        .yaw_rate_valid = THROTTLE_IMU_GYRO && s->imu_yaw_rate_valid,   // torque vectoring needs it
        .throttle_percent = t->pedal_1000 * 0.1f,
        .throttle_valid = !t->apps_error,
        .brake_pressure_bar = t->brake_bar,
        .brake_pressure_valid = t->brake_valid,
    };
    vehicle_dynamics_torque_t torque = vehicle_dynamics_compute(&vehicle_dynamics, &input, dt_s);

    /* --- 3-5. Regen limits, torque rise limit, digital BSPD (last) --- */
    t->current_left_a = limit_current(t, torque.rear_left.motor_current_a, t->current_left_a, dt_s);
    t->current_right_a = limit_current(t, torque.rear_right.motor_current_a, t->current_right_a, dt_s);

    /* --- Status for Live Expressions --- */
    bool regen_requested = vehicle_dynamics.regen_fraction > 0.0f;
    if (t->bspd_active) {
        t->status = THROTTLE_STATUS_BSPD_CUT;
    } else if (t->apps_error) {
        t->status = THROTTLE_STATUS_APPS_ERROR;
    } else if (t->current_left_a < 0.0f || t->current_right_a < 0.0f) {
        t->status = THROTTLE_STATUS_REGEN;
    } else if (t->current_left_a > 0.0f || t->current_right_a > 0.0f) {
        t->status = THROTTLE_STATUS_DRIVE;
    } else if (regen_requested && t->inverter_fault) {
        t->status = THROTTLE_STATUS_INVERTER_FAULT;
    } else if (regen_requested && t->voltage_factor_1000 == 0) {
        t->status = THROTTLE_STATUS_REGEN_OFF_DC_VOLTAGE;
    } else {
        t->status = THROTTLE_STATUS_COAST;
    }
}

void throttle_control_stop(CAN_HandleTypeDef *hcan) {
    throttle_control_reset();
    // Zero relative current: ends drive and regen (brake current) alike
    can_bus_send_FSIC_SetRelCurrent(1, 0, hcan);
    can_bus_send_FSIC_SetRelCurrent(2, 0, hcan);
}

void throttle_control_send(CAN_HandleTypeDef *hcan, uint32_t now_ms) {
    // One control mode per inverter per cycle - the inverter follows the last command it received
    send_motor_current(INVERTER_ID_REAR_LEFT, throttle.current_left_a, hcan);
    send_motor_current(INVERTER_ID_REAR_RIGHT, throttle.current_right_a, hcan);

#if THROTTLE_SEND_BRAKE_LIMITS
    // Max brake currents (negative, scale 0.1 A), resent periodically in case a frame is lost
    static uint32_t last_limits_ms = 0;
    if (now_ms - last_limits_ms >= BRAKE_LIMITS_PERIOD_MS) {
        int16_t max_ac_brake = (int16_t)(-THROTTLE_MAX_AC_BRAKE_CURRENT_A * 10);
        int16_t max_dc_brake = (int16_t)(-THROTTLE_MAX_DC_BRAKE_CURRENT_A * 10);
        can_bus_send_FSIC_SetMaxAcBrakeCurrent(1, max_ac_brake, hcan);
        can_bus_send_FSIC_SetMaxAcBrakeCurrent(2, max_ac_brake, hcan);
        can_bus_send_FSIC_SetMaxDcBrakeCurrent(1, max_dc_brake, hcan);
        can_bus_send_FSIC_SetMaxDcBrakeCurrent(2, max_dc_brake, hcan);
        last_limits_ms = now_ms;
    }
#else
    (void)now_ms;
#endif
}
