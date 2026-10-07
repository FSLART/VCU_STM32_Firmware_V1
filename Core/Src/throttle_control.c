/**
 * @file throttle_control.c
 * @brief Manual driving: accelerator -> relative current request (see throttle_control.h)
 */

#include "throttle_control.h"

#include <string.h>

#include "pau_control.h"
#include "traction_control_torque_vectoring.h"
#include "throttle_map.h"

#if (THROTTLE_REGEN_RAMP_UP_MS <= 0) || (THROTTLE_REGEN_RAMP_DOWN_MS <= 0)
#error "throttle_control.h: regen ramp times must be above 0"
#endif
#if MOTOR_POLE_PAIRS < 1
#error "CAN_utils.h: MOTOR_POLE_PAIRS must be at least 1 (divides by it)"
#endif
#if (THROTTLE_REGEN_CUTOFF_V > 0) && (THROTTLE_REGEN_FADE_START_V >= THROTTLE_REGEN_CUTOFF_V)
#error "throttle_control.h: THROTTLE_REGEN_FADE_START_V must be below THROTTLE_REGEN_CUTOFF_V"
#endif

#define MAX_DT_MS 50                // Longest time step the ramps will integrate
#define BRAKE_LIMITS_PERIOD_MS 100  // How often the brake-limit frames are resent

throttle_t throttle;

/* ============================== HELPERS ============================== */

/**
 * @brief Speed of the slower rear motor, mechanical rpm
 * @details The slower wheel decides, so a wheel slowing down (locking) under regen
 *          reduces regen on both sides. Absolute value: the motors may turn in opposite
 *          electrical directions depending on how they are mounted.
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
 * @brief Move a value towards a target by at most rate_per_s * dt (at least 1 per step)
 */
static uint16_t rate_limit(uint16_t current, uint16_t target, uint32_t rate_per_s, uint32_t dt_ms) {
    uint32_t max_step = (rate_per_s * dt_ms + 999u) / 1000u;
    if (target > current) {
        uint32_t step = target - current;
        return (uint16_t)(current + (step < max_step ? step : max_step));
    }
    uint32_t step = current - target;
    return (uint16_t)(current - (step < max_step ? step : max_step));
}

/* ============================= PUBLIC API ============================ */

void throttle_control_reset(void) {
    memset(&throttle, 0, sizeof(throttle));
    throttle.first_update = true;
    traction_control_torque_vectoring_reset();
}

void throttle_control_update(const APPS_Result_t *apps, bool bspd_active, const FSIC_t *inv1,
                             const FSIC_t *inv2, int32_t power_w, uint32_t now_ms) {
    throttle_t *t = &throttle;

    uint32_t dt_ms = t->first_update ? 0 : (now_ms - t->last_update_ms);
    if (dt_ms > MAX_DT_MS) dt_ms = MAX_DT_MS;
    t->last_update_ms = now_ms;
    t->first_update = false;

    /* --- 1. Inputs --- */
    t->pedal_1000 = apps->percentage_1000;
    t->apps_error = apps->error;
    t->inverter_fault = (inv1->Actual_FaultCode != 0) || (inv2->Actual_FaultCode != 0);
    t->bspd_active = bspd_active;
    t->motor_rpm = slowest_motor_rpm(inv1->Actual_ERPM, inv2->Actual_ERPM);
    t->speed_kmh = speed_kmh_from_motor_rpm(t->motor_rpm);
    t->dc_voltage_v = (inv1->Actual_InputVoltage > inv2->Actual_InputVoltage) ? inv1->Actual_InputVoltage
                                                                              : inv2->Actual_InputVoltage;

    /* --- 2. Throttle map: + drive, - regen --- */
    int16_t request = throttle_map_lookup(t->pedal_1000, t->speed_kmh);

    /* --- 3. PAU power limit (drive only, regen passes through) --- */
#if PAU_CONTROL_ENABLE
    request = pau_limit_accelerator(request, power_w);
#else
    (void)power_w;
#endif
    t->map_1000 = request;
    uint32_t drive = (request > 0) ? (uint32_t)request : 0;
    uint32_t regen = (request < 0) ? (uint32_t)(-request) : 0;

    /* --- 4. Limits --- */
    t->voltage_factor_1000 = dc_voltage_factor(t->dc_voltage_v);
    regen = regen * t->voltage_factor_1000 / 1000u;
    if (t->apps_error) {
        drive = 0;
    }
    if (!THROTTLE_REGEN_ENABLE || t->apps_error || t->inverter_fault) {
        regen = 0;
        t->regen_cmd_1000 = 0;  // Instant cut, no ramp
    }
    if (drive > 0) {
        regen = 0;  // Driver asks for torque: release regen
    }
    t->drive_target_1000 = (uint16_t)drive;
    t->regen_target_1000 = (uint16_t)regen;

    /* --- 5. Regen ramp; drive only once regen has fully released --- */
    uint32_t regen_ramp_ms = (regen > t->regen_cmd_1000) ? THROTTLE_REGEN_RAMP_UP_MS : THROTTLE_REGEN_RAMP_DOWN_MS;
    t->regen_cmd_1000 = rate_limit(t->regen_cmd_1000, (uint16_t)regen, 1000u * 1000u / regen_ramp_ms, dt_ms);
    if (t->regen_cmd_1000 > 0) {
        drive = 0;
    }

    /* --- 6. Torque rise limit: drive rises at most THROTTLE_TORQUE_RISE_MAX_PCT_S, falls instantly --- */
    if (THROTTLE_TORQUE_RISE_MAX_PCT_S == 0 || drive <= t->drive_ramped) {
        t->drive_ramped = (float)drive;
    } else {
        // %/s -> per mille per ms: x 10 / 1000
        t->drive_ramped += THROTTLE_TORQUE_RISE_MAX_PCT_S * 0.01f * (float)dt_ms;
        if (t->drive_ramped > drive) t->drive_ramped = (float)drive;
    }
    t->drive_cmd_1000 = (uint16_t)t->drive_ramped;

    /* --- 7. Digital BSPD - LAST, overrides everything above --- */
    if (t->bspd_active) {
        t->drive_cmd_1000 = 0;
        t->drive_ramped = 0.0f;  // Torque rises again from 0 once the BSPD clears
        t->regen_cmd_1000 = 0;
    }

    /* --- 8. Traction control + torque vectoring: one drive command per motor --- */
    const FSIC_t *inverter_rear_left = (INVERTER_ID_REAR_LEFT == 1) ? inv1 : inv2;
    const FSIC_t *inverter_rear_right = (INVERTER_ID_REAR_LEFT == 1) ? inv2 : inv1;
    traction_control_torque_vectoring_update(t->drive_cmd_1000, inverter_rear_left->Actual_ERPM,
                                             inverter_rear_right->Actual_ERPM, now_ms, dt_ms,
                                             &t->drive_command_left_1000, &t->drive_command_right_1000);

    /* --- Status for Live Expressions --- */
    if (t->bspd_active) {
        t->status = THROTTLE_STATUS_BSPD_CUT;
    } else if (t->apps_error) {
        t->status = THROTTLE_STATUS_APPS_ERROR;
    } else if (t->regen_cmd_1000 > 0) {
        t->status = THROTTLE_STATUS_REGEN;
    } else if (t->drive_cmd_1000 > 0) {
        t->status = THROTTLE_STATUS_DRIVE;
    } else if (t->map_1000 < 0 && t->inverter_fault) {
        t->status = THROTTLE_STATUS_INVERTER_FAULT;
    } else if (t->map_1000 < 0 && t->voltage_factor_1000 == 0) {
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
    // One control mode per inverter per cycle - the inverter follows the last command
    // it received, so sending both would make it switch modes every cycle.
    if (throttle.regen_cmd_1000 > 0) {
        can_bus_send_FSIC_SetRelBrakeCurrent(1, (int16_t)throttle.regen_cmd_1000, hcan);
        can_bus_send_FSIC_SetRelBrakeCurrent(2, (int16_t)throttle.regen_cmd_1000, hcan);
    } else {
        // Drive: per motor, after traction control / torque vectoring (= drive_cmd_1000 when off)
        can_bus_send_FSIC_SetRelCurrent(INVERTER_ID_REAR_LEFT, (int16_t)throttle.drive_command_left_1000, hcan);
        can_bus_send_FSIC_SetRelCurrent(INVERTER_ID_REAR_RIGHT, (int16_t)throttle.drive_command_right_1000, hcan);
    }

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
