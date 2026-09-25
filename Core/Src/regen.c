/**
 * @file regen.c
 * @brief Lift-off regenerative braking for the FSIC inverters (manual driving)
 *
 * See regen.h for how the strategy works and for all tuning values.
 */

#include "regen.h"

#include <string.h>

// Compile-time sanity checks on the tuning values
#if (REGEN_PEDAL_FULL_REGEN_1000 >= REGEN_COAST_PEDAL_LOW_1000) || \
    (REGEN_COAST_PEDAL_LOW_1000 > REGEN_COAST_PEDAL_HIGH_1000) ||  \
    (REGEN_COAST_PEDAL_HIGH_1000 + REGEN_COAST_BAND_1000 >= 1000)
#error "regen.h: pedal map must satisfy FULL_REGEN < COAST_LOW <= COAST_HIGH and COAST_HIGH + BAND < 1000"
#endif
#if REGEN_COAST_SPEED_HIGH_KMH <= 0
#error "regen.h: REGEN_COAST_SPEED_HIGH_KMH must be above 0"
#endif
#if REGEN_SPEED_MIN_KMH >= REGEN_SPEED_FULL_KMH
#error "regen.h: REGEN_SPEED_MIN_KMH must be below REGEN_SPEED_FULL_KMH"
#endif
#if (REGEN_MAX_1000 < 0) || (REGEN_MAX_1000 > 1000)
#error "regen.h: REGEN_MAX_1000 must be 0..1000"
#endif
#if (REGEN_SOFT_MAX_1000 < 0) || (REGEN_SOFT_MAX_1000 > REGEN_MAX_1000)
#error "regen.h: REGEN_SOFT_MAX_1000 must be 0..REGEN_MAX_1000"
#endif
#if (REGEN_SOFT_LIFT_1000 <= 0) || (REGEN_SOFT_LIFT_1000 >= 1000)
#error "regen.h: REGEN_SOFT_LIFT_1000 must be between 0 and 1000"
#endif
#if (REGEN_RAMP_UP_MS <= 0) || (REGEN_RAMP_DOWN_MS <= 0)
#error "regen.h: REGEN_RAMP_UP_MS and REGEN_RAMP_DOWN_MS must be above 0"
#endif
#if (REGEN_DC_VOLTAGE_CUTOFF_V > 0) && (REGEN_DC_VOLTAGE_FADE_START_V >= REGEN_DC_VOLTAGE_CUTOFF_V)
#error "regen.h: REGEN_DC_VOLTAGE_FADE_START_V must be below REGEN_DC_VOLTAGE_CUTOFF_V"
#endif

#define REGEN_MAX_DT_MS 50               // Longest time step the ramp will integrate
#define REGEN_LIMITS_PERIOD_MS 100       // How often the brake-limit frames are resent

regen_t regen;

/* ========================= HELPER FUNCTIONS ========================= */

/**
 * @brief Linear interpolation that returns a factor 0..1000
 * @return 0 at or below x0, 1000 at or above x1, a straight line in between
 */
static uint16_t ramp_factor(float x, float x0, float x1) {
    if (x <= x0) return 0;
    if (x >= x1) return 1000;
    return (uint16_t)(((x - x0) * 1000.0f) / (x1 - x0));
}

/**
 * @brief Move a value towards a target by at most rate_per_s * dt
 */
static uint16_t rate_limit(uint16_t current, uint16_t target, uint32_t rate_per_s, uint32_t dt_ms) {
    // Round up so small time steps still move by at least 1
    uint32_t max_step = (rate_per_s * dt_ms + 999u) / 1000u;

    if (target > current) {
        uint32_t step = target - current;
        return (uint16_t)(current + (step < max_step ? step : max_step));
    } else {
        uint32_t step = current - target;
        return (uint16_t)(current - (step < max_step ? step : max_step));
    }
}

static uint32_t abs_i32(int32_t v) {
    return (v < 0) ? (uint32_t)(-v) : (uint32_t)v;
}

/* ===================== STEP 1: PEDAL MAP ============================ */

/**
 * @brief Pedal position that gives zero torque at the current speed
 * @details Grows in a straight line from REGEN_COAST_PEDAL_LOW_1000 at standstill to
 *          REGEN_COAST_PEDAL_HIGH_1000 at REGEN_COAST_SPEED_HIGH_KMH, then stays there.
 */
static uint16_t coast_pedal_at_speed(float speed_kmh) {
    uint32_t span = REGEN_COAST_PEDAL_HIGH_1000 - REGEN_COAST_PEDAL_LOW_1000;
    return (uint16_t)(REGEN_COAST_PEDAL_LOW_1000 + span * ramp_factor(speed_kmh, 0, REGEN_COAST_SPEED_HIGH_KMH) / 1000u);
}

/**
 * @brief How much regen the pedal position asks for
 * @return 1000 with the pedal released, fading to 0 at the coast point
 */
static uint16_t pedal_regen_factor(uint16_t pedal_1000, uint16_t coast_pedal_1000) {
    return 1000 - ramp_factor(pedal_1000, REGEN_PEDAL_FULL_REGEN_1000, coast_pedal_1000);
}

/**
 * @brief Two-stage regen curve over the lift depth
 * @param lift_depth_1000 0 = pedal at the coast point, 1000 = pedal fully released
 * @return Regen, 0..REGEN_MAX_1000
 * @details Soft stage: 0 -> REGEN_SOFT_MAX_1000 over the first REGEN_SOFT_LIFT_1000 of
 *          the lift (partial lift for a corner). Strong stage: REGEN_SOFT_MAX_1000 ->
 *          REGEN_MAX_1000 over the rest of the lift (foot off the pedal).
 */
static uint16_t regen_from_lift_depth(uint16_t lift_depth_1000) {
    if (lift_depth_1000 <= REGEN_SOFT_LIFT_1000) {
        return (uint16_t)((uint32_t)REGEN_SOFT_MAX_1000 * lift_depth_1000 / REGEN_SOFT_LIFT_1000);
    }
    uint32_t strong_part = ramp_factor(lift_depth_1000, REGEN_SOFT_LIFT_1000, 1000);
    return (uint16_t)(REGEN_SOFT_MAX_1000 + (REGEN_MAX_1000 - REGEN_SOFT_MAX_1000) * strong_part / 1000u);
}

/**
 * @brief Rescale the drive request so the drive zone still covers 0..100% torque
 * @details Pedal below the coast point plus the coast band gives zero torque. From there
 *          to full travel the torque goes 0..1000, so full pedal is still full torque.
 */
static uint16_t drive_from_pedal(uint16_t drive_request_1000, uint16_t coast_pedal_1000) {
    return ramp_factor(drive_request_1000, coast_pedal_1000 + REGEN_COAST_BAND_1000, 1000);
}

/* ===================== STEP 2: SPEED FADE =========================== */

/**
 * @brief Speed of the slower rear motor, in mechanical RPM
 * @details The slower wheel decides so a wheel that is slowing down (locking) under
 *          regen reduces regen on both sides. Absolute value: the two motors may turn
 *          in opposite electrical directions depending on how they are mounted.
 */
static uint16_t slowest_motor_rpm(int32_t erpm_left, int32_t erpm_right) {
    uint32_t left = abs_i32(erpm_left) / MOTOR_POLE_PAIRS;
    uint32_t right = abs_i32(erpm_right) / MOTOR_POLE_PAIRS;
    uint32_t slowest = (left < right) ? left : right;
    return (slowest > UINT16_MAX) ? UINT16_MAX : (uint16_t)slowest;
}

/**
 * @brief Vehicle speed from motor speed, through the gearbox and the tire
 */
static float speed_kmh_from_motor_rpm(uint16_t motor_rpm) {
    const float two_pi = 6.2831853f;
    float wheel_rpm = motor_rpm / REGEN_GEAR_RATIO;
    return wheel_rpm * two_pi * REGEN_WHEEL_RADIUS_M * 60.0f / 1000.0f;
}

/* ===================== STEP 3: DC VOLTAGE LIMIT ===================== */

/**
 * @brief Fade regen out as the pack approaches full voltage
 * @return 1000 = no limit, 0 = no regen allowed
 */
static uint16_t dc_voltage_factor(uint16_t dc_voltage_v) {
#if REGEN_DC_VOLTAGE_CUTOFF_V > 0
    return 1000 - ramp_factor(dc_voltage_v, REGEN_DC_VOLTAGE_FADE_START_V, REGEN_DC_VOLTAGE_CUTOFF_V);
#else
    (void)dc_voltage_v;
    return 1000;  // Limit disabled until the accumulator voltages are set in regen.h
#endif
}

/* ============================ PUBLIC API ============================ */

void regen_reset(void) {
    memset(&regen, 0, sizeof(regen));
    regen.first_update = true;
    regen.status = REGEN_ENABLE ? REGEN_STATUS_COAST : REGEN_STATUS_DISABLED;
}

void regen_update(const regen_inputs_t *in, uint32_t now_ms) {
    regen.in = *in;

    // Time since the last update, for the ramp
    uint32_t dt_ms = regen.first_update ? 0 : (now_ms - regen.last_update_ms);
    if (dt_ms > REGEN_MAX_DT_MS) dt_ms = REGEN_MAX_DT_MS;
    regen.last_update_ms = now_ms;
    regen.first_update = false;

    // Vehicle speed - always calculated, the torque ramp uses it even with regen disabled
    regen.motor_rpm = slowest_motor_rpm(in->erpm_left, in->erpm_right);
    regen.vehicle_speed_kmh = speed_kmh_from_motor_rpm(regen.motor_rpm);

#if !REGEN_ENABLE
    // Regen off: the pedal drives exactly like before
    regen.regen_target_1000 = 0;
    regen.regen_cmd_1000 = 0;
    regen.drive_cmd_1000 = in->drive_request_1000;
    regen.status = REGEN_STATUS_DISABLED;
    return;
#endif

    /* --- Hard blocks: cut regen instantly, no ramp --- */
    regen_status_t block = REGEN_STATUS_REGEN;
    if (in->apps_error) {
        block = REGEN_STATUS_OFF_APPS_ERROR;
    } else if (in->fault_left != 0 || in->fault_right != 0) {
        block = REGEN_STATUS_OFF_INVERTER_FAULT;
    }

    /* --- Factors --- */
    regen.coast_pedal_1000 = coast_pedal_at_speed(regen.vehicle_speed_kmh);
    regen.pedal_factor_1000 = pedal_regen_factor(in->pedal_1000, regen.coast_pedal_1000);
    regen.lift_regen_1000 = regen_from_lift_depth(regen.pedal_factor_1000);
    regen.speed_factor_1000 = ramp_factor(regen.vehicle_speed_kmh, REGEN_SPEED_MIN_KMH, REGEN_SPEED_FULL_KMH);
    regen.voltage_factor_1000 = dc_voltage_factor(in->dc_voltage_v);

    /* --- Regen target: max strength scaled by every factor --- */
    uint32_t target = regen.lift_regen_1000;
    target = target * regen.speed_factor_1000 / 1000u;
    target = target * regen.voltage_factor_1000 / 1000u;

    uint16_t drive = drive_from_pedal(in->drive_request_1000, regen.coast_pedal_1000);
    if (drive > 0) {
        target = 0;  // Driver is asking for torque: release regen
    }

    if (block != REGEN_STATUS_REGEN) {
        target = 0;
        regen.regen_cmd_1000 = 0;  // Instant cut
    }
    regen.regen_target_1000 = (uint16_t)target;

    /* --- Ramp: gentle build-in, fast release --- */
    uint32_t ramp_ms = (target > regen.regen_cmd_1000) ? REGEN_RAMP_UP_MS : REGEN_RAMP_DOWN_MS;
    uint32_t rate = (uint32_t)REGEN_MAX_1000 * 1000u / ramp_ms;  // per mille per second
    regen.regen_cmd_1000 = rate_limit(regen.regen_cmd_1000, regen.regen_target_1000, rate, dt_ms);

    /* --- Drive torque only once regen has fully released --- */
    regen.drive_cmd_1000 = (regen.regen_cmd_1000 == 0) ? drive : 0;

    /* --- Status for Live Expressions --- */
    if (block != REGEN_STATUS_REGEN) {
        regen.status = block;
    } else if (regen.regen_cmd_1000 > 0) {
        regen.status = REGEN_STATUS_REGEN;
    } else if (regen.drive_cmd_1000 > 0) {
        regen.status = REGEN_STATUS_DRIVE;
    } else if (regen.pedal_factor_1000 > 0 && regen.speed_factor_1000 == 0) {
        regen.status = REGEN_STATUS_OFF_LOW_SPEED;
    } else if (regen.pedal_factor_1000 > 0 && regen.voltage_factor_1000 == 0) {
        regen.status = REGEN_STATUS_OFF_DC_VOLTAGE;
    } else {
        regen.status = REGEN_STATUS_COAST;
    }
}

void regen_send_to_inverters(CAN_HandleTypeDef *hcan, uint16_t drive_1000, uint32_t now_ms) {
    // One control mode per inverter per cycle - the inverter follows the last command
    // it received, so sending both would make it switch modes every cycle.
    if (regen.regen_cmd_1000 > 0) {
        can_bus_send_FSIC_SetRelBrakeCurrent(1, (int16_t)regen.regen_cmd_1000, hcan);
        can_bus_send_FSIC_SetRelBrakeCurrent(2, (int16_t)regen.regen_cmd_1000, hcan);
    } else {
        can_bus_send_FSIC_SetRelCurrent(1, (int16_t)drive_1000, hcan);
        can_bus_send_FSIC_SetRelCurrent(2, (int16_t)drive_1000, hcan);
    }

#if REGEN_ENABLE && REGEN_SEND_INVERTER_LIMITS
    // Max brake currents (negative, scale 0.1 A), resent periodically in case a frame is lost
    static uint32_t last_limits_ms = 0;
    if (now_ms - last_limits_ms >= REGEN_LIMITS_PERIOD_MS) {
        int16_t max_ac_brake = (int16_t)(-REGEN_MAX_AC_BRAKE_CURRENT_A * 10);
        int16_t max_dc_brake = (int16_t)(-REGEN_MAX_DC_BRAKE_CURRENT_A * 10);
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
