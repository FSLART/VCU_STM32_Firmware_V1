/**
 * @file throttle_control.h
 * @brief Manual driving: accelerator -> relative current request for both inverters
 *
 * PIPELINE (throttle_control_update, every control cycle, in this order)
 *
 *   1. Inputs         pedal (APPS), vehicle speed (slower rear motor), faults, DC voltage
 *   2. Throttle map   pedal x speed -> relative current (throttle_map_data.h, edited with
 *                     tools/throttle_map_editor.py): > 0 drive, < 0 regen, 0 coast
 *   3. PAU            power limit on drive (pau_control.h, when PAU_CONTROL_ENABLE)
 *   4. Limits         APPS error -> no drive, no regen; inverter fault -> no regen;
 *                     regen faded near max pack voltage; THROTTLE_REGEN_ENABLE
 *   5. Regen ramp     regen builds in gently and releases fast; drive only starts once
 *                     regen has fully released (never drive and brake at the same time)
 *   6. Torque rise    torque rate limit: drive torque rises at most
 *      limit          THROTTLE_TORQUE_RISE_MAX_PCT_S, to soften motor current spikes when
 *                     the pedal is stabbed. Decreases are instant. (Low-speed traction
 *                     limiting is in the map itself: lower drive values at low speed.)
 *   7. Digital BSPD   LAST: when active, drive and regen are 0, whatever came before
 *
 *   throttle_control_send() then sends exactly one command per inverter:
 *     regen > 0 -> SetRelBrakeCurrent(regen), otherwise SetRelCurrent(drive)
 *   Both inverters always get the same command, so there is never a yaw moment.
 *
 * All values are per mille: 0..1000 = 0..100.0 % of the inverter's configured maximum
 * (max current for drive, max brake current for regen).
 *
 * LIVE EXPRESSIONS
 *   Add "throttle" to see every step: inputs, map value, targets, commands and status.
 */

#ifndef THROTTLE_CONTROL_H
#define THROTTLE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "APPS.h"
#include "CAN_utils.h"

/* ============================= SWITCHES ============================= */

#define THROTTLE_REGEN_ENABLE        1  // 0 = regen off: negative map values give coast

/* ========================= TORQUE RISE LIMIT ======================== */
// Max drive torque rise rate, %/s (500 = 0 -> 100 % in 0.2 s). 0 = no limit.

#define THROTTLE_TORQUE_RISE_MAX_PCT_S  500

/* =========================== REGEN RAMP ============================= */
// Time for the regen command to go between 0 and 100 %.

#define THROTTLE_REGEN_RAMP_UP_MS    250  // Build-in: 0 -> 100 % regen
#define THROTTLE_REGEN_RAMP_DOWN_MS   50  // Release:  100 % regen -> 0 (drive waits for this)

/* ========================== VEHICLE SPEED =========================== */
//   motor_rpm = ERPM / MOTOR_POLE_PAIRS
//   km/h      = motor_rpm / gear ratio * 2 * pi * wheel radius * 60 / 1000
// 1 km/h = ~192 motor rpm = ~769 ERPM; motor max 20000 rpm = ~104 km/h.

#define THROTTLE_GEAR_RATIO      14.73f   // Motor-to-wheel reduction
#define THROTTLE_WHEEL_RADIUS_M  0.2032f  // Tire radius (m)

/* ========================= DC VOLTAGE LIMIT ========================= */
// Regen charges the pack: faded out near max pack voltage (inverter DC input voltage).
// TODO: set these for the accumulator. 0 = limit disabled.

#define THROTTLE_REGEN_FADE_START_V  0  // Regen starts fading above this voltage
#define THROTTLE_REGEN_CUTOFF_V      0  // No regen at or above this voltage

/* ======================= INVERTER BRAKE LIMITS ====================== */
// Regen is relative to the inverter's max brake current. When enabled, the VCU sends
// these limits to both inverters every 100 ms while ready to drive.

#define THROTTLE_SEND_BRAKE_LIMITS       0
#define THROTTLE_MAX_AC_BRAKE_CURRENT_A  100  // Per inverter, motor side (A peak)
#define THROTTLE_MAX_DC_BRAKE_CURRENT_A   20  // Per inverter, battery side (A)

/* ============================== TYPES =============================== */

typedef enum {
    THROTTLE_STATUS_COAST = 0,             // Map gives zero torque here
    THROTTLE_STATUS_DRIVE,                 // Driving
    THROTTLE_STATUS_REGEN,                 // Regenerating
    THROTTLE_STATUS_REGEN_OFF_DC_VOLTAGE,  // Map asks for regen, pack voltage too high
    THROTTLE_STATUS_APPS_ERROR,            // APPS error: no drive, no regen
    THROTTLE_STATUS_INVERTER_FAULT,        // Inverter fault: no regen (drive unchanged)
    THROTTLE_STATUS_BSPD_CUT               // Digital BSPD active: no drive, no regen
} throttle_status_t;

// Complete throttle state - one global instance, add "throttle" to Live Expressions
typedef struct {
    // 1. Inputs
    uint16_t pedal_1000;           // Accelerator, 0..1000
    bool apps_error;               // APPS plausibility error
    bool inverter_fault;           // Any inverter fault code
    bool bspd_active;              // Digital BSPD active
    uint16_t motor_rpm;            // Slower rear motor, mechanical rpm
    float speed_kmh;               // Vehicle speed from motor_rpm
    uint16_t dc_voltage_v;         // Highest inverter DC input voltage

    // 2..4. Map and limits
    int16_t map_1000;              // Map value (after PAU): + drive, - regen
    uint16_t voltage_factor_1000;  // DC voltage limit on regen (1000 = no limit)
    uint16_t drive_target_1000;    // Drive wanted, before the ramps
    uint16_t regen_target_1000;    // Regen wanted, before the ramp

    // 5..7. Commands sent to the inverters
    uint16_t drive_cmd_1000;       // -> SetRelCurrent
    uint16_t regen_cmd_1000;       // -> SetRelBrakeCurrent
    throttle_status_t status;

    // Internal
    float drive_ramped;            // Drive with fractions, so slow ramps do not lose steps
    uint32_t last_update_ms;
    bool first_update;
} throttle_t;

extern throttle_t throttle;

/* ============================ FUNCTIONS ============================= */

/**
 * @brief Clear the state (commands to 0, fresh ramps). Call when entering ready-to-drive.
 */
void throttle_control_reset(void);

/**
 * @brief Run the whole pipeline for this cycle (see the top of this file)
 * @param apps        APPS result (pedal and error)
 * @param bspd_active Digital BSPD state - applied last, overrides everything
 * @param inv1, inv2  Inverter feedback (speed, faults, DC voltage)
 * @param power_w     Tractive power (IVT), used by the PAU power limit
 * @param now_ms      HAL_GetTick()
 */
void throttle_control_update(const APPS_Result_t *apps, bool bspd_active, const FSIC_t *inv1,
                             const FSIC_t *inv2, int32_t power_w, uint32_t now_ms);

/**
 * @brief Leaving ready-to-drive: clear the state and send zero current to both inverters,
 *        so they do not keep executing the last drive or regen command
 * @param hcan Powertrain CAN handle
 */
void throttle_control_stop(CAN_HandleTypeDef *hcan);

/**
 * @brief Send the commands to both inverters (one control mode per inverter)
 * @param hcan   Powertrain CAN handle
 * @param now_ms HAL_GetTick(), used to rate-limit the brake-limit frames
 */
void throttle_control_send(CAN_HandleTypeDef *hcan, uint32_t now_ms);

#endif  // THROTTLE_CONTROL_H
