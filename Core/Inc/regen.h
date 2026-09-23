/**
 * @file regen.h
 * @brief Lift-off regenerative braking for the FSIC inverters (manual driving)
 *
 * HOW IT FEELS
 *   Like a road EV (Tesla standard mode): the pedal asks for torque, and the pedal
 *   position that gives zero torque depends on speed. Lift off and the car slows down
 *   on its own. At high speed even a half-pressed pedal can still regen, because it is
 *   less than what is needed to hold that speed. The brake pedal stays purely
 *   hydraulic - it does not add regen.
 *
 * HOW IT WORKS (all values are per mille: 0..1000 = 0..100 %)
 *
 *   1. Pedal map - the pedal is split at a "coast point" (zero torque):
 *
 *        regen strength                                   drive torque
 *        100% |####                                              /  100%
 *             |    ####                                        /
 *             |        ####                                  /
 *          0% +------------####--------------------------  /------  0%
 *             0%   3%          coast   +4%                         100% pedal
 *             |full|    regen  point  coast|  drive (rescaled 0..100%)  |
 *
 *      The coast point moves with speed:
 *        standstill                   -> 8% pedal
 *        REGEN_COAST_SPEED_HIGH_KMH   -> 40% pedal  (straight line in between)
 *      So at low speed almost any pedal drives, and at high speed the lower part of
 *      the pedal is regen. Below the coast point regen grows the further the pedal is
 *      lifted; above it (plus a small coast band) torque grows up to 100% at full pedal.
 *
 *   2. Speed fade - regen is scaled by vehicle speed:
 *        below REGEN_SPEED_MIN_KMH   -> no regen (rules: no regen at walking speed)
 *        between MIN and FULL        -> fades in linearly
 *        above REGEN_SPEED_FULL_KMH  -> full regen
 *      Speed comes from the slower of the two rear motors, and both inverters always
 *      get the same command, so regen never creates a yaw moment.
 *
 *   3. Limits - regen is cut to zero immediately on an APPS error (a failed pedal
 *      reads 0%, which would otherwise mean full regen) or any inverter fault, and is
 *      faded out as the DC link approaches the pack's max voltage.
 *
 *   4. Ramp - the final command is rate limited: regen builds in gently (no jerk when
 *      lifting off) and releases quickly (no delay when pressing the accelerator).
 *      Drive torque is only sent after regen has fully released, so the inverter never
 *      receives drive and brake requests at the same time.
 *
 *   5. Output - exactly one command per cycle and per inverter:
 *        regen > 0  -> SetRelBrakeCurrent(regen)   (0..1000 = % of inverter max brake current)
 *        otherwise  -> SetRelCurrent(drive)        (0..1000 = % of inverter max current)
 *
 * TUNING
 *   Everything that changes the feel is a #define below. The absolute regen strength
 *   is set by the max brake current configured in the inverters; start that low on
 *   the first runs and raise it step by step.
 *
 * LIVE EXPRESSIONS
 *   Add "regen" to see every input, intermediate factor, output and why regen is off.
 */

#ifndef REGEN_H
#define REGEN_H

#include <stdbool.h>
#include <stdint.h>

#include "CAN_utils.h"

/* ============================== ENABLE ============================== */

// 0 = regen off, pedal behaves exactly like before (0..100% pedal -> 0..100% drive)
#define REGEN_ENABLE 1

/* ============================ STRENGTH ============================== */

// Maximum regen, in per mille of the inverter's configured max brake current.
// 1000 = 100.0% (steps of 0.1%). 250 = full lift-off asks for 25% of the brake
// current set in the inverters.
#define REGEN_MAX_1000 250

/* ============================ PEDAL MAP ============================= */
// Pedal positions in per mille of travel (APPS percentage_1000).

#define REGEN_PEDAL_FULL_REGEN_1000   30   // Below 3% pedal: full regen (absorbs pedal noise at rest)
#define REGEN_COAST_PEDAL_LOW_1000    80   // Coast point at standstill: 8% pedal
#define REGEN_COAST_PEDAL_HIGH_1000  400   // Coast point at REGEN_COAST_SPEED_HIGH_KMH and above: 40% pedal
#define REGEN_COAST_SPEED_HIGH_KMH   100   // Speed where the coast point reaches its highest (km/h), ~top speed
#define REGEN_COAST_BAND_1000         40   // Zero-torque band above the coast point: 4% pedal

/* ========================== VEHICLE SPEED =========================== */
// Speed is calculated from the motor ERPM:
//   motor_rpm = ERPM / MOTOR_POLE_PAIRS
//   wheel_rpm = motor_rpm / REGEN_GEAR_RATIO
//   km/h      = wheel_rpm * 2 * pi * REGEN_WHEEL_RADIUS_M * 60 / 1000
// With the values below: 1 km/h = ~192 motor rpm = ~769 ERPM, and the motor's max
// 20000 rpm = ~104 km/h (the car's top speed).

#define REGEN_GEAR_RATIO      14.73f   // Motor-to-wheel reduction
#define REGEN_WHEEL_RADIUS_M  0.2032f  // Tire radius (m)

/* ============================ SPEED FADE ============================ */

#define REGEN_SPEED_MIN_KMH    6   // No regen below this speed (km/h)
#define REGEN_SPEED_FULL_KMH  15   // Full regen above this speed (km/h)

/* =============================== RAMPS ============================== */
// Maximum change of the regen command, in per mille per second.

#define REGEN_RAMP_UP_PER_S    1000  // Build-in: 0 -> 250 takes 0.25 s
#define REGEN_RAMP_DOWN_PER_S  5000  // Release:  250 -> 0 takes 0.05 s

/* ========================= DC VOLTAGE LIMIT ========================= */
// Regen charges the pack. Near max pack voltage (full battery) regen is faded out
// so the cells are not pushed over their limit. Uses the inverter's DC input voltage.
// TODO: set these for the accumulator. 0 = limit disabled.

#define REGEN_DC_VOLTAGE_FADE_START_V  0  // Regen starts fading above this voltage
#define REGEN_DC_VOLTAGE_CUTOFF_V      0  // No regen at or above this voltage

/* ======================= INVERTER BRAKE LIMITS ====================== */
// Regen strength is relative to the inverter's max brake current. If the inverters
// are not configured with a max brake current (or it is 0) regen does nothing.
// When enabled, the VCU sends these limits to both inverters every 100 ms while
// ready to drive. The DC limit is the battery charge current limit per inverter.

#define REGEN_SEND_INVERTER_LIMITS         0
#define REGEN_MAX_AC_BRAKE_CURRENT_A     100  // Per inverter, motor side (A peak)
#define REGEN_MAX_DC_BRAKE_CURRENT_A      20  // Per inverter, battery side (A)

/* ============================== TYPES =============================== */

// What regen is doing right now (shown in Live Expressions)
typedef enum {
    REGEN_STATUS_DRIVE = 0,          // Accelerator pressed, driving
    REGEN_STATUS_COAST,              // Pedal in the coast zone, zero torque
    REGEN_STATUS_REGEN,              // Regenerating
    REGEN_STATUS_OFF_LOW_SPEED,      // Pedal released but too slow for regen
    REGEN_STATUS_OFF_DC_VOLTAGE,     // Pedal released but pack voltage too high
    REGEN_STATUS_OFF_APPS_ERROR,     // Blocked: APPS error
    REGEN_STATUS_OFF_INVERTER_FAULT, // Blocked: inverter fault code present
    REGEN_STATUS_DISABLED            // REGEN_ENABLE is 0
} regen_status_t;

// Inputs sampled by the caller every control cycle
typedef struct {
    uint16_t pedal_1000;          // Accelerator pedal position, 0..1000 (before PAU/BSPD)
    uint16_t drive_request_1000;  // Accelerator after PAU/BSPD limits, 0..1000
    bool apps_error;              // APPS plausibility error
    int32_t erpm_left;            // INV1 electrical RPM
    int32_t erpm_right;           // INV2 electrical RPM
    uint8_t fault_left;           // INV1 fault code (0 = OK)
    uint8_t fault_right;          // INV2 fault code (0 = OK)
    uint16_t dc_voltage_v;        // Inverter DC input voltage (V)
} regen_inputs_t;

// Complete regen state - one global instance, add "regen" to Live Expressions
typedef struct {
    regen_inputs_t in;            // Last inputs received

    uint16_t motor_rpm;           // Slower of the two motors, mechanical RPM (absolute)
    float vehicle_speed_kmh;      // Vehicle speed calculated from motor_rpm
    uint16_t coast_pedal_1000;    // Pedal position giving zero torque at this speed

    // Factors, 0..1000 each. Regen target = REGEN_MAX_1000 * pedal * speed * voltage.
    uint16_t pedal_factor_1000;   // From the pedal map (1000 = pedal fully released)
    uint16_t speed_factor_1000;   // From the speed fade (1000 = fast enough for full regen)
    uint16_t voltage_factor_1000; // From the DC voltage limit (1000 = no limit)

    uint16_t regen_target_1000;   // Regen wanted this cycle, before the ramp
    uint16_t regen_cmd_1000;      // Regen actually sent (ramped) -> SetRelBrakeCurrent
    uint16_t drive_cmd_1000;      // Drive actually sent          -> SetRelCurrent

    regen_status_t status;        // What regen is doing and why

    uint32_t last_update_ms;      // Tick of the previous update, for the ramp
    bool first_update;            // True until the first update after a reset
} regen_t;

extern regen_t regen;

/* ============================ FUNCTIONS ============================= */

/**
 * @brief Clear all regen state. Call when entering or leaving ready-to-drive.
 */
void regen_reset(void);

/**
 * @brief Compute the regen and drive commands for this cycle
 * @param in     Inputs sampled this cycle
 * @param now_ms HAL_GetTick()
 * @details Results are in regen.regen_cmd_1000 and regen.drive_cmd_1000.
 */
void regen_update(const regen_inputs_t *in, uint32_t now_ms);

/**
 * @brief Send the current commands to both inverters (one mode per inverter)
 * @param hcan   Powertrain CAN handle
 * @param now_ms HAL_GetTick(), used to rate-limit the brake-limit frames
 */
void regen_send_to_inverters(CAN_HandleTypeDef *hcan, uint32_t now_ms);

#endif  // REGEN_H
