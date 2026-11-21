/**
 * @file throttle_control.h
 * @brief Manual driving: pedals -> vehicle dynamics layer -> motor current (Iq) for each rear inverter
 *
 * PIPELINE (throttle_control_update, every control cycle, in this order)
 *
 *   1. Inputs         pedal (APPS), brake pressure, wheel speeds (front AQT2, rear motor ERPM),
 *                     steering, faults, DC voltage
 *   2. Vehicle        vehicle_dynamics.h, everything in one layer: the pedal is a share of what the
 *      dynamics       wheels can take NOW (min of motor torque, power / speed, tyre with lateral
 *                     reserve), so it modulates the power the car can put down at this speed.
 *                     Regen ONLY from the brake pressure, linear regen_start_bar -> regen_full_bar
 *                     (vehicle_dynamics.c). Brake priority, APPS / brake plausibility, traction
 *                     control and torque vectoring (TRACTION_CONTROL_ENABLE / TORQUE_VECTORING_ENABLE
 *                     in traction_control_torque_vectoring.h). Out: Iq [A] per rear motor.
 *   3. Limits         regen: THROTTLE_REGEN_ENABLE, inverter fault -> none; faded near max pack voltage
 *   4. Torque rise    drive current rises at most THROTTLE_TORQUE_RISE_MAX_PCT_S of the max current,
 *      limit          to soften motor current spikes when the pedal is stabbed. Decreases are instant.
 *   5. Digital BSPD   LAST: when active, both currents are 0, whatever came before
 *
 *   throttle_control_send() then sends exactly one command per inverter:
 *     current >= 0 -> SetAcCurrent(current), current < 0 -> SetBrakeCurrent(-current), 0.1 A per bit
 *
 * IMU (autonomous bus, THROTTLE_IMU_GYRO / _ACCELERATIONS) into the layer. Without the accelerations the vehicle
 * speed comes from the rear wheels, rate-limited per ERPM frame; without the gyro, no torque vectoring.
 * Unit tests: tests/vehicle_dynamics/run_tests.py.
 *
 * LIVE EXPRESSIONS
 *   "throttle" for the inputs and the currents sent, "vehicle_dynamics" for every step of the layer.
 */

#ifndef THROTTLE_CONTROL_H
#define THROTTLE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "APPS.h"
#include "CAN_utils.h"
#include "vehicle_dynamics.h"

/* ============================= SWITCHES ============================= */

#define THROTTLE_REGEN_ENABLE        0  // 0 = regen off: brake pressure gives no motor braking
                                        // 0 for the first wet runs: regen not yet driven with the layer, brake pressure
                                        // not in the bags, brake at rest vs regen_start_bar (2 bar) not checked
#define THROTTLE_FRONT_WHEEL_SPEEDS  0  // 1 = use the AQT2 front wheel speeds (0: rear motors only, AQT2 ignored)
// IMU (autonomous bus 0x530 / 0x531; missing for IMU_TIMEOUT_MS = without). 13-10 replay (replay_logs.py), TC on,
// Milagres endurance: gyro only cuts a gripping wheel 6.6 % of the time (rear wheels only 12.2 %, + accelerations 33.5 %:
// the IMU vx lags and the driven wheels may only pull it down while driving).
#define THROTTLE_IMU_GYRO            1  // 1 = yaw rate from the IMU gyro (torque vectoring needs it)
#define THROTTLE_IMU_ACCELERATIONS   0  // 1 = ax / ay from the IMU into the vehicle speed estimate

/* =========================== TRACK SURFACE ========================== */
// Tyre friction x the tyre model (tyre_LMUX / tyre_LMUY in vehicle_dynamics.c, fitted on dry asphalt): lowers
// what the pedal gives, the regen, the traction control slip and how fast the car may speed up without the IMU.

// LSR 20-11, full wet (old firmware): the rear tyres let go at 0.64-0.68 x the dry model on 3 of 4 wheels (actual iq
// at breakaway, 10-18 km/h), one wheel at ~0.5. Without the IMU, traction control only holds a spin of BOTH rear wheels
// if the friction here is at or below the real one (car_sim: 0.6 on a 0.5 track spins out, 0.55 on 0.55 keeps 3 % slip).
// Start low, raise after a run with no spins.
#define THROTTLE_FRICTION_DRY  1.00f  // Dry track
#define THROTTLE_FRICTION_WET  0.55f  // Wet track
#define THROTTLE_WET_TRACK     1      // 1 = wet track: THROTTLE_FRICTION_WET, 0 = dry: THROTTLE_FRICTION_DRY

/* ========================= TORQUE RISE LIMIT ======================== */
// Max drive current rise rate, % of the max drive current per s (500 = 0 -> 100 % in 0.2 s). 0 = no limit.

#define THROTTLE_TORQUE_RISE_MAX_PCT_S  500

/* ========================== VEHICLE SPEED =========================== */
//   motor_rpm = ERPM / MOTOR_POLE_PAIRS
//   km/h      = motor_rpm / gear ratio * 2 * pi * wheel radius * 60 / 1000
// 1 km/h = ~192 motor rpm = ~769 ERPM; motor max 20000 rpm = ~104 km/h.
// Live Expressions / vehicle_sensors only: the layer uses its own gear ratio and radius (vehicle_dynamics.c).

#define THROTTLE_GEAR_RATIO      14.73f   // Motor-to-wheel reduction
#define THROTTLE_WHEEL_RADIUS_M  0.2032f  // Tire radius (m)

/* ========================= DC VOLTAGE LIMIT ========================= */
// Regen charges the pack: faded out near max pack voltage (inverter DC input voltage).
// TODO: set these for the accumulator. 0 = limit disabled.

#define THROTTLE_REGEN_FADE_START_V  0  // Regen starts fading above this voltage
#define THROTTLE_REGEN_CUTOFF_V      0  // No regen at or above this voltage

/* ======================= INVERTER BRAKE LIMITS ====================== */
// When enabled, the VCU sends these limits to both inverters every 100 ms while ready to drive.

#define THROTTLE_SEND_BRAKE_LIMITS       0
#define THROTTLE_MAX_AC_BRAKE_CURRENT_A  100  // Per inverter, motor side (A peak)
#define THROTTLE_MAX_DC_BRAKE_CURRENT_A   20  // Per inverter, battery side (A)

/* ============================== TYPES =============================== */

typedef enum {  // Sent in VCU_states.throttle_status (powertrain DBC) - keep its value table in sync
    THROTTLE_STATUS_COAST = 0,             // No current
    THROTTLE_STATUS_DRIVE,                 // Driving
    THROTTLE_STATUS_REGEN,                 // Regenerating
    THROTTLE_STATUS_REGEN_OFF_DC_VOLTAGE,  // Brake asks for regen, pack voltage too high
    THROTTLE_STATUS_APPS_ERROR,            // APPS error: no drive
    THROTTLE_STATUS_INVERTER_FAULT,        // Inverter fault: no regen (drive unchanged)
    THROTTLE_STATUS_BSPD_CUT               // Digital BSPD active: no drive, no regen
} throttle_status_t;

// Complete throttle state - one global instance, add "throttle" to Live Expressions
typedef struct {
    // 1. Inputs
    uint16_t pedal_1000;           // Accelerator, 0..1000
    float brake_bar;               // Brake pressure given to the layer
    bool brake_valid;              // Brake frame newer than SENSOR_TIMEOUT_MS
    bool apps_error;               // APPS plausibility error
    bool inverter_fault;           // Any inverter fault code
    bool bspd_active;              // Digital BSPD active
    uint16_t motor_rpm;            // Slower rear motor, mechanical rpm
    float speed_kmh;               // Vehicle speed from motor_rpm
    uint16_t dc_voltage_v;         // Highest inverter DC input voltage
    uint16_t voltage_factor_1000;  // DC voltage limit on regen (1000 = no limit)

    // 2..5. Currents sent to the inverters, Iq [A]: >= 0 SetAcCurrent, < 0 SetBrakeCurrent
    float current_left_a;          // Rear left motor
    float current_right_a;         // Rear right motor
    throttle_status_t status;

    // Internal
    uint32_t last_update_ms;
    bool first_update;
} throttle_t;

extern throttle_t throttle;
extern vehicle_dynamics_t vehicle_dynamics;

/* ============================ FUNCTIONS ============================= */

/**
 * @brief Clear the state (currents to 0, fresh layer). Call when entering ready-to-drive.
 */
void throttle_control_reset(void);

/**
 * @brief Run the whole pipeline for this cycle (see the top of this file)
 * @param apps        APPS result (pedal and error)
 * @param bspd_active Digital BSPD state - applied last, overrides everything
 * @param inv1, inv2  Inverter feedback (speed, faults, DC voltage)
 * @param power_w     Tractive power (IVT), not used (the layer has its own power limit)
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
