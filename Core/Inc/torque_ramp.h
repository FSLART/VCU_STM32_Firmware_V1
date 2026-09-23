/**
 * @file torque_ramp.h
 * @brief Speed-dependent ramp on the drive torque request (manual driving)
 *
 * WHY
 *   A sudden 0 -> 100% torque request at standstill makes the motor current jump as
 *   fast as the inverter allows (no back-EMF yet to slow it down). That causes large
 *   current spikes, EMI on the CAN bus and a clunk through the driveline. Ramping the
 *   request spreads the rise over a short time.
 *
 * HOW IT WORKS
 *   Torque INCREASES are limited to a maximum rise speed. The ramp time is the time
 *   to go from 0 to 100% torque, and depends on vehicle speed:
 *
 *        ramp time
 *        800 ms |*
 *               |  *
 *               |    *
 *               |      *
 *        200 ms |        ***********
 *               |                   |
 *          none +-------------------*************  -> vehicle speed
 *               0       20         40 km/h
 *
 *     standstill .. 20 km/h   ramp time goes from 800 ms to 200 ms (straight line)
 *     20 .. 40 km/h           200 ms
 *     above 40 km/h           no ramp, the request passes straight through
 *
 *   Torque DECREASES are never ramped: lifting off, an APPS error or a BSPD cut
 *   removes torque immediately.
 *
 * LIVE EXPRESSIONS
 *   Add "torque_ramp" to see the requested torque, the output and the ramp time in use.
 */

#ifndef TORQUE_RAMP_H
#define TORQUE_RAMP_H

#include <stdbool.h>
#include <stdint.h>

/* ============================== ENABLE ============================== */

// 0 = torque ramp off, the drive request is sent to the inverters unchanged
#define TORQUE_RAMP_ENABLE 1

/* ============================ RAMP TIMES ============================ */
// Time for the torque request to go from 0 to 100%.

#define TORQUE_RAMP_MS_STANDSTILL       800  // At 0 km/h
#define TORQUE_RAMP_MS_MOVING           200  // From TORQUE_RAMP_SPEED_MOVING_KMH up to TORQUE_RAMP_SPEED_OFF_KMH
#define TORQUE_RAMP_SPEED_MOVING_KMH     20  // Speed where the ramp reaches TORQUE_RAMP_MS_MOVING (km/h)
#define TORQUE_RAMP_SPEED_OFF_KMH        40  // No ramp at or above this speed (km/h)

/* ============================== TYPES =============================== */

// Complete torque ramp state - one global instance, add "torque_ramp" to Live Expressions
typedef struct {
    uint16_t target_1000;    // Drive torque requested, 0..1000 (before the ramp)
    uint16_t output_1000;    // Drive torque sent, 0..1000 (after the ramp)
    float speed_kmh;         // Vehicle speed used to choose the ramp time
    uint16_t ramp_ms;        // Ramp time in use (0 = no ramp at this speed)

    float output_exact;      // Output with fractions, so slow ramps do not lose steps
    uint32_t last_update_ms; // Tick of the previous update
    bool first_update;       // True until the first update after a reset
} torque_ramp_t;

extern torque_ramp_t torque_ramp;

/* ============================ FUNCTIONS ============================= */

/**
 * @brief Clear the ramp state. Call when entering ready-to-drive.
 */
void torque_ramp_reset(void);

/**
 * @brief Apply the speed-dependent ramp to a drive torque request
 * @param target_1000 Drive torque requested, 0..1000
 * @param speed_kmh   Current vehicle speed (km/h)
 * @param now_ms      HAL_GetTick()
 * @return Drive torque to send to the inverters, 0..1000
 */
uint16_t torque_ramp_update(uint16_t target_1000, float speed_kmh, uint32_t now_ms);

#endif  // TORQUE_RAMP_H
