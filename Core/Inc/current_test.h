/**
 * @file current_test.h
 * @brief Bench test: constant relative current above an accelerator threshold
 *
 * PURPOSE
 *   Bench tests of the inverter response to a known, constant current request,
 *   while the rest of the pedal keeps working normally:
 *
 *        drive request
 *        CURRENT_TEST_REL_CURRENT |                   +--------------
 *                                 |       normal      |   constant
 *                                 |    (regen, ramp)  |
 *                               0 +-------------------+-------------> accelerator
 *                                 0%                 50%          100%
 *
 *     accelerator below the threshold  -> normal pedal map (regen + torque ramp)
 *     accelerator >= threshold         -> SetRelCurrent(CURRENT_TEST_REL_CURRENT_1000)
 *
 *   The constant value is sent as-is (no torque ramp), so the inverter sees exactly the
 *   value being tested. To stop flickering when the pedal sits right at the threshold,
 *   the constant only switches off when the pedal drops CURRENT_TEST_HYSTERESIS_1000
 *   below the threshold.
 *
 * SAFETY
 *   On an APPS error or when the digital BSPD cuts the accelerator the constant is
 *   never applied (the normal map then sends 0). Regen handover is unchanged: if regen
 *   is still releasing, drive (and the constant) waits for it, as in normal driving.
 *
 * LIVE EXPRESSIONS
 *   Add "current_test" to see the pedal, whether the constant is active and what is sent.
 *
 * All values are per mille: 0..1000 = 0..100.0 %.
 */

#ifndef CURRENT_TEST_H
#define CURRENT_TEST_H

#include <stdbool.h>
#include <stdint.h>

/* ============================== ENABLE ============================== */

// 1 = constant request above the threshold, 0 = normal driving over the whole pedal
#define CURRENT_TEST_ENABLE 1

/* ============================ SETTINGS ============================== */

#define CURRENT_TEST_APPS_ON_1000      500  // Constant starts at 50% accelerator
#define CURRENT_TEST_HYSTERESIS_1000    20  // Constant stops below 48% (50% - 2%)
#define CURRENT_TEST_REL_CURRENT_1000  600  // Constant relative current: 20.0% of inverter max

/* ============================== TYPES =============================== */

// Test state - one global instance, add "current_test" to Live Expressions
typedef struct {
    uint16_t pedal_1000;         // Accelerator used for the threshold (after BSPD), 0..1000
    bool apps_error;             // APPS error at the last update
    bool active;                 // True while the constant is applied
    uint16_t normal_drive_1000;  // Drive request from the normal pedal map, 0..1000
    uint16_t drive_1000;         // Drive request actually sent, 0..1000
} current_test_t;

extern current_test_t current_test;

/* ============================ FUNCTIONS ============================= */

/**
 * @brief Clear the test state. Call when entering ready-to-drive.
 */
void current_test_reset(void);

/**
 * @brief Replace the normal drive request with the constant above the threshold
 * @param normal_drive_1000 Drive request from the normal pedal map (after the torque ramp)
 * @param pedal_1000        Accelerator after PAU/BSPD limits, 0..1000 (apps_bspd_pau)
 * @param apps_error        APPS plausibility error
 * @return Drive request to send, 0..1000
 */
uint16_t current_test_apply(uint16_t normal_drive_1000, uint16_t pedal_1000, bool apps_error);

#endif  // CURRENT_TEST_H
