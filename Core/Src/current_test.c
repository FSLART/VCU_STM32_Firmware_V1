/**
 * @file current_test.c
 * @brief Bench test: constant relative current above an accelerator threshold
 *
 * See current_test.h for how the test works and for all settings.
 */

#include "current_test.h"

#include <string.h>

// Compile-time sanity checks on the settings
#if (CURRENT_TEST_APPS_ON_1000 <= 0) || (CURRENT_TEST_APPS_ON_1000 > 1000)
#error "current_test.h: CURRENT_TEST_APPS_ON_1000 must be 1..1000"
#endif
#if (CURRENT_TEST_HYSTERESIS_1000 < 0) || (CURRENT_TEST_HYSTERESIS_1000 >= CURRENT_TEST_APPS_ON_1000)
#error "current_test.h: CURRENT_TEST_HYSTERESIS_1000 must be 0..(CURRENT_TEST_APPS_ON_1000 - 1)"
#endif
#if (CURRENT_TEST_REL_CURRENT_1000 < 0) || (CURRENT_TEST_REL_CURRENT_1000 > 1000)
#error "current_test.h: CURRENT_TEST_REL_CURRENT_1000 must be 0..1000"
#endif

current_test_t current_test;

void current_test_reset(void) {
    memset(&current_test, 0, sizeof(current_test));
}

uint16_t current_test_apply(uint16_t normal_drive_1000, uint16_t pedal_1000, bool apps_error) {
    current_test.pedal_1000 = pedal_1000;
    current_test.apps_error = apps_error;
    current_test.normal_drive_1000 = normal_drive_1000;

    if (apps_error) {
        current_test.active = false;  // Safety: never apply the constant on an APPS error
    } else if (pedal_1000 >= CURRENT_TEST_APPS_ON_1000) {
        current_test.active = true;
    } else if (pedal_1000 < CURRENT_TEST_APPS_ON_1000 - CURRENT_TEST_HYSTERESIS_1000) {
        current_test.active = false;
    }
    // Between (threshold - hysteresis) and threshold: keep the previous state

    current_test.drive_1000 = current_test.active ? CURRENT_TEST_REL_CURRENT_1000 : normal_drive_1000;
    return current_test.drive_1000;
}
