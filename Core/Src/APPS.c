/**
 * @file APPS.c
 * @brief Accelerator Pedal Position Sensor (APPS) management module
 *
 * This module handles dual redundant APPS sensors, performs error
 * detection, and calculates throttle position percentage using
 * bit values directly from ADC (0-4095).
 *
 * The dual redundancy approach follows FSAE rules requirements for
 * throttle position sensing safety.
 */

/* ---------------------- Includes ---------------------- */
#include "APPS.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "main.h"

// Calibration sanity checks (values are also range-checked by tools/throttle_map_editor.py)
#if APPS_MAX_BITS <= APPS_MIN_BITS
#error "APPS.h: APPS_MAX_BITS must be greater than APPS_MIN_BITS (divides by the difference)"
#endif
#if APPS2_GAIN_X1000 == 0
#error "APPS.h: APPS2_GAIN_X1000 must not be 0 (divides by it)"
#endif
#if APPS_SINGLE_SENSOR
#warning "APPS.h: APPS_SINGLE_SENSOR = 1 - throttle from APPS1 only, APPS1/APPS2 plausibility check OFF (not for competition)"
#endif
#if APPS_MA_WINDOW_SIZE < 1
#error "APPS.h: APPS_MA_WINDOW_SIZE must be at least 1"
#endif

/* ---------------------- Constants ---------------------- */
/**
 * Configuration constants for APPS module
 */
#define APPS_ADC_RESOLUTION 4095      // 12-bit ADC resolution (0-4095)
#define APPS_MIN_VALID_VALUE 50       // Minimum valid sensor reading (detect shorts to GND)
#define APPS_MAX_VALID_VALUE 4050     // Maximum valid sensor reading (detect shorts to VCC)
#define APPS_SHORT_THRESHOLD 10       // Threshold for detecting sensors shorted together
#define APPS_TIMEOUT_MS 600           // Error timeout in milliseconds
#define APPS_PERCENTAGE_MAX 100       // Maximum percentage value (0-100%)
#define APPS_PERCENTAGE_1000_MAX 999  // Maximum high-resolution percentage value (0-999)


// Pedal calibration (0%/100% points, tolerance, hysteresis, APPS2 scale) is in APPS.h

/* ---------------------- Global Debug Instance ---------------------- */
/**
 * Single global instance grouping configuration, runtime state, and
 * calibration data. Add 'apps_data' to Live Expressions in STM32CubeIDE
 * to inspect all APPS internals in one place.
 */
APPS_Instance_t apps_data = {0};

/* ---------------------- Private Function Prototypes ---------------------- */
/**
 * Internal functions not exposed in the header
 */
static APPS_ErrorType_t check_apps_errors(uint16_t apps1, uint16_t apps2_raw, uint16_t apps2_adjusted);
static bool check_error_timeout(uint16_t apps1, uint16_t apps2_adjusted);
static inline void calculate_functional_range(void);

/* ---------------------- Core Functions ---------------------- */

/**
 * @brief Calculates the effective functional range of the sensors
 *
 * This range is used for mapping sensor values to throttle percentages.
 * Tolerance is NOT factored in here - it's only a plausibility margin
 * for the APPS1/APPS2 disagreement check, not a deadzone on the pedal
 * travel itself. min_value/max_value are the real 0%/100% points.
 */
static inline void calculate_functional_range(void) {
    apps_data.state.functional_range = apps_data.config.max_value - apps_data.config.min_value;
}

/**
 * @brief Initializes APPS module with the calibration in APPS.h
 */
void APPS_Init(void) {
    apps_data.config.min_value = APPS_MIN_BITS;
    apps_data.config.max_value = APPS_MAX_BITS;
    apps_data.config.tolerance = APPS_TOLERANCE;

    // Calculate functional range based on configuration
    calculate_functional_range();

    // Reset state
    apps_data.state.error = false;
    apps_data.state.error_type = APPS_ERROR_NONE;
    apps_data.state.apps1_raw = 0;
    apps_data.state.apps2_raw = 0;
    apps_data.state.apps2_adjusted = 0;
    apps_data.state.mean = 0;
    apps_data.state.mean_held = apps_data.config.min_value;
    apps_data.state.percentage = 0;
    apps_data.state.percentage_1000 = 0;
}

/**
 * @brief Processes APPS sensor values and returns throttle position
 *
 * This main processing function handles:
 * 1. Storing raw sensor values
 * 2. Applying sensor adjustments
 * 3. Checking for errors
 * 4. Calculating throttle position
 *
 * @param apps1 Raw ADC value from APPS1 sensor (0-4095)
 * @param apps2 Raw ADC value from APPS2 sensor (0-4095)
 * @return APPS_Result_t Structure with throttle position and error status
 */
uint8_t APPS_ToThrottlePercent(uint16_t apps1_bits) {
    if (apps1_bits <= APPS_MIN_BITS) return 0;
    if (apps1_bits >= APPS_MAX_BITS) return 100;
    return (uint8_t)(((uint32_t)(apps1_bits - APPS_MIN_BITS) * 100u) / (APPS_MAX_BITS - APPS_MIN_BITS));
}

APPS_Result_t APPS_Process(uint16_t apps1, uint16_t apps2) {
    APPS_Result_t result = {0};

    // Store raw values
    apps_data.state.apps1_raw = apps1;
    apps_data.state.apps2_raw = apps2;

    // Convert APPS2 to APPS1 scale (see APPS2_OFFSET / APPS2_GAIN_X1000, offset may be negative)
    int32_t apps2_above_offset = (int32_t)apps2 - (int32_t)APPS2_OFFSET;
    apps_data.state.apps2_adjusted = (apps2_above_offset > 0)
        ? (uint16_t)((uint32_t)apps2_above_offset * 1000u / APPS2_GAIN_X1000) : 0u;

    // Disagreement tracking for calibration (Live Expressions)
    apps_data.state.disagreement = (uint16_t)abs((int)apps1 - (int)apps_data.state.apps2_adjusted);
    if (apps_data.state.disagreement > apps_data.state.disagreement_max) {
        apps_data.state.disagreement_max = apps_data.state.disagreement;
        apps_data.state.disagreement_max_apps1 = apps1;
        apps_data.state.disagreement_max_apps2_raw = apps2;
    }

    	// Precalculate thresholds once - the real calibrated 0%/100% points.
    	// Tolerance is deliberately NOT applied here (see calculate_functional_range).
		uint16_t min_threshold = apps_data.config.min_value;
		uint16_t max_threshold = apps_data.config.max_value;

		// Make sure functional range is up to date
		calculate_functional_range();

    	// NORMAL MODE: Check for errors with timeout
    if (check_error_timeout(apps1, apps_data.state.apps2_adjusted)) {
        // Error condition - zero throttle
        apps_data.state.percentage = 0;
        apps_data.state.percentage_1000 = 0;
        apps_data.state.mean = 0;
        apps_data.state.mean_held = min_threshold;  // Restart hysteresis from 0% after the error
        result.error = true;
        result.error_type = apps_data.state.error_type;
    } else {
        // No error - throttle position comes from APPS1 only.
        // APPS2 is still used for the disagreement (plausibility) check, unless APPS_SINGLE_SENSOR.
        apps_data.state.mean = apps1;

        // Hysteresis: hold the pedal value until it moves more than APPS_HYSTERESIS_BITS.
        // Near 0% force exactly 0; at 100% follow immediately so full pedal is always reached.
        int32_t pedal_change = (int32_t)apps_data.state.mean - (int32_t)apps_data.state.mean_held;
        if (apps_data.state.mean <= min_threshold + APPS_HYSTERESIS_BITS) {
            apps_data.state.mean_held = min_threshold;
        } else if (apps_data.state.mean >= max_threshold) {
            apps_data.state.mean_held = max_threshold;
        } else if (abs(pedal_change) > APPS_HYSTERESIS_BITS) {
            apps_data.state.mean_held = apps_data.state.mean;
        }

        // Determine throttle percentage based on position (after hysteresis)
        if (apps_data.state.mean_held <= min_threshold) {
            // Below minimum threshold
            apps_data.state.percentage = 0;
            apps_data.state.percentage_1000 = 0;
        } else if (apps_data.state.mean_held >= max_threshold) {
            // Above maximum threshold
            apps_data.state.percentage = APPS_PERCENTAGE_MAX;
            apps_data.state.percentage_1000 = APPS_PERCENTAGE_1000_MAX;
        } else {
            // In the active range - map the value
            uint32_t numerator = (uint32_t)(apps_data.state.mean_held - min_threshold) * APPS_PERCENTAGE_MAX;
            apps_data.state.percentage = numerator / apps_data.state.functional_range;

            // Calculate higher resolution percentage
            numerator = (uint32_t)(apps_data.state.mean_held - min_threshold) * APPS_PERCENTAGE_1000_MAX;
            apps_data.state.percentage_1000 = numerator / apps_data.state.functional_range;

            // Apply bounds checking for calculated percentages
            if (apps_data.state.percentage_1000 > APPS_PERCENTAGE_1000_MAX) {
                apps_data.state.percentage_1000 = APPS_PERCENTAGE_1000_MAX;
            } else if (apps_data.state.percentage_1000 < 0) {
                apps_data.state.percentage_1000 = 0;
            }



            if (apps_data.state.percentage > APPS_PERCENTAGE_MAX) {
                apps_data.state.percentage = APPS_PERCENTAGE_MAX;
            } else if (apps_data.state.percentage < 0) {
                apps_data.state.percentage = 0;
            }
        }

        result.error = false;
        result.error_type = APPS_ERROR_NONE;
    }

    // Set result values
    result.percentage = apps_data.state.percentage;
    result.percentage_1000 = apps_data.state.percentage_1000;
    result.raw_value = apps_data.state.mean;

    return result;
}

/* ---------------------- Error Handling Functions ---------------------- */

/**
 * @brief Checks for errors in APPS sensor values
 *
 * Detects various error conditions:
 * - Short circuit to ground or VCC
 * - Sensors shorted together
 * - (Commented out) Value disagreement > 10%
 * - (Commented out) Values outside valid range
 *
 * @param apps1 APPS1 sensor value
 * @param apps2_raw Raw APPS2 sensor value (without delta adjustment)
 * @param apps2_adjusted APPS2 sensor value with delta adjustment
 * @return APPS_ErrorType_t Error type detected, or APPS_ERROR_NONE
 */
static APPS_ErrorType_t check_apps_errors(uint16_t apps1, uint16_t apps2_raw, uint16_t apps2_adjusted) {
#if APPS_SINGLE_SENSOR
    // APPS2 ignored (APPS.h): only an open or shorted APPS1 is an error
    (void)apps2_raw;
    (void)apps2_adjusted;
    return (apps1 < APPS_MIN_VALID_VALUE || apps1 > APPS_MAX_VALID_VALUE) ? APPS_ERROR_SHORT_CIRCUIT : APPS_ERROR_NONE;
#endif
    // Check if values differ by more than 10%

    uint16_t max_difference = apps_data.config.tolerance;
    if (abs((int)apps1 - (int)apps2_adjusted) > max_difference) {
        return APPS_ERROR_DISAGREEMENT;
    }

    // Check for values outside valid range
    /*
    if ((apps1 < (apps_data.config.min_value - apps_data.config.tolerance)) ||
        (apps1 > (apps_data.config.max_value + apps_data.config.tolerance)) ||
        (apps2_adjusted < (apps_data.config.min_value - apps_data.config.tolerance)) ||
        (apps2_adjusted > (apps_data.config.max_value + apps_data.config.tolerance))) {
        return APPS_ERROR_RANGE;
    }
    */

    // Check for short circuit to ground or VCC - use raw value for hardware issues
    if ((apps1 < APPS_MIN_VALID_VALUE) ||
        (apps2_raw < APPS_MIN_VALID_VALUE) ||
        (apps1 > APPS_MAX_VALID_VALUE) ||
        (apps2_raw > APPS_MAX_VALID_VALUE)) {
        return APPS_ERROR_SHORT_CIRCUIT;
    }

    // Check if sensors are shorted together - use raw value for hardware issues
    if (abs((int)apps1 - (int)apps2_raw) < APPS_SHORT_THRESHOLD) {
        return APPS_ERROR_SHORTED_TOGETHER;
    }

    return APPS_ERROR_NONE;
}

/**
 * @brief Checks if error condition has persisted beyond timeout
 *
 * Implements debouncing for error detection to avoid false triggering
 * on transient conditions. An error must persist for APPS_TIMEOUT_MS
 * before being considered valid.
 *
 * @param apps1 APPS1 sensor value
 * @param apps2_adjusted APPS2 sensor value with delta adjustment
 * @return bool True if error has timed out, false otherwise
 */
static bool check_error_timeout(uint16_t apps1, uint16_t apps2_adjusted) {
    APPS_ErrorType_t current_error = check_apps_errors(apps1, apps_data.state.apps2_raw, apps2_adjusted);
    uint32_t current_time = HAL_GetTick();

    if (current_error != APPS_ERROR_NONE) {
        if (!apps_data.state.error) {
            // First detection of error
            apps_data.state.error_start_time = current_time;
            apps_data.state.error = true;
            apps_data.state.error_type = current_error;
        } else if ((current_time - apps_data.state.error_start_time) > APPS_TIMEOUT_MS) {
            // Error has persisted beyond timeout
            return true;
        }
    } else {
        // No error detected
        apps_data.state.error = false;
        apps_data.state.error_type = APPS_ERROR_NONE;
    }

    return false;
}

/* ---------------------- Debugging Functions ---------------------- */

/**
 * @brief Prints current APPS status for debugging
 *
 * Outputs a JSON-formatted string containing all relevant APPS state
 * information for debugging and monitoring purposes.
 */
void APPS_PrintStatus(void) {
    DBG_PRINTF(
        "{"
        "\"APPS1\":%d,"
        "\"APPS2\":%d,"
        "\"APPS2_Adjusted\":%d,"
        "\"APPS_Mean\":%d,"
        "\"APPS_Percentage\":%d,"
        "\"APPS_Percentage_mil\":%d,"
        "\"APPS_Error\":%d,"
        "\"APPS_ErrorType\":%d,"
        "\"APPS_Min\":%d,"
        "\"APPS_Max\":%d,"
        "\"APPS_Tolerance\":%d,"
        "\"APPS_Functional_Range\":%d"
        "}\n\r",
        apps_data.state.apps1_raw,
        apps_data.state.apps2_raw,
        apps_data.state.apps2_adjusted,
        apps_data.state.mean,
        apps_data.state.percentage,
        apps_data.state.percentage_1000,
        apps_data.state.error,
        apps_data.state.error_type,
        apps_data.config.min_value,
        apps_data.config.max_value,
        apps_data.config.tolerance,
        apps_data.state.functional_range);
}
