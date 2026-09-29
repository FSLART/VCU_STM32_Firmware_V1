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
#include <stddef.h>
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
#if APPS_MA_WINDOW_SIZE < 1
#error "APPS.h: APPS_MA_WINDOW_SIZE must be at least 1"
#endif
#if APPS_CAL_MIN_TRAVEL_BITS <= APPS_REST_DEADZONE_BITS + APPS_FULL_MARGIN_BITS
#error "APPS.h: APPS_CAL_MIN_TRAVEL_BITS must be greater than APPS_REST_DEADZONE_BITS + APPS_FULL_MARGIN_BITS"
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

#define APPS_SINGLE_SENSOR_TEST 0     // Set to 1 to bypass APPS2 and error checks (for inverter testing)

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
 * @brief CRC32, same result as Python zlib.crc32 (tools/vcu_live.py writes the record with it)
 */
static uint32_t crc32(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    while (len--) {
        crc ^= *data++;
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }
    return ~crc;  // crc32("123456789") = 0xCBF43926
}

/**
 * @brief Applies the flash calibration record (APPS_FlashCal_t) if it is valid
 *
 * Rejected (returns false, config untouched) when the block is erased or corrupted
 * (magic/CRC), a reading is outside the valid ADC range, the pedal travel is shorter than
 * APPS_CAL_MIN_TRAVEL_BITS on either sensor, or the APPS2/APPS1 slope is more than 25 %
 * away from APPS2_GAIN_X1000 (one sensor wrong while calibrating).
 */
static bool load_flash_calibration(void) {
    const APPS_FlashCal_t* cal = &apps_cal_flash;
    if (cal->magic != APPS_CAL_MAGIC ||
        cal->crc != crc32((const uint8_t*)cal, offsetof(APPS_FlashCal_t, crc))) {
        return false;
    }
    if (cal->apps1_rest < APPS_MIN_VALID_VALUE || cal->apps2_rest < APPS_MIN_VALID_VALUE ||
        cal->apps1_full > APPS_MAX_VALID_VALUE || cal->apps2_full > APPS_MAX_VALID_VALUE ||
        cal->apps1_full < cal->apps1_rest + APPS_CAL_MIN_TRAVEL_BITS ||
        cal->apps2_full < cal->apps2_rest + APPS_CAL_MIN_TRAVEL_BITS) {
        return false;
    }
    uint32_t gain = (uint32_t)(cal->apps2_full - cal->apps2_rest) * 1000u / (cal->apps1_full - cal->apps1_rest);
    if (gain < APPS2_GAIN_X1000 * 3u / 4u || gain > APPS2_GAIN_X1000 * 5u / 4u) {
        return false;
    }
    apps_data.config.min_value = cal->apps1_rest + APPS_REST_DEADZONE_BITS;
    apps_data.config.max_value = cal->apps1_full - APPS_FULL_MARGIN_BITS;
    apps_data.config.apps2_gain_x1000 = (uint16_t)gain;
    apps_data.config.apps2_offset = (int16_t)((int32_t)cal->apps2_rest - (int32_t)(cal->apps1_rest * gain / 1000u));
    return true;
}

/**
 * @brief Initializes APPS module: calibration from the flash record if valid, else APPS.h
 */
void APPS_Init(void) {
    apps_data.config.min_value = APPS_MIN_BITS;
    apps_data.config.max_value = APPS_MAX_BITS;
    apps_data.config.tolerance = APPS_TOLERANCE;
    apps_data.config.apps2_offset = APPS2_OFFSET;
    apps_data.config.apps2_gain_x1000 = APPS2_GAIN_X1000;
    apps_data.config.from_flash = load_flash_calibration();

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
    uint16_t min = apps_data.config.min_value, max = apps_data.config.max_value;
    if (apps1_bits <= min) return 0;
    if (apps1_bits >= max) return 100;
    return (uint8_t)(((uint32_t)(apps1_bits - min) * 100u) / (max - min));
}

APPS_Result_t APPS_Process(uint16_t apps1, uint16_t apps2) {
    APPS_Result_t result = {0};

    // Store raw values
    apps_data.state.apps1_raw = apps1;
    apps_data.state.apps2_raw = apps2;

    // Convert APPS2 to APPS1 scale (config.apps2_offset / apps2_gain_x1000, see APPS_Init)
    int32_t apps2_above_offset = (int32_t)apps2 - apps_data.config.apps2_offset;
    apps_data.state.apps2_adjusted = (apps2_above_offset > 0)
        ? (uint16_t)((uint32_t)apps2_above_offset * 1000u / apps_data.config.apps2_gain_x1000) : 0u;

    // Disagreement tracking for calibration (Live Expressions)
    apps_data.state.disagreement = (uint16_t)abs((int)apps1 - (int)apps_data.state.apps2_adjusted);
    if (apps_data.state.disagreement > apps_data.state.disagreement_max) {
        apps_data.state.disagreement_max = apps_data.state.disagreement;
        apps_data.state.disagreement_max_apps1 = apps1;
        apps_data.state.disagreement_max_apps2_raw = apps2;
    }

#if APPS_SINGLE_SENSOR_TEST
    // TEST MODE: Ignore errors and APPS2, use APPS1 directly
    apps_data.state.mean = apps2;
    
    // Safety check: STILL detect short to VCC or GND!
    if (apps2 < APPS_MIN_VALID_VALUE || apps2 > APPS_MAX_VALID_VALUE) {
        apps_data.state.percentage = 0;
        apps_data.state.percentage_1000 = 0;
        apps_data.state.mean = 0;
        result.error = true;
        result.error_type = APPS_ERROR_SHORT_CIRCUIT;
    }
    
    if (!result.error) {
#else
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
        // APPS2 is still used above for the disagreement (plausibility) check.
        apps_data.state.mean = apps1;
#endif

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
