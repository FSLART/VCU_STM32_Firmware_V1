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

/* ---------------------- Constants ---------------------- */
/**
 * Configuration constants for APPS module
 */
#define APPS_ADC_RESOLUTION 4095      // 12-bit ADC resolution (0-4095)
#define APPS_MIN_VALID_VALUE 50       // Minimum valid sensor reading (detect shorts to GND)
#define APPS_MAX_VALID_VALUE 4050     // Maximum valid sensor reading (detect shorts to VCC)
#define APPS_SHORT_THRESHOLD 10       // Threshold for detecting sensors shorted together
#define APPS_TIMEOUT_MS 100           // Error timeout in milliseconds
#define APPS_PERCENTAGE_MAX 100       // Maximum percentage value (0-100%)
#define APPS_PERCENTAGE_1000_MAX 999  // Maximum high-resolution percentage value (0-999)

#define APPS_SINGLE_SENSOR_TEST 0     // Set to 1 to bypass APPS2 and error checks (for inverter testing)

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
 * This range is used for mapping sensor values to throttle percentages
 * and factors in the tolerance values.
 */
static inline void calculate_functional_range(void) {
    uint16_t min_threshold = apps_data.config.min_value + apps_data.config.tolerance;
    uint16_t max_threshold = apps_data.config.max_value - apps_data.config.tolerance;
    apps_data.state.functional_range = max_threshold - min_threshold;
}

/**
 * @brief Initializes APPS module with calibration values
 *
 * @param min_value Minimum ADC value (0% throttle position)
 * @param max_value Maximum ADC value (100% throttle position)
 * @param tolerance Tolerance in ADC bits for error detection
 */
void APPS_Init(uint16_t min_value, uint16_t max_value, uint16_t tolerance) {
    // Store configuration
    apps_data.config.min_value = min_value;
    apps_data.config.max_value = max_value;
    apps_data.config.tolerance = tolerance;

    // Calculate functional range based on configuration
    calculate_functional_range();

    // Reset state
    apps_data.state.error = false;
    apps_data.state.error_type = APPS_ERROR_NONE;
    apps_data.state.apps1_raw = 0;
    apps_data.state.apps2_raw = 0;
    apps_data.state.apps2_adjusted = 0;
    apps_data.state.mean = 0;
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
APPS_Result_t APPS_Process(uint16_t apps1, uint16_t apps2) {
    APPS_Result_t result = {0};

    // Store raw values
    apps_data.state.apps1_raw = apps1;
    apps_data.state.apps2_raw = apps2;

    // Apply proportional adjustment to APPS2 (APPS2 is 2x APPS1)
    apps_data.state.apps2_adjusted = apps2 >> 1;

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
    // NORMAL MODE: Check for errors with timeout
    if (check_error_timeout(apps1, apps_data.state.apps2_adjusted)) {
        // Error condition - zero throttle
        apps_data.state.percentage = 0;
        apps_data.state.percentage_1000 = 0;
        apps_data.state.mean = 0;
        result.error = true;
        result.error_type = apps_data.state.error_type;
    } else {
        // No error - calculate throttle position
        // Use bit shift for division by 2 (faster than division)
        apps_data.state.mean = (apps1 + apps_data.state.apps2_adjusted) >> 1;
#endif

        // Precalculate thresholds once
        uint16_t min_threshold = apps_data.config.min_value + apps_data.config.tolerance;
        // uint16_t min_threshold = apps_data.config.min_value;
        uint16_t max_threshold = apps_data.config.max_value - apps_data.config.tolerance;
        // uint16_t max_threshold = apps_data.config.max_value;

        // Make sure functional range is up to date
        calculate_functional_range();

        // Determine throttle percentage based on position
        if (apps_data.state.mean <= min_threshold) {
            // Below minimum threshold
            apps_data.state.percentage = 0;
            apps_data.state.percentage_1000 = 0;
        } else if (apps_data.state.mean >= max_threshold) {
            // Above maximum threshold
            apps_data.state.percentage = APPS_PERCENTAGE_MAX;
            apps_data.state.percentage_1000 = APPS_PERCENTAGE_1000_MAX;
        } else {
            // In the active range - map the value
            uint32_t numerator = (uint32_t)(apps_data.state.mean - min_threshold) * APPS_PERCENTAGE_MAX;
            apps_data.state.percentage = numerator / apps_data.state.functional_range;

            // Calculate higher resolution percentage
            numerator = (uint32_t)(apps_data.state.mean - min_threshold) * APPS_PERCENTAGE_1000_MAX;
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

    uint16_t max_difference = apps_data.state.functional_range / 10;
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

/* ---------------------- Calibration Functions ---------------------- */

/**
 * @brief Starts APPS calibration
 *
 * Initializes the calibration state and begins collecting min/max
 * values from APPS sensors over a 10-second period.
 */
void APPS_StartCalibration(void) {
    apps_data.calib.is_calibrating = true;
    apps_data.calib.is_complete = false;
    apps_data.calib.start_time = HAL_GetTick();
    apps_data.calib.apps1_min = UINT16_MAX;
    apps_data.calib.apps1_max = 0;
    apps_data.calib.apps2_min = UINT16_MAX;
    apps_data.calib.apps2_max = 0;
    apps_data.calib.sample_count = 0;

    DBG_PRINTF("APPS Calibration started. Press and release pedal several times over the next 10 seconds...\n");
}

/**
 * @brief Updates APPS calibration with new sensor values
 *
 * Call this function regularly from main loop with ADC values to perform calibration.
 * Captures min/max values and calculates average delta between sensors.
 *
 * @param apps1 Current APPS1 sensor value
 * @param apps2 Current APPS2 sensor value
 * @return true if calibration is complete, false if still in progress
 */
bool APPS_Calibrate(uint16_t apps1, uint16_t apps2) {
    // If not calibrating or already complete, do nothing
    if (!apps_data.calib.is_calibrating || apps_data.calib.is_complete) {
        return apps_data.calib.is_complete;
    }

    uint32_t current_time = HAL_GetTick();

    // Check if calibration period is over
    if (current_time - apps_data.calib.start_time >= 10000) {  // 10 seconds
        // Calculate suggested values
        apps_data.calib.suggested_min = apps_data.calib.apps1_min;
        apps_data.calib.suggested_max = apps_data.calib.apps1_max;
        apps_data.calib.suggested_tolerance = 50;  // Default suggested tolerance

        // Print results
        DBG_PRINTF("\nAPPS Calibration Results:\n");
        DBG_PRINTF("APPS1 - Min: %u, Max: %u (Range: %u)\n",
               apps_data.calib.apps1_min, apps_data.calib.apps1_max,
               apps_data.calib.apps1_max - apps_data.calib.apps1_min);
        DBG_PRINTF("APPS2 - Min: %u, Max: %u (Range: %u)\n",
               apps_data.calib.apps2_min, apps_data.calib.apps2_max,
               apps_data.calib.apps2_max - apps_data.calib.apps2_min);

        // Print calibration recommendations
        DBG_PRINTF("\nRecommended Calibration Values:\n");
        DBG_PRINTF("min_value: %u\n", apps_data.calib.suggested_min);
        DBG_PRINTF("max_value: %u\n", apps_data.calib.suggested_max);
        DBG_PRINTF("tolerance: %u (adjust based on sensor stability)\n", apps_data.calib.suggested_tolerance);

        DBG_PRINTF("\nUse these values with APPS_Init(min_value, max_value, tolerance)\n");

        apps_data.calib.is_calibrating = false;
        apps_data.calib.is_complete = true;
        return true;
    }

    // Continue collecting data
    // Update min/max for APPS1
    if (apps1 < apps_data.calib.apps1_min && apps1 > APPS_MIN_VALID_VALUE) apps_data.calib.apps1_min = apps1;
    if (apps1 > apps_data.calib.apps1_max && apps1 < APPS_MAX_VALID_VALUE) apps_data.calib.apps1_max = apps1;

    // Update min/max for APPS2
    if (apps2 < apps_data.calib.apps2_min && apps2 > APPS_MIN_VALID_VALUE) apps_data.calib.apps2_min = apps2;
    if (apps2 > apps_data.calib.apps2_max && apps2 < APPS_MAX_VALID_VALUE) apps_data.calib.apps2_max = apps2;

    apps_data.calib.sample_count++;

    return false;
}

/**
 * @brief Checks if APPS calibration is in progress
 *
 * @return true if calibration is in progress, false otherwise
 */
bool APPS_IsCalibrating(void) {
    return apps_data.calib.is_calibrating;
}

/**
 * @brief Gets the suggested calibration values
 *
 * @param min_value Pointer to store suggested min value
 * @param max_value Pointer to store suggested max value
 * @param tolerance Pointer to store suggested tolerance
 * @return true if values are valid (calibration complete), false otherwise
 */
bool APPS_GetCalibrationValues(uint16_t* min_value, uint16_t* max_value,
                               uint16_t* tolerance) {
    if (!apps_data.calib.is_complete) {
        return false;
    }

    if (min_value) *min_value = apps_data.calib.suggested_min;
    if (max_value) *max_value = apps_data.calib.suggested_max;
    if (tolerance) *tolerance = apps_data.calib.suggested_tolerance;

    return true;
}
