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

/* ---------------------- Module State ---------------------- */
/**
 * Combined APPS module state containing configuration, current readings,
 * and error information.
 */
static struct {
    // Configuration parameters
    uint16_t min_value;  // Minimum ADC value (0% throttle)
    uint16_t max_value;  // Maximum ADC value (100% throttle)
    uint16_t tolerance;  // Tolerance in ADC bits for error detection
    uint16_t delta;      // Offset between APPS1 and APPS2 sensors

    // Current sensor values and calculations
    uint16_t apps1_raw;         // Raw APPS1 value from ADC
    uint16_t apps2_raw;         // Raw APPS2 value from ADC
    uint16_t delta_real_time;   // Real-time calculated delta between sensors
    uint16_t apps2_adjusted;    // APPS2 with delta adjustment applied
    uint16_t mean;              // Mean of both sensor values (used for throttle calculation)
    uint16_t percentage;        // Throttle percentage (0-100)
    uint16_t percentage_1000;   // Higher resolution throttle percentage (0-999)
    uint16_t functional_range;  // Range between min and max thresholds

    // Error tracking
    bool error;                   // Error flag (true if error detected)
    APPS_ErrorType_t error_type;  // Current error type
    uint32_t error_start_time;    // Time when error was first detected
} apps_state = {0};

/* ---------------------- Calibration State ---------------------- */
/**
 * Separate state for APPS calibration process that stores min/max
 * values observed and calculates suggested calibration parameters.
 */
static struct {
    bool is_calibrating;           // Flag to indicate calibration in progress
    bool is_complete;              // Flag to indicate calibration complete
    uint32_t start_time;           // Start time of calibration
    uint16_t apps1_min;            // Min value of APPS1 observed during calibration
    uint16_t apps1_max;            // Max value of APPS1 observed during calibration
    uint16_t apps2_min;            // Min value of APPS2 observed during calibration
    uint16_t apps2_max;            // Max value of APPS2 observed during calibration
    int32_t delta_sum;             // Sum of deltas for average calculation
    uint16_t sample_count;         // Number of samples collected during calibration
    uint16_t suggested_min;        // Suggested min value for calibration
    uint16_t suggested_max;        // Suggested max value for calibration
    uint16_t suggested_delta;      // Suggested delta for calibration
    uint16_t suggested_tolerance;  // Suggested tolerance for calibration
} calib_state = {0};

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
    uint16_t min_threshold = apps_state.min_value + apps_state.tolerance;
    uint16_t max_threshold = apps_state.max_value - apps_state.tolerance;
    apps_state.functional_range = max_threshold - min_threshold;
}

/**
 * @brief Initializes APPS module with calibration values
 *
 * @param min_value Minimum ADC value (0% throttle position)
 * @param max_value Maximum ADC value (100% throttle position)
 * @param tolerance Tolerance in ADC bits for error detection
 * @param delta Offset between APPS1 and APPS2 sensors
 */
void APPS_Init(uint16_t min_value, uint16_t max_value, uint16_t tolerance, uint16_t delta) {
    // Store configuration
    apps_state.min_value = min_value;
    apps_state.max_value = max_value;
    apps_state.tolerance = tolerance;
    apps_state.delta = delta;

    // Calculate functional range based on configuration
    calculate_functional_range();

    // Reset state
    apps_state.error = false;
    apps_state.error_type = APPS_ERROR_NONE;
    apps_state.apps1_raw = 0;
    apps_state.apps2_raw = 0;
    apps_state.apps2_adjusted = 0;
    apps_state.mean = 0;
    apps_state.percentage = 0;
    apps_state.percentage_1000 = 0;
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
    apps_state.apps1_raw = apps1;
    apps_state.apps2_raw = apps2;

    // Apply delta adjustment to APPS2
    apps_state.apps2_adjusted = (apps2 > apps_state.delta) ? (apps2 - apps_state.delta) : 0;

    // Calculate real-time delta for debugging/calibration
    apps_state.delta_real_time = apps_state.apps2_adjusted - apps1;

    // Check for errors with timeout
    if (check_error_timeout(apps1, apps_state.apps2_adjusted)) {
        // Error condition - zero throttle
        apps_state.percentage = 0;
        apps_state.percentage_1000 = 0;
        apps_state.mean = 0;
        result.error = true;
        result.error_type = apps_state.error_type;
    } else {
        // No error - calculate throttle position
        // Use bit shift for division by 2 (faster than division)
        apps_state.mean = (apps1 + apps_state.apps2_adjusted) >> 1;

        // Precalculate thresholds once
        uint16_t min_threshold = apps_state.min_value + apps_state.tolerance;
        // uint16_t min_threshold = apps_state.min_value;
        uint16_t max_threshold = apps_state.max_value - apps_state.tolerance;
        // uint16_t max_threshold = apps_state.max_value;

        // Make sure functional range is up to date
        calculate_functional_range();

        // Determine throttle percentage based on position
        if (apps_state.mean <= min_threshold) {
            // Below minimum threshold
            apps_state.percentage = 0;
            apps_state.percentage_1000 = 0;
        } else if (apps_state.mean >= max_threshold) {
            // Above maximum threshold
            apps_state.percentage = APPS_PERCENTAGE_MAX;
            apps_state.percentage_1000 = APPS_PERCENTAGE_1000_MAX;
        } else {
            // In the active range - map the value
            uint32_t numerator = (uint32_t)(apps_state.mean - min_threshold) * APPS_PERCENTAGE_MAX;
            apps_state.percentage = numerator / apps_state.functional_range;

            // Calculate higher resolution percentage
            numerator = (uint32_t)(apps_state.mean - min_threshold) * APPS_PERCENTAGE_1000_MAX;
            apps_state.percentage_1000 = numerator / apps_state.functional_range;

            // Apply bounds checking for calculated percentages
            if (apps_state.percentage_1000 > APPS_PERCENTAGE_1000_MAX) {
                apps_state.percentage_1000 = APPS_PERCENTAGE_1000_MAX;
            } else if (apps_state.percentage_1000 < 0) {
                apps_state.percentage_1000 = 0;
            }

            if (apps_state.percentage > APPS_PERCENTAGE_MAX) {
                apps_state.percentage = APPS_PERCENTAGE_MAX;
            } else if (apps_state.percentage < 0) {
                apps_state.percentage = 0;
            }
        }

        result.error = false;
        result.error_type = APPS_ERROR_NONE;
    }

    // Set result values
    result.percentage = apps_state.percentage;
    result.percentage_1000 = apps_state.percentage_1000;
    result.raw_value = apps_state.mean;

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

    uint16_t max_difference = apps_state.functional_range / 10;
    if (abs((int)apps1 - (int)apps2_adjusted) > max_difference) {
        return APPS_ERROR_DISAGREEMENT;
    }

    // Check for values outside valid range
    /*
    if ((apps1 < (apps_state.min_value - apps_state.tolerance)) ||
        (apps1 > (apps_state.max_value + apps_state.tolerance)) ||
        (apps2_adjusted < (apps_state.min_value - apps_state.tolerance)) ||
        (apps2_adjusted > (apps_state.max_value + apps_state.tolerance))) {
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
    APPS_ErrorType_t current_error = check_apps_errors(apps1, apps_state.apps2_raw, apps2_adjusted);
    uint32_t current_time = HAL_GetTick();

    if (current_error != APPS_ERROR_NONE) {
        if (!apps_state.error) {
            // First detection of error
            apps_state.error_start_time = current_time;
            apps_state.error = true;
            apps_state.error_type = current_error;
        } else if ((current_time - apps_state.error_start_time) > APPS_TIMEOUT_MS) {
            // Error has persisted beyond timeout
            return true;
        }
    } else {
        // No error detected
        apps_state.error = false;
        apps_state.error_type = APPS_ERROR_NONE;
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
    printf(
        "{"
        "\"APPS1\":%d,"
        "\"APPS2\":%d,"
        "\"APPS2_Adjusted\":%d,"
        "\"APPS_Delta_Real_Time\":%d,"
        "\"APPS_Mean\":%d,"
        "\"APPS_Percentage\":%d,"
        "\"APPS_Percentage_mil\":%d,"
        "\"APPS_Error\":%d,"
        "\"APPS_ErrorType\":%d,"
        "\"APPS_Min\":%d,"
        "\"APPS_Max\":%d,"
        "\"APPS_Tolerance\":%d,"
        "\"APPS_Delta\":%d,"
        "\"APPS_Functional_Range\":%d"
        "}\n\r",
        apps_state.apps1_raw,
        apps_state.apps2_raw,
        apps_state.apps2_adjusted,
        apps_state.delta_real_time,
        apps_state.mean,
        apps_state.percentage,
        apps_state.percentage_1000,
        apps_state.error,
        apps_state.error_type,
        apps_state.min_value,
        apps_state.max_value,
        apps_state.tolerance,
        apps_state.delta,
        apps_state.functional_range);
}

/* ---------------------- Calibration Functions ---------------------- */

/**
 * @brief Starts APPS calibration
 *
 * Initializes the calibration state and begins collecting min/max
 * values from APPS sensors over a 10-second period.
 */
void APPS_StartCalibration(void) {
    calib_state.is_calibrating = true;
    calib_state.is_complete = false;
    calib_state.start_time = HAL_GetTick();
    calib_state.apps1_min = UINT16_MAX;
    calib_state.apps1_max = 0;
    calib_state.apps2_min = UINT16_MAX;
    calib_state.apps2_max = 0;
    calib_state.delta_sum = 0;
    calib_state.sample_count = 0;

    printf("APPS Calibration started. Press and release pedal several times over the next 10 seconds...\n");
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
    if (!calib_state.is_calibrating || calib_state.is_complete) {
        return calib_state.is_complete;
    }

    uint32_t current_time = HAL_GetTick();

    // Check if calibration period is over
    if (current_time - calib_state.start_time >= 10000) {  // 10 seconds
        // Finalize calibration
        // Calculate average delta
        int32_t avg_delta = (calib_state.sample_count > 0) ? (calib_state.delta_sum / calib_state.sample_count) : 0;

        // Ensure delta is positive (as expected by the code)
        calib_state.suggested_delta = (avg_delta > 0) ? (uint16_t)avg_delta : 0;

        // Calculate suggested values
        calib_state.suggested_min = calib_state.apps1_min;
        calib_state.suggested_max = calib_state.apps1_max;
        calib_state.suggested_tolerance = 50;  // Default suggested tolerance

        // Print results
        printf("\nAPPS Calibration Results:\n");
        printf("APPS1 - Min: %u, Max: %u (Range: %u)\n",
               calib_state.apps1_min, calib_state.apps1_max,
               calib_state.apps1_max - calib_state.apps1_min);
        printf("APPS2 - Min: %u, Max: %u (Range: %u)\n",
               calib_state.apps2_min, calib_state.apps2_max,
               calib_state.apps2_max - calib_state.apps2_min);
        printf("Average Delta between sensors: %d\n", avg_delta);

        // Print calibration recommendations
        printf("\nRecommended Calibration Values:\n");
        printf("min_value: %u\n", calib_state.suggested_min);
        printf("max_value: %u\n", calib_state.suggested_max);
        printf("delta: %u\n", calib_state.suggested_delta);
        printf("tolerance: %u (adjust based on sensor stability)\n", calib_state.suggested_tolerance);

        printf("\nUse these values with APPS_Init(min_value, max_value, tolerance, delta)\n");

        calib_state.is_calibrating = false;
        calib_state.is_complete = true;
        return true;
    }

    // Continue collecting data
    // Update min/max for APPS1
    if (apps1 < calib_state.apps1_min && apps1 > APPS_MIN_VALID_VALUE) calib_state.apps1_min = apps1;
    if (apps1 > calib_state.apps1_max && apps1 < APPS_MAX_VALID_VALUE) calib_state.apps1_max = apps1;

    // Update min/max for APPS2
    if (apps2 < calib_state.apps2_min && apps2 > APPS_MIN_VALID_VALUE) calib_state.apps2_min = apps2;
    if (apps2 > calib_state.apps2_max && apps2 < APPS_MAX_VALID_VALUE) calib_state.apps2_max = apps2;

    // Calculate instantaneous delta (APPS2 - APPS1)
    int32_t current_delta = (int32_t)apps2 - (int32_t)apps1;
    calib_state.delta_sum += current_delta;
    calib_state.sample_count++;

    return false;
}

/**
 * @brief Checks if APPS calibration is in progress
 *
 * @return true if calibration is in progress, false otherwise
 */
bool APPS_IsCalibrating(void) {
    return calib_state.is_calibrating;
}

/**
 * @brief Gets the suggested calibration values
 *
 * @param min_value Pointer to store suggested min value
 * @param max_value Pointer to store suggested max value
 * @param tolerance Pointer to store suggested tolerance
 * @param delta Pointer to store suggested delta
 * @return true if values are valid (calibration complete), false otherwise
 */
bool APPS_GetCalibrationValues(uint16_t* min_value, uint16_t* max_value,
                               uint16_t* tolerance, uint16_t* delta) {
    if (!calib_state.is_complete) {
        return false;
    }

    if (min_value) *min_value = calib_state.suggested_min;
    if (max_value) *max_value = calib_state.suggested_max;
    if (tolerance) *tolerance = calib_state.suggested_tolerance;
    if (delta) *delta = calib_state.suggested_delta;

    return true;
}
