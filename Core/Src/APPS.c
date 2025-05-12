/**
 * @file APPS.c
 * @brief Accelerator Pedal Position Sensor (APPS) management module
 *
 * This module handles the dual redundant APPS sensors, performs error
 * detection, and calculates throttle position percentage.
 */

#include "APPS.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "main.h"

/* ---------------------- Constants and Types ---------------------- */

// ADC configuration
#define APPS_ADC_RESOLUTION 4095        // 12-bit ADC resolution
#define APPS_REFERENCE_VOLTAGE 3.3f     // ADC reference voltage
#define APPS_TIMEOUT_MS 100             // Error timeout in milliseconds
#define APPS_SHORT_THRESHOLD 20         // Threshold for detecting sensors shorted together
#define APPS_MIN_VALID_VALUE 100        // Minimum valid sensor reading
#define APPS_MAX_VALID_VALUE 4000       // Maximum valid sensor reading
#define APPS_CALIBRATION_TIME_MS 10000  // Time for calibration in ms

// Additional bounds
#define APPS_PERCENTAGE_MAX 100
#define APPS_PERCENTAGE_1000_MAX 999

// Throttle curve configuration
#define THROTTLE_CURVE_POINTS 11  // Number of points in each throttle curve

// Make our struct definition match the header file
APPS_ThrottleCurveType_t APPS_ActiveCurveType = THROTTLE_CURVE_STANDARD;

typedef enum {
    APPS_ERROR_NONE = 0,
    APPS_ERROR_TOLERANCE,
    APPS_ERROR_RANGE,
    APPS_ERROR_SHORT_CIRCUIT,
    APPS_ERROR_SHORTED_TOGETHER
} APPS_ErrorType;

/* ---------------------- Module Variables ---------------------- */

// Configuration parameters (combined voltage and bit values in one section)
static struct {
    // Configuration values
    float min_volts;             // Minimum voltage
    float max_volts;             // Maximum voltage
    float tolerance_volts;       // Voltage tolerance
    uint16_t min_bits;           // Minimum value in ADC bits
    uint16_t max_bits;           // Maximum value in ADC bits
    uint16_t tolerance_bits;     // Tolerance in ADC bits
    uint16_t delta;              // Offset between sensors
    uint16_t functional_region;  // Valid range in ADC bits

    // Current state
    uint16_t apps1_value;       // Current APPS1 value
    uint16_t apps2_value;       // Current APPS2 value
    uint16_t mean;              // Mean value
    int16_t percentage;         // Percentage (-100 to 100) - Changed from uint16_t to int16_t
    int16_t percentage_1000;    // Extended percentage (-999 to 999) - Changed from uint16_t to int16_t
    bool error;                 // Error flag
    uint32_t error_start_time;  // Time when error was first detected
} apps_state = {0};

// Throttle curves for different modes
// Standard curve (0-100%)
static int16_t throttle_curve_standard[THROTTLE_CURVE_POINTS] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};

// Regenerative braking curve (-100 to 100%)
// First half for regen braking (negative values), second half for acceleration
static int16_t throttle_curve_regen[THROTTLE_CURVE_POINTS] = {-100, -75, -50, -25, -10, 0, 25, 50, 75, 90, 100};

// Custom curves that can be configured by user
static int16_t throttle_curve_custom1[THROTTLE_CURVE_POINTS] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
static int16_t throttle_curve_custom2[THROTTLE_CURVE_POINTS] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};

// Pointer to the current active curve
static int16_t* active_throttle_curve = throttle_curve_standard;

/* ---------------------- Private Function Declarations ---------------------- */

static inline uint16_t convert_volts_to_bits(float volts);
static inline void calculate_functional_region(void);
static inline uint16_t calculate_mean(uint16_t apps1, uint16_t apps2);
static inline APPS_ErrorType check_apps_errors(uint16_t apps1, uint16_t apps2);
static bool check_error_timeout(uint16_t apps1, uint16_t apps2);
static int32_t map_value(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
static int16_t apply_throttle_curve(uint16_t input_percentage);

/* ---------------------- Utility Functions ---------------------- */

/**
 * @brief Maps a value from one range to another with overflow protection
 */
static int32_t map_value(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max) {
    // Prevent division by zero
    if (in_min == in_max) return out_min;

    // Clamp input to range
    if (x < in_min) x = in_min;
    if (x > in_max) x = in_max;

    // Use 64-bit intermediate calculation to prevent overflow
    int64_t numerator = (int64_t)(x - in_min) * (int64_t)(out_max - out_min);
    int32_t result = (int32_t)(numerator / (in_max - in_min) + out_min);

    // Ensure result is in output range
    if (result < out_min) return out_min;
    if (result > out_max) return out_max;

    return result;
}

/**
 * @brief Converts voltage to ADC bits
 */
static inline uint16_t convert_volts_to_bits(float volts) {
    return (uint16_t)((volts * APPS_ADC_RESOLUTION) / APPS_REFERENCE_VOLTAGE);
}

/**
 * @brief Calculates functional region (valid range) for APPS
 */
static inline void calculate_functional_region(void) {
    apps_state.functional_region = (apps_state.max_bits - apps_state.tolerance_bits) -
                                   (apps_state.min_bits + apps_state.tolerance_bits);
}

/* ---------------------- Sensor Value Processing ---------------------- */

/**
 * @brief Calculates mean value of both APPS sensors
 */
static inline uint16_t calculate_mean(uint16_t apps1, uint16_t apps2) {
    return (apps1 + apps2) / 2;
}

/* ---------------------- Error Detection ---------------------- */

/**
 * @brief Performs all error checks on APPS values and returns error type
 */
static inline APPS_ErrorType check_apps_errors(uint16_t apps1, uint16_t apps2) {
    // Check if values differ by more than 10%
    if (apps1 < (apps2 * 0.9) || apps1 > (apps2 * 1.1)) {
        return APPS_ERROR_TOLERANCE;
    }

    // Check if values are within tolerance range
    if ((apps1 < (apps_state.min_bits - apps_state.tolerance_bits)) ||
        (apps1 > (apps_state.max_bits + (2 * apps_state.tolerance_bits))) ||
        (apps2 < (apps_state.min_bits - apps_state.tolerance_bits)) ||
        (apps2 > (apps_state.max_bits + (2 * apps_state.tolerance_bits)))) {
        return APPS_ERROR_RANGE;
    }

    // Check for short circuit to ground or VCC
    if ((apps1 < APPS_MIN_VALID_VALUE) ||
        (apps2 < APPS_MIN_VALID_VALUE) ||
        (apps1 > APPS_MAX_VALID_VALUE) ||
        (apps2 > APPS_MAX_VALID_VALUE)) {
        return APPS_ERROR_SHORT_CIRCUIT;
    }

    // Check if sensors are shorted together
    if (abs((int)apps1 - ((int)apps2 + apps_state.delta)) < APPS_SHORT_THRESHOLD) {
        return APPS_ERROR_SHORTED_TOGETHER;
    }

    return APPS_ERROR_NONE;
}

/**
 * @brief Checks if error condition has persisted beyond timeout
 */
static bool check_error_timeout(uint16_t apps1, uint16_t apps2) {
    APPS_ErrorType error = check_apps_errors(apps1, apps2);
    uint32_t current_time = HAL_GetTick();

    if (error != APPS_ERROR_NONE) {
        if (!apps_state.error) {
            // First detection of error, start timer
            apps_state.error_start_time = current_time;
            apps_state.error = true;
        } else if ((current_time - apps_state.error_start_time) > APPS_TIMEOUT_MS) {
            // Error has persisted beyond timeout
            return true;
        }
    } else {
        // No error detected
        apps_state.error = false;
    }

    return false;
}

/* ---------------------- Public Interface Functions ---------------------- */

/**
 * @brief Initializes APPS module with calibration values
 *
 * @param min_volts Minimum voltage from APPS sensor
 * @param max_volts Maximum voltage from APPS sensor
 * @param tolerance_volts Tolerance voltage for measurements
 * @param apps_delta_value Delta value between APPS1 and APPS2
 */
void APPS_Init(float min_volts, float max_volts, float tolerance_volts, uint16_t apps_delta_value) {
    // Store configuration values
    apps_state.min_volts = min_volts;
    apps_state.max_volts = max_volts;
    apps_state.tolerance_volts = tolerance_volts;
    apps_state.min_bits = convert_volts_to_bits(min_volts);
    apps_state.max_bits = convert_volts_to_bits(max_volts);
    apps_state.tolerance_bits = convert_volts_to_bits(tolerance_volts);
    apps_state.delta = apps_delta_value;

    // Initialize other state
    apps_state.error = false;
    apps_state.apps1_value = 0;
    apps_state.apps2_value = 0;
    apps_state.mean = 0;
    apps_state.percentage = 0;
    apps_state.percentage_1000 = 0;

    calculate_functional_region();
}

/**
 * @brief Processes APPS values and returns throttle position and error status
 */
APPS_Result_t APPS_Function(uint16_t apps1, uint16_t apps2) {
    APPS_Result_t result = {0};

    // Update current values
    apps_state.apps1_value = apps1;
    apps_state.apps2_value = apps2 - apps_state.delta;  // Apply delta adjustment to APPS2

    // Check for errors
    if (check_error_timeout(apps_state.apps1_value, apps_state.apps2_value)) {
        // Error condition - zero throttle
        apps_state.percentage = 0;
        apps_state.percentage_1000 = 0;
        apps_state.mean = 0;
        result.error = true;
    } else {
        // No error - calculate throttle position using bit shift instead of division
        apps_state.mean = (apps_state.apps1_value + apps_state.apps2_value) >> 1;

        // Precalculate thresholds once to avoid repeated calculations
        uint16_t min_threshold = apps_state.min_bits + apps_state.tolerance_bits;
        uint16_t max_threshold = apps_state.max_bits - apps_state.tolerance_bits;

        // Determine throttle percentage based on pedal position
        if (apps_state.mean <= min_threshold) {
            // Below minimum threshold
            apps_state.percentage = 0;
            apps_state.percentage_1000 = 0;
        } else if (apps_state.mean >= max_threshold) {
            // Above maximum threshold
            apps_state.percentage = APPS_PERCENTAGE_MAX;
            apps_state.percentage_1000 = APPS_PERCENTAGE_1000_MAX;
        } else {
            // In the active range - inline the mapping calculation to avoid function call
            uint32_t numerator = (uint32_t)(apps_state.mean - min_threshold) * APPS_PERCENTAGE_MAX;
            uint16_t raw_percentage = numerator / (max_threshold - min_threshold);

            // Get throttle value from curve
            int16_t throttle_value = apply_throttle_curve(raw_percentage);

            // Handle both positive and negative values (for regenerative braking)
            apps_state.percentage = throttle_value;

            // Optimize percentage_1000 calculation by factoring out common code
            apps_state.percentage_1000 = (throttle_value * APPS_PERCENTAGE_1000_MAX) / APPS_PERCENTAGE_MAX;
        }

        result.error = false;
    }

    // Set the result values (including negative values for regen)
    result.percentage = apps_state.percentage;
    result.percentage_1000 = apps_state.percentage_1000;

    return result;
}

/**
 * @brief Prints current APPS values to serial port in JSON format
 */
void APPS_PrintValues(void) {
    // Get the active curve for display
    int16_t curve_values[THROTTLE_CURVE_POINTS];
    APPS_GetThrottleCurve(curve_values, APPS_ActiveCurveType);

    printf(
        "{"
        "\"APPS1\":%d,"
        "\"APPS2\":%d,"
        "\"APPS_Mean\":%d,"
        "\"APPS_Percentage\":%d,"
        "\"APPS_Percentage_mil\":%d,"
        "\"APPS_Error\":%d,"
        "\"APPS_MIN_bits\":%d,"
        "\"APPS_MAX_bits\":%d,"
        "\"APPS_Tolerance_bits\":%d,"
        "\"APPS_functional_region\":%d,"
        "\"Active_Curve_Type\":%d,"
        "\"Throttle_Curve\":[%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]"
        "}\n",
        apps_state.apps1_value, apps_state.apps2_value, apps_state.mean,
        apps_state.percentage, apps_state.percentage_1000,
        apps_state.error, apps_state.min_bits, apps_state.max_bits,
        apps_state.tolerance_bits, apps_state.functional_region,
        APPS_ActiveCurveType,
        curve_values[0], curve_values[1], curve_values[2], curve_values[3],
        curve_values[4], curve_values[5], curve_values[6], curve_values[7],
        curve_values[8], curve_values[9], curve_values[10]);
}

/**
 * @brief Sets the active throttle curve type
 *
 * @param curve_type The curve type to set active
 * @return bool true if successful, false otherwise
 */
bool APPS_SetActiveCurveType(APPS_ThrottleCurveType_t curve_type) {
    if (curve_type >= THROTTLE_CURVE_COUNT) {
        return false;
    }

    APPS_ActiveCurveType = curve_type;

    // Set the appropriate curve array
    switch (curve_type) {
        case THROTTLE_CURVE_STANDARD:
            active_throttle_curve = throttle_curve_standard;
            break;
        case THROTTLE_CURVE_REGEN:
            active_throttle_curve = throttle_curve_regen;
            break;
        case THROTTLE_CURVE_CUSTOM1:
            active_throttle_curve = throttle_curve_custom1;
            break;
        case THROTTLE_CURVE_CUSTOM2:
            active_throttle_curve = throttle_curve_custom2;
            break;
        default:
            active_throttle_curve = throttle_curve_standard;
            return false;
    }

    return true;
}

/**
 * @brief Gets the active throttle curve type
 *
 * @return APPS_ThrottleCurveType_t The active curve type
 */
APPS_ThrottleCurveType_t APPS_GetActiveCurveType(void) {
    return APPS_ActiveCurveType;
}

/**
 * @brief Updates a specified throttle curve with custom values
 *
 * @param new_curve Array of percentages for throttle response
 * @param points Number of points in the array (must match THROTTLE_CURVE_POINTS)
 * @param curve_type The curve type to update
 * @return bool true if update was successful, false otherwise
 */
bool APPS_UpdateThrottleCurve(const int16_t* new_curve, uint8_t points, APPS_ThrottleCurveType_t curve_type) {
    if (points != THROTTLE_CURVE_POINTS || new_curve == NULL || curve_type >= THROTTLE_CURVE_COUNT) {
        return false;
    }

    // Validate curve values (must be monotonically increasing)
    for (uint8_t i = 1; i < THROTTLE_CURVE_POINTS; i++) {
        if (new_curve[i] < new_curve[i - 1]) {
            return false;
        }

        // Check range based on curve type
        if (curve_type == THROTTLE_CURVE_REGEN) {
            // Regen curve can have values from -100 to 100
            if (new_curve[i] < -100 || new_curve[i] > 100) {
                return false;
            }
        } else {
            // Standard curves have values from 0 to 100
            if (new_curve[i] < 0 || new_curve[i] > 100) {
                return false;
            }
        }
    }

    // Get pointer to the target curve
    int16_t* target_curve;
    switch (curve_type) {
        case THROTTLE_CURVE_STANDARD:
            target_curve = throttle_curve_standard;
            break;
        case THROTTLE_CURVE_REGEN:
            target_curve = throttle_curve_regen;
            break;
        case THROTTLE_CURVE_CUSTOM1:
            target_curve = throttle_curve_custom1;
            break;
        case THROTTLE_CURVE_CUSTOM2:
            target_curve = throttle_curve_custom2;
            break;
        default:
            return false;
    }

    // Copy new curve values
    for (uint8_t i = 0; i < THROTTLE_CURVE_POINTS; i++) {
        target_curve[i] = new_curve[i];
    }

    return true;
}

/**
 * @brief Gets a specified throttle curve
 *
 * @param curve_buffer Buffer to receive curve values (must be THROTTLE_CURVE_POINTS size)
 * @param curve_type The curve type to retrieve
 * @return uint8_t Number of points copied, 0 if error
 */
uint8_t APPS_GetThrottleCurve(int16_t* curve_buffer, APPS_ThrottleCurveType_t curve_type) {
    if (curve_buffer == NULL || curve_type >= THROTTLE_CURVE_COUNT) {
        return 0;
    }

    // Get pointer to the requested curve
    int16_t* source_curve;
    switch (curve_type) {
        case THROTTLE_CURVE_STANDARD:
            source_curve = throttle_curve_standard;
            break;
        case THROTTLE_CURVE_REGEN:
            source_curve = throttle_curve_regen;
            break;
        case THROTTLE_CURVE_CUSTOM1:
            source_curve = throttle_curve_custom1;
            break;
        case THROTTLE_CURVE_CUSTOM2:
            source_curve = throttle_curve_custom2;
            break;
        default:
            return 0;
    }

    // Copy curve values
    for (uint8_t i = 0; i < THROTTLE_CURVE_POINTS; i++) {
        curve_buffer[i] = source_curve[i];
    }

    return THROTTLE_CURVE_POINTS;
}

/**
 * @brief Applies the active throttle curve to an input percentage
 *
 * @param input_percentage Input pedal position percentage (0-100)
 * @return int16_t Mapped throttle value after curve application (-100 to 100)
 */
static int16_t apply_throttle_curve(uint16_t input_percentage) {
    // Clamp input to valid range
    if (input_percentage > 100) {
        input_percentage = 100;
    }

    // Calculate indices and fractions for interpolation
    float index_float = input_percentage / (100.0f / (THROTTLE_CURVE_POINTS - 1));
    uint8_t index_lower = (uint8_t)index_float;
    uint8_t index_upper = index_lower + 1;
    float fraction = index_float - index_lower;

    // Handle edge case for 100% input
    if (index_upper >= THROTTLE_CURVE_POINTS) {
        return active_throttle_curve[THROTTLE_CURVE_POINTS - 1];
    }

    // Interpolate between lower and upper points
    return (int16_t)((1.0f - fraction) * active_throttle_curve[index_lower] +
                     fraction * active_throttle_curve[index_upper]);
}

/**
 * @brief Performs auto-calibration of APPS sensors
 *
 * Records min/max values for 10 seconds, then calculates
 * recommended calibration parameters.
 */
void AUTO_CALIBRATION(uint16_t apps1, uint16_t apps2) {
    static struct {
        uint16_t apps1_min;
        uint16_t apps1_max;
        uint16_t apps2_min;
        uint16_t apps2_max;
        uint32_t start_time;
        bool active;
    } cal = {0};

    // Start calibration if not already active
    if (!cal.active) {
        cal.active = true;
        cal.start_time = HAL_GetTick();
        cal.apps1_min = APPS_ADC_RESOLUTION;
        cal.apps1_max = 0;
        cal.apps2_min = APPS_ADC_RESOLUTION;
        cal.apps2_max = 0;
    }

    // Update min/max values
    cal.apps1_min = (apps1 < cal.apps1_min) ? apps1 : cal.apps1_min;
    cal.apps1_max = (apps1 > cal.apps1_max) ? apps1 : cal.apps1_max;
    cal.apps2_min = (apps2 < cal.apps2_min) ? apps2 : cal.apps2_min;
    cal.apps2_max = (apps2 > cal.apps2_max) ? apps2 : cal.apps2_max;

    // Print current calibration values
    printf("APPS1_MIN %d APPS1_MAX %d APPS2_MIN %d APPS2_MAX %d\r\n",
           cal.apps1_min, cal.apps1_max, cal.apps2_min, cal.apps2_max);

    // Stop calibration after time period
    if (HAL_GetTick() - cal.start_time > APPS_CALIBRATION_TIME_MS) {
        cal.active = false;

        // Calculate recommended parameters
        uint16_t calculated_delta = cal.apps1_min - cal.apps2_min;
        float min_volts = (float)cal.apps1_min * APPS_REFERENCE_VOLTAGE / APPS_ADC_RESOLUTION;
        float max_volts = (float)cal.apps1_max * APPS_REFERENCE_VOLTAGE / APPS_ADC_RESOLUTION;
        float tolerance_volts = (float)(cal.apps1_max - cal.apps1_min) * 0.05 *
                                APPS_REFERENCE_VOLTAGE / APPS_ADC_RESOLUTION;

        printf(
            "Calibration complete!\r\n"
            "Recommended values for APPS_Init:\r\n"
            "min_volts: %.2f, max_volts: %.2f, tolerance_volts: %.2f, delta: %d\r\n",
            min_volts, max_volts, tolerance_volts, calculated_delta);
    }
}