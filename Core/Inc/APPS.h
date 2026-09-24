/*
 * File:   APPS.h
 * Author: pedro
 *
 * Created on 24 de Fevereiro de 2024, 20:29
 */

#ifndef APPS_H
#define APPS_H

#include <stdbool.h>
#include <stdint.h>

/* ========================= PEDAL CALIBRATION ======================== */
// All values in APPS1 bits. The throttle is taken from APPS1 only; APPS2 is only used
// for the disagreement (plausibility) check.

#define APPS_MIN_BITS   1238U  // 0% throttle point  (rest measured 1233 + 5 bits dead zone)
#define APPS_MAX_BITS   1381U  // 100% throttle point (full pedal measured 1385, -4 bits margin)
#define APPS_TOLERANCE    20U  // Max APPS1 vs APPS2 disagreement (~14% of the 143-bit range)

// Hysteresis on APPS1 (5..10 bits; 1 bit ~= 0.7% of pedal travel). The throttle output
// only changes once APPS1 moves more than this many bits away from the last accepted
// value. Within this many bits of the 0% point the output is forced to exactly 0, so a
// released pedal always reads 0 (with the values above: 0 up to APPS1 1243).
#define APPS_HYSTERESIS_BITS 5

// APPS2 -> APPS1 scale. APPS2 is not exactly 2x APPS1: a straight-line fit of measured
// points (APPS1 1185/1246/1260/1406 <-> APPS2 2343/2460/2476/2767) gives
//   APPS2 = 1.928 * APPS1 + 55   ->   apps2_adjusted = (APPS2 - 55) / 1.928
// so apps2_adjusted reads the same as APPS1 along the whole pedal travel.
#define APPS2_OFFSET       55U  // APPS2 reading where APPS1 would be 0
#define APPS2_GAIN_X1000 1928U  // APPS2 / APPS1 slope, x1000

/* ====================== FILTERING AND SAFETY ======================== */
// Used by main.c (moving average, CAN timeout check and calibration mode).

#define APPS_MA_WINDOW_SIZE     5  // Moving average window, in samples of the ~100 Hz APPS timer
#define MAX_APPS_TIMEOUT_MS   250  // No APPS CAN frame for this long -> throttle forced to 0
#define CALIBRATE_APPS          0  // 1 = run APPS_Calibrate() and apply the measured min/max/tolerance

// Error types
typedef enum {
    APPS_ERROR_NONE = 0,
    APPS_ERROR_DISAGREEMENT,     // Sensors disagree beyond tolerance
    APPS_ERROR_RANGE,            // Values outside valid range
    APPS_ERROR_SHORT_CIRCUIT,    // Sensor shorted to VCC or GND
    APPS_ERROR_SHORTED_TOGETHER  // Sensors shorted together
} APPS_ErrorType_t;

// Result structure
typedef struct {
    bool error;                   // Error status
    APPS_ErrorType_t error_type;  // Type of error detected
    uint16_t percentage;          // 0 to 100 value
    uint16_t percentage_1000;     // 0 to 1000 value (higher resolution)
    uint16_t raw_value;           // Raw calculated throttle value
} APPS_Result_t;

// Configuration structure
typedef struct {
    uint16_t min_value;  // ADC value at 0% throttle
    uint16_t max_value;  // ADC value at 100% throttle
    uint16_t tolerance;  // Tolerance in ADC bits
} APPS_Config_t;

// Internal runtime state (sensor readings, calculations, error tracking)
typedef struct {
    uint16_t apps1_raw;         // Raw APPS1 value from ADC
    uint16_t apps2_raw;         // Raw APPS2 value from ADC
    uint16_t apps2_adjusted;    // APPS2 proportionally adjusted
    uint16_t mean;              // Pedal value used for throttle = APPS1 (before hysteresis)
    uint16_t mean_held;         // Mean after hysteresis, used for the throttle percentage
    uint16_t disagreement;      // |APPS1 - APPS2 adjusted| right now (error above tolerance)
    uint16_t disagreement_max;  // Worst disagreement seen - set to 0 in Live Expressions to reset
    uint16_t disagreement_max_apps1;      // APPS1 raw at the worst disagreement
    uint16_t disagreement_max_apps2_raw;  // APPS2 raw at the worst disagreement
    uint16_t percentage;        // Throttle percentage (0-100)
    uint16_t percentage_1000;   // Higher resolution throttle percentage (0-999)
    uint16_t functional_range;  // Range between min and max thresholds
    bool error;                   // Error flag (true if error detected)
    APPS_ErrorType_t error_type;  // Current error type
    uint32_t error_start_time;    // Time when error was first detected
} APPS_State_t;

// Calibration state (populated during APPS_Calibrate())
typedef struct {
    bool is_calibrating;           // Flag to indicate calibration in progress
    bool is_complete;              // Flag to indicate calibration complete
    uint32_t start_time;           // Start time of calibration
    uint16_t apps1_min;            // Min value of APPS1 observed during calibration
    uint16_t apps1_max;            // Max value of APPS1 observed during calibration
    uint16_t apps2_min;            // Min value of APPS2 observed during calibration
    uint16_t apps2_max;            // Max value of APPS2 observed during calibration
    uint16_t sample_count;         // Number of samples collected during calibration
    uint16_t suggested_min;        // Suggested min value for calibration
    uint16_t suggested_max;        // Suggested max value for calibration
    uint16_t suggested_tolerance;  // Suggested tolerance for calibration
} APPS_CalibState_t;

// Top-level instance: add this to Live Expressions in STM32CubeIDE for full visibility
typedef struct {
    APPS_Config_t config;      // Calibration limits and tolerance
    APPS_State_t state;        // Runtime values: raws, mean, percentages, errors
    APPS_CalibState_t calib;   // Calibration process data
} APPS_Instance_t;

// Global instance — accessible from anywhere that includes APPS.h
extern APPS_Instance_t apps_data;

// Core functions
void APPS_Init(uint16_t min_value, uint16_t max_value, uint16_t tolerance);
APPS_Result_t APPS_Process(uint16_t apps1, uint16_t apps2);
APPS_ErrorType_t APPS_GetErrorType(uint16_t apps1, uint16_t apps2);
void APPS_PrintStatus(void);
APPS_Config_t APPS_GetConfig(void);
bool APPS_SetConfig(APPS_Config_t config);

/**
 * @brief Starts APPS calibration
 */
void APPS_StartCalibration(void);

/**
 * @brief Updates APPS calibration with new sensor values
 *
 * Call this function regularly from main loop with ADC values to perform calibration
 *
 * @param apps1 Current APPS1 sensor value
 * @param apps2 Current APPS2 sensor value
 * @return true if calibration is complete, false if still in progress
 */
bool APPS_Calibrate(uint16_t apps1, uint16_t apps2);

/**
 * @brief Checks if APPS calibration is in progress
 *
 * @return true if calibration is in progress, false otherwise
 */
bool APPS_IsCalibrating(void);

/**
 * @brief Gets the suggested calibration values
 *
 * @param min_value Pointer to store suggested min value
 * @param max_value Pointer to store suggested max value
 * @param tolerance Pointer to store suggested tolerance
 * @return true if values are valid (calibration complete), false otherwise
 */
bool APPS_GetCalibrationValues(uint16_t* min_value, uint16_t* max_value,
                               uint16_t* tolerance);

#endif /* APPS_H */
