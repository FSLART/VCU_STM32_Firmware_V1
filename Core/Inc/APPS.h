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
//
// APPS_MIN_BITS / APPS_MAX_BITS: button "Calibrar APPS" in tools/throttle_map_editor.py
// (VCU connected) measures the pedal and writes them here. Then rebuild and flash.
#define APPS_MIN_BITS   1235U  // 0% throttle point
#define APPS_MAX_BITS   1373U  // 100% throttle point
#define APPS_TOLERANCE    20U  // Max APPS1 vs APPS2 disagreement (10% rule with the values above: <= 13)

// Hysteresis on APPS1 (5..10 bits; 1 bit ~= 0.7% of pedal travel). The throttle output
// only changes once APPS1 moves more than this many bits away from the last accepted
// value. Within this many bits of the 0% point the output is forced to exactly 0, so a
// released pedal always reads 0 (with the values above: 0 up to APPS1 1345).
#define APPS_HYSTERESIS_BITS 5

// APPS2 -> APPS1 scale, measured 2026-09-29 (APPS1 1474/1599 <-> APPS2 2340/2756) - NOT
// re-measured with the APPS1 values above; unused while APPS_SINGLE_SENSOR = 1.
//   APPS2 = 3.328 * APPS1 - 2565   ->   apps2_adjusted = (APPS2 + 2565) / 3.328
// so apps2_adjusted reads the same as APPS1 along the whole pedal travel.
#define APPS2_OFFSET    -2565  // APPS2 reading where APPS1 would be 0 (line extended, can be negative)
#define APPS2_GAIN_X1000 3328U  // APPS2 / APPS1 slope, x1000

/* ====================== FILTERING AND SAFETY ======================== */
// Used by main.c (moving average and CAN timeout check).

// 1 = throttle from APPS1 only (CAN apps2_raw): APPS2 and the APPS1/APPS2 disagreement are
// ignored, only an open/shorted APPS1 cuts the throttle. NOT FS-legal (two sensors +
// plausibility check) - bench/inverter tests only. The build shows a #warning while it is 1.
#define APPS_SINGLE_SENSOR      1

#define APPS_MA_WINDOW_SIZE     5  // Moving average window, in samples of the ~100 Hz APPS timer
#define MAX_APPS_TIMEOUT_MS   250  // No APPS CAN frame for this long -> throttle forced to 0

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

// Top-level instance: add this to Live Expressions in STM32CubeIDE for full visibility
typedef struct {
    APPS_Config_t config;      // Calibration limits and tolerance
    APPS_State_t state;        // Runtime values: raws, mean, percentages, errors
} APPS_Instance_t;

// Global instance — accessible from anywhere that includes APPS.h
extern APPS_Instance_t apps_data;

// Core functions
void APPS_Init(void);  // Calibration from APPS.h (APPS_MIN_BITS, APPS_MAX_BITS, APPS_TOLERANCE)
APPS_Result_t APPS_Process(uint16_t apps1, uint16_t apps2);

/**
 * @brief Convert an APPS1 reading to throttle percentage
 * @param apps1_bits APPS1 value (same units as APPS_MIN_BITS / APPS_MAX_BITS)
 * @return 0..100 %, straight line between APPS_MIN_BITS (0%) and APPS_MAX_BITS (100%),
 *         clamped. No hysteresis and no error checks - it always converts.
 */
uint8_t APPS_ToThrottlePercent(uint16_t apps1_bits);
APPS_ErrorType_t APPS_GetErrorType(uint16_t apps1, uint16_t apps2);
void APPS_PrintStatus(void);
APPS_Config_t APPS_GetConfig(void);
bool APPS_SetConfig(APPS_Config_t config);

#endif /* APPS_H */
