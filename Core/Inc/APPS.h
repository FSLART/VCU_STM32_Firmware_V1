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
// Normally the calibration comes from FLASH: with the VCU connected, the buttons
// "APPS repouso" / "APPS a fundo" in tools/throttle_map_editor.py save the raw readings
// (APPS_FlashCal_t below) and reset the VCU. APPS_Init() turns them into the 0%/100% points
// and the APPS2 scale. APPS_MIN_BITS, APPS_MAX_BITS, APPS2_OFFSET and APPS2_GAIN_X1000 are
// only used when there is no valid record in flash (apps_data.config.from_flash = 0).

#define APPS_REST_DEADZONE_BITS    9U  // Flash calibration: 0% point = measured rest + this
#define APPS_FULL_MARGIN_BITS      8U  // Flash calibration: 100% point = measured full pedal - this
#define APPS_CAL_MIN_TRAVEL_BITS 100U  // Flash record rejected if full - rest is smaller (either sensor)

#define APPS_MIN_BITS   1265U  // 0% throttle point  (rest measured 1113 + 9 bits dead zone)
#define APPS_MAX_BITS   1394U  // 100% throttle point (full pedal measured 1375, -8 bits margin for drift)
#define APPS_TOLERANCE    20U  // Max APPS1 vs APPS2 disagreement (~8% of the 249-bit range)

// Hysteresis on APPS1 (5..10 bits; 1 bit ~= 0.4% of pedal travel). The throttle output
// only changes once APPS1 moves more than this many bits away from the last accepted
// value. Within this many bits of the 0% point the output is forced to exactly 0, so a
// released pedal always reads 0 (with the values above: 0 up to APPS1 1127).
#define APPS_HYSTERESIS_BITS 5

// APPS2 -> APPS1 scale. APPS2 is not exactly 2x APPS1: a straight-line fit of measured
// points (APPS1 1185/1246/1260/1406 <-> APPS2 2343/2460/2476/2767) gives
//   APPS2 = 1.928 * APPS1 + 55   ->   apps2_adjusted = (APPS2 - 55) / 1.928
// so apps2_adjusted reads the same as APPS1 along the whole pedal travel.
#define APPS2_OFFSET       55U  // APPS2 reading where APPS1 would be 0
#define APPS2_GAIN_X1000 1928U  // APPS2 / APPS1 slope, x1000

// Record at the start of the reserved flash block (STM32F767VGTX_FLASH.ld, 0x080C0000).
// Written only by tools/vcu_live.py (write_apps_cal) - keep both layouts the same.
#define APPS_CAL_MAGIC 0x43505041U  // "APPC"
typedef struct {
    uint32_t magic;       // APPS_CAL_MAGIC
    uint16_t apps1_rest;  // APPS1 raw, pedal released
    uint16_t apps1_full;  // APPS1 raw, pedal fully pressed
    uint16_t apps2_rest;  // APPS2 raw, pedal released
    uint16_t apps2_full;  // APPS2 raw, pedal fully pressed
    uint32_t crc;         // CRC32 of the 12 bytes above (same as Python zlib.crc32)
} APPS_FlashCal_t;

extern const APPS_FlashCal_t apps_cal_flash;  // Defined by the linker script

/* ====================== FILTERING AND SAFETY ======================== */
// Used by main.c (moving average and CAN timeout check).

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
    uint16_t min_value;         // ADC value at 0% throttle
    uint16_t max_value;         // ADC value at 100% throttle
    uint16_t tolerance;         // Tolerance in ADC bits
    int16_t apps2_offset;       // APPS2 reading where APPS1 would be 0
    uint16_t apps2_gain_x1000;  // APPS2 / APPS1 slope, x1000
    bool from_flash;            // true = calibration from the flash record, false = APPS.h values
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
void APPS_Init(void);  // Loads the calibration: flash record if valid, else APPS.h
APPS_Result_t APPS_Process(uint16_t apps1, uint16_t apps2);

/**
 * @brief Convert an APPS1 reading to throttle percentage
 * @param apps1_bits APPS1 value (same units as apps_data.config.min_value / max_value)
 * @return 0..100 %, straight line between config.min_value (0%) and config.max_value (100%),
 *         clamped. No hysteresis and no error checks - it always converts.
 */
uint8_t APPS_ToThrottlePercent(uint16_t apps1_bits);
APPS_ErrorType_t APPS_GetErrorType(uint16_t apps1, uint16_t apps2);
void APPS_PrintStatus(void);
APPS_Config_t APPS_GetConfig(void);
bool APPS_SetConfig(APPS_Config_t config);

#endif /* APPS_H */
