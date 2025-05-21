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
    uint16_t delta;      // Offset between APPS1 and APPS2
} APPS_Config_t;

// Core functions
void APPS_Init(uint16_t min_value, uint16_t max_value, uint16_t tolerance, uint16_t delta);
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
 * @param delta Pointer to store suggested delta
 * @return true if values are valid (calibration complete), false otherwise
 */
bool APPS_GetCalibrationValues(uint16_t* min_value, uint16_t* max_value,
                               uint16_t* tolerance, uint16_t* delta);

#endif /* APPS_H */
