#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Linear temperature sensor configuration
// Temperature is computed as: T = (V - v_offset) / v_per_c
// Example for LM35: v_offset = 0.0f, v_per_c = 0.010f (10 mV/°C)
// Example for TMP36: v_offset = 0.500f, v_per_c = 0.010f
// Example for 5V sensor scaled to 3.3V ADC: set vref = 3.3f and correct divider in v_per_c
typedef struct {
    float vref;        // ADC reference voltage (V)
    uint16_t adc_max;  // ADC max code (12-bit -> 4095)
    float v_offset;    // Sensor voltage at 0°C (V)
    float v_per_c;     // Sensor voltage per °C (V/°C)
} temp_linear_cfg_t;

// NTC Beta model configuration (voltage divider)
// Assumes NTC to GND and series resistor to Vref.
// R_ntc = R_series * Vout / (Vref - Vout)
// 1/T = 1/T0 + (1/B) * ln(R_ntc / R0)
typedef struct {
    float vref;        // ADC reference voltage (V)
    uint16_t adc_max;  // ADC max code (12-bit -> 4095)
    float r_series;    // Series resistor (ohms)
    float r0;          // NTC resistance at T0 (ohms)
    float t0_c;        // T0 in °C (e.g., 25.0)
    float beta;        // Beta value (e.g., 3977)
} temp_ntc_beta_cfg_t;

float temp_adc_to_voltage(uint16_t raw, const temp_linear_cfg_t* cfg);
float temp_voltage_to_celsius(float voltage, const temp_linear_cfg_t* cfg);
float temp_adc_to_celsius(uint16_t raw, const temp_linear_cfg_t* cfg);

// Returns temperature in deci-°C (one decimal place), e.g. 225 -> 22.5°C
int16_t temp_adc_to_deci_c(uint16_t raw, const temp_linear_cfg_t* cfg);

// Convert an array of raw ADC samples to deci-°C values
void temp_adc_vector_to_deci_c(const uint16_t* raw, size_t len, int16_t* out_deci_c,
                               const temp_linear_cfg_t* cfg);

float temp_ntc_adc_to_celsius(uint16_t raw, const temp_ntc_beta_cfg_t* cfg);
int16_t temp_ntc_adc_to_deci_c(uint16_t raw, const temp_ntc_beta_cfg_t* cfg);
void temp_ntc_vector_to_deci_c(const uint16_t* raw, size_t len, int16_t* out_deci_c,
                               const temp_ntc_beta_cfg_t* cfg);

#ifdef __cplusplus
}
#endif

#endif
