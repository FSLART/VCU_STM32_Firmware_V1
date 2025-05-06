#ifndef PROPORTIONAL_INTEGRAL_CONTROLLER_H
#define PROPORTIONAL_INTEGRAL_CONTROLLER_H

#include <stdint.h>  // Add this to define uint8_t

typedef struct {
    float Kp;                 // Proportional gain
    float Ki;                 // Integral gain
    float setpoint;           // Target temperature
    float output_min;         // Minimum output value (0)
    float output_max;         // Maximum output value (100)
    float last_valid_output;  // Last valid output
    float integral_sum;       // Accumulated error for integral term
} ProportionalController;

// Initialize controller parameters
void PI_Init(ProportionalController *pc, float Kp, float Ki, float setpoint);

// Set temperature setpoint
void PI_SetSetpoint(ProportionalController *pc, float setpoint);

// Update proportional gain
void PI_SetKp(ProportionalController *pc, float Kp);

// Update integral gain
void PI_SetKi(ProportionalController *pc, float Ki);

// Set output limits
void PI_SetOutputLimits(ProportionalController *pc, float min, float max);

// Calculate controller output (0-100)
uint8_t PI_Calculate(ProportionalController *pc, float temp1, float temp2, float temp3);

float PI_ConvertADCToTemperature(uint16_t adc_value, uint16_t adc_ref, uint8_t adc_bits);

#endif