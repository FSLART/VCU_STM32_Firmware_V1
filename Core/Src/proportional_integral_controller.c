#include "proportional_integral_controller.h"

#include <math.h>    // For isnan
#include <stddef.h>  // For NULL
#include <stdint.h>  // For int8_t

/**
 * Initialize the Proportional-Integral controller with given parameters
 *
 * @param pc       Pointer to controller structure
 * @param Kp       Proportional gain - controls response to current error
 * @param Ki       Integral gain - controls response to accumulated error
 * @param setpoint Target temperature value the controller will maintain
 */
void PI_Init(ProportionalController *pc, float Kp, float Ki, float setpoint) {
    if (pc == NULL) return;  // Safety check for null pointer

    pc->Kp = Kp;
    pc->Ki = Ki;
    pc->setpoint = setpoint;
    pc->output_min = 0;         // Output range minimum (0-100 scale)
    pc->output_max = 100;       // Output range maximum (0-100 scale)
    pc->last_valid_output = 0;  // Initialize output memory for sensor failure
    pc->integral_sum = 0;       // Reset accumulated error to prevent initial kick
}

/**
 * Update the controller's target temperature
 *
 * @param pc        Pointer to controller structure
 * @param setpoint  New target temperature
 */
void PI_SetSetpoint(ProportionalController *pc, float setpoint) {
    if (pc == NULL) return;  // Safety check for null pointer
    pc->setpoint = setpoint;
    pc->integral_sum = 0;  // Reset integral sum to prevent control bump
}

/**
 * Update the proportional gain parameter
 *
 * @param pc  Pointer to controller structure
 * @param Kp  New proportional gain value (higher = stronger immediate response)
 */
void PI_SetKp(ProportionalController *pc, float Kp) {
    if (pc == NULL) return;  // Safety check for null pointer
    pc->Kp = Kp;
}

/**
 * Update the integral gain parameter
 *
 * @param pc  Pointer to controller structure
 * @param Ki  New integral gain value (higher = stronger response to steady-state error)
 */
void PI_SetKi(ProportionalController *pc, float Ki) {
    if (pc == NULL) return;  // Safety check for null pointer
    pc->Ki = Ki;
}

/**
 * Set the minimum and maximum limits for controller output
 *
 * @param pc   Pointer to controller structure
 * @param min  Minimum output value (typically 0)
 * @param max  Maximum output value (typically 100)
 */
void PI_SetOutputLimits(ProportionalController *pc, float min, float max) {
    if (pc == NULL || min > max) return;  // Safety checks
    pc->output_min = min;
    pc->output_max = max;
}

// Simple macro for clamping values between min and max boundaries
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/**
 * Calculate controller output based on current temperature readings
 *
 * This function implements a PI (Proportional-Integral) controller for cooling:
 * - Higher temperatures produce higher output (more cooling)
 * - Lower temperatures produce lower output (less cooling)
 *
 * @param pc     Pointer to controller structure
 * @param temp1  First temperature sensor reading
 * @param temp2  Second temperature sensor reading
 * @param temp3  Third temperature sensor reading
 * @return       Controller output value (0-100)
 */
uint8_t PI_Calculate(ProportionalController *pc, float temp1, float temp2, float temp3) {
    if (pc == NULL) return 0;  // Safety check for null pointer

    // Check for invalid sensor readings using bitwise OR for efficiency
    if (isnan(temp1) | isnan(temp2) | isnan(temp3)) {
        return pc->last_valid_output;
    }

    // Calculate average temperature from three sensors for reliability
    const float average_temp = (temp1 + temp2 + temp3) * 0.333333f;

    // Calculate error for cooling mode: positive error means cooling needed
    const float error = average_temp - pc->setpoint;

    // Accumulate error over time for the integral term
    pc->integral_sum += error;

    // Anti-windup protection: prevent integral term from growing too large
    float i_term = pc->Ki * pc->integral_sum;
    if (i_term > pc->output_max) {
        i_term = pc->output_max;
        pc->integral_sum = pc->output_max / pc->Ki;
    } else if (i_term < pc->output_min) {
        i_term = pc->output_min;
        pc->integral_sum = pc->output_min / pc->Ki;
    }

    // Calculate proportional term based on current error
    float p_term = pc->Kp * error;

    // Combine P and I terms to get raw controller output
    float raw_output = p_term + i_term;

    // Ensure output stays within defined limits
    float clamped_output = CLAMP(raw_output, pc->output_min, pc->output_max);

    // Convert float to integer with proper rounding for actuator control
    uint8_t output = (uint8_t)(clamped_output + 0.5f);

    // Store last valid output in case of future sensor failure
    pc->last_valid_output = output;
    return output;
}

/**
 * Convert ADC reading to temperature using Steinhart-Hart equation
 * For thermistor at top of voltage divider configuration (thermistor to VCC, resistor to GND)
 *
 * @param adc_value    Raw ADC reading from temperature sensor
 * @param adc_ref      ADC reference voltage in millivolts (e.g., 3300 for 3.3V)
 * @param adc_bits     ADC resolution in bits (e.g., 12 for 12-bit ADC)
 * @return             Temperature in degrees Celsius with one decimal precision
 */
float PI_ConvertADCToTemperature(uint16_t adc_value, uint16_t adc_ref, uint8_t adc_bits) {
    // Safety checks for valid ADC parameters
    if (adc_bits < 1 || adc_bits > 16) return -99.9f;  // Invalid ADC bit resolution
    const uint16_t maxADC = (1 << adc_bits) - 1;
    if (adc_value == 0 || adc_value > maxADC) return -99.9f;  // Invalid ADC reading

    // Constants (adjust based on your hardware)
    const float SERIESRESISTOR = 10000.0f;
    const float THERMISTORNOMINAL = 10000.0f;
    const float TEMPERATURENOMINAL = 25.0f;  // 25°C in Celsius
    const float BCOEFFICIENT = 3976.0f;

    // adc_value = (float)adc_value / 1.51515;

    // Calculate thermistor resistance with voltage divider formula
    float rThermistor = (float)(SERIESRESISTOR * adc_value) / (maxADC - adc_value);

    // Steinhart-Hart equation (B-parameter approximation)
    float steinhart = rThermistor / THERMISTORNOMINAL;
    steinhart = logf(steinhart) / BCOEFFICIENT;          // Natural log + B coefficient
    steinhart += 1.0f / (TEMPERATURENOMINAL + 273.15f);  // Add reciprocal of T0 in Kelvin
    steinhart = 1.0f / steinhart - 273.15f;              // Convert to Celsius

    return roundf(steinhart * 10.0f) / 10.0f;  // Round to 1 decimal place
}
