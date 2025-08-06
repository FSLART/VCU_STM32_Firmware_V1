/**
 * @file bspd.h
 * @brief Simple Digital BSPD for Formula Student
 */

#ifndef BSPD_H
#define BSPD_H

#include <stdbool.h>
#include <stdint.h>

// Thresholds - modify these as needed
#define BSPD_BRAKE_THRESHOLD 20.0f  // Brake pressure threshold (bar)
#define BSPD_ACCEL_THRESHOLD 0   // Accelerator threshold (0-1000)

// Timing delays (milliseconds)
#define BSPD_ACTIVATION_DELAY 100    // Delay before BSPD activates (ms)
#define BSPD_DEACTIVATION_DELAY 1  // Delay before BSPD deactivates (ms)

// BSPD state structure
typedef struct {
    bool bspd_active;
    uint32_t fault_start_time;
    uint32_t last_update_time;
} bspd_state_t;

// Initialize BSPD system
void bspd_init(bspd_state_t *state);

// BSPD function with timing - call this in your main loop
uint16_t bspd_process(bspd_state_t *state, float brake_pressure, uint16_t accel_input, uint32_t current_time_ms);

#endif  // BSPD_H
