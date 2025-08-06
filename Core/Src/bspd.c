/**
 * @file bspd.c
 * @brief Simple Digital BSPD Implementation
 */

#include "bspd.h"

void bspd_init(bspd_state_t *state) {
    state->bspd_active = false;
    state->fault_start_time = 0;
    state->last_update_time = 0;
}

uint16_t bspd_process(bspd_state_t *state, float brake_pressure, uint16_t accel_input, uint32_t current_time_ms) {
    // Check if both brake and accelerator thresholds are exceeded
    bool brake_active = (brake_pressure >= BSPD_BRAKE_THRESHOLD);
    bool accel_active = (accel_input >= BSPD_ACCEL_THRESHOLD);
    bool fault_condition = brake_active && accel_active;

    // Handle timing delays
    if (!state->bspd_active) {
        // BSPD is currently inactive
        if (fault_condition) {
            // Fault detected, start or continue timing
            if (state->fault_start_time == 0) {
                state->fault_start_time = current_time_ms;
            } else if ((current_time_ms - state->fault_start_time) >= BSPD_ACTIVATION_DELAY) {
                // Activation delay elapsed, activate BSPD
                state->bspd_active = true;
            }
        } else {
            // No fault, reset timer
            state->fault_start_time = 0;
        }
    } else {
        // BSPD is currently active
        if (!fault_condition) {
            // Fault cleared, check deactivation delay
            if ((current_time_ms - state->fault_start_time) >=
                (BSPD_ACTIVATION_DELAY + BSPD_DEACTIVATION_DELAY)) {
                // Deactivation delay elapsed, deactivate BSPD
                state->bspd_active = false;
                state->fault_start_time = 0;
            }
        }
        // If fault persists, stay active
    }

    state->last_update_time = current_time_ms;

    // If BSPD is active, cut throttle to 0, otherwise pass through input
    return state->bspd_active ? 0 : accel_input;
}
