/**
 * @file torque_ramp.c
 * @brief Speed-dependent ramp on the drive torque request (manual driving)
 *
 * See torque_ramp.h for how the ramp works and for all tuning values.
 */

#include "torque_ramp.h"

#include <string.h>

// Compile-time sanity checks on the tuning values
#if (TORQUE_RAMP_MS_STANDSTILL <= 0) || (TORQUE_RAMP_MS_MOVING <= 0)
#error "torque_ramp.h: ramp times must be above 0"
#endif
#if (TORQUE_RAMP_SPEED_MOVING_KMH <= 0) || (TORQUE_RAMP_SPEED_MOVING_KMH >= TORQUE_RAMP_SPEED_OFF_KMH)
#error "torque_ramp.h: must satisfy 0 < TORQUE_RAMP_SPEED_MOVING_KMH < TORQUE_RAMP_SPEED_OFF_KMH"
#endif

#define TORQUE_RAMP_MAX_DT_MS 50  // Longest time step the ramp will integrate

torque_ramp_t torque_ramp;

/**
 * @brief Ramp time (0 -> 100%) to use at the given speed
 * @return Ramp time in ms, or 0 when no ramp applies at this speed
 */
static uint16_t ramp_time_at_speed(float speed_kmh) {
    if (speed_kmh >= TORQUE_RAMP_SPEED_OFF_KMH) {
        return 0;  // Fast enough: no ramp
    }
    if (speed_kmh >= TORQUE_RAMP_SPEED_MOVING_KMH) {
        return TORQUE_RAMP_MS_MOVING;
    }
    if (speed_kmh <= 0.0f) {
        return TORQUE_RAMP_MS_STANDSTILL;
    }

    // Standstill .. MOVING speed: straight line from the standstill time to the moving time
    float fraction = speed_kmh / TORQUE_RAMP_SPEED_MOVING_KMH;
    float ramp_ms = TORQUE_RAMP_MS_STANDSTILL + (TORQUE_RAMP_MS_MOVING - TORQUE_RAMP_MS_STANDSTILL) * fraction;
    return (uint16_t)ramp_ms;
}

void torque_ramp_reset(void) {
    memset(&torque_ramp, 0, sizeof(torque_ramp));
    torque_ramp.first_update = true;
}

uint16_t torque_ramp_update(uint16_t target_1000, float speed_kmh, uint32_t now_ms) {
    // Time since the last update
    uint32_t dt_ms = torque_ramp.first_update ? 0 : (now_ms - torque_ramp.last_update_ms);
    if (dt_ms > TORQUE_RAMP_MAX_DT_MS) dt_ms = TORQUE_RAMP_MAX_DT_MS;
    torque_ramp.last_update_ms = now_ms;
    torque_ramp.first_update = false;

    torque_ramp.target_1000 = target_1000;
    torque_ramp.speed_kmh = speed_kmh;
    torque_ramp.ramp_ms = TORQUE_RAMP_ENABLE ? ramp_time_at_speed(speed_kmh) : 0;

    if (torque_ramp.ramp_ms == 0 || target_1000 <= torque_ramp.output_exact) {
        // No ramp at this speed, or torque is going down: follow the request immediately
        torque_ramp.output_exact = target_1000;
    } else {
        // Torque is going up: rise by at most 100% per ramp time
        float max_step = 1000.0f * (float)dt_ms / (float)torque_ramp.ramp_ms;
        torque_ramp.output_exact += max_step;
        if (torque_ramp.output_exact > target_1000) {
            torque_ramp.output_exact = target_1000;
        }
    }

    torque_ramp.output_1000 = (uint16_t)torque_ramp.output_exact;
    return torque_ramp.output_1000;
}
