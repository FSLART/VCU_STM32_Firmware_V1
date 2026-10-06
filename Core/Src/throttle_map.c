/**
 * @file throttle_map.c
 * @brief Interpolation in the throttle map (throttle_map_data.h)
 */

#include "throttle_map.h"

#include "throttle_map_data.h"

_Static_assert(THROTTLE_MAP_APPS_POINTS >= 2 && THROTTLE_MAP_APPS_POINTS <= 255, "throttle map: 2..255 accelerator points");
_Static_assert(THROTTLE_MAP_SPEED_POINTS >= 2 && THROTTLE_MAP_SPEED_POINTS <= 255, "throttle map: 2..255 speed points");

/**
 * @brief Find where x sits on an increasing axis
 * @param frac Fraction 0..1 from axis[index] to axis[index + 1]
 * @return Index of the lower point (clamped to the axis ends)
 */
static uint8_t axis_find(const uint16_t *axis, uint8_t points, float x, float *frac) {
    if (x <= axis[0]) {
        *frac = 0.0f;
        return 0;
    }
    if (x >= axis[points - 1]) {
        *frac = 1.0f;
        return points - 2;
    }
    uint8_t i = 0;
    while (x > axis[i + 1]) i++;
    uint16_t span = axis[i + 1] - axis[i];
    *frac = (span > 0) ? (x - axis[i]) / (float)span : 0.0f;  // Guard a hand-edited flat axis
    return i;
}

int16_t throttle_map_lookup(uint16_t pedal_1000, float speed_kmh) {
    float fa, fs;
    uint8_t a = axis_find(throttle_map_apps_axis, THROTTLE_MAP_APPS_POINTS, pedal_1000, &fa);
    uint8_t s = axis_find(throttle_map_speed_axis, THROTTLE_MAP_SPEED_POINTS, speed_kmh, &fs);

    // Interpolate along the accelerator on the two speed rows, then between the rows
    const int16_t *slow = throttle_map[s];
    const int16_t *fast = throttle_map[s + 1];
    float at_slow = slow[a] + (slow[a + 1] - slow[a]) * fa;
    float at_fast = fast[a] + (fast[a + 1] - fast[a]) * fa;
    float value = at_slow + (at_fast - at_slow) * fs;

    if (value > 1000.0f) value = 1000.0f;  // Guard hand-edited values
    if (value < -1000.0f) value = -1000.0f;
    return (int16_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}