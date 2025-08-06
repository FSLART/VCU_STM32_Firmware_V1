// power adjust utility (pau) control header file

#ifndef PAU_CONTROL_H
#define PAU_CONTROL_H

#include <stdint.h>

// Constants
#define PAU_POWER_LIMIT_THRESHOLD_W 65000    // Start power limiting above 60kW
#define PAU_MAX_POWER_W 70000                // Maximum allowed power in watts (80 kW)
#define PAU_ACCELERATOR_MIN -1000            // Minimum accelerator value (full regen)
#define PAU_ACCELERATOR_MAX 1000             // Maximum accelerator value (full throttle)
#define PAU_MAX_THROTTLE_AT_80KW_PERCENT 20  // Maximum throttle percentage when at 80kW

/**
 * @brief Calculate power-limited accelerator value with regen support
 * @param apps_input Raw accelerator/brake pedal input (-1000 to 1000)
 *                   Negative values = regenerative braking
 *                   Positive values = acceleration/throttle
 * @param current_power_w Current power reading in watts from CAN
 * @return Limited accelerator value (-1000 to 1000)
 *         Negative values pass through without limiting (regen braking)
 *         Positive values are power-limited above 60kW
 */
int16_t pau_limit_accelerator(int16_t apps_input, int32_t current_power_w);

#endif  // PAU_CONTROL_H
