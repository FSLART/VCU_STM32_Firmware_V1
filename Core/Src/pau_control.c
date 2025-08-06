// power adjust utility (pau) control implementation

#include "pau_control.h"

/**
 * @brief Calculate power-limited accelerator value with regen support
 *
 * This function reduces the accelerator pedal signal when power exceeds 60kW
 * to prevent the vehicle from exceeding 80kW maximum power. At 80kW, the throttle
 * is limited to a maximum of 20%. Negative values (regenerative braking) pass
 * through without any power limiting.
 *
 * @param apps_input Raw accelerator/brake pedal input (-1000 to 1000)
 * @param current_power_w Current power reading in watts from CAN
 * @return Limited accelerator value (-1000 to 1000)
 */
int16_t pau_limit_accelerator(int16_t apps_input, int32_t current_power_w) {
    // Validate input range and clamp if necessary
    if (apps_input > PAU_ACCELERATOR_MAX) {
        apps_input = PAU_ACCELERATOR_MAX;
    } else if (apps_input < PAU_ACCELERATOR_MIN) {
        apps_input = PAU_ACCELERATOR_MIN;
    }

    // Negative values (regen braking) pass through without limiting
    if (apps_input < 0) {
        return apps_input;
    }

    // Zero input returns zero
    if (apps_input == 0) {
        return 0;
    }

    // No limiting needed if power is below 60kW threshold
    if (current_power_w <= PAU_POWER_LIMIT_THRESHOLD_W) {
        return apps_input;
    }

    // Calculate power excess above 60kW threshold
    uint32_t power_above_threshold = current_power_w - PAU_POWER_LIMIT_THRESHOLD_W;
    uint32_t power_range = PAU_MAX_POWER_W - PAU_POWER_LIMIT_THRESHOLD_W;  // 20kW range (60-80kW)

    // Calculate target throttle percentage based on how close we are to 80kW limit
    // Linear reduction: 100% throttle at 60kW, 20% throttle at 80kW
    uint32_t progress_to_max = (power_above_threshold * 100) / power_range;

    // Cap progress at 100%
    if (progress_to_max > 100) {
        progress_to_max = 100;
    }

    // Calculate target throttle percentage
    uint32_t target_throttle_percent = 100 - ((100 - PAU_MAX_THROTTLE_AT_80KW_PERCENT) * progress_to_max) / 100;

    // Apply the throttle limitation
    int16_t limited_value = ((int32_t)apps_input * target_throttle_percent) / 100;

    return limited_value;
}
