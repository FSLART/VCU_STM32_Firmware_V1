#include "temperature.h"

#include <math.h>

float temp_adc_to_voltage(uint16_t raw, const temp_linear_cfg_t* cfg) {
    if (cfg == NULL || cfg->adc_max == 0) {
        return 0.0f;
    }

    if (raw > cfg->adc_max) {
        raw = cfg->adc_max;
    }

    return ((float)raw * cfg->vref) / (float)cfg->adc_max;
}

float temp_voltage_to_celsius(float voltage, const temp_linear_cfg_t* cfg) {
    if (cfg == NULL || cfg->v_per_c == 0.0f) {
        return 0.0f;
    }

    return (voltage - cfg->v_offset) / cfg->v_per_c;
}

float temp_adc_to_celsius(uint16_t raw, const temp_linear_cfg_t* cfg) {
    float voltage = temp_adc_to_voltage(raw, cfg);
    return temp_voltage_to_celsius(voltage, cfg);
}

int16_t temp_adc_to_deci_c(uint16_t raw, const temp_linear_cfg_t* cfg) {
    float temp_c = temp_adc_to_celsius(raw, cfg);
    return (int16_t)lroundf(temp_c * 10.0f);
}

void temp_adc_vector_to_deci_c(const uint16_t* raw, size_t len, int16_t* out_deci_c,
                               const temp_linear_cfg_t* cfg) {
    if (raw == NULL || out_deci_c == NULL || cfg == NULL) {
        return;
    }

    for (size_t i = 0; i < len; i++) {
        out_deci_c[i] = temp_adc_to_deci_c(raw[i], cfg);
    }
}

static float ntc_adc_to_resistance(uint16_t raw, const temp_ntc_beta_cfg_t* cfg) {
    if (cfg == NULL || cfg->adc_max == 0 || cfg->vref <= 0.0f || cfg->r_series <= 0.0f) {
        return 0.0f;
    }

    if (raw >= cfg->adc_max) {
        raw = cfg->adc_max - 1;
    }

    float vout = ((float)raw * cfg->vref) / (float)cfg->adc_max;
    float denom = cfg->vref - vout;
    if (denom <= 0.0f) {
        return 0.0f;
    }

    return cfg->r_series * (vout / denom);
}

float temp_ntc_adc_to_celsius(uint16_t raw, const temp_ntc_beta_cfg_t* cfg) {
    if (cfg == NULL || cfg->beta <= 0.0f || cfg->r0 <= 0.0f) {
        return 0.0f;
    }

    float r_ntc = ntc_adc_to_resistance(raw, cfg);
    if (r_ntc <= 0.0f) {
        return 0.0f;
    }

    const float kelvin_offset = 273.15f;
    float t0_k = cfg->t0_c + kelvin_offset;
    float inv_t = (1.0f / t0_k) + (1.0f / cfg->beta) * logf(r_ntc / cfg->r0);
    float t_k = 1.0f / inv_t;

    return t_k - kelvin_offset;
}

int16_t temp_ntc_adc_to_deci_c(uint16_t raw, const temp_ntc_beta_cfg_t* cfg) {
    float temp_c = temp_ntc_adc_to_celsius(raw, cfg);
    return (int16_t)lroundf(temp_c * 10.0f);
}

void temp_ntc_vector_to_deci_c(const uint16_t* raw, size_t len, int16_t* out_deci_c,
                               const temp_ntc_beta_cfg_t* cfg) {
    if (raw == NULL || out_deci_c == NULL || cfg == NULL) {
        return;
    }

    for (size_t i = 0; i < len; i++) {
        out_deci_c[i] = temp_ntc_adc_to_deci_c(raw[i], cfg);
    }
}
