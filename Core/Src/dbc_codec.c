#include "dbc_codec.h"
#include "fsic.h"
#include "data_dbc.h"
#include "autonomous_t26.h"
#include <string.h>

/* ==================== FSIC INV1 Decode ==================== */

bool dbc_decode_inv1_erpm(const uint8_t *data, uint8_t dlc, int32_t *erpm, int16_t *duty, int16_t *voltage) {
    if (dlc < FSIC_INV1_ERPM_DUTY_VOLTAGE_LENGTH) return false;
    struct fsic_inv1_erpm_duty_voltage_t msg;
    if (fsic_inv1_erpm_duty_voltage_unpack(&msg, data, FSIC_INV1_ERPM_DUTY_VOLTAGE_LENGTH) != 0) return false;
    *erpm = msg.inv1_actual_erpm;
    *duty = msg.inv1_actual_duty;
    *voltage = msg.inv1_actual_input_voltage;
    return true;
}

bool dbc_decode_inv1_current(const uint8_t *data, uint8_t dlc, int16_t *ac_current, int16_t *dc_current) {
    if (dlc < FSIC_INV1_AC_DC_CURRENT_LENGTH) return false;
    struct fsic_inv1_ac_dc_current_t msg;
    if (fsic_inv1_ac_dc_current_unpack(&msg, data, FSIC_INV1_AC_DC_CURRENT_LENGTH) != 0) return false;
    *ac_current = msg.inv1_actual_ac_current;
    *dc_current = msg.inv1_actual_dc_current;
    return true;
}

bool dbc_decode_inv1_temps(const uint8_t *data, uint8_t dlc, int16_t *temp_ctrl, int16_t *temp_motor, uint8_t *fault_code) {
    if (dlc < FSIC_INV1_TEMPERATURES_LENGTH) return false;
    struct fsic_inv1_temperatures_t msg;
    if (fsic_inv1_temperatures_unpack(&msg, data, FSIC_INV1_TEMPERATURES_LENGTH) != 0) return false;
    *temp_ctrl = msg.inv1_actual_temp_controller;
    *temp_motor = msg.inv1_actual_temp_motor;
    *fault_code = msg.inv1_actual_fault_code;
    return true;
}

bool dbc_decode_inv1_misc(const uint8_t *data, uint8_t dlc, void *out_struct) {
    if (dlc < FSIC_INV1_MISC_LENGTH) return false;
    if (out_struct == NULL) return false;
    struct fsic_inv1_misc_t *dst = (struct fsic_inv1_misc_t *)out_struct;
    if (fsic_inv1_misc_unpack(dst, data, FSIC_INV1_MISC_LENGTH) != 0) return false;
    return true;
}

/* ==================== FSIC INV1 Encode ==================== */

bool dbc_encode_inv1_set_erpm(uint8_t *data, uint8_t *dlc, int32_t target_erpm) {
    struct fsic_inv1_set_erpm_t msg;
    fsic_inv1_set_erpm_init(&msg);
    msg.inv1_cmd_target_speed = target_erpm;
    int ret = fsic_inv1_set_erpm_pack(data, &msg, FSIC_INV1_SET_ERPM_LENGTH);
    *dlc = FSIC_INV1_SET_ERPM_LENGTH;
    return (ret == FSIC_INV1_SET_ERPM_LENGTH);
}

bool dbc_encode_inv1_set_rel_current(uint8_t *data, uint8_t *dlc, int16_t target_pct) {
    struct fsic_inv1_set_rel_current_t msg;
    fsic_inv1_set_rel_current_init(&msg);
    msg.inv1_cmd_target_relative_current = target_pct;
    int ret = fsic_inv1_set_rel_current_pack(data, &msg, FSIC_INV1_SET_REL_CURRENT_LENGTH);
    *dlc = FSIC_INV1_SET_REL_CURRENT_LENGTH;
    return (ret == FSIC_INV1_SET_REL_CURRENT_LENGTH);
}

bool dbc_encode_inv1_set_drive_enable(uint8_t *data, uint8_t *dlc, uint8_t enable) {
    struct fsic_inv1_set_drive_enable_t msg;
    fsic_inv1_set_drive_enable_init(&msg);
    msg.inv1_cmd_drive_enable = enable;
    int ret = fsic_inv1_set_drive_enable_pack(data, &msg, FSIC_INV1_SET_DRIVE_ENABLE_LENGTH);
    *dlc = FSIC_INV1_SET_DRIVE_ENABLE_LENGTH;
    return (ret == FSIC_INV1_SET_DRIVE_ENABLE_LENGTH);
}

/* ==================== FSIC INV2 Decode ==================== */

bool dbc_decode_inv2_erpm(const uint8_t *data, uint8_t dlc, int32_t *erpm, int16_t *duty, int16_t *voltage) {
    if (dlc < FSIC_INV2_ERPM_DUTY_VOLTAGE_LENGTH) return false;
    struct fsic_inv2_erpm_duty_voltage_t msg;
    if (fsic_inv2_erpm_duty_voltage_unpack(&msg, data, FSIC_INV2_ERPM_DUTY_VOLTAGE_LENGTH) != 0) return false;
    *erpm = msg.inv2_actual_erpm;
    *duty = msg.inv2_actual_duty;
    *voltage = msg.inv2_actual_input_voltage;
    return true;
}

bool dbc_decode_inv2_current(const uint8_t *data, uint8_t dlc, int16_t *ac_current, int16_t *dc_current) {
    if (dlc < FSIC_INV2_AC_DC_CURRENT_LENGTH) return false;
    struct fsic_inv2_ac_dc_current_t msg;
    if (fsic_inv2_ac_dc_current_unpack(&msg, data, FSIC_INV2_AC_DC_CURRENT_LENGTH) != 0) return false;
    *ac_current = msg.inv2_actual_ac_current;
    *dc_current = msg.inv2_actual_dc_current;
    return true;
}

bool dbc_decode_inv2_temps(const uint8_t *data, uint8_t dlc, int16_t *temp_ctrl, int16_t *temp_motor, uint8_t *fault_code) {
    if (dlc < FSIC_INV2_TEMPERATURES_LENGTH) return false;
    struct fsic_inv2_temperatures_t msg;
    if (fsic_inv2_temperatures_unpack(&msg, data, FSIC_INV2_TEMPERATURES_LENGTH) != 0) return false;
    *temp_ctrl = msg.inv2_actual_temp_controller;
    *temp_motor = msg.inv2_actual_temp_motor;
    *fault_code = msg.inv2_actual_fault_code;
    return true;
}

/* ==================== FSIC INV2 Encode ==================== */

bool dbc_encode_inv2_set_erpm(uint8_t *data, uint8_t *dlc, int32_t target_erpm) {
    struct fsic_inv2_set_erpm_t msg;
    fsic_inv2_set_erpm_init(&msg);
    msg.inv2_cmd_target_speed = target_erpm;
    int ret = fsic_inv2_set_erpm_pack(data, &msg, FSIC_INV2_SET_ERPM_LENGTH);
    *dlc = FSIC_INV2_SET_ERPM_LENGTH;
    return (ret == FSIC_INV2_SET_ERPM_LENGTH);
}

bool dbc_encode_inv2_set_rel_current(uint8_t *data, uint8_t *dlc, int16_t target_pct) {
    struct fsic_inv2_set_rel_current_t msg;
    fsic_inv2_set_rel_current_init(&msg);
    msg.inv2_cmd_target_relative_current = target_pct;
    int ret = fsic_inv2_set_rel_current_pack(data, &msg, FSIC_INV2_SET_REL_CURRENT_LENGTH);
    *dlc = FSIC_INV2_SET_REL_CURRENT_LENGTH;
    return (ret == FSIC_INV2_SET_REL_CURRENT_LENGTH);
}

bool dbc_encode_inv2_set_drive_enable(uint8_t *data, uint8_t *dlc, uint8_t enable) {
    struct fsic_inv2_set_drive_enable_t msg;
    fsic_inv2_set_drive_enable_init(&msg);
    msg.inv2_cmd_drive_enable = enable;
    int ret = fsic_inv2_set_drive_enable_pack(data, &msg, FSIC_INV2_SET_DRIVE_ENABLE_LENGTH);
    *dlc = FSIC_INV2_SET_DRIVE_ENABLE_LENGTH;
    return (ret == FSIC_INV2_SET_DRIVE_ENABLE_LENGTH);
}

/* ==================== Data Bus Encode ==================== */

bool dbc_encode_vcu_0(uint8_t *data, uint8_t *dlc, uint8_t apps, uint8_t bps, uint32_t trgt_power, uint32_t cnsm_power) {
    struct data_dbc_vcu__t msg;
    data_dbc_vcu__init(&msg);
    msg.apps = apps;
    msg.bps = bps;
    msg.trgt_power = trgt_power;
    msg.cnsm_power = cnsm_power;
    int ret = data_dbc_vcu__pack(data, &msg, DATA_DBC_VCU__LENGTH);
    *dlc = DATA_DBC_VCU__LENGTH;
    return (ret == DATA_DBC_VCU__LENGTH);
}

bool dbc_encode_vcu_1(uint8_t *data, uint8_t *dlc, uint16_t inv_temp, uint16_t motor_temp, uint16_t bms_voltage, uint8_t soc) {
    struct data_dbc_vcu_1_t msg;
    data_dbc_vcu_1_init(&msg);
    msg.inv_temperature = inv_temp;
    msg.motor_temperature = motor_temp;
    msg.bms_voltage = bms_voltage;
    msg.soc_hv = soc;
    int ret = data_dbc_vcu_1_pack(data, &msg, DATA_DBC_VCU_1_LENGTH);
    *dlc = DATA_DBC_VCU_1_LENGTH;
    return (ret == DATA_DBC_VCU_1_LENGTH);
}

bool dbc_encode_vcu_3(uint8_t *data, uint8_t *dlc, uint16_t inv_voltage, uint16_t rpm, uint8_t ign, uint8_t r2d) {
    struct data_dbc_vcu_3_t msg;
    data_dbc_vcu_3_init(&msg);
    msg.inv_voltage = inv_voltage;
    msg.rpm = rpm;
    msg.ign = ign;
    msg.r2_d = r2d;
    int ret = data_dbc_vcu_3_pack(data, &msg, DATA_DBC_VCU_3_LENGTH);
    *dlc = DATA_DBC_VCU_3_LENGTH;
    return (ret == DATA_DBC_VCU_3_LENGTH);
}

/* ==================== Autonomous Bus Encode ==================== */

bool dbc_encode_vcu_rpm(uint8_t *data, uint8_t *dlc, uint16_t rpm) {
    struct autonomous_t26_vcu_rpm_t msg;
    autonomous_t26_vcu_rpm_init(&msg);
    msg.motor_rpm_left = rpm;
    int ret = autonomous_t26_vcu_rpm_pack(data, &msg, AUTONOMOUS_T26_VCU_RPM_LENGTH);
    *dlc = AUTONOMOUS_T26_VCU_RPM_LENGTH;
    return (ret == AUTONOMOUS_T26_VCU_RPM_LENGTH);
}

bool dbc_encode_vcu_hv(uint8_t *data, uint8_t *dlc, uint8_t hv_state, uint8_t brake_pressure_front) {
    struct autonomous_t26_vcu_hv_t msg;
    autonomous_t26_vcu_hv_init(&msg);
    msg.hv = hv_state;
    msg.brake_pressure_front = brake_pressure_front;
    int ret = autonomous_t26_vcu_hv_pack(data, &msg, AUTONOMOUS_T26_VCU_HV_LENGTH);
    *dlc = AUTONOMOUS_T26_VCU_HV_LENGTH;
    return (ret == AUTONOMOUS_T26_VCU_HV_LENGTH);
}

/* ==================== Autonomous Bus Decode ==================== */

bool dbc_decode_acu_ign(const uint8_t *data, uint8_t dlc, uint8_t *ign, uint8_t *asms, uint8_t *emergency) {
    if (dlc < AUTONOMOUS_T26_ACU_LENGTH) return false;
    struct autonomous_t26_acu_t msg;
    if (autonomous_t26_acu_unpack(&msg, data, AUTONOMOUS_T26_ACU_LENGTH) != 0) return false;
    *ign       = msg.ign;
    *asms      = msg.asms;
    *emergency = msg.emergency;
    return true;
}

bool dbc_decode_as_state(const uint8_t *data, uint8_t dlc, uint8_t *state) {
    if (dlc < AUTONOMOUS_T26_DV_STATUS_LENGTH) return false;
    struct autonomous_t26_dv_status_t msg;
    if (autonomous_t26_dv_status_unpack(&msg, data, AUTONOMOUS_T26_DV_STATUS_LENGTH) != 0) return false;
    *state = msg.as_status;
    return true;
}

bool dbc_decode_rpm_target(const uint8_t *data, uint8_t dlc, uint16_t *rpm_target) {
    if (dlc < AUTONOMOUS_T26_VCU_RPM_TARGET_LENGTH) return false;
    struct autonomous_t26_vcu_rpm_target_t msg;
    if (autonomous_t26_vcu_rpm_target_unpack(&msg, data, AUTONOMOUS_T26_VCU_RPM_TARGET_LENGTH) != 0) return false;
    *rpm_target = msg.rpm_target;
    return true;
}
