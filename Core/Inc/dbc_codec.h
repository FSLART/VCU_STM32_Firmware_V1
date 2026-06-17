#ifndef DBC_CODEC_H
#define DBC_CODEC_H

#include "stm32f7xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* ==================== FSIC Inverter Messages (Powertrain bus / CAN2) ==================== */

/* Decode: INV1 ERPM/Duty/Voltage (0x404) */
bool dbc_decode_inv1_erpm(const uint8_t *data, uint8_t dlc, int32_t *erpm, int16_t *duty, int16_t *voltage);

/* Decode: INV1 AC/DC Current (0x424) */
bool dbc_decode_inv1_current(const uint8_t *data, uint8_t dlc, int16_t *ac_current, int16_t *dc_current);

/* Decode: INV1 Temperatures (0x444) */
bool dbc_decode_inv1_temps(const uint8_t *data, uint8_t dlc, int16_t *temp_ctrl, int16_t *temp_motor, uint8_t *fault_code);

/* Decode: INV1 MISC (0x484) — all limits, digital I/O, drive enable */
bool dbc_decode_inv1_misc(const uint8_t *data, uint8_t dlc, void *out_struct);

/* Encode: INV1 SetERPM (0x64) */
bool dbc_encode_inv1_set_erpm(uint8_t *data, uint8_t *dlc, int32_t target_erpm);

/* Encode: INV1 SetRelCurrent (0xa4) */
bool dbc_encode_inv1_set_rel_current(uint8_t *data, uint8_t *dlc, int16_t target_pct);

/* Encode: INV1 SetDriveEnable (0x184) */
bool dbc_encode_inv1_set_drive_enable(uint8_t *data, uint8_t *dlc, uint8_t enable);

/* Same set for INV2 (IDs offset by +1: 0x405, 0x425, etc.) */
bool dbc_decode_inv2_erpm(const uint8_t *data, uint8_t dlc, int32_t *erpm, int16_t *duty, int16_t *voltage);
bool dbc_decode_inv2_current(const uint8_t *data, uint8_t dlc, int16_t *ac_current, int16_t *dc_current);
bool dbc_decode_inv2_temps(const uint8_t *data, uint8_t dlc, int16_t *temp_ctrl, int16_t *temp_motor, uint8_t *fault_code);
bool dbc_encode_inv2_set_erpm(uint8_t *data, uint8_t *dlc, int32_t target_erpm);
bool dbc_encode_inv2_set_rel_current(uint8_t *data, uint8_t *dlc, int16_t target_pct);
bool dbc_encode_inv2_set_drive_enable(uint8_t *data, uint8_t *dlc, uint8_t enable);

/* ==================== Data Bus Messages (CAN1) ==================== */

/* Encode VCU_0 (0x20): apps, bps, trgt_power, cnsm_power */
bool dbc_encode_vcu_0(uint8_t *data, uint8_t *dlc, uint8_t apps, uint8_t bps, uint32_t trgt_power, uint32_t cnsm_power);

/* Encode VCU_1 (0x21): inv_temp, motor_temp, bms_voltage, soc */
bool dbc_encode_vcu_1(uint8_t *data, uint8_t *dlc, uint16_t inv_temp, uint16_t motor_temp, uint16_t bms_voltage, uint8_t soc);

/* Encode VCU_3 (0x23): inv_voltage, rpm, ign, r2d */
bool dbc_encode_vcu_3(uint8_t *data, uint8_t *dlc, uint16_t inv_voltage, uint16_t rpm, uint8_t ign, uint8_t r2d);

/* ==================== Autonomous Bus Messages (CAN3) ==================== */

/* Encode VCU_RPM (0x509) */
bool dbc_encode_vcu_rpm(uint8_t *data, uint8_t *dlc, uint16_t rpm);

/* Encode VCU_HV (0x81) */
bool dbc_encode_vcu_hv(uint8_t *data, uint8_t *dlc, uint8_t hv_state, uint8_t brake_pressure_front);

/* Decode ACU_IGN (0x71) */
bool dbc_decode_acu_ign(const uint8_t *data, uint8_t dlc, uint8_t *ign, uint8_t *asms, uint8_t *emergency);

/* Decode AS_STATE (0x503) */
bool dbc_decode_as_state(const uint8_t *data, uint8_t dlc, uint8_t *state);

/* Decode RPM_TARGET (0x499) */
bool dbc_decode_rpm_target(const uint8_t *data, uint8_t dlc, uint16_t *rpm_target);

#endif /* DBC_CODEC_H */
