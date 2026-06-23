"""CAN message encoders for VCU Powertrain Simulator.

Byte layouts match firmware CAN_utils.c / fsic.c exactly.
All multi-byte fields: big-endian (MSB first) per FSIC DBC.
APPS and R2D/IGN: little-endian (LSB first) per firmware decode.
"""

import struct

# ──────────────────────────────────────────────
# CAN IDs
# ──────────────────────────────────────────────
R2D_AND_IGN_ID = 0x740
APPS_ADC_RAW_ID = 0x710
BMS_PRECHARGE_STATE_ID = 0x702
IVT_RESULT_W_ID = 0x526

INV1_ERPM_DUTY_VOLTAGE_ID = 0x404
INV1_AC_DC_CURRENT_ID = 0x424
INV1_TEMPERATURES_ID = 0x444
INV1_FOC_ID = 0x464
INV1_MISC_ID = 0x484

INV2_ERPM_DUTY_VOLTAGE_ID = 0x405
INV2_AC_DC_CURRENT_ID = 0x425
INV2_TEMPERATURES_ID = 0x445
INV2_FOC_ID = 0x465
INV2_MISC_ID = 0x485

# VCU telemetry IDs (CAN1 Data Bus) — for RX decode
VCU_0_ID = 0x20
VCU_1_ID = 0x21
VCU_2_ID = 0x22
VCU_3_ID = 0x23
VCU_4_ID = 0x24


# ──────────────────────────────────────────────
# Driver input messages (CAN2 Powertrain)
# ──────────────────────────────────────────────

def encode_r2d_and_ign(ignition: bool, r2d_button: bool) -> bytes:
    """0x740: data[0]=ignition, data[1]=r2d_button."""
    return bytes([1 if ignition else 0, 1 if r2d_button else 0, 0, 0, 0, 0, 0, 0])


def encode_apps_raw(apps1_adc: int, apps2_adc: int) -> bytes:
    """0x710: data[0..1]=apps1 uint16 LE, data[2..3]=apps2 uint16 LE."""
    apps1 = max(0, min(65535, int(apps1_adc)))
    apps2 = max(0, min(65535, int(apps2_adc)))
    return struct.pack('<HH', apps1, apps2) + b'\x00\x00'


# ──────────────────────────────────────────────
# BMS mock messages
# ──────────────────────────────────────────────

def encode_bms_precharge_state(state: int) -> bytes:
    """0x702: data[1]=precharge_circuit_state (9=done)."""
    return bytes([0, max(0, min(255, state)), 0, 0, 0, 0, 0, 0])


def encode_ivt_result_w(power_watts: int) -> bytes:
    """0x526: bytes 2-5 = power in watts, big-endian."""
    data = bytearray(8)
    val = max(-2147483648, min(2147483647, power_watts))
    data[2] = (val >> 24) & 0xFF
    data[3] = (val >> 16) & 0xFF
    data[4] = (val >> 8) & 0xFF
    data[5] = val & 0xFF
    return bytes(data)


# ──────────────────────────────────────────────
# FSIC Inverter mock messages — big-endian per DBC
# ──────────────────────────────────────────────

def encode_inv_erpm_duty_voltage(erpm: int, duty: int, voltage: int) -> bytes:
    """ERPM/Duty/Voltage: bytes 0-3=ERPM BE, bytes 4-5=duty BE, bytes 6-7=voltage BE."""
    data = bytearray(8)
    erpm_u = erpm & 0xFFFFFFFF
    data[0] = (erpm_u >> 24) & 0xFF
    data[1] = (erpm_u >> 16) & 0xFF
    data[2] = (erpm_u >> 8) & 0xFF
    data[3] = erpm_u & 0xFF
    duty_u = duty & 0xFFFF
    data[4] = (duty_u >> 8) & 0xFF
    data[5] = duty_u & 0xFF
    voltage_u = voltage & 0xFFFF
    data[6] = (voltage_u >> 8) & 0xFF
    data[7] = voltage_u & 0xFF
    return bytes(data)


def encode_inv_ac_dc_current(ac_current: int, dc_current: int) -> bytes:
    """AC/DC Current: bytes 0-1=AC current BE, bytes 2-3=DC current BE."""
    data = bytearray(8)
    ac_u = ac_current & 0xFFFF
    data[0] = (ac_u >> 8) & 0xFF
    data[1] = ac_u & 0xFF
    dc_u = dc_current & 0xFFFF
    data[2] = (dc_u >> 8) & 0xFF
    data[3] = dc_u & 0xFF
    return bytes(data)


def encode_inv_temperatures(temp_controller: int, temp_motor: int, fault_code: int) -> bytes:
    """Temperatures: bytes 0-1=controller temp BE, bytes 2-3=motor temp BE, byte 4=fault code."""
    data = bytearray(8)
    tc_u = temp_controller & 0xFFFF
    data[0] = (tc_u >> 8) & 0xFF
    data[1] = tc_u & 0xFF
    tm_u = temp_motor & 0xFFFF
    data[2] = (tm_u >> 8) & 0xFF
    data[3] = tm_u & 0xFF
    data[4] = fault_code & 0xFF
    return bytes(data)


def encode_inv_foc(foc_id: int, foc_iq: int) -> bytes:
    """FOC: bytes 0-3=foc_id BE, bytes 4-7=foc_iq BE."""
    data = bytearray(8)
    fid = foc_id & 0xFFFFFFFF
    data[0] = (fid >> 24) & 0xFF
    data[1] = (fid >> 16) & 0xFF
    data[2] = (fid >> 8) & 0xFF
    data[3] = fid & 0xFF
    fiq = foc_iq & 0xFFFFFFFF
    data[4] = (fiq >> 24) & 0xFF
    data[5] = (fiq >> 16) & 0xFF
    data[6] = (fiq >> 8) & 0xFF
    data[7] = fiq & 0xFF
    return bytes(data)


def encode_inv_misc(throttle: int, brake: int, digital_inputs: int,
                    drive_enable: bool, limits_byte4: int = 0,
                    limits_byte5: int = 0, can_map_version: int = 1) -> bytes:
    """MISC: byte 0=throttle, byte 1=brake, byte 2=digital I/O,
    byte 3=drive_enable, byte 4=limits1, byte 5=limits2, byte 7=can_map_version."""
    data = bytearray(8)
    data[0] = max(-128, min(127, throttle)) & 0xFF
    data[1] = max(-128, min(127, brake)) & 0xFF
    data[2] = digital_inputs & 0xFF
    data[3] = 1 if drive_enable else 0
    data[4] = limits_byte4 & 0xFF
    data[5] = limits_byte5 & 0xFF
    data[7] = can_map_version & 0xFF
    return bytes(data)


# ──────────────────────────────────────────────
# Message cycle groups
# ──────────────────────────────────────────────

def get_all_inv1_messages(erpm: int, duty: int, voltage: int,
                          ac_current: int, dc_current: int,
                          temp_controller: int, temp_motor: int,
                          fault_code: int, foc_id: int, foc_iq: int,
                          throttle: int, brake: int,
                          drive_enable: bool) -> list[tuple[int, bytes]]:
    """Return list of (can_id, data) for all INV1 status frames."""
    return [
        (INV1_ERPM_DUTY_VOLTAGE_ID, encode_inv_erpm_duty_voltage(erpm, duty, voltage)),
        (INV1_AC_DC_CURRENT_ID, encode_inv_ac_dc_current(ac_current, dc_current)),
        (INV1_TEMPERATURES_ID, encode_inv_temperatures(temp_controller, temp_motor, fault_code)),
        (INV1_FOC_ID, encode_inv_foc(foc_id, foc_iq)),
        (INV1_MISC_ID, encode_inv_misc(throttle, brake, 0x0F, drive_enable)),
    ]


def get_all_inv2_messages(erpm: int, duty: int, voltage: int,
                          ac_current: int, dc_current: int,
                          temp_controller: int, temp_motor: int,
                          fault_code: int, foc_id: int, foc_iq: int,
                          throttle: int, brake: int,
                          drive_enable: bool) -> list[tuple[int, bytes]]:
    """Return list of (can_id, data) for all INV2 status frames."""
    return [
        (INV2_ERPM_DUTY_VOLTAGE_ID, encode_inv_erpm_duty_voltage(erpm, duty, voltage)),
        (INV2_AC_DC_CURRENT_ID, encode_inv_ac_dc_current(ac_current, dc_current)),
        (INV2_TEMPERATURES_ID, encode_inv_temperatures(temp_controller, temp_motor, fault_code)),
        (INV2_FOC_ID, encode_inv_foc(foc_id, foc_iq)),
        (INV2_MISC_ID, encode_inv_misc(throttle, brake, 0x0F, drive_enable)),
    ]


# ──────────────────────────────────────────────
# Telemetry decode (VCU → CAN1 Data Bus)
# ──────────────────────────────────────────────

def decode_vcu_3(data: bytes) -> dict:
    """Decode VCU_3 frame (0x23): voltage, rpm, ign, r2d."""
    if len(data) < 8:
        return {}
    voltage = (data[0] << 8) | data[1]
    rpm = (data[2] << 8) | data[3]
    ign = data[4]
    r2d = data[5]
    return {"voltage": voltage, "rpm": rpm, "ign": ign, "r2d": r2d}


def decode_vcu_2(data: bytes) -> dict:
    """Decode VCU_2 frame (0x22): faults, limits."""
    if len(data) < 8:
        return {}
    faults = (data[0] << 8) | data[1]
    lmt1 = data[2]
    lmt2 = data[3]
    power_plan = data[5]
    return {"faults": faults, "lmt1": lmt1, "lmt2": lmt2, "power_plan": power_plan}
