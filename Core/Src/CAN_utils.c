#include "CAN_utils.h"

#include <string.h>  // Add this for memset function

#include "can_queue.h"

#include "autonomous_t26.h"
#include "data_dbc.h"
#include "fsic.h"
#include "powertrain_t26.h"

// RPM to KM/H conversion macro: 1000 RPM = 24.54 km/h
#define RPM_TO_KMH(rpm) ((rpm) * 0.02454f)

/* CAN ID DEFINITIONS */
//CAN 1 - DATA

//CAN 2 - POWERTRAIN
//CAN 3 - AUTONOMOUS
#define BRAKE_PRESSURE_ID 0x710
// Initialize all signals to 0
VCU_Signals_t vcu = {
    .precharge_signal = false,

    .r2d_button_signal = false,
    .r2d_toggle_signal = false,
    .r2d_button_prev = false,
    // All other fields will be initialized to 0/false by default
};

//Variables
//APPS Loss of comms tick
volatile uint32_t last_apps_can_rx_time = 0; // keeps track of the last time a valid 0x710 message came through
volatile uint32_t last_acu_can_rx_time = 0;
__attribute__((section(".adcarray"))) uint16_t ADC2_APPS[2];  // ADC2_IN5(apps 1) and ADC2_IN6(apps 2)
volatile uint8_t debug_res_signal = 0;

#pragma region Basic CAN Functions

void can_bus_send(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint8_t len) {
    can_msg_t msg = {
        .id = id,
        .dlc = len,
        .timestamp = HAL_GetTick(),
    };
    memcpy(msg.data, data, len);

    if (hcan == &hcan1)      msg.bus = CAN_BUS_1;
    else if (hcan == &hcan2) msg.bus = CAN_BUS_2;
    else if (hcan == &hcan3) msg.bus = CAN_BUS_3;
    else return;

    can_tx_enqueue(&msg);
}

#pragma endregion Basic CAN Functions

#pragma region FSIC Inverter Control Functions

/*
 * FSIC Dual-Inverter Control Functions (replaces FSIC_t single-inverter)
 *
 * Each function sends a command to either INV1 or INV2 based on inv_id.
 * The fsic_inv1_* pack functions produce identical byte layout for both
 * inverters — only the CAN frame ID selects which inverter receives the command.
 *
 * SAFETY: Wrong inv_id = wrong inverter gets the command = potential vehicle dynamics issue.
 * inv_id == 1 selects INV1 frame IDs, inv_id == 2 selects INV2 frame IDs.
 */

/*Send_CAN_FSIC_SetAcCurrent*/
void can_bus_send_FSIC_SetAcCurrent(uint8_t inv_id, int16_t ac_current, CAN_HandleTypeDef *hcan) {
    struct fsic_inv1_set_ac_current_t msg;
    fsic_inv1_set_ac_current_init(&msg);
    msg.inv1_cmd_target_ac_current = ac_current;
    uint8_t data[8];
    fsic_inv1_set_ac_current_pack(data, &msg, sizeof(data));
    uint32_t frame_id = (inv_id == 2) ? FSIC_INV2_SET_AC_CURRENT_FRAME_ID
                                      : FSIC_INV1_SET_AC_CURRENT_FRAME_ID;
    can_bus_send(hcan, frame_id, data, FSIC_INV1_SET_AC_CURRENT_LENGTH);
}

/*Send_CAN_FSIC_SetBrakeCurrent*/
void can_bus_send_FSIC_SetBrakeCurrent(uint8_t inv_id, int16_t brake_current, CAN_HandleTypeDef *hcan) {
    struct fsic_inv1_set_brake_current_t msg;
    fsic_inv1_set_brake_current_init(&msg);
    msg.inv1_cmd_target_brake_current = brake_current;
    uint8_t data[8];
    fsic_inv1_set_brake_current_pack(data, &msg, sizeof(data));
    uint32_t frame_id = (inv_id == 2) ? FSIC_INV2_SET_BRAKE_CURRENT_FRAME_ID
                                      : FSIC_INV1_SET_BRAKE_CURRENT_FRAME_ID;
    can_bus_send(hcan, frame_id, data, FSIC_INV1_SET_BRAKE_CURRENT_LENGTH);
}

/*Send_CAN_FSIC_SetERPM*/
void can_bus_send_FSIC_SetERPM(uint8_t inv_id, int32_t erpm, CAN_HandleTypeDef *hcan) {
    uint8_t data[4];
    data[0] = (uint8_t)(erpm >> 24);
    data[1] = (uint8_t)(erpm >> 16);
    data[2] = (uint8_t)(erpm >> 8);
    data[3] = (uint8_t)(erpm & 0xFF);
    uint32_t frame_id = (inv_id == 2) ? FSIC_INV2_SET_ERPM_FRAME_ID
                                      : FSIC_INV1_SET_ERPM_FRAME_ID;
    can_bus_send(hcan, frame_id, data, 4);
}

/*Send_CAN_FSIC_SetPosition*/
void can_bus_send_FSIC_SetPosition(uint8_t inv_id, int16_t position, CAN_HandleTypeDef *hcan) {
    struct fsic_inv1_set_position_t msg;
    fsic_inv1_set_position_init(&msg);
    msg.inv1_cmd_target_position = position;
    uint8_t data[8];
    fsic_inv1_set_position_pack(data, &msg, sizeof(data));
    uint32_t frame_id = (inv_id == 2) ? FSIC_INV2_SET_POSITION_FRAME_ID
                                      : FSIC_INV1_SET_POSITION_FRAME_ID;
    can_bus_send(hcan, frame_id, data, FSIC_INV1_SET_POSITION_LENGTH);
}

/*Send_CAN_FSIC_SetRelCurrent*/
void can_bus_send_FSIC_SetRelCurrent(uint8_t inv_id, int16_t rel_current, CAN_HandleTypeDef *hcan) {
    uint8_t data[2];
    data[0] = (uint8_t)(rel_current >> 8);
    data[1] = (uint8_t)(rel_current & 0xFF);
    uint32_t frame_id = (inv_id == 2) ? FSIC_INV2_SET_REL_CURRENT_FRAME_ID
                                      : FSIC_INV1_SET_REL_CURRENT_FRAME_ID;
    can_bus_send(hcan, frame_id, data, 2);
}

/*Send_CAN_FSIC_SetRelBrakeCurrent*/
void can_bus_send_FSIC_SetRelBrakeCurrent(uint8_t inv_id, int16_t rel_brake_current, CAN_HandleTypeDef *hcan) {
    struct fsic_inv1_set_rel_brake_current_t msg;
    fsic_inv1_set_rel_brake_current_init(&msg);
    msg.inv1_cmd_tgt_rel_brake_current = rel_brake_current;
    uint8_t data[8];
    fsic_inv1_set_rel_brake_current_pack(data, &msg, sizeof(data));
    uint32_t frame_id = (inv_id == 2) ? FSIC_INV2_SET_REL_BRAKE_CURRENT_FRAME_ID
                                      : FSIC_INV1_SET_REL_BRAKE_CURRENT_FRAME_ID;
    can_bus_send(hcan, frame_id, data, FSIC_INV1_SET_REL_BRAKE_CURRENT_LENGTH);
}

/*Send_CAN_FSIC_SetMaxAcCurrent*/
void can_bus_send_FSIC_SetMaxAcCurrent(uint8_t inv_id, int16_t max_ac_current, CAN_HandleTypeDef *hcan) {
    struct fsic_inv1_set_max_ac_current_t msg;
    fsic_inv1_set_max_ac_current_init(&msg);
    msg.inv1_cmd_max_ac_current = max_ac_current;
    uint8_t data[8];
    fsic_inv1_set_max_ac_current_pack(data, &msg, sizeof(data));
    uint32_t frame_id = (inv_id == 2) ? FSIC_INV2_SET_MAX_AC_CURRENT_FRAME_ID
                                      : FSIC_INV1_SET_MAX_AC_CURRENT_FRAME_ID;
    can_bus_send(hcan, frame_id, data, FSIC_INV1_SET_MAX_AC_CURRENT_LENGTH);
}

/*Send_CAN_FSIC_SetMaxAcBrakeCurrent*/
void can_bus_send_FSIC_SetMaxAcBrakeCurrent(uint8_t inv_id, int16_t max_ac_brake_current, CAN_HandleTypeDef *hcan) {
    struct fsic_inv1_set_max_ac_brake_current_t msg;
    fsic_inv1_set_max_ac_brake_current_init(&msg);
    msg.inv1_cmd_max_ac_brake_current = max_ac_brake_current;
    uint8_t data[8];
    fsic_inv1_set_max_ac_brake_current_pack(data, &msg, sizeof(data));
    uint32_t frame_id = (inv_id == 2) ? FSIC_INV2_SET_MAX_AC_BRAKE_CURRENT_FRAME_ID
                                      : FSIC_INV1_SET_MAX_AC_BRAKE_CURRENT_FRAME_ID;
    can_bus_send(hcan, frame_id, data, FSIC_INV1_SET_MAX_AC_BRAKE_CURRENT_LENGTH);
}

/*Send_CAN_FSIC_SetMaxDcCurrent*/
void can_bus_send_FSIC_SetMaxDcCurrent(uint8_t inv_id, int16_t max_dc_current, CAN_HandleTypeDef *hcan) {
    struct fsic_inv1_set_max_dc_current_t msg;
    fsic_inv1_set_max_dc_current_init(&msg);
    msg.inv1_cmd_max_dc_current = max_dc_current;
    uint8_t data[8];
    fsic_inv1_set_max_dc_current_pack(data, &msg, sizeof(data));
    uint32_t frame_id = (inv_id == 2) ? FSIC_INV2_SET_MAX_DC_CURRENT_FRAME_ID
                                      : FSIC_INV1_SET_MAX_DC_CURRENT_FRAME_ID;
    can_bus_send(hcan, frame_id, data, FSIC_INV1_SET_MAX_DC_CURRENT_LENGTH);
}

/*Send_CAN_FSIC_SetMaxDcBrakeCurrent*/
void can_bus_send_FSIC_SetMaxDcBrakeCurrent(uint8_t inv_id, int16_t max_dc_brake_current, CAN_HandleTypeDef *hcan) {
    struct fsic_inv1_set_max_dc_brake_current_t msg;
    fsic_inv1_set_max_dc_brake_current_init(&msg);
    msg.inv1_cmd_max_dc_brake_current = max_dc_brake_current;
    uint8_t data[8];
    fsic_inv1_set_max_dc_brake_current_pack(data, &msg, sizeof(data));
    uint32_t frame_id = (inv_id == 2) ? FSIC_INV2_SET_MAX_DC_BRAKE_CURRENT_FRAME_ID
                                      : FSIC_INV1_SET_MAX_DC_BRAKE_CURRENT_FRAME_ID;
    can_bus_send(hcan, frame_id, data, FSIC_INV1_SET_MAX_DC_BRAKE_CURRENT_LENGTH);
}

/*Send_CAN_FSIC_SetDriveEnable*/
void can_bus_send_FSIC_SetDriveEnable(uint8_t inv_id, uint8_t drive_enable, CAN_HandleTypeDef *hcan) {
    uint8_t data[2];
    data[0] = drive_enable;
    data[1] = 0;
    uint32_t frame_id = (inv_id == 2) ? FSIC_INV2_SET_DRIVE_ENABLE_FRAME_ID
                                      : FSIC_INV1_SET_DRIVE_ENABLE_FRAME_ID;
    can_bus_send(hcan, frame_id, data, 2);
}

/*Send_Powertrain_Bus_1 — VCU status to BMS
 * TODO: The CAN ID for this VCU status message needs to be verified against the vehicle CAN DB.
 *       0x123 is a placeholder — powertrain_t26.h does not define a VCU-to-BMS status frame.
 *       WARNING: 0x123 conflicts with POWERTRAIN_T26_SLAVE_06_VOLTAGE_ID_1_FRAME_ID — DO NOT use
 *       this ID on CAN2 without confirming the correct VCU status frame ID first.
 */
void can_bus_send_pwtbus_1(uint8_t r2d, uint8_t ignition, CAN_HandleTypeDef *hcan) {
    uint8_t data[8] = {0};
    data[0] = r2d;
    data[1] = ignition;
    /* TODO: Replace 0x123 with the correct VCU-to-BMS status CAN ID */
    can_bus_send(hcan, 0x123, data, 8);
}

void can_bus_send_bms_close_contactors(uint8_t close_contactors, CAN_HandleTypeDef *hcan) {
    static uint32_t last_send_time = 0;
    static uint8_t last_value = 0xFF; // Start with invalid value to force first send
    uint32_t current_time = HAL_GetTick();

    // If the value changes, send immediately. Otherwise, rate-limit to 100Hz (10ms)
    if (close_contactors == last_value && (current_time - last_send_time < 10)) {
        return;
    }

    last_send_time = current_time;
    last_value = close_contactors;

    can_data_t data;
    data.id = 0x83;
    data.length = 1;

    memset(data.message, 0x00, sizeof(data.message));

    data.message[0] = close_contactors;

    can_bus_send(hcan, data.id, data.message, data.length);
}

/*
=========================================================
                POWERTRAIN DECODE
=========================================================
*/

void decode_powertrain_bus(const can_msg_t *msg, BMSvars_t* bms, FSIC_t* fsic1, FSIC_t* fsic2, IVT_t* ivt) {
    /* Powertrain DBC IDs are all 11-bit standard. Reject extended frames
       for the same reason as decode_autonomous_bus — see comment there. */
    if (msg->is_extended) {
        return;
    }

    const uint8_t *data = msg->data;

    switch (msg->id) {

        /* ---- BMS Messages (TODO: old CAN_PWT_BMS_ID_* are undefined, need new DBC definitions) ---- */
        // TODO: BMS messages need new DBC definitions - old CAN_PWT_BMS_ID_* are undefined
        // case CAN_PWT_BMS_ID_3:
        //     bms->high_cell_temp = MAP_DECODE_PWT_BMS_PACK_HIGH_CELL_TEMP(data);
        //     bms->low_cell_temp = MAP_DECODE_PWT_BMS_PACK_LOW_CELL_TEMP(data);
        //     break;

        case 0x702:
            bms->precharge_circuit_state = data[1];  // TODO NEW BMS PRE CHARGE STATE ID, CHECK THIS
            break;

        /* ---- FSIC INV1 RX: ERPM/Duty/Voltage (0x404) ---- */
        case FSIC_INV1_ERPM_DUTY_VOLTAGE_FRAME_ID: {
            struct fsic_inv1_erpm_duty_voltage_t m;
            fsic_inv1_erpm_duty_voltage_init(&m);
            fsic_inv1_erpm_duty_voltage_unpack(&m, data, msg->dlc);
            fsic1->Actual_ERPM = m.inv1_actual_erpm;
            fsic1->Actual_Duty = m.inv1_actual_duty;
            fsic1->Actual_InputVoltage = (uint16_t)m.inv1_actual_input_voltage;
            break;
        }

        /* ---- FSIC INV2 RX: ERPM/Duty/Voltage (0x405) ---- */
        case FSIC_INV2_ERPM_DUTY_VOLTAGE_FRAME_ID: {
            struct fsic_inv2_erpm_duty_voltage_t m;
            fsic_inv2_erpm_duty_voltage_init(&m);
            fsic_inv2_erpm_duty_voltage_unpack(&m, data, msg->dlc);
            fsic2->Actual_ERPM = m.inv2_actual_erpm;
            fsic2->Actual_Duty = m.inv2_actual_duty;
            fsic2->Actual_InputVoltage = (uint16_t)m.inv2_actual_input_voltage;
            break;
        }

        /* ---- IVT Result W (0x526) ---- */
        case 0x526:
            // byte 2 to 5 its power in motorola (big endian)
            ivt->result_W = (int32_t)((data[2] << 24) | (data[3] << 16) | (data[4] << 8) | data[5]);
            break;

        /* ---- FSIC INV1 RX: AC/DC Current (0x424) ---- */
        case FSIC_INV1_AC_DC_CURRENT_FRAME_ID: {
            struct fsic_inv1_ac_dc_current_t m;
            fsic_inv1_ac_dc_current_init(&m);
            fsic_inv1_ac_dc_current_unpack(&m, data, msg->dlc);
            fsic1->Actual_ACCurrent = m.inv1_actual_ac_current;
            fsic1->Actual_DCCurrent = m.inv1_actual_dc_current;
            break;
        }

        /* ---- FSIC INV2 RX: AC/DC Current (0x425) ---- */
        case FSIC_INV2_AC_DC_CURRENT_FRAME_ID: {
            struct fsic_inv2_ac_dc_current_t m;
            fsic_inv2_ac_dc_current_init(&m);
            fsic_inv2_ac_dc_current_unpack(&m, data, msg->dlc);
            fsic2->Actual_ACCurrent = m.inv2_actual_ac_current;
            fsic2->Actual_DCCurrent = m.inv2_actual_dc_current;
            break;
        }

        /* ---- FSIC INV1 RX: Temperatures (0x444) ---- */
        case FSIC_INV1_TEMPERATURES_FRAME_ID: {
            struct fsic_inv1_temperatures_t m;
            fsic_inv1_temperatures_init(&m);
            fsic_inv1_temperatures_unpack(&m, data, msg->dlc);
            fsic1->Actual_TempController = m.inv1_actual_temp_controller;
            fsic1->Actual_TempMotor = m.inv1_actual_temp_motor;
            fsic1->Actual_FaultCode = m.inv1_actual_fault_code;
            
            /* Forward to Data bus via queue */
            can_msg_t tx_msg1;
            tx_msg1.id = FSIC_INV1_TEMPERATURES_FRAME_ID;
            tx_msg1.dlc = msg->dlc;
            tx_msg1.bus = CAN_BUS_1;
            tx_msg1.is_extended = 0;
            tx_msg1.timestamp = HAL_GetTick();
            memcpy(tx_msg1.data, data, msg->dlc);
            can_tx_enqueue(&tx_msg1);
            break;
        }

        /* ---- FSIC INV2 RX: Temperatures (0x445) ---- */
        case FSIC_INV2_TEMPERATURES_FRAME_ID: {
            struct fsic_inv2_temperatures_t m;
            fsic_inv2_temperatures_init(&m);
            fsic_inv2_temperatures_unpack(&m, data, msg->dlc);
            fsic2->Actual_TempController = m.inv2_actual_temp_controller;
            fsic2->Actual_TempMotor = m.inv2_actual_temp_motor;
            fsic2->Actual_FaultCode = m.inv2_actual_fault_code;
            
            /* Forward to Data bus via queue */
            can_msg_t tx_msg2;
            tx_msg2.id = FSIC_INV2_TEMPERATURES_FRAME_ID;
            tx_msg2.dlc = msg->dlc;
            tx_msg2.bus = CAN_BUS_1;
            tx_msg2.is_extended = 0;
            tx_msg2.timestamp = HAL_GetTick();
            memcpy(tx_msg2.data, data, msg->dlc);
            can_tx_enqueue(&tx_msg2);
            break;
        }

        /* ---- FSIC INV1 RX: FOC (0x464) ---- */
        case FSIC_INV1_FOC_FRAME_ID: {
            struct fsic_inv1_foc_t m;
            fsic_inv1_foc_init(&m);
            fsic_inv1_foc_unpack(&m, data, msg->dlc);
            fsic1->Actual_FOC_id = m.inv1_actual_foc_id;
            fsic1->Actual_FOC_iq = m.inv1_actual_foc_iq;
            break;
        }

        /* ---- FSIC INV2 RX: FOC (0x465) ---- */
        case FSIC_INV2_FOC_FRAME_ID: {
            struct fsic_inv2_foc_t m;
            fsic_inv2_foc_init(&m);
            fsic_inv2_foc_unpack(&m, data, msg->dlc);
            fsic2->Actual_FOC_id = m.inv2_actual_foc_id;
            fsic2->Actual_FOC_iq = m.inv2_actual_foc_iq;
            break;
        }

        /* ---- FSIC INV1 RX: MISC (0x484) ---- */
        case FSIC_INV1_MISC_FRAME_ID: {
            struct fsic_inv1_misc_t m;
            fsic_inv1_misc_init(&m);
            fsic_inv1_misc_unpack(&m, data, msg->dlc);
            fsic1->Actual_Throttle = m.inv1_actual_throttle;
            fsic1->Actual_Brake = m.inv1_actual_brake;
            fsic1->Digital_input_1 = m.inv1_digital_input_1;
            fsic1->Digital_input_2 = m.inv1_digital_input_2;
            fsic1->Digital_input_3 = m.inv1_digital_input_3;
            fsic1->Digital_input_4 = m.inv1_digital_input_4;
            fsic1->Digital_output_1 = m.inv1_digital_output_1;
            fsic1->Digital_output_2 = m.inv1_digital_output_2;
            fsic1->Digital_output_3 = m.inv1_digital_output_3;
            fsic1->Digital_output_4 = m.inv1_digital_output_4;
            fsic1->Drive_enable = m.inv1_drive_enable;
            fsic1->Capacitor_temp_limit = m.inv1_capacitor_temp_limit;
            fsic1->DC_current_limit = m.inv1_dc_current_limit;
            fsic1->Drive_enable_limit = m.inv1_drive_enable_limit;
            fsic1->IGBT_accel_limit = m.inv1_igbt_accel_limit;
            fsic1->IGBT_temp_limit = m.inv1_igbt_temp_limit;
            fsic1->Input_voltage_limit = m.inv1_input_voltage_limit;
            fsic1->Motor_accel_limit = m.inv1_motor_accel_limit;
            fsic1->Motor_temp_limit = m.inv1_motor_temp_limit;
            fsic1->RPM_min_limit = m.inv1_rpm_min_limit;
            fsic1->RPM_max_limit = m.inv1_rpm_max_limit;
            fsic1->Power_limit = m.inv1_power_limit;
            fsic1->CAN_map_version = m.inv1_can_map_version;
            break;
        }

        /* ---- FSIC INV2 RX: MISC (0x485) ---- */
        case FSIC_INV2_MISC_FRAME_ID: {
            struct fsic_inv2_misc_t m;
            fsic_inv2_misc_init(&m);
            fsic_inv2_misc_unpack(&m, data, msg->dlc);
            fsic2->Actual_Throttle = m.inv2_actual_throttle;
            fsic2->Actual_Brake = m.inv2_actual_brake;
            fsic2->Digital_input_1 = m.inv2_digital_input_1;
            fsic2->Digital_input_2 = m.inv2_digital_input_2;
            fsic2->Digital_input_3 = m.inv2_digital_input_3;
            fsic2->Digital_input_4 = m.inv2_digital_input_4;
            fsic2->Digital_output_1 = m.inv2_digital_output_1;
            fsic2->Digital_output_2 = m.inv2_digital_output_2;
            fsic2->Digital_output_3 = m.inv2_digital_output_3;
            fsic2->Digital_output_4 = m.inv2_digital_output_4;
            fsic2->Drive_enable = m.inv2_drive_enable;
            fsic2->Capacitor_temp_limit = m.inv2_capacitor_temp_limit;
            fsic2->DC_current_limit = m.inv2_dc_current_limit;
            fsic2->Drive_enable_limit = m.inv2_drive_enable_limit;
            fsic2->IGBT_accel_limit = m.inv2_igbt_accel_limit;
            fsic2->IGBT_temp_limit = m.inv2_igbt_temp_limit;
            fsic2->Input_voltage_limit = m.inv2_input_voltage_limit;
            fsic2->Motor_accel_limit = m.inv2_motor_accel_limit;
            fsic2->Motor_temp_limit = m.inv2_motor_temp_limit;
            fsic2->RPM_min_limit = m.inv2_rpm_min_limit;
            fsic2->RPM_max_limit = m.inv2_rpm_max_limit;
            fsic2->Power_limit = m.inv2_power_limit;
            fsic2->CAN_map_version = m.inv2_can_map_version;
            break;
        }

        /* ---- BMS Messages (TODO: old CAN_PWT_BMS_ID_* are undefined, need new DBC definitions) ---- */
        // TODO: BMS messages need new DBC definitions - old CAN_PWT_BMS_ID_* are undefined
        // case CAN_PWT_BMS_ID_1:
        //     bms->instant_voltage = MAP_DECODE_PWT_BMS_PACK_INSTANT_VOLTAGE(data);
        //     bms->soc = MAP_DECODE_PWT_BMS_PACK_SOC(data);
        //     break;

        // case CAN_PWT_BMS_ID_2:
        //     bms->high_cell_voltage = MAP_DECODE_PWT_BMS_PACK_HIGH_CELL_VOLTAGE(data);
        //     bms->low_cell_voltage = MAP_DECODE_PWT_BMS_PACK_LOW_CELL_VOLTAGE(data);
        //     bms->avg_cell_voltage = MAP_DECODE_PWT_BMS_PACK_AVG_CELL_VOLTAGE(data);
        //     break;

        case POWERTRAIN_T26_APPS_ADC_RAW_FRAME_ID: {
            struct powertrain_t26_apps_adc_raw_t apps_msg;
            powertrain_t26_apps_adc_raw_init(&apps_msg);
            powertrain_t26_apps_adc_raw_unpack(&apps_msg, data, msg->dlc);
            
            __disable_irq(); // Lock interrupts so this update doesn't happen while MovingAverage_Update() is trying to read it
#ifdef POWERTRAIN_T26_APPS_ADC_RAW_APPS1_RAW_NAME
            ADC2_APPS[0] = (uint16_t)apps_msg.apps1_raw / 10;
            ADC2_APPS[1] = (uint16_t)apps_msg.apps2_raw / 10;
#else
            ADC2_APPS[0] = (uint16_t)apps_msg.apps1 / 10;
            ADC2_APPS[1] = (uint16_t)apps_msg.apps2 / 10;
#endif
            __enable_irq();  // Unlock interrupts so MovingAverage_Update() can get to reading them

            last_apps_can_rx_time = HAL_GetTick(); // Reset the safety timer (For checking comms)
            break;
        }
        case POWERTRAIN_T26_DASH_BOARD_FRAME_ID: {
            struct powertrain_t26_dash_board_t db_msg;
            powertrain_t26_dash_board_init(&db_msg);
            powertrain_t26_dash_board_unpack(&db_msg, data, msg->dlc);
            uint32_t current_time = HAL_GetTick();

            // Process ignition as a momentary button with 50ms debounce (toggle on falling edge / button release)
            if (db_msg.ignition_switch_raw != vcu.ignition_button_prev && (current_time - vcu.ignition_last_toggle_time > 50)) {
                if (!db_msg.ignition_switch_raw) {
                    vcu.ignition_toggle_signal = !vcu.ignition_toggle_signal;
                }
                vcu.ignition_button_prev = db_msg.ignition_switch_raw;
                vcu.ignition_last_toggle_time = current_time;
            }
            vcu.ignition_switch_signal = vcu.ignition_toggle_signal;

            // Process r2d as a momentary button with 50ms debounce (toggle on falling edge / button release)
            if (db_msg.r2d_button_raw != vcu.r2d_button_prev && (current_time - vcu.r2d_last_toggle_time > 50)) {
                if (!db_msg.r2d_button_raw) {
                    vcu.r2d_toggle_signal = !vcu.r2d_toggle_signal;
                }
                vcu.r2d_button_prev = db_msg.r2d_button_raw;
                vcu.r2d_last_toggle_time = current_time;
            }
            vcu.r2d_button_signal = vcu.r2d_toggle_signal;
            
            break;
        }
        default:
            break;
    }
}

/*======================================= Autonomous bus =======================================*/
#pragma region Autonomous Bus
/**
 * @brief Sends rpm to the autonomous system
 *
 * @param state The state of the autonomous system
 * @param hcan CAN handle for the VCU bus (CAN3)
 */
void can_send_vcu_rpm(CAN_HandleTypeDef *hcan, int32_t erpm_left, int32_t erpm_right) {
    // Scale down the ERPM value to mechanical RPM
    int32_t rpm_left = erpm_left / MOTOR_POLE_PAIRS;
    int32_t rpm_right = erpm_right / MOTOR_POLE_PAIRS;

    // Clamp to uint16_t range for safety
    if (rpm_left > 65535) {
        rpm_left = 65535;
    } else if (rpm_left < 0) {
        rpm_left = 0;
    }
    
    if (rpm_right > 65535) {
        rpm_right = 65535;
    } else if (rpm_right < 0) {
        rpm_right = 0;
    }

    struct autonomous_t26_vcu_rpm_t vcu_rpm_msg;
    uint8_t data[8];

    autonomous_t26_vcu_rpm_init(&vcu_rpm_msg);
    vcu_rpm_msg.motor_rpm_left = (uint16_t)rpm_left;
    vcu_rpm_msg.motor_rpm_right = (uint16_t)rpm_right;
    vcu_rpm_msg.motor_current_left = 0;
    vcu_rpm_msg.motor_current_right = 0;

    int size = autonomous_t26_vcu_rpm_pack(data, &vcu_rpm_msg, sizeof(data));
    if (size >= 0) {
        can_bus_send(hcan, AUTONOMOUS_T26_VCU_RPM_FRAME_ID, data, AUTONOMOUS_T26_VCU_RPM_LENGTH);
    }
}

/**
 * @brief Sends the HV signal and brake pressure front to the autonomous system
 *
 * @param hcan CAN handle for the VCU bus (CAN3)
 * @param hv_state The state of the HV (0 or 1)
 * @param brake_pressure_front The front brake pressure value (0-255)
 */
void can_send_autonomous_HV_signal(CAN_HandleTypeDef *hcan, uint8_t hv_state, uint8_t brake_pressure_front) {
    struct autonomous_t26_vcu_hv_t hv_signal_msg;
    uint8_t data[8];

    autonomous_t26_vcu_hv_init(&hv_signal_msg);
    hv_signal_msg.hv = hv_state;
    hv_signal_msg.brake_pressure_front = brake_pressure_front;
    hv_signal_msg.brake_pressure_rear = 0;

    int size = autonomous_t26_vcu_hv_pack(data, &hv_signal_msg, sizeof(data));
    if (size >= 0) {
        can_bus_send(hcan, AUTONOMOUS_T26_VCU_HV_FRAME_ID, data, AUTONOMOUS_T26_VCU_HV_LENGTH);
    }
}

void can_send_vcu_ign_r2d_signals(CAN_HandleTypeDef *hcan, uint8_t ignition_manual, uint8_t r2d_manual, uint8_t ignition_auto, uint8_t r2d_auto, uint8_t shutdown_signal, uint8_t vcu_state) {
    struct autonomous_t26_vcu_ign_r2_d_t ign_r2d_msg;
    uint8_t data[8];

    autonomous_t26_vcu_ign_r2_d_init(&ign_r2d_msg);
    ign_r2d_msg.ignition_manual = ignition_manual;
    ign_r2d_msg.r2d_manual = r2d_manual;
    ign_r2d_msg.ignition_auto = ignition_auto;
    ign_r2d_msg.r2d_auto = r2d_auto;
    ign_r2d_msg.shutdown_signal = shutdown_signal;
    ign_r2d_msg.vcu_state = vcu_state;
    ign_r2d_msg.r2_d_button_raw = 0;
    ign_r2d_msg.ignition_switch_raw = 0;

    int size = autonomous_t26_vcu_ign_r2_d_pack(data, &ign_r2d_msg, sizeof(data));
    if (size >= 0) {
        can_bus_send(hcan, AUTONOMOUS_T26_VCU_IGN_R2_D_FRAME_ID, data, AUTONOMOUS_T26_VCU_IGN_R2_D_LENGTH);
    }
}

/**
 * @brief Sends the emergency state to the autonomous system
 *
 * @param hcan CAN handle for the VCU bus (CAN3)
 */
void decode_autonomous_bus(const can_msg_t *msg, AS_System_t *as_system, ACU_t *acu, RES_t *res) {
    /* Every frame ID in the autonomous DBC is an 11-bit standard ID. An
       extended (29-bit) frame is either foreign traffic or a corrupted
       capture, and msg->id for it must NOT be compared against the
       standard-ID defines below (id-space overlap would decode it as the
       wrong signal with the wrong payload). Drop it before the switch. */
    if (msg->is_extended) {
        return;
    }

    const uint8_t *data = msg->data;
    switch (msg->id) {
        case AUTONOMOUS_T26_ACU_FRAME_ID: {
            struct autonomous_t26_acu_t acu_msg;
            autonomous_t26_acu_init(&acu_msg);
            if (autonomous_t26_acu_unpack(&acu_msg, data, msg->dlc) == 0) {
                acu->mission_select = acu_msg.mission_select;
                acu->ignition_ad = acu_msg.ign;
                acu->ASMS = acu_msg.asms;
                acu->is_in_emergency = acu_msg.emergency;
                last_acu_can_rx_time = HAL_GetTick();
            }
            break;
        }
        case AUTONOMOUS_T26_JETSON_FRAME_ID: {
            struct autonomous_t26_jetson_t jetson;
            autonomous_t26_jetson_init(&jetson);
            if (autonomous_t26_jetson_unpack(&jetson, data, msg->dlc) == 0) {
                as_system->mission_select = jetson.as_mission;
            }
            break;
        }
        case (0x181u): {
        //case AUTONOMOUS_T26_RES_FRAME_ID: {
            /* 
            struct autonomous_t26_res_t res_ad;
            autonomous_t26_res_init(&res_ad);
            if (autonomous_t26_res_unpack(&res_ad, data, msg->dlc) == 0) {
                res->signal = res_ad.signal;
            }
            */
            
            // Manual Decode
            if (msg->dlc >= 1) {
                res->signal = data[0];
                debug_res_signal = data[0];
            }
            break;
        }
        case AUTONOMOUS_T26_VCU_RPM_TARGET_FRAME_ID: {
            struct autonomous_t26_vcu_rpm_target_t target_rpm;
            autonomous_t26_vcu_rpm_target_init(&target_rpm);
            if (autonomous_t26_vcu_rpm_target_unpack(&target_rpm, data, msg->dlc) == 0) {
                as_system->target_rpm = target_rpm.rpm_target;
                as_system->control_mode = AS_CONTROL_MODE_RPM;
                as_system->last_control_cmd_ms = HAL_GetTick();
            }
            break;
        }
        case AUTONOMOUS_T26_VCU_TORQUE_TARGET_FRAME_ID: {
            struct autonomous_t26_vcu_torque_target_t target_torque;
            autonomous_t26_vcu_torque_target_init(&target_torque);
            if (autonomous_t26_vcu_torque_target_unpack(&target_torque, data, msg->dlc) == 0) {
                as_system->target_torque = target_torque.torque_target;
                as_system->control_mode = AS_CONTROL_MODE_TORQUE;
                as_system->last_control_cmd_ms = HAL_GetTick();
            }
            break;
        }
        case AUTONOMOUS_T26_DV_STATUS_FRAME_ID: {
            struct autonomous_t26_dv_status_t as_state;
            autonomous_t26_dv_status_init(&as_state);
            if (autonomous_t26_dv_status_unpack(&as_state, data, msg->dlc) == 0) {
                as_system->state = as_state.as_status;
            }
            break;
        }
        case BRAKE_PRESSURE_ID: {
            uint16_t raw_pressure = (data[1] << 8) | data[0];
            uint8_t resulting_pressure = raw_pressure * 0.1;
            vcu.brake_pressure = resulting_pressure;
            break;
        }
        default:
            break;
    }
}
#pragma endregion Autonomous Bus

// ----------------------------------------------
//---------- SEND DATA TO DATA BUS -----------
// ----------------------------------------------

/**
 * @brief Send VCU_ frame (0x20) - Basic pedal and power data
 * @param hcan CAN handle for the data bus
 * @param hv500 Pointer to FSIC_t struct containing inverter data
 */
void send_vcu_0(CAN_HandleTypeDef *hcan, const FSIC_t *hv500) {
    struct data_dbc_vcu__t vcu_frame;
    uint8_t data[8];

    // Initialize the frame
    data_dbc_vcu__init(&vcu_frame);

    // Populate with actual VCU data
    // extern APPS_Result_t result;  // Get APPS result from main
    // extern VCU_Signals_t vcu;     // Get VCU signals from main

    // vcu_frame.apps = (uint8_t)(result.percentage_1000 / 10);                                           // Convert from 0-1000 to 0-100%
    // vcu_frame.bps = vcu.brake_pressure;                                                                // Brake pressure from ADC
    vcu_frame.trgt_power = (uint32_t)(hv500->Actual_ERPM * hv500->Actual_DCCurrent / 1000);          // Estimated target power
    vcu_frame.cnsm_power = (uint32_t)(hv500->Actual_InputVoltage * hv500->Actual_DCCurrent / 1000);  // Consumed power

    // Pack the data
    int pack_result = data_dbc_vcu__pack(data, &vcu_frame, sizeof(data));
    if (pack_result > 0) {
        can_bus_send(hcan, DATA_DBC_VCU__FRAME_ID, data, DATA_DBC_VCU__LENGTH);
    }
}

/**
 * @brief Send VCU_1 frame (0x21) - Temperature and voltage data
 * @param hcan CAN handle for the data bus
 * @param hv500 Pointer to FSIC_t struct containing inverter data
 * @param bms Pointer to BMSvars_t struct containing BMS data
 */
void send_vcu_1(CAN_HandleTypeDef *hcan, const FSIC_t *hv500, const BMSvars_t *bms) {
    struct data_dbc_vcu_1_t vcu1_frame;
    uint8_t data[8];

    // Initialize the frame
    data_dbc_vcu_1_init(&vcu1_frame);

    // Populate with actual VCU data from FSIC_t and BMS
    vcu1_frame.inv_temperature = (uint16_t)hv500->Actual_TempController;  // Inverter temperature
    vcu1_frame.motor_temperature = (uint16_t)hv500->Actual_TempMotor;     // Motor temperature
    vcu1_frame.bms_voltage = bms->instant_voltage;                        // BMS voltage
    vcu1_frame.soc_hv = bms->soc;                                         // SOC HV

    // Pack the data
    int pack_result = data_dbc_vcu_1_pack(data, &vcu1_frame, sizeof(data));
    if (pack_result > 0) {
        can_bus_send(hcan, DATA_DBC_VCU_1_FRAME_ID, data, DATA_DBC_VCU_1_LENGTH);
    }
}

/**
 * @brief Send VCU_2 frame (0x22) - Faults and state data
 * @param hcan CAN handle for the data bus
 * @param hv500 Pointer to FSIC_t struct containing inverter data
 */
void send_vcu_2(CAN_HandleTypeDef *hcan, const FSIC_t *hv500) {
    struct data_dbc_vcu_2_t vcu2_frame;
    uint8_t data[8];

    // Initialize the frame
    data_dbc_vcu_2_init(&vcu2_frame);

    // Populate with actual VCU data
    vcu2_frame.inv_faults = (uint16_t)hv500->Actual_FaultCode;  // Inverter faults
    vcu2_frame.lmt1 = (uint8_t)hv500->RPM_max_limit;            // RPM power limit
    vcu2_frame.lmt2 = (uint8_t)hv500->Motor_temp_limit;         // Current limit motor temp
    vcu2_frame.vcu_state = (uint8_t)current_state;               // VCU state machine current state
    vcu2_frame.power_plan = 0;  // Power plan switch - add variable when available

    // Pack the data
    int pack_result = data_dbc_vcu_2_pack(data, &vcu2_frame, sizeof(data));
    if (pack_result > 0) {
        can_bus_send(hcan, DATA_DBC_VCU_2_FRAME_ID, data, DATA_DBC_VCU_2_LENGTH);
    }
}

/**
 * @brief Send VCU_3 frame (0x23) - Motor and system status
 * @param hcan CAN handle for the data bus
 * @param r2d_manual Manual R2D signal
 * @param ignition_manual Manual ignition signal
 * @param r2d_auto Autonomous R2D signal
 * @param ignition_auto Autonomous ignition signal
 * @param hv500 Pointer to FSIC_t struct containing inverter data
 */
void send_vcu_3(CAN_HandleTypeDef *hcan, bool r2d_manual, bool ignition_manual, bool r2d_auto, bool ignition_auto, const FSIC_t *hv500) {
    struct data_dbc_vcu_3_t vcu3_frame;
    uint8_t data[8];

    // Initialize the frame
    data_dbc_vcu_3_init(&vcu3_frame);

    // Populate with actual VCU data
    // extern VCU_Signals_t vcu;  // VCU signals structure

    vcu3_frame.inv_voltage = (uint16_t)hv500->Actual_InputVoltage;  // DC link inverter voltage
    vcu3_frame.rpm = (uint16_t)(hv500->Actual_ERPM / 10);           // RPM (convert from ERPM)
    vcu3_frame.ign = (uint8_t)(ignition_manual || ignition_auto);   // Combined ignition signal
    vcu3_frame.r2_d = (uint8_t)(r2d_manual || r2d_auto);            // Combined R2D signal

    // Pack the data
    int pack_result = data_dbc_vcu_3_pack(data, &vcu3_frame, sizeof(data));
    if (pack_result > 0) {
        can_bus_send(hcan, DATA_DBC_VCU_3_FRAME_ID, data, DATA_DBC_VCU_3_LENGTH);
    }
}

/**
 * @brief Send VCU_4 frame (0x24) - System states and LV data
 * @param hcan CAN handle for the data bus
 * @param acu Pointer to ACU_t struct containing autonomous control unit data
 */
void send_vcu_4(CAN_HandleTypeDef *hcan, const ACU_t *acu) {
    struct data_dbc_vcu_4_t vcu4_frame;
    uint8_t data[8];

    // Initialize the frame
    data_dbc_vcu_4_init(&vcu4_frame);

    // Populate with actual VCU data
    vcu4_frame.tcu_state = 0;                             // TCU state - replace with actual TCU state
    vcu4_frame.acu_state = (uint8_t)acu->mission_select;  // ACU state
    vcu4_frame.alc_state = 0;                             // ALC state - replace with actual ALC state
    // vcu4_frame.lv_soc = 0;                                // LV SOC - replace with actual LV SOC
    // vcu4_frame.lv_voltage = 0;                            // LV voltage - replace with actual LV voltage

    // Pack the data
    int pack_result = data_dbc_vcu_4_pack(data, &vcu4_frame, sizeof(data));
    if (pack_result > 0) {
        can_bus_send(hcan, DATA_DBC_VCU_4_FRAME_ID, data, DATA_DBC_VCU_4_LENGTH);
    }
}

/**
 * @brief Send all VCU frames to the data bus
 * @param hcan CAN handle for the data bus
 * @param hv500 Pointer to FSIC_t struct containing inverter data
 * @param bms Pointer to BMSvars_t struct containing BMS data
 * @param acu Pointer to ACU_t struct containing autonomous control unit data
 * @note send_vcu_3 is excluded as it requires VCU state parameters
 */
void send_all_vcu_frames(CAN_HandleTypeDef *hcan, const FSIC_t *hv500, const BMSvars_t *bms, const ACU_t *acu) {
    send_vcu_0(hcan, hv500);
    send_vcu_1(hcan, hv500, bms);
    send_vcu_2(hcan, hv500);
    // send_vcu_3 excluded - requires VCU state parameters (r2d_manual, ignition_manual, r2d_auto, ignition_auto)
    send_vcu_4(hcan, acu);
}

/**
 * @brief Send brake pressure data to CAN bus
 * @param hcan CAN handle for the target bus
 * @param brake_pressure Brake pressure value (0-255 or scaled as needed)
 */
void can_bus_send_brake_pressure(CAN_HandleTypeDef *hcan, uint16_t brake_pressure) {
    can_data_t data;
    data.id = 0x25;  // Brake pressure CAN ID - adjust as needed
    data.length = 1;

    memset(data.message, 0x00, sizeof(data.message));

    // Pack brake pressure as 16-bit value (little endian)
    data.message[0] = brake_pressure;

    can_bus_send(hcan, data.id, data.message, data.length);
}

void can_bus_send_vcu_apps_raw(CAN_HandleTypeDef *hcan, uint8_t apps1_raw, uint8_t apps2_raw, uint8_t apps_delta_raw, uint8_t cpu_temp, uint8_t flag_digital_bspd, uint8_t apps_error_type, int16_t apps_1000) {
    can_data_t data;
    data.id = 0x610;  // VCU Apps Raw CAN ID - adjust as needed
    data.length = 8;

    memset(data.message, 0x00, sizeof(data.message));

    // Pack the data into the message
    data.message[0] = apps1_raw;                           // Apps1 raw value
    data.message[1] = apps2_raw;                           // Apps2 raw value
    data.message[2] = apps_delta_raw;                      // Apps delta raw value
    data.message[3] = cpu_temp;                            // CPU temperature
    data.message[4] = flag_digital_bspd;                   // Digital BSPD flag
    data.message[5] = apps_error_type;                     // Apps error type
    data.message[6] = (uint8_t)(apps_1000 & 0xFF);         // Low byte of apps_1000
    data.message[7] = (uint8_t)((apps_1000 >> 8) & 0xFF);  // High byte of apps_1000

    can_bus_send(hcan, data.id, data.message, data.length);
}

void can_bus_send_vcu_state(void) {
    // Send VCU_state DBC message on CAN2 (via TX queue) 
    struct powertrain_t26_vcu_states_t state_msg; 
    powertrain_t26_vcu_states_init(&state_msg); 
    state_msg.vcu_state = (uint8_t)current_state; 
 
    uint8_t state_data[8]; 
    int pack_size = powertrain_t26_vcu_states_pack(state_data, &state_msg, sizeof(state_data)); 
    if (pack_size >= 0) { 
        can_msg_t state_can_msg = { 
            .id = POWERTRAIN_T26_VCU_STATES_FRAME_ID, 
            .dlc = POWERTRAIN_T26_VCU_STATES_LENGTH, 
            .bus = CAN_BUS_2, 
            .timestamp = HAL_GetTick(), 
        }; 
        memcpy(state_can_msg.data, state_data, 8); 
        can_tx_enqueue(&state_can_msg); 
    } 
}
