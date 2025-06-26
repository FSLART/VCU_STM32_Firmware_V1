#include "CAN_utils.h"

#include <string.h>  // Add this for memset function

// #include "../../CAN_DBC/generated/Autonomous_temporary/autonomous_temporary.h"
#include "../../Can-Header-Map/CAN_asdb.h"
#include "../../Can-Header-Map/CAN_datadb.h"
#include "../../Can-Header-Map/CAN_pwtdb.h"
#include "autonomous_temporary.h"
#include "data_dbc.h"

void can_bus_send(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint8_t len) {
    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.StdId = id;
    TxHeader.ExtId = 0;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = len;
    TxHeader.TransmitGlobalTime = DISABLE;

    uint32_t TxMailbox;

    HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &TxMailbox);
}

// Process DATADB messages (header and data already retrieved in callback)
void decode_DATA_DB(CAN_RxHeaderTypeDef RxHeader, uint8_t *data) {
    switch (RxHeader.StdId) {
        // Add your data bus message processing here
        // Example message IDs from CAN_datadb.h:
        // case CAN_VCU_ID_1:
        //     // Process VCU message
        //     break;
        // case CAN_PDM_ID_1:
        //     // Process PDM message
        //     break;
        default:
            break;
    }
}

/*
░█████╗░░█████╗░███╗░░██╗  ██████╗░  ░░░░░░██████╗░░█████╗░░██╗░░░░░░░██╗███████╗██████╗░████████╗██████╗░░█████╗░██╗███╗░░██╗
██╔══██╗██╔══██╗████╗░██║  ╚════██╗  ░░░░░░██╔══██╗██╔══██╗░██║░░██╗░░██║██╔════╝██╔══██╗╚══██╔══╝██╔══██╗██╔══██╗██║████╗░██║
██║░░╚═╝███████║██╔██╗██║  ░░███╔═╝  █████╗██████╔╝██║░░██║░╚██╗████╗██╔╝█████╗░░██████╔╝░░░██║░░░██████╔╝███████║██║██╔██╗██║
██║░░██╗██╔══██║██║╚████║  ██╔══╝░░  ╚════╝██╔═══╝░██║░░██║░░████╔═████║░██╔══╝░░██╔══██╗░░░██║░░░██╔══██╗██╔══██║██║██║╚████║
╚█████╔╝██║░░██║██║░╚███║  ███████╗  ░░░░░░██║░░░░░╚█████╔╝░░╚██╔╝░╚██╔╝░███████╗██║░░██║░░░██║░░░██║░░██║██║░░██║██║██║░╚███║
░╚════╝░╚═╝░░╚═╝╚═╝░░╚══╝  ╚══════╝  ░░░░░░╚═╝░░░░░░╚════╝░░░░╚═╝░░░╚═╝░░╚══════╝╚═╝░░╚═╝░░░╚═╝░░░╚═╝░░╚═╝╚═╝░░╚═╝╚═╝╚═╝░░╚══╝
*/

/*Send_CAN_HV500_SetAcCurrent*/
void can_bus_send_HV500_SetAcCurrent(uint16_t ac_current, CAN_HandleTypeDef *hcan) {
    can_data_t data;
    data.id = CAN_HV500_SetAcCurrent_ID;
    data.length = 8;

    memset(data.message, 0x00, sizeof(data.message));

    MAP_ENCODE_CMD_AcCurrent(data.message, ac_current);

    can_bus_send(hcan, data.id, data.message, data.length);
}

/*Send_CAN_HV500_SetBrakeCurrent*/
void can_bus_send_HV500_SetBrakeCurrent(uint16_t brake_current, CAN_HandleTypeDef *hcan) {
    can_data_t data;
    data.id = CAN_HV500_SetBrakeCurrent_ID;
    data.length = 8;

    memset(data.message, 0x00, sizeof(data.message));

    MAP_ENCODE_CMD_BrakeCurrent(data.message, brake_current);

    can_bus_send(hcan, data.id, data.message, data.length);
}

/*Send_CAN_HV500_SetERPM*/
void can_bus_send_HV500_SetERPM(uint32_t erpm, CAN_HandleTypeDef *hcan) {
    can_data_t data;
    data.id = CAN_HV500_SetERPM_ID;
    data.length = 4;

    memset(data.message, 0x00, sizeof(data.message));

    MAP_ENCODE_CMD_ERPM(data.message, erpm);

    can_bus_send(hcan, data.id, data.message, data.length);
}

/*Send_CAN_HV500_SetPosition*/
void can_bus_send_HV500_SetPosition(uint32_t position, CAN_HandleTypeDef *hcan) {
    can_data_t data;
    data.id = CAN_HV500_SetPosition_ID;
    data.length = 8;

    memset(data.message, 0x00, sizeof(data.message));

    MAP_ENCODE_CMD_Position(data.message, position);

    can_bus_send(hcan, data.id, data.message, data.length);
}

/*Send_CAN_HV500_SetRelCurrent*/
void can_bus_send_HV500_SetRelCurrent(uint32_t rel_current, CAN_HandleTypeDef *hcan) {
    can_data_t data;
    data.id = CAN_HV500_SetRelCurrent_ID;
    data.length = 2;

    memset(data.message, 0x00, sizeof(data.message));

    MAP_ENCODE_CMD_RelCurrent(data.message, rel_current);

    can_bus_send(hcan, data.id, data.message, data.length);
}

/*Send_CAN_HV500_SetRelBrakeCurrent*/
void can_bus_send_HV500_SetRelBrakeCurrent(uint32_t rel_brake_current, CAN_HandleTypeDef *hcan) {
    can_data_t data;
    data.id = CAN_HV500_SetRelBrakeCurrent_ID;
    data.length = 8;

    memset(data.message, 0x00, sizeof(data.message));

    MAP_ENCODE_CMD_RelBrakeCurrent(data.message, rel_brake_current);

    can_bus_send(hcan, data.id, data.message, data.length);
}

/*Send_CAN_HV500_SetMaxAcCurrent*/
void can_bus_send_HV500_SetMaxAcCurrent(uint32_t max_ac_current, CAN_HandleTypeDef *hcan) {
    can_data_t data;
    data.id = CAN_HV500_SetMaxAcCurrent_ID;
    data.length = 8;

    memset(data.message, 0x00, sizeof(data.message));

    MAP_ENCODE_CMD_MaxAcCurrent(data.message, max_ac_current);

    can_bus_send(hcan, data.id, data.message, data.length);
}

/*Send_CAN_HV500_SetMaxAcBrakeCurrent*/
void can_bus_send_HV500_SetMaxAcBrakeCurrent(uint32_t max_ac_brake_current, CAN_HandleTypeDef *hcan) {
    can_data_t data;
    data.id = CAN_HV500_SetMaxAcBrakeCurrent_ID;
    data.length = 8;

    memset(data.message, 0x00, sizeof(data.message));

    MAP_ENCODE_CMD_MaxAcBrakeCurrent(data.message, max_ac_brake_current);

    can_bus_send(hcan, data.id, data.message, data.length);
}

/*CAN_HV500_SetMaxDcCurrent_ID*/
void can_bus_send_HV500_SetMaxDcCurrent(uint32_t max_dc_current, CAN_HandleTypeDef *hcan) {
    can_data_t data;
    data.id = CAN_HV500_SetMaxDcCurrent_ID;
    data.length = 8;

    memset(data.message, 0x00, sizeof(data.message));

    MAP_ENCODE_CMD_MaxDcCurrent(data.message, max_dc_current);

    can_bus_send(hcan, data.id, data.message, data.length);
}

/*CAN_HV500_SetMaxDcBrakeCurrent_ID*/
void can_bus_send_HV500_SetMaxDcBrakeCurrent(uint32_t max_dc_brake_current, CAN_HandleTypeDef *hcan) {
    can_data_t data;
    data.id = CAN_HV500_SetMaxDcBrakeCurrent_ID;
    data.length = 8;

    memset(data.message, 0x00, sizeof(data.message));

    //    MAP_ENCODE_CMD_MaxDcBrakeCurrent(data.message, max_dc_brake_current);

    can_bus_send(hcan, data.id, data.message, data.length);
}

/*Send_CAN_HV500_SetDriveEnable*/
void can_bus_send_HV500_SetDriveEnable(uint32_t drive_enable, CAN_HandleTypeDef *hcan) {
    can_data_t data;
    data.id = CAN_HV500_SetDriveEnable_ID;
    data.length = 1;

    memset(data.message, 0x00, sizeof(data.message));

    MAP_ENCODE_CMD_DriveEnable(data.message, drive_enable);

    can_bus_send(hcan, data.id, data.message, data.length);
}

void can_bus_send_pwtbus_1(uint8_t r2d, uint8_t ignition, CAN_HandleTypeDef *hcan) {
    can_data_t data;
    data.id = CAN_PWT_VCU_ID_1;
    data.length = 8;

    memset(data.message, 0x00, sizeof(data.message));

    MAP_ENCODE_PWT_R2D_STATE(data.message, r2d);
    MAP_ENCODE_PWT_IGNITION_STATE(data.message, ignition);

    can_bus_send(hcan, data.id, data.message, data.length);
}

void can_bus_send_bms_precharge_state(uint8_t precharge_state, CAN_HandleTypeDef *hcan) {
    can_data_t data;
    data.id = 0x83;
    data.length = 1;

    memset(data.message, 0x00, sizeof(data.message));

    data.message[0] = precharge_state;

    can_bus_send(hcan, data.id, data.message, data.length);
}

/*
=========================================================
                POWERTRAIN DECODE
=========================================================
*/

void decode_PWT_DB(CAN_RxHeaderTypeDef RxHeader, uint8_t *data) {
    switch (RxHeader.StdId) {
        case CAN_HV500_ERPM_DUTY_VOLTAGE_ID:
            myHV500.Actual_ERPM = MAP_DECODE_Actual_ERPM(data);
            myHV500.Actual_Duty = MAP_DECODE_Actual_Duty(data);
            myHV500.Actual_InputVoltage = MAP_DECODE_Actual_InputVoltage(data);
            break;

        case CAN_HV500_AC_DC_current_ID:
            myHV500.Actual_ACCurrent = MAP_DECODE_Actual_ACCurrent(data);
            myHV500.Actual_DCCurrent = MAP_DECODE_Actual_DCCurrent(data);
            break;

        case CAN_HV500_Temperatures_ID:
            myHV500.Actual_TempController = MAP_DECODE_Actual_TempController(data);
            myHV500.Actual_TempMotor = MAP_DECODE_Actual_TempMotor(data);
            myHV500.Actual_FaultCode = MAP_DECODE_Actual_FaultCode(data);
            break;

        case CAN_HV500_FOC_ID:
            myHV500.Actual_FOC_id = MAP_DECODE_Actual_FOC_id(data);
            myHV500.Actual_FOC_iq = MAP_DECODE_Actual_FOC_iq(data);
            break;

        case CAN_HV500_MISC_ID:
            myHV500.Actual_Throttle = MAP_DECODE_Actual_Throttle(data);
            myHV500.Actual_Brake = MAP_DECODE_Actual_Brake(data);
            myHV500.Digital_input_1 = MAP_DECODE_Digital_input_1(data);
            myHV500.Digital_input_2 = MAP_DECODE_Digital_input_2(data);
            myHV500.Digital_input_3 = MAP_DECODE_Digital_input_3(data);
            myHV500.Digital_input_4 = MAP_DECODE_Digital_input_4(data);
            myHV500.Digital_output_1 = MAP_DECODE_Digital_output_1(data);
            myHV500.Digital_output_2 = MAP_DECODE_Digital_output_2(data);
            myHV500.Digital_output_3 = MAP_DECODE_Digital_output_3(data);
            myHV500.Digital_output_4 = MAP_DECODE_Digital_output_4(data);
            myHV500.Drive_enable = MAP_DECODE_Drive_enable(data);
            myHV500.Capacitor_temp_limit = MAP_DECODE_Capacitor_temp_limit(data);
            myHV500.DC_current_limit = MAP_DECODE_DC_current_limit(data);
            myHV500.Drive_enable_limit = MAP_DECODE_Drive_enable_limit(data);
            myHV500.IGBT_accel_limit = MAP_DECODE_IGBT_accel_limit(data);
            myHV500.IGBT_temp_limit = MAP_DECODE_IGBT_temp_limit(data);
            myHV500.Input_voltage_limit = MAP_DECODE_Input_voltage_limit(data);
            myHV500.Motor_accel_limit = MAP_DECODE_Motor_accel_limit(data);
            myHV500.Motor_temp_limit = MAP_DECODE_Motor_temp_limit(data);
            myHV500.RPM_min_limit = MAP_DECODE_RPM_min_limit(data);
            myHV500.RPM_max_limit = MAP_DECODE_RPM_max_limit(data);
            myHV500.Power_limit = MAP_DECODE_Power_limit(data);
            myHV500.CAN_map_version = MAP_DECODE_CAN_map_version(data);
            break;

        case CAN_PWT_BMS_ID_1:
            bms.instant_voltage = MAP_DECODE_PWT_BMS_PACK_INSTANT_VOLTAGE(data);
            bms.soc = MAP_DECODE_PWT_BMS_PACK_SOC(data);
            break;

        case CAN_PWT_BMS_ID_2:
            bms.high_cell_voltage = MAP_DECODE_PWT_BMS_PACK_HIGH_CELL_VOLTAGE(data);
            bms.low_cell_voltage = MAP_DECODE_PWT_BMS_PACK_LOW_CELL_VOLTAGE(data);
            bms.avg_cell_voltage = MAP_DECODE_PWT_BMS_PACK_AVG_CELL_VOLTAGE(data);
            break;

        case CAN_PWT_BMS_ID_3:
            bms.high_cell_temp = MAP_DECODE_PWT_BMS_PACK_HIGH_CELL_TEMP(data);
            bms.low_cell_temp = MAP_DECODE_PWT_BMS_PACK_LOW_CELL_TEMP(data);
            bms.precharge_circuit_state = data[6];
            break;

        default:
            break;
    }
}

/*======================================= Autonomous bus =======================================*/

/**
 * @brief Sends RPM to the autonomous system
 *
 * @param hcan CAN handle for the VCU bus (CAN3)
 * @param rpm RPM value to send (0-100 RPM range)
 */
void can_send_vcu_rpm(CAN_HandleTypeDef *hcan, long rpm) {
    struct autonomous_temporary_vcu_rpm_t vcu_rpm_msg;
    uint8_t data[8];

    // Initialize the message structure
    autonomous_temporary_vcu_rpm_init(&vcu_rpm_msg);

    vcu_rpm_msg.rpm = (uint16_t)rpm;

    // Pack the message and check for errors
    int pack_result = autonomous_temporary_vcu_rpm_pack(data, &vcu_rpm_msg, sizeof(data));
    if (pack_result > 0) {
        // Send the message only if packing was successful
        can_bus_send(hcan, AUTONOMOUS_TEMPORARY_VCU_RPM_FRAME_ID, data, AUTONOMOUS_TEMPORARY_VCU_RPM_LENGTH);
    }
}

/**
 * @brief Sends the HV signal to the autonomous system
 *
 * @param hcan CAN handle for the VCU bus (CAN3)
 * @param hv_state The state of the HV (0 or 1)
 */
void can_send_autonomous_HV_signal(CAN_HandleTypeDef *hcan, uint8_t hv_state) {
    struct autonomous_temporary_vcu_hv_t hv_signal_msg;
    uint8_t data[8];
    data[0] = hv_state;
    data[1] = 0;
    data[2] = 0;
    // autonomous_temporary_vcu_hv_init(&hv_signal_msg);
    // hv_signal_msg.hv = hv_state;  // Set the HV state
    // autonomous_temporary_vcu_hv_pack(data, &hv_signal_msg, sizeof(data));
    can_bus_send(hcan, AUTONOMOUS_TEMPORARY_VCU_HV_FRAME_ID, data, 3);
    // can_bus_send(hcan, 0x100, data, AUTONOMOUS_TEMPORARY_VCU_HV_LENGTH);
    // can_bus_send(hcan, 0x420, data, AUTONOMOUS_TEMPORARY_VCU_HV_LENGTH);
}

/**
 * @brief Sends the emergency state to the autonomous system
 *
 * @param hcan CAN handle for the VCU bus (CAN3)
 */
void decode_AUTO_DB(CAN_RxHeaderTypeDef RxHeader, uint8_t *data) {
    uint8_t dlc_bits = RxHeader.DLC * 8;
    switch (RxHeader.StdId) {
        case AUTONOMOUS_TEMPORARY_ACU_MS_FRAME_ID:
            struct autonomous_temporary_acu_ms_t acu_ms;

            int unpack_result_ms = autonomous_temporary_acu_ms_unpack(&acu_ms, data, dlc_bits);
            if (unpack_result_ms == 0) {
                acu.mission_select = acu_ms.mission_select;
            }
            break;
        case AUTONOMOUS_TEMPORARY_ACU_IGN_FRAME_ID:
            struct autonomous_temporary_acu_ign_t acu_ign;

            int unpack_result_ign = autonomous_temporary_acu_ign_unpack(&acu_ign, data, dlc_bits);
            if (unpack_result_ign == 0) {
                acu.ignition_ad = acu_ign.ign;
                acu.ASMS = acu_ign.asms;
                acu.is_in_emergency = acu_ign.emergency;
            }
            break;
        case AUTONOMOUS_TEMPORARY_JETSON_MS_FRAME_ID:
            struct autonomous_temporary_jetson_ms_t jetson_ms;

            int unpack_result_jetson_ms = autonomous_temporary_jetson_ms_unpack(&jetson_ms, data, dlc_bits);
            if (unpack_result_jetson_ms == 0) {
                as_system.mission_select = jetson_ms.mission_select;
            }
            break;
        case AUTONOMOUS_TEMPORARY_RD_JETSON_FRAME_ID:
            struct autonomous_temporary_rd_jetson_t rd_jetson;

            int unpack_result_rd = autonomous_temporary_rd_jetson_unpack(&rd_jetson, data, dlc_bits);
            if (unpack_result_rd == 0) {
                as_system.ready_to_drive_ad = rd_jetson.rd;
            }
            break;
        case AUTONOMOUS_TEMPORARY_RES_FRAME_ID:
            struct autonomous_temporary_res_t res_ad;

            int unpack_result_res = autonomous_temporary_res_unpack(&res_ad, data, dlc_bits);
            if (unpack_result_res == 0) {
                res.signal = res_ad.signal;
            }
            break;
        case AUTONOMOUS_TEMPORARY_TARGET_RPM_FRAME_ID:
            struct autonomous_temporary_target_rpm_t target_rpm;

            int unpack_result_rpm = autonomous_temporary_target_rpm_unpack(&target_rpm, data, dlc_bits);
            if (unpack_result_rpm == 0) {
                as_system.target_rpm = target_rpm.rpm;
            }
            break;
        case AUTONOMOUS_TEMPORARY_AS_STATE_FRAME_ID:
            struct autonomous_temporary_as_state_t as_state;

            int unpack_result_state = autonomous_temporary_as_state_unpack(&as_state, data, dlc_bits);
            if (unpack_result_state == 0) {
                as_system.state = as_state.state;
            }
            break;

        default:
            break;
    }
}

void can_send_st_wheel_data(CAN_HandleTypeDef *hcan, uint16_t apps, uint16_t brake, uint16_t inv_temp, uint16_t motor_temp, uint16_t bms_voltage, uint16_t soc_hv, uint16_t apps_error, uint16_t inv_voltage, uint16_t rpm, uint16_t ign_signal, uint16_t r2d_signal) {
    uint8_t data[8];

    // Send VCU_ message (0x20) - APPS and BPS data
    struct data_dbc_vcu__t vcu_msg;
    data_dbc_vcu__init(&vcu_msg);
    vcu_msg.apps = (uint8_t)apps;
    vcu_msg.bps = (uint8_t)brake;
    vcu_msg.trgt_power = 0;  // Not provided in parameters
    vcu_msg.cnsm_power = 0;  // Not provided in parameters

    if (data_dbc_vcu__pack(data, &vcu_msg, sizeof(data)) > 0) {
        can_bus_send(hcan, DATA_DBC_VCU__FRAME_ID, data, DATA_DBC_VCU__LENGTH);
    }

    // Send VCU_1 message (0x21) - Temperatures, BMS voltage, SOC
    struct data_dbc_vcu_1_t vcu1_msg;
    data_dbc_vcu_1_init(&vcu1_msg);
    vcu1_msg.inv_temperature = inv_temp;
    vcu1_msg.motor_temperature = motor_temp;
    vcu1_msg.bms_voltage = bms_voltage;
    vcu1_msg.soc_hv = (uint8_t)soc_hv;

    if (data_dbc_vcu_1_pack(data, &vcu1_msg, sizeof(data)) > 0) {
        can_bus_send(hcan, DATA_DBC_VCU_1_FRAME_ID, data, DATA_DBC_VCU_1_LENGTH);
    }

    // Send VCU_2 message (0x22) - APPS error and other states
    struct data_dbc_vcu_2_t vcu2_msg;
    data_dbc_vcu_2_init(&vcu2_msg);
    vcu2_msg.inv_faults = 0;  // Not provided in parameters
    vcu2_msg.lmt1 = 0;        // Not provided in parameters
    vcu2_msg.lmt2 = 0;        // Not provided in parameters
    vcu2_msg.vcu_state = 0;   // Not provided in parameters
    vcu2_msg.apps_error = (uint8_t)apps_error;
    vcu2_msg.power_plan = 0;  // Not provided in parameters

    if (data_dbc_vcu_2_pack(data, &vcu2_msg, sizeof(data)) > 0) {
        can_bus_send(hcan, DATA_DBC_VCU_2_FRAME_ID, data, DATA_DBC_VCU_2_LENGTH);
    }

    // Send VCU_3 message (0x23) - Inverter voltage, RPM, IGN, R2D
    struct data_dbc_vcu_3_t vcu3_msg;
    data_dbc_vcu_3_init(&vcu3_msg);
    vcu3_msg.inv_voltage = inv_voltage;
    vcu3_msg.rpm = rpm;
    vcu3_msg.ign = (uint8_t)ign_signal;
    vcu3_msg.r2_d = (uint8_t)r2d_signal;

    if (data_dbc_vcu_3_pack(data, &vcu3_msg, sizeof(data)) > 0) {
        can_bus_send(hcan, DATA_DBC_VCU_3_FRAME_ID, data, DATA_DBC_VCU_3_LENGTH);
    }
}
