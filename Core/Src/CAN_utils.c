#include "CAN_utils.h"

#include <string.h>  // Add this for memset function

// #include "../../CAN_DBC/generated/Autonomous_temporary/autonomous_temporary.h"
#include "../../Can-Header-Map/CAN_asdb.h"
#include "../../Can-Header-Map/CAN_datadb.h"
#include "../../Can-Header-Map/CAN_pwtdb.h"
#include "autonomous_temporary.h"

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

// read for multiple can bus
void can_bus_read_DATADB(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, data);

    switch (RxHeader.StdId) {
        default:
            break;
    }
}

// Implement can_bus_read_ASDB function to handle messages from autonomous system
void can_bus_read_ASDB(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, data);

    switch (RxHeader.StdId) {
        case AUTONOMOUS_TEMPORARY_ACU_MS_FRAME_ID:
            // Handle ACU Mission Select message
            struct autonomous_temporary_acu_ms_t acu_ms;
            autonomous_temporary_acu_ms_unpack(&acu_ms, data, sizeof(data));
            // Store mission select value for later use
            // E.g.: as_system.mission_select = acu_ms.mission_select;
            break;

        case AUTONOMOUS_TEMPORARY_JETSON_MS_FRAME_ID:
            // Handle Jetson Mission Select message
            struct autonomous_temporary_jetson_ms_t jetson_ms;
            autonomous_temporary_jetson_ms_unpack(&jetson_ms, data, sizeof(data));
            // Store mission select value for later use
            break;

        case AUTONOMOUS_TEMPORARY_TARGET_RPM_FRAME_ID:
            // Handle Target RPM message from autonomous system
            struct autonomous_temporary_target_rpm_t target_rpm;
            autonomous_temporary_target_rpm_unpack(&target_rpm, data, sizeof(data));
            // Store target RPM for use in autonomous mode
            // E.g.: as_system.target_rpm = target_rpm.rpm;
            break;

        case AUTONOMOUS_TEMPORARY_ACU_IGN_FRAME_ID:
            // Handle ACU Ignition message
            struct autonomous_temporary_acu_ign_t acu_ign;
            autonomous_temporary_acu_ign_unpack(&acu_ign, data, sizeof(data));
            // Update VCU signals based on ignition state
            // vcu.ignition_ad = acu_ign.ign;
            break;

        case AUTONOMOUS_TEMPORARY_RD_JETSON_FRAME_ID:
            // Handle Ready to Drive signal from Jetson
            struct autonomous_temporary_rd_jetson_t rd_jetson;
            autonomous_temporary_rd_jetson_unpack(&rd_jetson, data, sizeof(data));
            // Update R2D signal from autonomous system
            // vcu.r2d_autonomous_signal = (rd_jetson.rd > 0) ? true : false;
            break;

        case AUTONOMOUS_TEMPORARY_AS_STATE_FRAME_ID:
            // Handle Autonomous System State message
            struct autonomous_temporary_as_state_t as_state;
            autonomous_temporary_as_state_unpack(&as_state, data, sizeof(data));
            // Process autonomous system state
            // E.g.: as_system.state = as_state.state;
            break;

        case AUTONOMOUS_TEMPORARY_RES_FRAME_ID:
            // Handle Response message
            struct autonomous_temporary_res_t res;
            autonomous_temporary_res_unpack(&res, data, sizeof(data));
            // Check for emergency signal
            if (res.signal == AUTONOMOUS_TEMPORARY_RES_SIGNAL_EMERGENCY_CHOICE) {
                // vcu.AS_emergency = true;
                // HAL_GPIO_WritePin(LED_DATA_GPIO_Port, LED_DATA_Pin, GPIO_PIN_RESET);

            } else if (res.signal == AUTONOMOUS_TEMPORARY_RES_SIGNAL_GO_SIGNAL_CHOICE ||
                       res.signal == AUTONOMOUS_TEMPORARY_RES_SIGNAL_GO_SIGNAL_2_CHOICE) {
                // vcu.AS_emergency = false;

                // blink data led
                // HAL_GPIO_WritePin(LED_DATA_GPIO_Port, LED_DATA_Pin, GPIO_PIN_SET);
            }
            break;

        default:
            // Unknown message ID
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

void can_filter_id_bus2(CAN_RxHeaderTypeDef RxHeader, uint8_t *data) {
    switch (RxHeader.StdId) {
        case CAN_HV500_ERPM_DUTY_VOLTAGE_ID:
            myHV500.Actual_ERPM = MAP_DECODE_Actual_ERPM(data);
            //printf("Actual ERPM: %ld\n", myHV500.Actual_ERPM);
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
 * @brief Sends rpm to the autonomous system
 *
 * @param state The state of the autonomous system
 * @param hcan CAN handle for the VCU bus (CAN3)
 */
void can_send_vcu_rpm(CAN_HandleTypeDef *hcan, uint32_t rpm) {
    // Scale down the RPM value
    rpm = rpm / 10;

    // Clamp to uint16_t range for safety
    if (rpm > 65535) {
        rpm = 65535;
    } else if (rpm <= 0) {
        rpm = 0;
    }

    /*
        struct autonomous_temporary_vcu_rpm_t vcu_rpm_msg;
    uint8_t data[8];

    autonomous_temporary_vcu_rpm_init(&vcu_rpm_msg);
    vcu_rpm_msg.rpm = rpm;  // Convert long to uint16_t

    uint8_t error = autonomous_temporary_vcu_rpm_pack(data, &vcu_rpm_msg, sizeof(data));
    if (error > 0) {
        can_bus_send(hcan, AUTONOMOUS_TEMPORARY_VCU_RPM_FRAME_ID, data, AUTONOMOUS_TEMPORARY_VCU_RPM_LENGTH);
    }
    */

    uint16_t rpm_u16 = (uint16_t)rpm;

    //printf("RPM: %ld , RPM_uint16_t: %d \n", rpm, rpm_u16);
    // Perdu é gay
    can_data_t data;
    data.id = 0x510;
    data.length = 2;
    data.message[0] = rpm_u16 & 0xFF;         // Low byte (LSB first)
    data.message[1] = (rpm_u16 >> 8) & 0xFF;  // High byte (MSB second)
    can_bus_send(hcan, data.id, data.message, data.length);
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
}

/**
 * @brief Sends the emergency state to the autonomous system
 *
 * @param hcan CAN handle for the VCU bus (CAN3)
 */
void decode_auto_bus(CAN_RxHeaderTypeDef RxHeader, uint8_t *data) {
    uint8_t dlc_bits = RxHeader.DLC * 8;
    switch (RxHeader.StdId) {
        case AUTONOMOUS_TEMPORARY_ACU_MS_FRAME_ID:
            struct autonomous_temporary_acu_ms_t acu_ms;
            autonomous_temporary_acu_ms_init(&acu_ms);
            acu_ms.mission_select = 0;
            autonomous_temporary_acu_ms_unpack(&acu_ms, data, dlc_bits);
            acu.mission_select = acu_ms.mission_select;
            break;
        case AUTONOMOUS_TEMPORARY_ACU_IGN_FRAME_ID:
            struct autonomous_temporary_acu_ign_t acu_ign;
            autonomous_temporary_acu_ign_init(&acu_ign);
            acu_ign.ign = 0;
            autonomous_temporary_acu_ign_unpack(&acu_ign, data, dlc_bits);
            acu.ignition_ad = acu_ign.ign;
            acu.ASMS = acu_ign.asms;
            acu.is_in_emergency = acu_ign.emergency;
            break;
        case AUTONOMOUS_TEMPORARY_JETSON_MS_FRAME_ID:
            struct autonomous_temporary_jetson_ms_t jetson_ms;
            autonomous_temporary_jetson_ms_init(&jetson_ms);
            jetson_ms.mission_select = 0;
            autonomous_temporary_jetson_ms_unpack(&jetson_ms, data, dlc_bits);
            as_system.mission_select = jetson_ms.mission_select;
            break;
        case AUTONOMOUS_TEMPORARY_RD_JETSON_FRAME_ID:
            struct autonomous_temporary_rd_jetson_t rd_jetson;
            autonomous_temporary_rd_jetson_init(&rd_jetson);
            rd_jetson.rd = 0;
            autonomous_temporary_rd_jetson_unpack(&rd_jetson, data, dlc_bits);
            as_system.ready_to_drive_ad = rd_jetson.rd;
            break;
        case AUTONOMOUS_TEMPORARY_RES_FRAME_ID:
            struct autonomous_temporary_res_t res_ad;
            autonomous_temporary_res_init(&res_ad);
            res_ad.signal = 0;
            autonomous_temporary_res_unpack(&res_ad, data, dlc_bits);
            res.signal = res_ad.signal;
            break;
        case AUTONOMOUS_TEMPORARY_TARGET_RPM_FRAME_ID:
            struct autonomous_temporary_target_rpm_t target_rpm;
            autonomous_temporary_target_rpm_init(&target_rpm);
            target_rpm.rpm = 0;
            autonomous_temporary_target_rpm_unpack(&target_rpm, data, dlc_bits);
            as_system.target_rpm = target_rpm.rpm;
            break;
        case AUTONOMOUS_TEMPORARY_AS_STATE_FRAME_ID:
            struct autonomous_temporary_as_state_t as_state;
            autonomous_temporary_as_state_init(&as_state);
            as_state.state = 0;
            autonomous_temporary_as_state_unpack(&as_state, data, dlc_bits);
            as_system.state = as_state.state;
            break;

        default:
            break;
    }
}
