#include "CAN_utils.h"

#include <string.h>  // Add this for memset function

#include "../../Can-Header-Map/CAN_asdb.h"
#include "../../Can-Header-Map/CAN_datadb.h"
#include "../../Can-Header-Map/CAN_pwtdb.h"




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

/*
=========================================================
                POWERTRAIN DECODE
=========================================================
*/

void can_filter_id_bus2(can_data_t *data) {
    switch (data->id) {
        case CAN_HV500_ERPM_DUTY_VOLTAGE_ID:
            myHV500.Actual_ERPM = MAP_DECODE_Actual_ERPM(data->message);
            myHV500.Actual_Duty = MAP_DECODE_Actual_Duty(data->message);
            myHV500.Actual_InputVoltage = MAP_DECODE_Actual_InputVoltage(data->message);

            break;
        case CAN_HV500_AC_DC_current_ID:

            myHV500.Actual_ACCurrent = MAP_DECODE_Actual_ACCurrent(data->message);
            myHV500.Actual_DCCurrent = MAP_DECODE_Actual_DCCurrent(data->message);
            break;
        case CAN_HV500_Temperatures_ID:
            myHV500.Actual_TempController = MAP_DECODE_Actual_TempController(data->message);
            myHV500.Actual_TempMotor = MAP_DECODE_Actual_TempMotor(data->message);
            myHV500.Actual_FaultCode = MAP_DECODE_Actual_FaultCode(data->message);
            break;
        case CAN_HV500_FOC_ID:
            myHV500.Actual_FOC_id = MAP_DECODE_Actual_FOC_id(data->message);
            myHV500.Actual_FOC_iq = MAP_DECODE_Actual_FOC_iq(data->message);
            break;
        case CAN_HV500_MISC_ID:
            myHV500.Actual_Throttle = MAP_DECODE_Actual_Throttle(data->message);
            myHV500.Actual_Brake = MAP_DECODE_Actual_Brake(data->message);
            myHV500.Digital_input_1 = MAP_DECODE_Digital_input_1(data->message);
            myHV500.Digital_input_2 = MAP_DECODE_Digital_input_2(data->message);
            myHV500.Digital_input_3 = MAP_DECODE_Digital_input_3(data->message);
            myHV500.Digital_input_4 = MAP_DECODE_Digital_input_4(data->message);
            myHV500.Digital_output_1 = MAP_DECODE_Digital_output_1(data->message);
            myHV500.Digital_output_2 = MAP_DECODE_Digital_output_2(data->message);
            myHV500.Digital_output_3 = MAP_DECODE_Digital_output_3(data->message);
            myHV500.Digital_output_4 = MAP_DECODE_Digital_output_4(data->message);
            myHV500.Drive_enable = MAP_DECODE_Drive_enable(data->message);
            myHV500.Capacitor_temp_limit = MAP_DECODE_Capacitor_temp_limit(data->message);
            myHV500.DC_current_limit = MAP_DECODE_DC_current_limit(data->message);
            myHV500.Drive_enable_limit = MAP_DECODE_Drive_enable_limit(data->message);
            myHV500.IGBT_accel_limit = MAP_DECODE_IGBT_accel_limit(data->message);
            myHV500.IGBT_temp_limit = MAP_DECODE_IGBT_temp_limit(data->message);
            myHV500.Input_voltage_limit = MAP_DECODE_Input_voltage_limit(data->message);
            myHV500.Motor_accel_limit = MAP_DECODE_Motor_accel_limit(data->message);
            myHV500.Motor_temp_limit = MAP_DECODE_Motor_temp_limit(data->message);
            myHV500.RPM_min_limit = MAP_DECODE_RPM_min_limit(data->message);
            myHV500.RPM_max_limit = MAP_DECODE_RPM_max_limit(data->message);
            myHV500.Power_limit = MAP_DECODE_Power_limit(data->message);
            myHV500.CAN_map_version = MAP_DECODE_CAN_map_version(data->message);
            break;

        case CAN_TCU_ID_2:
            // tcu.TCU_STATE = MAP_DECODE_TCU_STATE(data->message);
            // tcu.Precharge_done = MAP_DECODE_PRECHARGE_DONE(data->message);
            // tcu.SDC_State = MAP_DECODE_SDC_STATE(data->message);
            break;

        case CAN_PWT_BMS_ID_1:
            bms.instant_voltage = MAP_DECODE_PWT_BMS_PACK_INSTANT_VOLTAGE(data->message);
            bms.soc = MAP_DECODE_PWT_BMS_PACK_SOC(data->message);
            break;
        case CAN_PWT_BMS_ID_2:
            bms.high_cell_voltage = MAP_DECODE_PWT_BMS_PACK_HIGH_CELL_VOLTAGE(data->message);
            bms.low_cell_voltage = MAP_DECODE_PWT_BMS_PACK_LOW_CELL_VOLTAGE(data->message);
            bms.avg_cell_voltage = MAP_DECODE_PWT_BMS_PACK_AVG_CELL_VOLTAGE(data->message);
            break;
        case CAN_PWT_BMS_ID_3:
            bms.high_cell_temp = MAP_DECODE_PWT_BMS_PACK_HIGH_CELL_TEMP(data->message);
            bms.low_cell_temp = MAP_DECODE_PWT_BMS_PACK_LOW_CELL_TEMP(data->message);
            break;

        default:
            break;
    }
}