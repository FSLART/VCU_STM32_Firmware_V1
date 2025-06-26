#ifndef CAN_UTILS_H
#define CAN_UTILS_H

#include <stdbool.h>

#include "main.h"
#include "stm32f7xx_hal.h"

/* External CAN handle declarations */
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern CAN_HandleTypeDef hcan3;

typedef struct {
    uint32_t id;
    uint8_t message[64];
    uint8_t length;
} can_data_t;

/*data stucture for interating with the HV500 inverter*/
typedef struct {
    // Received data
    long Actual_ERPM;
    uint32_t Actual_Duty;
    uint32_t Actual_InputVoltage;
    uint32_t Actual_ACCurrent;
    uint32_t Actual_DCCurrent;
    uint32_t Actual_TempController;
    uint32_t Actual_TempMotor;
    uint32_t Actual_FaultCode;
    uint32_t Actual_FOC_id;
    uint32_t Actual_FOC_iq;
    uint32_t Actual_Throttle;
    uint32_t Actual_Brake;
    uint32_t Digital_input_1;
    uint32_t Digital_input_2;
    uint32_t Digital_input_3;
    uint32_t Digital_input_4;
    uint32_t Digital_output_1;
    uint32_t Digital_output_2;
    uint32_t Digital_output_3;
    uint32_t Digital_output_4;
    uint32_t Drive_enable;
    uint32_t Capacitor_temp_limit;
    uint32_t DC_current_limit;
    uint32_t Drive_enable_limit;
    uint32_t IGBT_accel_limit;
    uint32_t IGBT_temp_limit;
    uint32_t Input_voltage_limit;
    uint32_t Motor_accel_limit;
    uint32_t Motor_temp_limit;
    uint32_t RPM_min_limit;
    uint32_t RPM_max_limit;
    uint32_t Power_limit;
    uint32_t CAN_map_version;

    uint16_t SetCurrent;
    uint16_t SetBrakeCurrent;
    uint32_t SetERPM;
    uint16_t SetPosition;
    uint16_t SetRelativeCurrent;
    uint16_t SetRelativeBrakeCurrent;
    uint32_t SetDigitalOutput;
    uint16_t SetMaxACCurrent;
    uint16_t SetMaxACBrakeCurrent;
    uint16_t SetMaxDCCurrent;
    uint16_t SetMaxDCBrakeCurrent;
    uint8_t DriveEnable;
} HV500;
extern volatile HV500 myHV500;

typedef enum {
    AS_STATE_OFF = 1,    // System is off or not initialized
    AS_STATE_READY = 2,  // System is ready to engage autonomous mode
    AS_STATE_DRIVING = 3,
    AS_STATE_EMERGENCY = 4,  // System is in emergency state
    AS_STATE_FINISHED = 5,   // Mission or task completed
} AS_State_t;

typedef struct {
    uint8_t mission_select;
    uint16_t target_rpm;
    // uint8_t state;
    AS_State_t state;
    uint8_t ready_to_drive_ad;
} AS_System_t;
extern volatile AS_System_t as_system;

typedef struct {
    uint8_t mission_select;
    uint8_t ignition_ad;
    uint8_t ASMS;
    bool is_in_emergency;
} ACU_t;
extern volatile ACU_t acu;

typedef enum {
    RES_SIGNAL_EMERGENCY = 0,
    RES_SIGNAl_DEFAULT_1 = 1,
    RES_SIGNAl_DEFAULT_2 = 3,
    RES_SIGNAL_R2D_1 = 5,
    RES_SIGNAL_R2D_2 = 7
} Res_Signal_t;
typedef struct {
    Res_Signal_t signal;  // Current signal state
    // uint8_t signal;
} RES_t;
extern volatile RES_t res;

typedef struct {
    uint16_t instant_voltage;
    uint16_t open_voltage;
    uint8_t soc;
    uint16_t pack_current;

    uint16_t high_cell_voltage;
    uint8_t high_cell_voltage_id;
    uint16_t low_cell_voltage;
    uint8_t low_cell_voltage_id;
    uint16_t avg_cell_voltage;

    uint16_t high_cell_temp;
    uint8_t high_cell_temp_id;
    uint16_t low_cell_temp;
    uint8_t low_cell_temp_id;
    uint8_t ambient_temp;

    uint16_t max_discharge_current;
    uint8_t precharge_circuit_state;

} BMSvars_t;
extern volatile BMSvars_t bms;

/**
 * @brief Sends a CAN message
 * @param hcan CAN handle
 * @param id CAN message ID
 * @param data Pointer to data buffer
 * @param len Length of data (0-8 bytes)
 */
void can_bus_send(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint8_t len);

/**
 * @brief Sends HV500 AC current setting
 * @param ac_current AC current value
 * @param hcan CAN handle
 */
void can_bus_send_HV500_SetAcCurrent(uint16_t ac_current, CAN_HandleTypeDef *hcan);

/**
 * @brief Sends HV500 brake current setting
 * @param brake_current Brake current value
 * @param hcan CAN handle
 */
void can_bus_send_HV500_SetBrakeCurrent(uint16_t brake_current, CAN_HandleTypeDef *hcan);

/**
 * @brief Sends HV500 ERPM setting
 * @param erpm ERPM value
 * @param hcan CAN handle
 */
void can_bus_send_HV500_SetERPM(uint32_t erpm, CAN_HandleTypeDef *hcan);

/**
 * @brief Sends HV500 position setting
 * @param position Position value
 * @param hcan CAN handle
 */
void can_bus_send_HV500_SetPosition(uint32_t position, CAN_HandleTypeDef *hcan);

/**
 * @brief Sends HV500 relative current setting
 * @param rel_current Relative current value
 * @param hcan CAN handle
 */
void can_bus_send_HV500_SetRelCurrent(uint32_t rel_current, CAN_HandleTypeDef *hcan);

/**
 * @brief Sends HV500 relative brake current setting
 * @param rel_brake_current Relative brake current value
 * @param hcan CAN handle
 */
void can_bus_send_HV500_SetRelBrakeCurrent(uint32_t rel_brake_current, CAN_HandleTypeDef *hcan);

/**
 * @brief Sends HV500 maximum AC current setting
 * @param max_ac_current Maximum AC current value
 * @param hcan CAN handle
 */
void can_bus_send_HV500_SetMaxAcCurrent(uint32_t max_ac_current, CAN_HandleTypeDef *hcan);

/**
 * @brief Sends HV500 maximum AC brake current setting
 * @param max_ac_brake_current Maximum AC brake current value
 * @param hcan CAN handle
 */
void can_bus_send_HV500_SetMaxAcBrakeCurrent(uint32_t max_ac_brake_current, CAN_HandleTypeDef *hcan);

/**
 * @brief Sends HV500 maximum DC current setting
 * @param max_dc_current Maximum DC current value
 * @param hcan CAN handle
 */
void can_bus_send_HV500_SetMaxDcCurrent(uint32_t max_dc_current, CAN_HandleTypeDef *hcan);

/**
 * @brief Sends HV500 maximum DC brake current setting
 * @param max_dc_brake_current Maximum DC brake current value
 * @param hcan CAN handle
 */
void can_bus_send_HV500_SetMaxDcBrakeCurrent(uint32_t max_dc_brake_current, CAN_HandleTypeDef *hcan);

/**
 * @brief Sends HV500 drive enable setting
 * @param drive_enable Drive enable value
 * @param hcan CAN handle
 */
void can_bus_send_HV500_SetDriveEnable(uint32_t drive_enable, CAN_HandleTypeDef *hcan);

/**
 * @brief Sends powertrain bus message with R2D and ignition states
 * @param r2d Ready to drive state
 * @param ignition Ignition state
 * @param hcan CAN handle
 */
void can_bus_send_pwtbus_1(uint8_t r2d, uint8_t ignition, CAN_HandleTypeDef *hcan);

void can_bus_send_bms_precharge_state(uint8_t precharge_state, CAN_HandleTypeDef *hcan);

/* Autonomous bus functions */

void can_send_vcu_rpm(CAN_HandleTypeDef *hcan, long rpm);

void can_send_autonomous_HV_signal(CAN_HandleTypeDef *hcan, uint8_t hv_state);

/* CAN Bus Processing Functions */

/**
 * @brief Processes data bus (CAN1) messages
 * @param RxHeader CAN message header with ID and metadata
 * @param data CAN message data payload (8 bytes)
 */
void decode_DATA_DB(CAN_RxHeaderTypeDef RxHeader, uint8_t *data);

/**
 * @brief Processes autonomous system bus (CAN3) messages
 * @param RxHeader CAN message header with ID and metadata
 * @param data CAN message data payload (8 bytes)
 */
void decode_AS_DB(CAN_RxHeaderTypeDef RxHeader, uint8_t *data);

/**
 * @brief Processes autonomous bus messages (CAN3)
 * @param RxHeader CAN message header with ID and metadata
 * @param data CAN message data payload (8 bytes)
 */
void decode_AUTO_DB(CAN_RxHeaderTypeDef RxHeader, uint8_t *data);

/**
 * @brief Processes powertrain bus messages (CAN2)
 * @param RxHeader CAN message header with ID and metadata
 * @param data CAN message data payload (8 bytes)
 */
void decode_PWT_DB(CAN_RxHeaderTypeDef RxHeader, uint8_t *data);

void can_send_st_wheel_data(CAN_HandleTypeDef *hcan, uint16_t apps, uint16_t brake, uint16_t inv_temp, uint16_t motor_temp, uint16_t bms_voltage, uint16_t soc_hv, uint16_t apps_error, uint16_t inv_voltage, uint16_t rpm, uint16_t ign_signal, uint16_t r2d_signal);

    /**
     * @brief CAN mailbox used for transmitting messages
     */
    extern uint32_t TxMailbox;

#endif /* CAN_UTILS_H */
