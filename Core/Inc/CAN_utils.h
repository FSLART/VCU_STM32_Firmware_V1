#ifndef CAN_UTILS_H
#define CAN_UTILS_H

#include <stdbool.h>

#include "main.h"
#include "stm32f7xx_hal.h"
#include "can_queue.h"
#include "fsic.h"
#include "powertrain_t26.h"

/* External CAN handle declarations */
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern CAN_HandleTypeDef hcan3;

typedef struct {
    uint32_t id;
    uint8_t message[64];
    uint8_t length;
} can_data_t;

/*data structure for interacting with FSIC inverter (INV1 or INV2)*/
typedef struct {
    int32_t Actual_ERPM;            // 32-bit signed, bytes 0-3
    int16_t Actual_Duty;            // 16-bit signed, bytes 4-5, scale 10
    uint16_t Actual_InputVoltage;   // 16-bit unsigned, bytes 6-7
    int16_t Actual_ACCurrent;       // 16-bit signed, bytes 0-1, scale 10
    int16_t Actual_DCCurrent;       // 16-bit signed, bytes 2-3, scale 10
    int16_t Actual_TempController;  // 16-bit signed, bytes 0-1, scale 10
    int16_t Actual_TempMotor;       // 16-bit signed, bytes 2-3, scale 10
    uint8_t Actual_FaultCode;       // 8-bit unsigned, byte 4
    int32_t Actual_FOC_id;          // 32-bit signed, bytes 0-3, scale 100
    int32_t Actual_FOC_iq;          // 32-bit signed, bytes 4-7, scale 100
    int8_t Actual_Throttle;         // 8-bit signed, byte 0
    int8_t Actual_Brake;            // 8-bit signed, byte 1
    uint8_t Digital_input_1;        // 1-bit, bit 16
    uint8_t Digital_input_2;        // 1-bit, bit 17
    uint8_t Digital_input_3;        // 1-bit, bit 18
    uint8_t Digital_input_4;        // 1-bit, bit 19
    uint8_t Digital_output_1;       // 1-bit, bit 20
    uint8_t Digital_output_2;       // 1-bit, bit 21
    uint8_t Digital_output_3;       // 1-bit, bit 22
    uint8_t Digital_output_4;       // 1-bit, bit 23
    uint8_t Drive_enable;           // 1-bit, bit 24
    uint8_t Capacitor_temp_limit;   // 1-bit, bit 32
    uint8_t DC_current_limit;       // 1-bit, bit 33
    uint8_t Drive_enable_limit;     // 1-bit, bit 34
    uint8_t IGBT_accel_limit;       // 1-bit, bit 35
    uint8_t IGBT_temp_limit;        // 1-bit, bit 36
    uint8_t Input_voltage_limit;    // 1-bit, bit 37
    uint8_t Motor_accel_limit;      // 1-bit, bit 38
    uint8_t Motor_temp_limit;       // 1-bit, bit 39
    uint8_t RPM_min_limit;          // 1-bit, bit 40
    uint8_t RPM_max_limit;          // 1-bit, bit 41
    uint8_t Power_limit;            // 1-bit, bit 42
    uint8_t CAN_map_version;        // 8-bit unsigned, byte 7

    // Transmit data (command values to send to FSIC_t)
    int16_t SetCurrent;               // Set AC current, 16-bit signed, scale 10
    int16_t SetBrakeCurrent;          // Set brake current, 16-bit signed, scale 10 (positive only)
    int32_t SetERPM;                  // Set speed (ERPM), 32-bit signed
    int16_t SetPosition;              // Set position, 16-bit signed, scale 10
    int16_t SetRelativeCurrent;       // Set relative current, 16-bit signed, scale 10 (-100 to +100%)
    int16_t SetRelativeBrakeCurrent;  // Set relative brake current, 16-bit signed, scale 10 (0 to 100%)
    uint8_t SetDigitalOutput;         // Set digital output, 4-bit values (bits 0-3)
    int16_t SetMaxACCurrent;          // Set max AC current, 16-bit signed, scale 10
    int16_t SetMaxACBrakeCurrent;     // Set max AC brake current, 16-bit signed, scale 10 (negative only)
    int16_t SetMaxDCCurrent;          // Set max DC current, 16-bit signed, scale 10
    int16_t SetMaxDCBrakeCurrent;     // Set max DC brake current, 16-bit signed, scale 10 (negative only)
    uint8_t DriveEnable;              // Drive enable, 8-bit unsigned (0 or 1)
} FSIC_t;

typedef enum {
    STATE_INIT,                    // Initial startup state
    STATE_SHUTDOWN,                // Shutdown state
    STATE_STANDBY,                 // Ignition off
    STATE_PRECHARGE,               // Ignition on, precharging
    STATE_WAITING_FOR_R2D_MANUAL,  // Waiting for manual R2D signal
    STATE_WAITING_FOR_R2D_AUTO,    // Waiting for autonomous R2D signal
    STATE_READY_MANUAL,            // Ready to drive in manual mode
    STATE_READY_AUTONOMOUS,        // Ready to drive in autonomous mode
    STATE_AS_EMERGENCY             // Emergency condition detected
} VCU_STATE_t;

extern VCU_STATE_t current_state;

typedef enum {
    AS_STATE_OFF = 1,    // System is off or not initialized
    AS_STATE_READY = 2,  // System is ready to engage autonomous mode
    AS_STATE_DRIVING = 3,
    AS_STATE_EMERGENCY = 4,  // System is in emergency state
    AS_STATE_FINISHED = 5,   // Mission or task completed
} AS_State_t;

typedef enum {
    AS_CONTROL_MODE_NONE = 0,
    AS_CONTROL_MODE_RPM = 1,
    AS_CONTROL_MODE_TORQUE = 2
} AS_ControlMode_t;

typedef struct {
    uint8_t mission_select;
    uint16_t target_rpm;
    int16_t target_torque;
    AS_ControlMode_t control_mode;
    uint32_t last_control_cmd_ms;
    // uint8_t state;
    AS_State_t state;
    uint8_t ready_to_drive_ad;
} AS_System_t;

typedef struct {
    uint8_t mission_select;
    uint8_t ignition_ad;
    uint8_t ASMS;
    bool is_in_emergency;
} ACU_t;

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

typedef struct {

    int32_t result_W;  // Resulting power in watts
} IVT_t;

// VCU signals structure
typedef struct {
    bool r2d_button_signal;  // R2D signal
    bool r2d_toggle_signal;  // R2D toggle signal
    bool r2d_button_prev;    // Previous state of button for edge detection

    bool r2d_autonomous_signal;  // R2D signal from autonomous system

    bool shutdown_signal;  // Shutdown signal

    uint8_t brake_pressure;  // Brake pressure signal

    bool ignition_ad;             // ignition coming from autonomous system
    bool ignition_ad_prev;        // Previous state for edge detection
    bool ignition_switch_signal;  // Ignition signal

    bool precharge_signal;  // Precharge signal
    bool manual;            // Manual mode signal
    bool autonomous;        // Autonomous mode signal

    bool AS_emergency;

    // R2D sound control variables
    bool r2d_sound_playing;         // Flag indicating if R2D sound is currently playing
    bool r2d_sound_completed;       // Flag indicating if R2D sound has been played for current R2D event
    uint32_t r2d_sound_start_time;  // Timestamp when R2D sound started

    // Emergency sound control variables
    bool emergency_sound_playing;          // Flag indicating if emergency sound is currently playing
    bool emergency_sound_completed;        // Flag indicating if emergency sound has been played for current emergency event
    uint32_t emergency_sound_start_time;   // Timestamp when emergency sound started
    uint32_t emergency_sound_last_toggle;  // Last toggle time for intermittent sound
    bool emergency_sound_state;            // Current state of the emergency sound (ON/OFF)
} VCU_Signals_t;

// VCU signals (defined in CAN_utils.c)
extern VCU_Signals_t vcu;

//Variables
//APPS Loss of comms tick
extern volatile uint32_t last_apps_can_rx_time; // keeps track of the last time a valid 0x710 message came through
extern volatile uint32_t last_acu_can_rx_time;
extern __attribute__((section(".adcarray"))) uint16_t ADC2_APPS[2];  // ADC2_IN5(apps 1) and ADC2_IN6(apps 2)

/**
 * @brief Sends a CAN message
 * @param hcan CAN handle
 * @param id CAN message ID
 * @param data Pointer to data buffer
 * @param len Length of data (0-8 bytes)
 */
void can_bus_send(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint8_t len);

/**
 * @brief Sets AC current on FSIC inverter
 * @param inv_id Inverter ID (1 for INV1, 2 for INV2)
 * @param ac_current AC current value (int16_t)
 * @param hcan CAN handle
 */
void can_bus_send_FSIC_SetAcCurrent(uint8_t inv_id, int16_t ac_current, CAN_HandleTypeDef *hcan);

/**
 * @brief Sets brake current on FSIC inverter
 * @param inv_id Inverter ID (1 for INV1, 2 for INV2)
 * @param brake_current Brake current value (int16_t)
 * @param hcan CAN handle
 */
void can_bus_send_FSIC_SetBrakeCurrent(uint8_t inv_id, int16_t brake_current, CAN_HandleTypeDef *hcan);

/**
 * @brief Sets electrical RPM on FSIC inverter
 * @param inv_id Inverter ID (1 for INV1, 2 for INV2)
 * @param erpm ERPM value (int32_t)
 * @param hcan CAN handle
 */
void can_bus_send_FSIC_SetERPM(uint8_t inv_id, int32_t erpm, CAN_HandleTypeDef *hcan);

/**
 * @brief Sets motor position on FSIC inverter
 * @param inv_id Inverter ID (1 for INV1, 2 for INV2)
 * @param position Position value (int16_t)
 * @param hcan CAN handle
 */
void can_bus_send_FSIC_SetPosition(uint8_t inv_id, int16_t position, CAN_HandleTypeDef *hcan);

/**
 * @brief Sets relative current on FSIC inverter
 * @param inv_id Inverter ID (1 for INV1, 2 for INV2)
 * @param rel_current Relative current value (int16_t)
 * @param hcan CAN handle
 */
void can_bus_send_FSIC_SetRelCurrent(uint8_t inv_id, int16_t rel_current, CAN_HandleTypeDef *hcan);

/**
 * @brief Sets relative brake current on FSIC inverter
 * @param inv_id Inverter ID (1 for INV1, 2 for INV2)
 * @param rel_brake_current Relative brake current value (int16_t)
 * @param hcan CAN handle
 */
void can_bus_send_FSIC_SetRelBrakeCurrent(uint8_t inv_id, int16_t rel_brake_current, CAN_HandleTypeDef *hcan);

/**
 * @brief Sets maximum AC current on FSIC inverter
 * @param inv_id Inverter ID (1 for INV1, 2 for INV2)
 * @param max_ac_current Maximum AC current value (int16_t)
 * @param hcan CAN handle
 */
void can_bus_send_FSIC_SetMaxAcCurrent(uint8_t inv_id, int16_t max_ac_current, CAN_HandleTypeDef *hcan);

/**
 * @brief Sets maximum AC brake current on FSIC inverter
 * @param inv_id Inverter ID (1 for INV1, 2 for INV2)
 * @param max_ac_brake_current Maximum AC brake current value (int16_t)
 * @param hcan CAN handle
 */
void can_bus_send_FSIC_SetMaxAcBrakeCurrent(uint8_t inv_id, int16_t max_ac_brake_current, CAN_HandleTypeDef *hcan);

/**
 * @brief Sets maximum DC current on FSIC inverter
 * @param inv_id Inverter ID (1 for INV1, 2 for INV2)
 * @param max_dc_current Maximum DC current value (int16_t)
 * @param hcan CAN handle
 */
void can_bus_send_FSIC_SetMaxDcCurrent(uint8_t inv_id, int16_t max_dc_current, CAN_HandleTypeDef *hcan);

/**
 * @brief Sets maximum DC brake current on FSIC inverter
 * @param inv_id Inverter ID (1 for INV1, 2 for INV2)
 * @param max_dc_brake_current Maximum DC brake current value (int16_t)
 * @param hcan CAN handle
 */
void can_bus_send_FSIC_SetMaxDcBrakeCurrent(uint8_t inv_id, int16_t max_dc_brake_current, CAN_HandleTypeDef *hcan);

/**
 * @brief Sets drive enable on FSIC inverter
 * @param inv_id Inverter ID (1 for INV1, 2 for INV2)
 * @param drive_enable Drive enable value (uint8_t, 0 or 1)
 * @param hcan CAN handle
 */
void can_bus_send_FSIC_SetDriveEnable(uint8_t inv_id, uint8_t drive_enable, CAN_HandleTypeDef *hcan);

/**
 * @brief Sends powertrain bus message with R2D and ignition states
 * @param r2d Ready to drive state
 * @param ignition Ignition state
 * @param hcan CAN handle
 */
void can_bus_send_pwtbus_1(uint8_t r2d, uint8_t ignition, CAN_HandleTypeDef *hcan);

void can_bus_send_bms_close_contactors(uint8_t close_contactors, CAN_HandleTypeDef *hcan);

/**
 * @brief Send brake pressure data to CAN bus
 * @param hcan CAN handle for the target bus
 * @param brake_pressure Brake pressure value (0-65535)
 */
void can_bus_send_brake_pressure(CAN_HandleTypeDef *hcan, uint16_t brake_pressure);

/* Autonomous bus functions */

void can_send_vcu_rpm(CAN_HandleTypeDef *hcan, uint32_t rpm_left, uint32_t rpm_right);

void can_send_autonomous_HV_signal(CAN_HandleTypeDef *hcan, uint8_t hv_state, uint8_t brake_pressure_front);
void can_send_vcu_ign_r2d_signals(CAN_HandleTypeDef *hcan, uint8_t ignition_manual, uint8_t r2d_manual, uint8_t ignition_auto, uint8_t r2d_auto, uint8_t shutdown_signal, uint8_t vcu_state);

void can_bus_read_ASDB(CAN_HandleTypeDef *hcan, AS_System_t *as_system, ACU_t *acu, RES_t *res);

void decode_autonomous_bus(const can_msg_t *msg, AS_System_t *as_system, ACU_t *acu, RES_t *res);

void decode_powertrain_bus(const can_msg_t *msg, BMSvars_t *bms, FSIC_t *fsic1, FSIC_t *fsic2, IVT_t *ivt);

void send_vcu_0(CAN_HandleTypeDef *hcan, const FSIC_t *hv500);
void send_vcu_1(CAN_HandleTypeDef *hcan, const FSIC_t *hv500, const BMSvars_t *bms);
void send_vcu_2(CAN_HandleTypeDef *hcan, const FSIC_t *hv500);
void send_vcu_3(CAN_HandleTypeDef *hcan, bool r2d_manual, bool ignition_manual, bool r2d_auto, bool ignition_auto, const FSIC_t *hv500);
void send_vcu_4(CAN_HandleTypeDef *hcan, const ACU_t *acu);
void send_all_vcu_frames(CAN_HandleTypeDef *hcan, const FSIC_t *hv500, const BMSvars_t *bms, const ACU_t *acu);
void can_bus_send_vcu_apps_raw(CAN_HandleTypeDef *hcan, uint8_t apps1_raw, uint8_t apps2_raw, uint8_t apps_delta_raw, uint8_t cpu_temp, uint8_t flag_digital_bspd, uint8_t apps_error_type, int16_t apps_1000);
void can_bus_send_vcu_state(void);

/**
 * @brief CAN mailbox used for transmitting messages
 */
extern uint32_t TxMailbox;


#endif /* CAN_UTILS_H */
