/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : CAN Test Isolation — stripped-down VCU
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* -------------------- CONFIGURATION DEFINES -------------------- */
#define __APPS_MIN_BITS 1710U
#define __APPS_MAX_BITS 2690U
#define __APPS_TOLERANCE 50U  // tolerancia para o erro
#define __APPS_DELTA 339U     // usado para normalizar o valor do APPS

#define APPS_MA_WINDOW_SIZE 5  // Window size for moving average

#define CALIBRATE_APPS 0

/*----CAN SAFETY TIMEOUTS----*/
#define MAX_APPS_TIMEOUT_MS 50 //TIMEOUT for APPS loss of communication
//UNCOMMENT IF YOU WANT COMMUNICATIONS CHECKS ON R2D
//#define MAX_R2D_IGN_TIMEOUT_MS 50 //TIMEOUT for IGN and R2D loss of communication
/*---------------------------*/

#define DIGI_BSPD_ENABLE 1    // Enable digital BSPD
#define PAU_CONTROL_ENABLE 0  // Enable power adjust utility (PAU) control

#define print_state 0
#define print_variables 0
#define print_apps 0

#define CAN_DATA_BUS &hcan1
#define CAN_POWERTRAIN_BUS &hcan2
#define CAN_AUTONOMOUS &hcan3

/* BYPASS VARIABLES*/
#define Bypass_brake_pressure 0
#define Bypass_precharge 0

#define BRAKE_PRESSURE_THRESHOLD 20  // Minimum brake pressure (bar) required for R2D

#define SHUTDOWN_DEBOUNCE_TIME_MS 50  // Shutdown signal debounce time in milliseconds
#define IGNITION_DEBOUNCE_TIME_MS 50  // Ignition switch debounce time in milliseconds

#define MAX_RPM_AD 1800  // 44.17 km/h

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)


#pragma region Includes
/* -------------------- STANDARD INCLUDES -------------------- */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stm32f767xx.h>
#include <stm32f7xx_hal_can.h>
#include <string.h>

/* ---- Test isolation flags ---- */
#define TEST_CAN_IRQ 1   /* 1 = use can_driver for interrupt-driven RX */
#define TEST_CAN_QUEUE 1 /* 1 = drain queues each loop, expose debug vars */
// #define TEST_DBC_CODEC   1   /* 1 = run DBC encode/decode demo once */
#define TEST_CAN_TX 1 /* 1 = send periodic test frames on all 3 buses */

#pragma endregion Includes
/* -------------------- GLOBAL VARIABLES -------------------- */
#pragma region Global Variables
// ADC buffers
__attribute__((section(".adcarray"))) uint16_t ADC1_VAL[4];
//__attribute__((section(".adcarray"))) uint16_t ADC2_APPS[2];  // ADC2_IN5(apps 1) and ADC2_IN6(apps 2)

// APPS moving average variables
uint16_t apps1_buffer[APPS_MA_WINDOW_SIZE];
uint16_t apps2_buffer[APPS_MA_WINDOW_SIZE];
uint8_t apps_buffer_pos = 0;
uint16_t apps1_avg = 0;
uint16_t apps2_avg = 0;


//UNCOMMENT IF YOU WANT COMMUNICATIONS CHECKS ON R2D
//volatile uint32_t last_r2d_ign_can_rx_time = 0; //keeps track of the last time a valid 0x740 message came through

int value = 0;

// CAN variables
CAN_TxHeaderTypeDef TxHeader_CAN1;
uint8_t TxData_CAN1[8];
uint32_t TxMailbox_CAN1;

CAN_TxHeaderTypeDef TxHeader_CAN2;
uint8_t TxData_CAN2[8];
uint32_t TxMailbox_CAN2;

CAN_TxHeaderTypeDef TxHeader_CAN3;
uint8_t TxData_CAN3[8];
uint32_t TxMailbox_CAN3;

CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

can_data_t can_data1;  // CAN data structure for CAN1
can_data_t can_data2;  // CAN data structure for CAN2
can_data_t can_data3;  // CAN data structure for CAN3

volatile bool can1_rx_flag = false;
volatile bool can2_rx_flag = false;
volatile bool can3_rx_flag = false;

APPS_Result_t result = {0};  // Result structure for APPS processing

bspd_state_t bspd_state;

uint16_t apps_bspd_pau = 0;

// vars on can utils file
volatile HV500 myHV500;
volatile AS_System_t as_system;
volatile ACU_t acu;
volatile RES_t res;
volatile BMSvars_t bms;
volatile IVT_t ivt;

/* -------------------- STATE MACHINE DEFINITIONS -------------------- */
// VCU state enumeration
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

// State machine variables
VCU_STATE_t current_state = STATE_INIT;   // Current state of the VCU
VCU_STATE_t previous_state = STATE_INIT;  // Previous state of the VCU

// State names for debug output
const char *state_names[] = {
    "STATE_INIT",
    "STATE_SHUTDOWN",
    "STATE_STANDBY",
    "STATE_PRECHARGE",
    "STATE_WAITING_FOR_R2D_MANUAL",
    "STATE_WAITING_FOR_R2D_AUTO",
    "STATE_READY_MANUAL",
    "STATE_READY_AUTONOMOUS",
    "STATE_AS_EMERGENCY"};


long erpm_temporary = 0;  // Temporary variable for ERPM calculations


#pragma endregion Global Variables

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Debug variables — watch these in debugger */
#if TEST_CAN_IRQ
/* can1/2/3_rx_count and can1/2/3_tx_count are defined in can_driver.c
   and declared extern in can_driver.h — watch them there. */
#endif

#if TEST_CAN_QUEUE
/* Snapshot variables updated each loop iteration */
uint32_t can1_queue_depth = 0;
uint32_t can2_queue_depth = 0;
uint32_t can3_queue_depth = 0;
can_msg_t last_can1_msg = {0};
can_msg_t last_can2_msg = {0};
can_msg_t last_can3_msg = {0};
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MPU_Config(void);
/* USER CODE BEGIN PFP */

// Timed task function declarations
void execute_10ms_tasks(void);
void execute_100ms_tasks(void);
void execute_immediate_tasks(void);

// Button debounce function declaration
//void debounce_r2d_button(void);
void debounce_shutdown_signal(void);
//void debounce_ignition_switch(void);
#pragma endregion Function Prototypes
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ---- CAN RX interrupt callback ---- */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan) {
#if TEST_CAN_IRQ
    /* New path: route all three buses through can_driver */
    can_driver_rx_isr(hcan);
#else
    /* Legacy path: only handle CAN2 (powertrain) */
    if (hcan->Instance == CAN2) {
        CAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
            /* User: add legacy processing here if needed */
        }
    }
#endif
}
/* USER CODE END 0 */

/**
 * @brief Measure the brake pressure from an ADC reading
 * @param bits Raw ADC value (12-bit resolution, 0-4095)
 * @return Calculated brake pressure in bar
 * @details Converts ADC reading to voltage, then to pressure using calibration formula.
 *          Sensor characteristics: 28.57mV/bar with 0.5V offset (0 bar = 0.5V)
 */

/*
float MeasureBrakePressure(uint16_t bits) {
    // Constants for clarity
    const float ADC_MAX = 4095.0f;
    const float MCU_VREF = 3.3f;             // MCU reference voltage
    const float SENSOR_VREF = 5.0f;          // Sensor reference voltage
    const float CONVERSION_FACTOR = 0.667f;  // Factor to convert 3.3V to 5V scale
    const float OFFSET_VOLTAGE = 0.5f;       // 0 bar = 0.5V
    const float SENSITIVITY = 0.02857f;      // 28.57mV/bar

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

    return pressure;  // Return the brake pressure in bar
}*/

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CAN3_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
#if TEST_CAN_IRQ
    /* Init queues, filters, IRQs, start CAN buses */
    can_queue_init(&can1_rx_queue);
    can_queue_init(&can2_rx_queue);
    can_queue_init(&can3_rx_queue);
    can_driver_init();
    can_queue_init(&can1_tx_queue);
    can_queue_init(&can2_tx_queue);
    can_queue_init(&can3_tx_queue);
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

#if TEST_CAN_IRQ
        /* Main-context queue drain — increment counters for debugger watch */
        can_msg_t msg;

        while (can_queue_pop(&can1_rx_queue, &msg)) {
            /* User: set breakpoint here and inspect msg.id, msg.data, msg.dlc */
            /* can1_rx_count is incremented in the ISR (can_driver_rx_isr) */
        }

        while (can_queue_pop(&can2_rx_queue, &msg)) {
            /* can2_rx_count is incremented in the ISR */
        }

        while (can_queue_pop(&can3_rx_queue, &msg)) {
            /* can3_rx_count is incremented in the ISR */
        }
#endif

#if TEST_CAN_QUEUE
        /* Snapshot queue depths for debugger watch */
        can1_queue_depth = can_queue_count(&can1_rx_queue);
        can2_queue_depth = can_queue_count(&can2_rx_queue);
        can3_queue_depth = can_queue_count(&can3_rx_queue);

        /* Peek at latest messages */
        can_queue_peek(&can1_rx_queue, &last_can1_msg);
        can_queue_peek(&can2_rx_queue, &last_can2_msg);
        can_queue_peek(&can3_rx_queue, &last_can3_msg);
#endif

#if TEST_DBC_CODEC
        /* Run once — encode + send a frame on each bus */
        static bool dbc_demo_done = false;
        if (!dbc_demo_done) {
            uint8_t data[8];
            uint8_t dlc;

/**
 * @brief Reset the Emergency sound state
 * @details Resets the emergency sound flags when leaving the emergency state.
 */
void ResetEmergencySound(void) {
    vcu.emergency_sound_playing = false;
    vcu.emergency_sound_completed = false;
    vcu.emergency_sound_state = false;
    HAL_GPIO_WritePin(GPIOD, dout4_R2D_Buzzer_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Handle emergency stop condition
 * @details Sounds the buzzer with required pattern if an emergency stop is detected
 */
void HandleEmergencyStop(void) {
    if ((acu.is_in_emergency == 1) || (as_system.state == 4) || (res.signal == 0)) {
        // Start emergency sound if not already playing or completed
        if (!vcu.emergency_sound_playing && !vcu.emergency_sound_completed) {
            StartEmergencySound();
            // Disable R2D state
            vcu.r2d_toggle_signal = false;
            vcu.r2d_autonomous_signal = false;
        }

        // Update emergency sound pattern
        UpdateEmergencySound();
    } else {
        // Reset emergency sound state when emergency condition clears
        ResetEmergencySound();
    }
}

/* -------------------- STATE MACHINE FUNCTIONS -------------------- */
/**
 * @brief Print state transition for debugging
 * @param from Previous state
 * @param to New state
 */
void print_state_transition(VCU_STATE_t from, VCU_STATE_t to) {
    printf("\n\rState transition: %s -> %s\n", state_names[from], state_names[to]);
}

#pragma region UPDATE STATE

/**
 * @brief Update the state of the VCU based on inputs and conditions
 */
void UpdateState(void) {
    // Store current state for change detection
    previous_state = current_state;

    // Process emergency condition with highest priority
    if (((acu.is_in_emergency == 1) || (as_system.state == 4) || (res.signal == 0)) && current_state != STATE_AS_EMERGENCY) {
        current_state = STATE_AS_EMERGENCY;
    }

    //NO LONGER NEEDED
    //debounce_shutdown_signal();         // Update shutdown signal with debounce
    //debounce_ignition_switch();         // Update ignition switch signal with debounce
    vcu.ignition_ad = acu.ignition_ad;  // Read ignition signal from autonomous system

    // State transitions
    switch (current_state) {
        case STATE_INIT:

            if (vcu.ignition_switch_signal || vcu.ignition_ad) {
                current_state = STATE_INIT;
            } else {
                current_state = STATE_STANDBY;  // Transition to standby state after initialization
            }

            /* Data bus (CAN1): encode + send VCU_0 */
            if (dbc_encode_vcu_0(data, &dlc, 0, 0, 1000, 800)) {
                can_driver_tx(&hcan1, DATA_DBC_VCU__FRAME_ID, data, dlc);
            }

            /* Autonomous bus (CAN3): encode + send VCU_RPM */
            if (dbc_encode_vcu_rpm(data, &dlc, 1200)) {
                can_driver_tx(&hcan3, AUTONOMOUS_TEMPORARY_VCU_RPM_FRAME_ID, data, dlc);
            }

        case STATE_PRECHARGE:
            // when the precharge is done, wait for the ready to drive signal to start the ready to drive
            // either from manual or autonomous mode-

            // Check if precharge is complete
            if ((!vcu.ignition_switch_signal && !vcu.ignition_ad) || !vcu.shutdown_signal) {
                current_state = STATE_STANDBY;
                if (!vcu.shutdown_signal) {
                    current_state = STATE_SHUTDOWN;
                }
            } else if (Bypass_precharge || bms.precharge_circuit_state == 9) {
                current_state = vcu.manual ? STATE_WAITING_FOR_R2D_MANUAL : STATE_WAITING_FOR_R2D_AUTO;
            }
            break;

        case STATE_WAITING_FOR_R2D_MANUAL:

            if (!vcu.ignition_switch_signal || !vcu.shutdown_signal) {
                current_state = STATE_STANDBY;
                if (!vcu.shutdown_signal) {
                    current_state = STATE_SHUTDOWN;
                }
                //} else if (vcu.r2d_toggle_signal && (Bypass_brake_pressure || vcu.brake_pressure > BRAKE_PRESSURE_THRESHOLD)) {
            } else if ((Bypass_brake_pressure || ((vcu.brake_pressure > BRAKE_PRESSURE_THRESHOLD) && (result.percentage == 0))) && vcu.r2d_button_signal) {
            	current_state = STATE_READY_MANUAL;
            	/*
            	// Handle R2D button debounce
                //debounce_r2d_button();
                if (vcu.r2d_toggle_signal) {
                    current_state = STATE_READY_MANUAL;
                }*/
            }
            break;

        case STATE_WAITING_FOR_R2D_AUTO:
            if (!vcu.ignition_ad || !vcu.shutdown_signal) {
                current_state = STATE_STANDBY;
                if (!vcu.shutdown_signal) {
                    current_state = STATE_SHUTDOWN;
                }
            } else if (as_system.state == 3) {  // driving state
                current_state = STATE_READY_AUTONOMOUS;
            }
            break;

        case STATE_READY_MANUAL:
            //debounce_r2d_button();
            if (!vcu.ignition_switch_signal || !vcu.shutdown_signal) {
                // Change to standby if ignition is off
                current_state = STATE_STANDBY;
                if (!vcu.shutdown_signal) {
                    current_state = STATE_SHUTDOWN;
                }
            } else if (!vcu.r2d_toggle_signal) {
                // Change to waiting for R2D manual if the button is pressed again
                current_state = STATE_WAITING_FOR_R2D_MANUAL;
            }
            break;

        case STATE_READY_AUTONOMOUS:
            // serve as gateway to the computer to send the commands to the inverter
            if (!vcu.ignition_ad) {
                current_state = STATE_STANDBY;
            } else if (as_system.state != 3 && as_system.state != 5) {
                current_state = STATE_WAITING_FOR_R2D_AUTO;
            }

            // as_system.state = 5 finish autonoma

            // if (!vcu.ignition_ad || !vcu.shutdown_signal) {
            //     current_state = STATE_STANDBY;
            //     if (!vcu.shutdown_signal) {
            //         current_state = STATE_SHUTDOWN;
            //     }
            // } else if (as_system.state != 3) {
            //     current_state = STATE_WAITING_FOR_R2D_AUTO;
            // }
            break;

        case STATE_AS_EMERGENCY:
            // printf("\n\rAS Emergency: %d\n\r", as_system.state);

            // Only exit emergency state when the 9-second sound has completed
            if (!(acu.is_in_emergency) && !(as_system.state == 4) && !(res.signal == 0)) {
                current_state = STATE_STANDBY;
                if (!vcu.shutdown_signal) {
                    current_state = STATE_SHUTDOWN;
                }
            }

            // if (!vcu.ignition_ad || !vcu.shutdown_signal) {
            //     // current_state = STATE_STANDBY;
            //     // if (!vcu.shutdown_signal) {
            //     //     // so quando o tocar todo o som da emergencia é que vai para shutdown
            //     //     if (vcu.emergency_sound_completed) {
            //     //         current_state = STATE_SHUTDOWN;
            //     //     }
            //     // }
            // } else if (as_system.state != 4 && res.signal != 0) {
            //     current_state = STATE_STANDBY;
            // }

            // if (!vcu.ignition_ad || !vcu.shutdown_signal) {
            //     current_state = STATE_STANDBY;
            //     if (!vcu.shutdown_signal) {
            //         // so quando o tocar todo o som da emergencia é que vai para shutdown
            //         if (vcu.emergency_sound_completed) {
            //             current_state = STATE_SHUTDOWN;
            //         }
            //     }
            // } else if (as_system.state != 4 && res.signal != 0) {
            //     current_state = STATE_STANDBY;
            // }
            break;
    }

    // Execute entry actions when state has changed
    if (current_state != previous_state) {
#ifdef print_state
        print_state_transition(previous_state, current_state);
#endif

#if TEST_CAN_TX
        /* Periodic test TX — 10ms rate, unique ID per bus, via TX queue */
        {
            static uint32_t last_tx_tick = 0;
            uint32_t now = HAL_GetTick();

            if ((now - last_tx_tick) >= 100) {
                last_tx_tick = now;

                /* Sequence counters — embedded in TX payload bytes [0..3] so the
                 * receiver can detect lost/reordered frames. Named _seq to avoid
                 * shadowing the hardware-level can1/2/3_tx_count in can_driver.c. */
                static uint32_t can1_tx_seq = 0;
                static uint32_t can2_tx_seq = 0;
                static uint32_t can3_tx_seq = 0;

                can_msg_t tx_msg;
                tx_msg.dlc = 8;
                tx_msg.timestamp = now;

                /* CAN3 (Autonomous): ID 0x303 */
                memset(tx_msg.data, 0, 8);
                tx_msg.id = 0x303;
                tx_msg.data[0] = (uint8_t)(can3_tx_seq & 0xFF);
                tx_msg.data[1] = (uint8_t)((can3_tx_seq >> 8) & 0xFF);
                tx_msg.data[2] = (uint8_t)((can3_tx_seq >> 16) & 0xFF);
                tx_msg.data[3] = (uint8_t)((can3_tx_seq >> 24) & 0xFF);
                tx_msg.bus = CAN_BUS_3;
                can_tx_enqueue(&tx_msg);
                can3_tx_seq++;

                /* CAN1 (Data bus): ID 0x301 */
                memset(tx_msg.data, 0, 8);
                tx_msg.id = 0x301;
                tx_msg.data[0] = (uint8_t)(can1_tx_seq & 0xFF);
                tx_msg.data[1] = (uint8_t)((can1_tx_seq >> 8) & 0xFF);
                tx_msg.data[2] = (uint8_t)((can1_tx_seq >> 16) & 0xFF);
                tx_msg.data[3] = (uint8_t)((can1_tx_seq >> 24) & 0xFF);
                tx_msg.bus = CAN_BUS_1;
                can_tx_enqueue(&tx_msg);
                can1_tx_seq++;

                /* CAN2 (Powertrain): ID 0x302 */
                memset(tx_msg.data, 0, 8);
                tx_msg.id = 0x302;
                tx_msg.data[0] = (uint8_t)(can2_tx_seq & 0xFF);
                tx_msg.data[1] = (uint8_t)((can2_tx_seq >> 8) & 0xFF);
                tx_msg.data[2] = (uint8_t)((can2_tx_seq >> 16) & 0xFF);
                tx_msg.data[3] = (uint8_t)((can2_tx_seq >> 24) & 0xFF);
                tx_msg.bus = CAN_BUS_2;
                can_tx_enqueue(&tx_msg);
                can2_tx_seq++;
            }

            break;

        case STATE_STANDBY:
            static uint32_t last_can_send_time_standby = 0;
            uint32_t current_time_standby = HAL_GetTick();
            if (current_time_standby - last_can_send_time_standby >= 1) {
                can_bus_send_bms_precharge_state(0, &hcan2);
                last_can_send_time_standby = current_time_standby;
            }

            break;

        case STATE_PRECHARGE:
            // Handle precharge process

            static uint32_t last_can_send_time_precharge = 0;
            uint32_t current_time_precharge = HAL_GetTick();
            if (current_time_precharge - last_can_send_time_precharge >= 10) {
                can_bus_send_bms_precharge_state(1, &hcan2);
                last_can_send_time_precharge = current_time_precharge;
            }

            break;

        case STATE_WAITING_FOR_R2D_MANUAL:
            // fade in and out the R2D LED
            led_fade_nonblocking(GPIOD, LED_R2D_Pin);
            led_fade_pwm(&htim4, TIM_CHANNEL_1);

            static uint32_t last_can_send_time_r2d_manual = 0;
            uint32_t current_time_r2d_manual = HAL_GetTick();
            if (current_time_r2d_manual - last_can_send_time_r2d_manual >= 10) {
                can_bus_send_bms_precharge_state(1, &hcan2);
                last_can_send_time_r2d_manual = current_time_r2d_manual;
            }

            break;

        case STATE_WAITING_FOR_R2D_AUTO:
            // Waiting indicator for autonomous mode
            led_fade_nonblocking(GPIOD, LED_R2D_Pin);
            led_fade_pwm(&htim4, TIM_CHANNEL_1);

            static uint32_t last_can_send_time_r2d_auto = 0;
            uint32_t current_time_r2d_auto = HAL_GetTick();

            if (current_time_r2d_auto - last_can_send_time_r2d_auto >= 10) {
                can_bus_send_bms_precharge_state(1, &hcan2);
                last_can_send_time_r2d_auto = current_time_r2d_auto;
            }
            break;

        case STATE_READY_MANUAL:
            // Process manual driving controls
            // Send pedal position to inverter

            static uint32_t last_can_send_time_manuel = 0;
            uint32_t current_time_manuel = HAL_GetTick();

            if (current_time_manuel - last_can_send_time_manuel >= 5) {
                can_bus_send_HV500_SetDriveEnable(1, &hcan2);
                can_bus_send_HV500_SetRelCurrent(apps_bspd_pau, &hcan2);
                can_bus_send_bms_precharge_state(1, &hcan2);
                last_can_send_time_manuel = current_time_manuel;
            }
            break;

        case STATE_READY_AUTONOMOUS:
            // Forward autonomous commands to inverter
            static uint32_t last_can_send_time_auto = 0;
            uint32_t current_time_auto = HAL_GetTick();

            if (current_time_auto - last_can_send_time_auto >= 10) {
                can_bus_send_HV500_SetDriveEnable(1, &hcan2);
                // finished
                if (as_system.state == 5) {
                    can_bus_send_HV500_SetERPM(0, &hcan2);
                    can_send_vcu_rpm(&hcan3, erpm_temporary);  // feedback to jetson
                    // can_send_vcu_rpm(&hcan3, myHV500.Actual_ERPM);
                    can_bus_send_bms_precharge_state(1, &hcan2);
                    last_can_send_time_auto = current_time_auto;

                    // driving
                } else if (as_system.state == 3) {
                    if (as_system.target_rpm > MAX_RPM_AD) {
                        as_system.target_rpm = MAX_RPM_AD;
                    } else if (as_system.target_rpm < 0) {
                        as_system.target_rpm = 0;
                    }

                    uint32_t erpm = as_system.target_rpm * 10;
                    can_bus_send_HV500_SetERPM(erpm, &hcan2);
                    can_send_vcu_rpm(&hcan3, erpm_temporary);  // feedback to jetson
                    // can_send_vcu_rpm(&hcan3, myHV500.Actual_ERPM);
                    can_bus_send_bms_precharge_state(1, &hcan2);
                    last_can_send_time_auto = current_time_auto;

                    // printf("\n\rRPM: %d\n\r", myHV500.Actual_ERPM / 10);
                    // printf("\n\rTarget RPM: %d\n\r", as_system.target_rpm);
                }
            }
            break;

        case STATE_AS_EMERGENCY:
            // Activate emergency indicators
            // Handle emergency state
            static uint32_t last_can_send_time_emergency = 0;
            uint32_t current_time_emergency = HAL_GetTick();
            if (current_time_emergency - last_can_send_time_emergency >= 1) {
                can_bus_send_bms_precharge_state(0, &hcan2);
                last_can_send_time_emergency = current_time_emergency;
            }
            HandleEmergencyStop();
            break;
    }
}
#pragma endregion HANDLE STATE

// Time-based debounce for R2D button

//---No longer needed - Comes in via CAN2----//
/*void debounce_r2d_button(void) {
    static uint32_t last_debounce_time = 0;
    static bool last_reading = false;
    static bool stable_state = false;

    bool current_reading = HAL_GPIO_ReadPin(int2_r2d_GPIO_Port, int2_r2d_Pin);

    // If state changed, reset debounce timer
    if (current_reading != last_reading) {
        last_debounce_time = HAL_GetTick();
    }

    // State is considered stable if it's been the same for the debounce delay
    if ((HAL_GetTick() - last_debounce_time) > 50) {  // 50ms debounce time
        // Only process state change when it's stable and different from previous
        if (current_reading != stable_state) {
            stable_state = current_reading;

            // Process rising edge (button press)
            if (stable_state && !vcu.r2d_button_prev) {
                vcu.r2d_toggle_signal = !vcu.r2d_toggle_signal;
            }

            vcu.r2d_button_prev = stable_state;
        }
    }

    // Save the reading for next comparison
    last_reading = current_reading;

    // Update raw signal for any code that needs it
    vcu.r2d_button_signal = stable_state;
}*/

// Time-based debounce for shutdown signal
void debounce_shutdown_signal(void) {
    static uint32_t last_debounce_time = 0;
    static bool last_reading = false;
    static bool stable_state = false;

    bool current_reading = HAL_GPIO_ReadPin(int3_shutdown_signal_GPIO_Port, int3_shutdown_signal_Pin);

    // If state changed, reset debounce timer
    if (current_reading != last_reading) {
        last_debounce_time = HAL_GetTick();
    }

    // State is considered stable if it's been the same for the debounce delay
    if ((HAL_GetTick() - last_debounce_time) > SHUTDOWN_DEBOUNCE_TIME_MS) {
        // Update stable state only if it has been stable for the debounce time
        if (current_reading != stable_state) {
            stable_state = current_reading;
        }
    }

    // Save the reading for next comparison
    last_reading = current_reading;

    // Update VCU signal with debounced value
    vcu.shutdown_signal = stable_state;
}

// Time-based debounce for ignition switch
//IGNITION DEBOUNCE LOGIC
/*void debounce_ignition_switch(void) {
    static uint32_t last_debounce_time = 0;
    static bool last_reading = false;
    static bool stable_state = false;

    bool current_reading = HAL_GPIO_ReadPin(int1_ign_GPIO_Port, int1_ign_Pin);

    // If state changed, reset debounce timer
    if (current_reading != last_reading) {
        last_debounce_time = HAL_GetTick();
    }

    // State is considered stable if it's been the same for the debounce delay
    if ((HAL_GetTick() - last_debounce_time) > IGNITION_DEBOUNCE_TIME_MS) {
        // Update stable state only if it has been stable for the debounce time
        if (current_reading != stable_state) {
            stable_state = current_reading;
        }
    }

    // Save the reading for next comparison
    last_reading = current_reading;

    // Update VCU signal with debounced value
    vcu.ignition_switch_signal = stable_state;
}*/

/* -------------------- TIMED TASK FUNCTIONS -------------------- */

/**
 * @brief Execute 10ms tasks (100Hz)
 * @details High-frequency tasks that need precise timing
 */
void execute_10ms_tasks(void) {

	if ((HAL_GetTick() - last_apps_can_rx_time) > MAX_APPS_TIMEOUT_MS) {
	    //at least cut off throttle...
	    ADC2_APPS[0] = 0;
	    ADC2_APPS[1] = 0;
	    // Maybe trigger state emergency??
	    // current_state = STATE_AS_EMERGENCY;
	}

	//MAYBE WE IMPLEMENT, IF THERES TIME (NOTE: CANT KILL IGNITION LIKE THAT!!)
	/*if ((HAL_GetTick() - last_r2d_ign_can_rx_time) > MAX_R2D_IGN_TIMEOUT_MS) {
		    vcu.r2d_button_signal = 0;
		    vcu.ignition_switch_signal = 0;
		}*/

    // Send autonomous HV signal
    can_send_autonomous_HV_signal(&hcan3, bms.precharge_circuit_state, vcu.brake_pressure);

    can_bus_send_vcu_apps_raw(&hcan3, ADC2_APPS[0], ADC2_APPS[1], 0, 0, bspd_state.bspd_active, result.error_type, result.percentage_1000);

    // Send heartbeat frame on CAN2
    uint8_t data[8] = {0};
    CAN_TxHeaderTypeDef TxHeader;
    TxHeader.StdId = 0x69;
    TxHeader.ExtId = 0;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 8;
    TxHeader.TransmitGlobalTime = DISABLE;

    uint32_t TxMailbox;
    HAL_CAN_AddTxMessage(&hcan2, &TxHeader, data, &TxMailbox);

#if CALIBRATE_APPS
    // If calibration is in progress, update it
    if (APPS_IsCalibrating()) {
        bool calibration_complete = APPS_Calibrate(apps1_avg, apps2_avg);
        if (calibration_complete) {
            // Optionally, automatically apply the calibration
            uint16_t min, max, tolerance, delta;
            APPS_GetCalibrationValues(&min, &max, &tolerance, &delta);
            APPS_Init(min, max, tolerance, delta);
        }
    }
#endif

        /* Drain TX queues — one attempt per bus per loop iteration */
        can_driver_tx_poll();

/**
 * @brief Execute 100ms tasks (10Hz)
 * @details Medium-frequency tasks for sensor reading and logging
 */
void execute_100ms_tasks(void) {
    // Read brake pressure from ADC
	//NOW DONE BY CAN
    //vcu.brake_pressure = MeasureBrakePressure(ADC1_VAL[0]);
    turn_on_brake_light(vcu.brake_pressure);

    // Debug prints (uncomment if needed)
    // printf("\n\rBrake Pressure: %d\n\r", vcu.brake_pressure);
    // printf("\n\rbits ADC_brake_pressure: %d\n\r", ADC1_VAL[0]);

    can_send_vcu_ign_r2d_signals(&hcan3,
                                 vcu.ignition_switch_signal,                 // manual ignition
                                 (current_state == STATE_READY_MANUAL),      // manual R2D
                                 vcu.ignition_ad,                            // auto ignition
                                 (current_state == STATE_READY_AUTONOMOUS),  // auto R2D
                                 vcu.shutdown_signal,                        // shutdown signal
                                 (uint8_t)current_state);                    // VCU state

    // Send VCU telemetry frames in rotation (one different frame each time)
    static uint8_t frame_index = 0;

    switch (frame_index) {
        case 0:
            send_vcu_0(&hcan1, &myHV500);
            send_vcu_3(&hcan1, vcu.r2d_toggle_signal, vcu.r2d_autonomous_signal, vcu.ignition_switch_signal,
                       vcu.ignition_ad, &myHV500);

            break;
        case 1:
            send_vcu_1(&hcan1, &myHV500, &bms);
            can_bus_send_brake_pressure(&hcan1, vcu.brake_pressure);
            break;
        case 2:
            send_vcu_2(&hcan1, &myHV500);
            break;
        case 3:
            send_vcu_3(&hcan1, vcu.r2d_toggle_signal, vcu.r2d_autonomous_signal, vcu.ignition_switch_signal,
                       vcu.ignition_ad, &myHV500);
            break;
        case 4:
            send_vcu_4(&hcan1, &acu);
            break;
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef RxHeader1;
    uint8_t RxData1[8];
    CAN_RxHeaderTypeDef RxHeader2;
    uint8_t RxData2[8];
    //CAN_RxHeaderTypeDef RxHeader3;
    //uint8_t RxData3[8];


    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader1, RxData1) == HAL_OK) {
            // CAN1 (DATA bus) - message read out to drain the FIFO, not handled yet.
    	}


    if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader2, RxData2) == HAL_OK) {
        if (RxHeader2.StdId == 0x14) {

            erpm_temporary = RxData2[0] << 24 | RxData2[1] << 16 | RxData2[2] << 8 | RxData2[3];
            // printf("\n\rERPM: %d\n\r", erpm_temporary);

        /*}else if (RxHeader2.StdId == APPS_ADC_RAW_ID) { //APPS_ADC_RAW - DBC Powertrain: 122

        	//Fill out the variables previously populated by ADC2
        	__disable_irq(); // Lock interrupts so this update doesnt happen while MovingAverage_Update() is trying to read it
        	ADC2_APPS[0] = (RxData2[1] << 8) | RxData2[0];
        	ADC2_APPS[1] = (RxData2[3] << 8) | RxData2[2];
        	__enable_irq();  // Unlock interrupts so MovingAverage_Update() can get to reading them

        	last_apps_can_rx_time = HAL_GetTick(); // Reset the safety timer (For checking comms)


          }else if(RxHeader2.StdId == R2D_AND_IGN_ID){//R2D Button and IGN button - DBC Powertrain: 123
        	vcu.ignition_switch_signal = RxData2[0];
			vcu.r2d_button_signal = RxData2[1];
			*/
        }else {
        	can_filter_id_bus2(RxHeader2, RxData2, &bms, &myHV500, &ivt);
        }
        //} else {
        //    can_filter_id_bus2(RxHeader2, RxData2);
        //}
        //Isto é BMS e HV500 parece, metemos num else, o default so da break, poupa tempo se a ID nao cair nos casos??
        /*can_filter_id_bus2(RxHeader2, RxData2, &bms, &myHV500, &ivt);*/

    }/*else if(HAL_CAN_GetRxMessage(CAN_AUTONOMOUS, CAN_RX_FIFO0, &RxHeader3, RxData3) == HAL_OK){

    	if (RxHeader3.StdId == BRAKE_PRESSURE_ID) {

    		uint16_t raw_pressure = (RxData3[1] << 8) | RxData3[0];
    		uint8_t resulting_pressure = raw_pressure*0.1;
    		vcu.brake_pressure = resulting_pressure;
    	}
    }*/

}

//----Legacy function for FIFO0 CAN Interrupt----//
/*void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    // CAN_RxHeaderTypeDef RxHeader1;
    // uint8_t RxData1[8];
    CAN_RxHeaderTypeDef RxHeader2;
    uint8_t RxData2[8];
    // CAN_RxHeaderTypeDef RxHeader3;
    // uint8_t RxData3[8];

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

        can_filter_id_bus2(RxHeader2, RxData2, &bms, &myHV500, &ivt);
    }
}*/

#pragma region MAIN
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
    current_state = STATE_INIT;
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

    // HAL_ADC_Start_DMA(&hadc3, (uint32_t *)&adc_buffer, 5);

    // HAL_ADC_Start(&hadc3);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_CAN3_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start_IT(&htim2);

    // Configure filters with different banks
    CAN1_Filter_Config();
    CAN2_Filter_Config();
    CAN3_Filter_Config();

    // Enable interrupts for all
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(&hcan3, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }

    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_Start(&hcan3);

    startup_leds_animation();
    MovingAverage_Init();  // Initialize moving average buffers

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    //__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 2000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1_VAL, 4);
    //HAL_ADC_Start_DMA(&hadc2, ADC2_APPS, 2);  // Start ADC2 for APPS
                                              // Calibrate APPS

    APPS_Init(__APPS_MIN_BITS, __APPS_MAX_BITS, __APPS_TOLERANCE, __APPS_DELTA);  // Initialize APPS
    bspd_init(&bspd_state);
    res.signal = RES_SIGNAl_DEFAULT_1;  // start with a value different than 0 to avoid emergency state
    printf("\n\n\n\n\n======================== RESET ========================\n\n\n\n\n\r");

#if CALIBRATE_APPS
    APPS_StartCalibration();
#endif

    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

        // Execute immediate tasks every loop iteration
        execute_immediate_tasks();

        // Timing-based tasks
        static uint32_t previous_tick_10ms = 0;
        static uint32_t previous_tick_100ms = 0;
        uint32_t current_tick = HAL_GetTick();

        // 10ms tasks (100Hz) - High frequency control tasks
        if (current_tick - previous_tick_10ms >= 10) {
            execute_10ms_tasks();
            previous_tick_10ms = current_tick;
        }

        // 100ms tasks (10Hz) - Medium frequency monitoring tasks
        if (current_tick - previous_tick_100ms >= 100) {
            execute_100ms_tasks();
            previous_tick_100ms = current_tick;
        }

        // Optional: Add more timing intervals here if needed
        // Example for 1000ms (1Hz) tasks:
        // static uint32_t previous_tick_1s = 0;
        // if (current_tick - previous_tick_1s >= 1000) {
        //     execute_1s_tasks();s
        //     previous_tick_1s = current_tick;
        // }
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32B;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
