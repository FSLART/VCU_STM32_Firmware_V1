/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
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
#define __APPS_MIN_BITS 1160U
#define __APPS_MAX_BITS 2645U
#define __APPS_TOLERANCE 50U  // tolerancia para o erro
#define __APPS_DELTA 304U     // usado para normalizar o valor do APPS

#define APPS_MA_WINDOW_SIZE 10  // Window size for moving average

#define CALIBRATE_APPS 0

#define print_state 0
#define print_variables 0
#define print_apps 0

#define CAN_DATA_BUS &hcan1
#define CAN_POWERTRAIN_BUS &hcan2
#define CAN_AUTONOMOUS &hcan3

/* BYPASS VARIABLES*/
#define Bypass_brake_pressure 1
#define Bypass_precharge 0

#define BRAKE_PRESSURE_THRESHOLD 10  // Minimum brake pressure (bar) required for R2D

#define MAX_RPM_AD 1500

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

/* -------------------- STANDARD INCLUDES -------------------- */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stm32f767xx.h>
#include <stm32f7xx_hal_can.h>
#include <stm32f7xx_hal_cortex.h>

/* -------------------- PROJECT INCLUDES -------------------- */
#include "../Inc/proportional_integral_controller.h"
#include "APPS.h"
#include "CAN_utils.h"

/* -------------------- GLOBAL VARIABLES -------------------- */
// ADC buffers
__attribute__((section(".adcarray"))) uint16_t ADC1_VAL[4];
__attribute__((section(".adcarray"))) uint16_t ADC2_APPS[2];  // ADC2_IN5(apps 1) and ADC2_IN6(apps 2)

// APPS moving average variables
uint16_t apps1_buffer[APPS_MA_WINDOW_SIZE];
uint16_t apps2_buffer[APPS_MA_WINDOW_SIZE];
uint8_t apps_buffer_pos = 0;
uint16_t apps1_avg = 0;
uint16_t apps2_avg = 0;

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

// vars on can utils file
HV500 myHV500;
AS_System_t as_system;
ACU_t acu;
RES_t res;
BMSvars_t bms;

/* -------------------- STATE MACHINE DEFINITIONS -------------------- */
// VCU state enumeration
typedef enum {
    STATE_INIT,                    // Initial startup state
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
    "STATE_STANDBY",
    "STATE_PRECHARGE",
    "STATE_WAITING_FOR_R2D_MANUAL",
    "STATE_WAITING_FOR_R2D_AUTO",
    "STATE_READY_MANUAL",
    "STATE_READY_AUTONOMOUS",
    "STATE_AS_EMERGENCY"};

// VCU signals structure
typedef struct {
    bool r2d_button_signal;  // R2D signal
    bool r2d_toggle_signal;  // R2D toggle signal
    bool r2d_button_prev;    // Previous state of button for edge detection

    bool r2d_autonomous_signal;  // R2D signal from autonomous system

    uint8_t brake_pressure;  // Brake pressure signal

    bool ignition_ad;             // ignition comming from autonomous system
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

// Initialize all signals to 0
VCU_Signals_t vcu = {
    .precharge_signal = false,

    .r2d_button_signal = false,
    .r2d_toggle_signal = false,
    .r2d_button_prev = false,
    // All other fields will be initialized to 0/false by default
};

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    // Process the data
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void CAN1_Filter_Config(void) {
    CAN_FilterTypeDef canfilterconfig;

    canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
    canfilterconfig.FilterBank = 0;
    canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    canfilterconfig.FilterIdHigh = 0x0000;  // Accept all IDs
    canfilterconfig.FilterIdLow = 0x0000;
    canfilterconfig.FilterMaskIdHigh = 0x0000;
    canfilterconfig.FilterMaskIdLow = 0x0000;
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;

    if (HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig) != HAL_OK) {
        Error_Handler();
    }
}

void CAN2_Filter_Config(void) {
    CAN_FilterTypeDef canfilterconfig;

    canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
    canfilterconfig.FilterBank = 14;
    canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    canfilterconfig.FilterIdHigh = 0x0000;  // Accept all IDs
    canfilterconfig.FilterIdLow = 0x0000;
    canfilterconfig.FilterMaskIdHigh = 0x0000;
    canfilterconfig.FilterMaskIdLow = 0x0000;
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;

    if (HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig) != HAL_OK) {
        Error_Handler();
    }
}

void CAN3_Filter_Config(void) {
    CAN_FilterTypeDef canfilterconfig;

    canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
    canfilterconfig.FilterBank = 0;
    canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;

    canfilterconfig.FilterIdHigh = 0x0000;  // Accept all IDs
    canfilterconfig.FilterIdLow = 0x0000;
    canfilterconfig.FilterMaskIdHigh = 0x0000;
    canfilterconfig.FilterMaskIdLow = 0x0000;

    canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;

    if (HAL_CAN_ConfigFilter(&hcan3, &canfilterconfig) != HAL_OK) {
        Error_Handler();
    }
}

/* -------------------- SENSOR FUNCTIONS -------------------- */
/**
 * @brief Measure the brake pressure from an ADC reading
 * @param bits Raw ADC value (12-bit resolution, 0-4095)
 * @return Calculated brake pressure in bar
 * @details Converts ADC reading to voltage, then to pressure using calibration formula.
 *          Sensor characteristics: 28.57mV/bar with 0.5V offset (0 bar = 0.5V)
 */
float MeasureBrakePressure(uint16_t bits) {
    // Constants for clarity
    const float ADC_MAX = 4095.0f;
    const float MCU_VREF = 3.3f;             // MCU reference voltage
    const float SENSOR_VREF = 5.0f;          // Sensor reference voltage
    const float CONVERSION_FACTOR = 0.667f;  // Factor to convert 3.3V to 5V scale
    const float OFFSET_VOLTAGE = 0.5f;       // 0 bar = 0.5V
    const float SENSITIVITY = 0.02857f;      // 28.57mV/bar

    // Calculate voltage from ADC reading (0-3.3V range)
    float volts = (float)bits * MCU_VREF / ADC_MAX;

    // Scale to sensor voltage range (0-5V)
    volts = volts / CONVERSION_FACTOR;

    // Apply boundary checking for voltage
    if (volts < 0.0f) {
        volts = 0.0f;
    } else if (volts > SENSOR_VREF) {
        volts = SENSOR_VREF;
    }

    // Calculate pressure from voltage using calibration formula:
    // P(bar) = (V - 0.5V) / 0.02857V/bar
    float pressure = 0.0f;
    if (volts <= OFFSET_VOLTAGE) {
        pressure = 0.0f;  // Anything below offset voltage is 0 bar
    } else {
        pressure = (volts - OFFSET_VOLTAGE) / SENSITIVITY;
    }

    // Limit maximum pressure if needed
    const float MAX_PRESSURE = 100.0f;  // Maximum measurable pressure
    if (pressure > MAX_PRESSURE) {
        pressure = MAX_PRESSURE;
    }

    return pressure;  // Return the brake pressure in bar
}

/**
 * @brief Initialize moving average buffers for APPS
 */
void MovingAverage_Init(void) {
    for (int i = 0; i < APPS_MA_WINDOW_SIZE; i++) {
        apps1_buffer[i] = 0;
        apps2_buffer[i] = 0;
    }
    apps_buffer_pos = 0;
    apps1_avg = 0;
    apps2_avg = 0;
}

/**
 * @brief Update moving average with new APPS values
 * @param apps1_raw Raw value from APPS1 sensor
 * @param apps2_raw Raw value from APPS2 sensor
 * @return None, updates apps1_avg and apps2_avg global variables
 */
void MovingAverage_Update(uint16_t apps1_raw, uint16_t apps2_raw) {
    // Add new values to buffers
    apps1_buffer[apps_buffer_pos] = apps1_raw;
    apps2_buffer[apps_buffer_pos] = apps2_raw;

    // Update position for next entry
    apps_buffer_pos = (apps_buffer_pos + 1) % APPS_MA_WINDOW_SIZE;

    // Calculate averages
    uint32_t sum1 = 0;
    uint32_t sum2 = 0;

    for (int i = 0; i < APPS_MA_WINDOW_SIZE; i++) {
        sum1 += apps1_buffer[i];
        sum2 += apps2_buffer[i];
    }

    apps1_avg = (uint16_t)(sum1 / APPS_MA_WINDOW_SIZE);
    apps2_avg = (uint16_t)(sum2 / APPS_MA_WINDOW_SIZE);
}

/* -------------------- LED CONTROL FUNCTIONS -------------------- */
/**
 * @brief Heartbeat indicator with non-blocking double blink pattern
 * @param GPIO_Port GPIO port of the LED
 * @param GPIO_Pin GPIO pin of the LED
 */
void heartbeat_nonblocking(GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin) {
    static uint32_t previous_tick = 0;
    static uint8_t state = 0;

    uint32_t current_tick = HAL_GetTick();

    // State transitions based on timing
    switch (state) {
        case 0:  // First blink ON
            HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_SET);
            if (current_tick - previous_tick >= 100) {  // 100ms ON
                previous_tick = current_tick;
                state = 1;
            }
            break;

        case 1:  // First blink OFF
            HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_RESET);
            if (current_tick - previous_tick >= 100) {  // 100ms OFF
                previous_tick = current_tick;
                state = 2;
            }
            break;

        case 2:  // Second blink ON
            HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_SET);
            if (current_tick - previous_tick >= 100) {  // 100ms ON
                previous_tick = current_tick;
                state = 3;
            }
            break;

        case 3:  // Second blink OFF
            HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_RESET);
            if (current_tick - previous_tick >= 100) {  // 100ms OFF
                previous_tick = current_tick;
                state = 4;
            }
            break;

        case 4:                                         // Long pause
            if (current_tick - previous_tick >= 500) {  // 500ms pause
                previous_tick = current_tick;
                state = 0;  // Restart cycle
            }
            break;
    }
}

/**
 * @brief LED startup animation
 * @note  This function performs a startup animation for the LEDs.
 */
void startup_leds_animation(void) {
    // LED startup animation
    HAL_GPIO_WritePin(GPIOD, LED_IGN_Pin, GPIO_PIN_SET);
    HAL_Delay(40);
    HAL_GPIO_WritePin(GPIOD, LED_R2D_Pin, GPIO_PIN_SET);
    HAL_Delay(40);
    HAL_GPIO_WritePin(GPIOD, LED_AUTO_Pin, GPIO_PIN_SET);
    HAL_Delay(40);
    HAL_GPIO_WritePin(GPIOD, LED_PWT_Pin, GPIO_PIN_SET);
    HAL_Delay(40);
    HAL_GPIO_WritePin(GPIOB, LED_DATA_Pin, GPIO_PIN_SET);
    HAL_Delay(40);
    HAL_GPIO_WritePin(GPIOB, LED_Heartbeat_Pin, GPIO_PIN_SET);
    HAL_Delay(350);
    HAL_GPIO_WritePin(GPIOB, LED_Heartbeat_Pin, GPIO_PIN_RESET);
    HAL_Delay(40);
    HAL_GPIO_WritePin(GPIOB, LED_DATA_Pin, GPIO_PIN_RESET);
    HAL_Delay(40);
    HAL_GPIO_WritePin(GPIOD, LED_PWT_Pin, GPIO_PIN_RESET);
    HAL_Delay(40);
    HAL_GPIO_WritePin(GPIOD, LED_AUTO_Pin, GPIO_PIN_RESET);
    HAL_Delay(40);
    HAL_GPIO_WritePin(GPIOD, LED_R2D_Pin, GPIO_PIN_RESET);
    HAL_Delay(40);
    HAL_GPIO_WritePin(GPIOD, LED_IGN_Pin, GPIO_PIN_RESET);
    HAL_Delay(40);

    // Turn off all LEDs
    HAL_GPIO_WritePin(GPIOD, LED_IGN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, LED_R2D_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, LED_AUTO_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, LED_PWT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LED_DATA_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Fade LED in and out using software PWM
 * @param GPIO_Port GPIO port of the LED
 * @param GPIO_Pin GPIO pin of the LED
 */
void led_fade_nonblocking(GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin) {
    static uint32_t last_update_time = 0;
    static uint32_t fade_counter = 0;
    static uint32_t pwm_counter = 0;
    static const uint32_t PWM_PERIOD = 5;    // PWM period for software PWM
    static const uint32_t FADE_PERIOD = 50;  // How fast the LED brightness changes

    uint32_t current_time = HAL_GetTick();

    // Update fade counter (determines brightness level)
    if (current_time - last_update_time >= FADE_PERIOD) {
        fade_counter = (fade_counter + 1) % 200;  // 0-199 range for smooth transition
        last_update_time = current_time;
        pwm_counter = 0;  // Reset PWM counter on brightness change
    }

    // Calculate current brightness level (0-100)
    uint32_t brightness;
    if (fade_counter < 100) {
        brightness = fade_counter;  // Fade in (0-99)
    } else {
        brightness = 200 - fade_counter;  // Fade out (99-0)
    }

    // Implement software PWM
    pwm_counter = (pwm_counter + 2) % PWM_PERIOD;

    if (pwm_counter < brightness) {
        HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_SET);  // Turn LED on
    } else {
        HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_RESET);  // Turn LED off
    }
}

/**
 * @brief Update PWM value for LED fading effect using hardware timer
 * @param htim PWM timer handle
 * @param channel Timer channel to update
 */
void led_fade_pwm(TIM_HandleTypeDef *htim, uint32_t channel) {
    static uint32_t last_update_time = 0;
    static uint16_t brightness = 0;
    static int8_t direction = 1;                 // 1 = increasing, -1 = decreasing
    static const uint32_t FADE_PERIOD = 10;      // Update interval in ms
    static const uint16_t MAX_BRIGHTNESS = 100;  // Maximum PWM value

    uint32_t current_time = HAL_GetTick();

    // Update PWM value at regular intervals
    if (current_time - last_update_time >= FADE_PERIOD) {
        last_update_time = current_time;

        // Update brightness based on current direction
        brightness += direction;

        // Change direction at limits
        if (brightness >= MAX_BRIGHTNESS) {
            direction = -1;  // Start decreasing
        } else if (brightness <= 0) {
            direction = 1;  // Start increasing
        }

        // Apply new PWM value (constraining to valid range)
        if (brightness > MAX_BRIGHTNESS) brightness = MAX_BRIGHTNESS;
        if (brightness < 0) brightness = 0;

        __HAL_TIM_SET_COMPARE(htim, channel, brightness);
    }
}

/* -------------------- R2D SOUND FUNCTIONS -------------------- */
/**
 * @brief Start playing the Ready-To-Drive sound
 * @details This function activates the buzzer to play the R2D sound
 *          as required by EV4.12 regulations (80-90 dBA, 1-3 seconds).
 */
void StartR2DSound(void) {
    // Activate only if not already playing
    if (!vcu.r2d_sound_playing && !vcu.r2d_sound_completed) {
        vcu.r2d_sound_playing = true;
        vcu.r2d_sound_start_time = HAL_GetTick();

        // Activate buzzer - set the GPIO pin that controls the buzzer
        HAL_GPIO_WritePin(GPIOD, dout4_R2D_Buzzer_Pin, GPIO_PIN_SET);

        printf("\n\rR2D Sound: Started\n\r");
    }
}

/**
 * @brief Update the Ready-To-Drive sound state
 * @details Checks if R2D sound needs to be stopped based on timing
 *          requirements (1-3 seconds continuous sound).
 */
void UpdateR2DSound(void) {
    // If sound is playing, check if it's time to stop
    if (vcu.r2d_sound_playing) {
        uint32_t current_time = HAL_GetTick();
        uint32_t elapsed_time = current_time - vcu.r2d_sound_start_time;

        // Sound duration: 2 seconds (2000 ms)
        if (elapsed_time >= 1000) {
            // Stop the sound
            HAL_GPIO_WritePin(GPIOD, dout4_R2D_Buzzer_Pin, GPIO_PIN_RESET);
            vcu.r2d_sound_playing = false;
            vcu.r2d_sound_completed = true;

            printf("\n\rR2D Sound: Completed\n\r");
        }
    }
}

/**
 * @brief Reset the Ready-To-Drive sound state
 * @details Resets the R2D sound flags when leaving a ready state.
 */
void ResetR2DSound(void) {
    vcu.r2d_sound_playing = false;
    vcu.r2d_sound_completed = false;
    HAL_GPIO_WritePin(GPIOD, dout4_R2D_Buzzer_Pin, GPIO_PIN_RESET);
}

/**
 * @brief Start playing the Emergency sound
 * @details This function activates the buzzer to play an intermittent emergency sound
 *          with frequency 1-5Hz, 50% duty cycle, for 8-10 seconds.
 */
void StartEmergencySound(void) {
    // Only start if not already playing
    if (!vcu.emergency_sound_playing && !vcu.emergency_sound_completed) {
        vcu.emergency_sound_playing = true;
        vcu.emergency_sound_start_time = HAL_GetTick();
        vcu.emergency_sound_last_toggle = HAL_GetTick();
        vcu.emergency_sound_state = true;  // Start with buzzer ON

        // Activate buzzer
        HAL_GPIO_WritePin(GPIOD, dout4_R2D_Buzzer_Pin, GPIO_PIN_SET);

        printf("\n\rEmergency Sound: Started\n\r");
    }
}

/**
 * @brief Update the Emergency sound state
 * @details Manages the intermittent pattern (1-5Hz, 50% duty cycle) and duration (8-10s)
 *          of the emergency sound.
 */
void UpdateEmergencySound(void) {
    if (vcu.emergency_sound_playing) {
        uint32_t current_time = HAL_GetTick();
        uint32_t total_elapsed_time = current_time - vcu.emergency_sound_start_time;
        uint32_t toggle_elapsed_time = current_time - vcu.emergency_sound_last_toggle;

        // Using 2.5Hz frequency (200ms ON, 200ms OFF)
        const uint32_t TOGGLE_INTERVAL_MS = 200;

        // Toggle buzzer state at specified frequency (2.5Hz)
        if (toggle_elapsed_time >= TOGGLE_INTERVAL_MS) {
            vcu.emergency_sound_state = !vcu.emergency_sound_state;
            HAL_GPIO_WritePin(GPIOD, dout4_R2D_Buzzer_Pin,
                              vcu.emergency_sound_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
            vcu.emergency_sound_last_toggle = current_time;
        }

        // Total sound duration: 9 seconds
        const uint32_t EMERGENCY_SOUND_DURATION_MS = 9000;

        // Stop sound after duration completes
        if (total_elapsed_time >= EMERGENCY_SOUND_DURATION_MS) {
            // Stop the sound
            HAL_GPIO_WritePin(GPIOD, dout4_R2D_Buzzer_Pin, GPIO_PIN_RESET);
            vcu.emergency_sound_playing = false;
            vcu.emergency_sound_completed = true;

            printf("\n\rEmergency Sound: Completed\n\r");
        }
    }
}

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
    if ((as_system.state == 4) || res.signal == 0) {
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

/**
 * @brief Update the state of the VCU based on inputs and conditions
 */
void UpdateState(void) {
    // Store current state for change detection
    previous_state = current_state;

    // Process emergency condition with highest priority
    if (((as_system.state == 4) || (res.signal == 0)) && current_state != STATE_AS_EMERGENCY) {
        current_state = STATE_AS_EMERGENCY;
    }

    vcu.ignition_switch_signal = HAL_GPIO_ReadPin(int1_ign_GPIO_Port, int1_ign_Pin);
    vcu.ignition_ad = acu.ignition_ad;  // Read ignition signal from autonomous system

    // State transitions
    switch (current_state) {
        case STATE_INIT:
            current_state = STATE_STANDBY;  // Transition to standby state after initialization
            break;
        case STATE_STANDBY:
            // if precharge is request either for manual or autonomous mode start the precharge
            if (vcu.ignition_switch_signal) {
                current_state = STATE_PRECHARGE;  // Transition to precharge state
                vcu.manual = true;                // Set manual mode
                vcu.autonomous = false;           // Clear autonomous mode
            } else if (vcu.ignition_ad && acu.ASMS) {
                current_state = STATE_PRECHARGE;  // Transition to precharge state
                vcu.manual = false;               // Clear manual mode
                vcu.autonomous = true;            // Set autonomous mode
            }
            break;

        case STATE_PRECHARGE:
            // when the precharge is done, wait for the ready to drive signal to start the ready to drive
            // either from manual or autonomous mode-

            // Check if precharge is complete
            // TODO fazer a diferenca entre a tensao do inversor e do bms e ver se esta a 90% reais

            if (!vcu.ignition_switch_signal && !vcu.ignition_ad) {
                current_state = STATE_STANDBY;
            } else if (Bypass_precharge || (bms.precharge_circuit_state == 9) || myHV500.Actual_InputVoltage > 450) {
                current_state = vcu.manual ? STATE_WAITING_FOR_R2D_MANUAL : STATE_WAITING_FOR_R2D_AUTO;
            }
            break;

        case STATE_WAITING_FOR_R2D_MANUAL:

            if (!vcu.ignition_switch_signal) {
                current_state = STATE_STANDBY;
            } else if (vcu.r2d_toggle_signal && (Bypass_brake_pressure || vcu.brake_pressure > BRAKE_PRESSURE_THRESHOLD)) {
                current_state = STATE_READY_MANUAL;
            }
            break;

        case STATE_WAITING_FOR_R2D_AUTO:
            if (!vcu.ignition_ad) {
                current_state = STATE_STANDBY;
            } else if (as_system.state == 3) {  // driving state
                current_state = STATE_READY_AUTONOMOUS;
            }
            break;

        case STATE_READY_MANUAL:

            if (!vcu.ignition_switch_signal) {
                // Change to standby if ignition is off
                current_state = STATE_STANDBY;
            } else if (!vcu.r2d_toggle_signal) {
                // Change to waiting for R2D manual if the button is pressed again
                current_state = STATE_WAITING_FOR_R2D_MANUAL;
            }
            break;

        case STATE_READY_AUTONOMOUS:
            // serve as gateway to the computer to send the commands to the inverter

            if (!vcu.ignition_ad) {
                current_state = STATE_STANDBY;
            } else if (as_system.state != 3) {
                current_state = STATE_WAITING_FOR_R2D_AUTO;
            }
            break;

        case STATE_AS_EMERGENCY:
            // TODO fazer isto verificar com os autonomos e o bruno
            // printf("\n\rAS Emergency: %d\n\r", as_system.state);

            if (vcu.ignition_ad == false) {
                current_state = STATE_STANDBY;
            } else if (as_system.state != 4 && res.signal != 0) {
                current_state = STATE_STANDBY;
            }

            break;
    }

    // Execute entry actions when state has changed
    if (current_state != previous_state) {
#ifdef print_state
        print_state_transition(previous_state, current_state);
#endif
        // State entry actions
        switch (current_state) {
            case STATE_INIT:
                break;

            case STATE_STANDBY:
                // Reset critical flags when entering standby
                vcu.manual = false;
                vcu.autonomous = false;

                memset(&vcu, 0, sizeof(VCU_Signals_t));      // Reset all VCU signals
                memset(&as_system, 0, sizeof(AS_System_t));  // Reset autonomous system state
                memset(&acu, 0, sizeof(ACU_t));              // Reset ACU state
                // memset(&res, 0, sizeof(RES_t));              // Reset RES state
                // memset(&bms, 0, sizeof(BMSvars_t));  // Reset BMS variables
                memset(&myHV500, 0, sizeof(HV500));  // Reset HV500 variables

                vcu.r2d_button_signal = false;      // Reset R2D button signal
                vcu.r2d_toggle_signal = false;      // Reset R2D toggle signal
                vcu.r2d_autonomous_signal = false;  // Reset R2D autonomous signal
                ResetR2DSound();                    // Reset R2D sound state
                HAL_GPIO_WritePin(GPIOB, dout1_BMS_IGN_Pin, GPIO_PIN_RESET);

                //__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 0);  // Set PWM to 0% duty cycle
                // HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);         // Stop PWM for R2D LED

                HAL_GPIO_WritePin(GPIOD, LED_IGN_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOD, LED_R2D_Pin, GPIO_PIN_RESET);
                vcu.r2d_button_signal = false;  // Reset R2D button signal
                break;

            case STATE_PRECHARGE:
                // Precharge state entry actions
                HAL_GPIO_WritePin(GPIOD, LED_IGN_Pin, GPIO_PIN_SET);        // Turn on ignition LED debug
                HAL_GPIO_WritePin(GPIOB, dout1_BMS_IGN_Pin, GPIO_PIN_SET);  // Activate BMS ignition

                //__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 0);  // Set PWM to 0% duty cycle
                // HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);         // Stop PWM for R2D LED
                break;

            case STATE_WAITING_FOR_R2D_AUTO:
            case STATE_WAITING_FOR_R2D_MANUAL:
                HAL_GPIO_WritePin(GPIOB, dout1_BMS_IGN_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(GPIOD, LED_R2D_Pin, GPIO_PIN_RESET);  // Turn off R2D LED
                // HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
                ResetR2DSound();  // Reset R2D sound state
                break;

            case STATE_READY_MANUAL:
            case STATE_READY_AUTONOMOUS:
                // Common setup for ready states
                HAL_GPIO_WritePin(GPIOB, dout1_BMS_IGN_Pin, GPIO_PIN_SET);
                // r2d led 100% on
                //__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 100);  // Set PWM to 100% duty cycle
                HAL_GPIO_WritePin(GPIOD, LED_R2D_Pin, GPIO_PIN_SET);

                // Play R2D sound when entering a ready state
                StartR2DSound();
                break;

            case STATE_AS_EMERGENCY:
                // Emergency entry actions
                HAL_GPIO_WritePin(GPIOB, dout1_BMS_IGN_Pin, GPIO_PIN_RESET);
                // HAL_GPIO_WritePin(GPIOB, dout1_BMS_IGN_Pin, !GPIO_PIN_RESET);
                //__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 0);  // Set PWM to 0% duty cycle
                // HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);         // Stop PWM for R2D LED
                break;
        }
    }
}

/**
 * @brief Handle actions specific to the current state
 */
void HandleState(void) {
    // Update R2D sound status if it's playing
    UpdateR2DSound();

    switch (current_state) {
        case STATE_INIT:
            // Initialization actions
            break;

        case STATE_STANDBY:

            break;

        case STATE_PRECHARGE:
            // Handle precharge process

            break;

        case STATE_WAITING_FOR_R2D_MANUAL:
            // fade in and out the R2D LED
            led_fade_nonblocking(GPIOD, LED_R2D_Pin);
            // Use hardware PWM for LED fading
            // led_fade_pwm(&htim4, TIM_CHANNEL_1);
            break;

        case STATE_WAITING_FOR_R2D_AUTO:
            // Waiting indicator for autonomous mode
            led_fade_nonblocking(GPIOD, LED_R2D_Pin);
            // Use hardware PWM for LED fading
            // led_fade_pwm(&htim4, TIM_CHANNEL_1);

            break;

        case STATE_READY_MANUAL:
            // Process manual driving controls
            // Send pedal position to inverter

            static uint32_t last_can_send_time_manuel = 0;
            uint32_t current_time_manuel = HAL_GetTick();

            if (current_time_manuel - last_can_send_time_manuel >= 5) {
                can_bus_send_HV500_SetDriveEnable(1, &hcan2);
                can_bus_send_HV500_SetRelCurrent(result.percentage_1000, &hcan2);
                last_can_send_time_manuel = current_time_manuel;
            }

            break;

        case STATE_READY_AUTONOMOUS:
            // Forward autonomous commands to inverter
            static uint32_t last_can_send_time_auto = 0;
            uint32_t current_time_auto = HAL_GetTick();

            if (current_time_auto - last_can_send_time_auto >= 10) {
                can_bus_send_HV500_SetDriveEnable(1, &hcan2);

                if (as_system.target_rpm > MAX_RPM_AD) {
                    as_system.target_rpm = MAX_RPM_AD;
                } else if (as_system.target_rpm < 0) {
                    as_system.target_rpm = 0;
                }

                uint32_t erpm = as_system.target_rpm * 10;
                can_bus_send_HV500_SetERPM(erpm, &hcan2);
                can_send_vcu_rpm(&hcan3, myHV500.Actual_ERPM / 10);
                last_can_send_time_auto = current_time_auto;

                // printf("\n\rRPM: %d\n\r", myHV500.Actual_ERPM / 10);
                // printf("\n\rTarget RPM: %d\n\r", as_system.target_rpm);
            }
            break;

        case STATE_AS_EMERGENCY:
            // Activate emergency indicators
            // Handle emergency state
            HandleEmergencyStop();
            break;
    }
}

// Time-based debounce for R2D button
void debounce_r2d_button(void) {
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
}

/* -------------------- COMMUNICATION FUNCTIONS -------------------- */

/**
 * @brief Redirect printf to UART
 */
PUTCHAR_PROTOTYPE {
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART1 and Loop until the end of transmission */
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

    return ch;
}

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
  MX_ADC3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CAN3_Init();
  MX_TIM4_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */

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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    // HAL_ADC_Start_DMA(&hadc1, ADC1_VAL, 4);
    HAL_ADC_Start_DMA(&hadc2, ADC2_APPS, 2);  // Start ADC2 for APPS
                                              // Calibrate APPS

    APPS_Init(__APPS_MIN_BITS, __APPS_MAX_BITS, __APPS_TOLERANCE, __APPS_DELTA);  // Initialize APPS
    res.signal = 12;                                                              // start with a value different than 0 to avoid emergency state
    printf("\n\n\n\n\n======================== RESET ========================\n\n\n\n\n\r");

#if CALIBRATE_APPS
    APPS_StartCalibration();
#endif

    while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        heartbeat_nonblocking(GPIOB, LED_Heartbeat_Pin);

        if (HAL_CAN_GetRxFifoFillLevel(&hcan3, CAN_RX_FIFO0) > 0) {
            CAN_RxHeaderTypeDef RxHeader;
            uint8_t RxData[8];
            if (HAL_CAN_GetRxMessage(&hcan3, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
                decode_auto_bus(RxHeader, RxData);
            }
        }

        if (HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0) > 0) {
            CAN_RxHeaderTypeDef RxHeader2;
            uint8_t RxData2[8];
            if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader2, RxData2) == HAL_OK) {
                can_filter_id_bus2(RxHeader2, RxData2);
            }
        }

        static uint32_t previous_tick = 0;
        uint32_t current_tick = HAL_GetTick();
        if (current_tick - previous_tick >= 200) {
            can_send_autonomous_HV_signal(&hcan3, bms.precharge_circuit_state);

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

            result = APPS_Process(apps2_avg, apps1_avg);

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
#if print_apps
            APPS_PrintStatus();
#endif

            vcu.brake_pressure = MeasureBrakePressure(ADC1_VAL[0]);
            // printf("\n\rBrake Pressure: %d\n\r", vcu.brake_pressure);

            previous_tick = current_tick;
        }

        // Update moving average with new APPS values
        MovingAverage_Update(ADC2_APPS[0], ADC2_APPS[1]);

        // Check for CAN timeouts
        // CheckCANTimeouts();

        // Handle R2D button debounce
        debounce_r2d_button();

        // Update state based on inputs
        UpdateState();

        // Execute state-specific actions
        HandleState();
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
