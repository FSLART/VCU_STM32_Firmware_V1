/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define adc1_in1_brake_pressure_Pin GPIO_PIN_1
#define adc1_in1_brake_pressure_GPIO_Port GPIOA
#define APPS1_Signal_Pin GPIO_PIN_5
#define APPS1_Signal_GPIO_Port GPIOA
#define APPS2_Signal_Pin GPIO_PIN_6
#define APPS2_Signal_GPIO_Port GPIOA
#define int1_ign_Pin GPIO_PIN_7
#define int1_ign_GPIO_Port GPIOA
#define int2_r2d_Pin GPIO_PIN_4
#define int2_r2d_GPIO_Port GPIOC
#define dout3_Water_Pump_Pin GPIO_PIN_14
#define dout3_Water_Pump_GPIO_Port GPIOD
#define dout4_R2D_Buzzer_Pin GPIO_PIN_15
#define dout4_R2D_Buzzer_GPIO_Port GPIOD
#define LED_IGN_Pin GPIO_PIN_4
#define LED_IGN_GPIO_Port GPIOD
#define LED_R2D_Pin GPIO_PIN_5
#define LED_R2D_GPIO_Port GPIOD
#define LED_AUTO_Pin GPIO_PIN_6
#define LED_AUTO_GPIO_Port GPIOD
#define LED_PWT_Pin GPIO_PIN_7
#define LED_PWT_GPIO_Port GPIOD
#define LED_DATA_Pin GPIO_PIN_3
#define LED_DATA_GPIO_Port GPIOB
#define LED_Heartbeat_Pin GPIO_PIN_4
#define LED_Heartbeat_GPIO_Port GPIOB
#define dout1_BMS_IGN_Pin GPIO_PIN_7
#define dout1_BMS_IGN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
