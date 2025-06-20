/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, dout3_Water_Pump_Pin|dout4_R2D_Buzzer_Pin|LED_IGN_Pin|LED_R2D_Pin
                          |LED_AUTO_Pin|LED_PWT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_DATA_Pin|LED_Heartbeat_Pin|dout1_BMS_IGN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : int1_ign_Pin */
  GPIO_InitStruct.Pin = int1_ign_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(int1_ign_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : int2_r2d_Pin */
  GPIO_InitStruct.Pin = int2_r2d_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(int2_r2d_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : int3_shutdown_signal_Pin */
  GPIO_InitStruct.Pin = int3_shutdown_signal_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(int3_shutdown_signal_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : dout3_Water_Pump_Pin LED_IGN_Pin LED_R2D_Pin LED_AUTO_Pin
                           LED_PWT_Pin */
  GPIO_InitStruct.Pin = dout3_Water_Pump_Pin|LED_IGN_Pin|LED_R2D_Pin|LED_AUTO_Pin
                          |LED_PWT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : dout4_R2D_Buzzer_Pin */
  GPIO_InitStruct.Pin = dout4_R2D_Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(dout4_R2D_Buzzer_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_DATA_Pin LED_Heartbeat_Pin dout1_BMS_IGN_Pin */
  GPIO_InitStruct.Pin = LED_DATA_Pin|LED_Heartbeat_Pin|dout1_BMS_IGN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
