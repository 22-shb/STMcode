/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, motorDE1_Pin|motorDE2_Pin|motorCE2_Pin|motorCE1_Pin
                          |motorALLE_Pin|motorAE1_Pin|motorAE2_Pin|motorBE2_Pin
                          |motorBE1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : motorDE1_Pin motorDE2_Pin motorCE2_Pin motorCE1_Pin
                           motorALLE_Pin motorAE1_Pin motorAE2_Pin motorBE2_Pin
                           motorBE1_Pin */
  GPIO_InitStruct.Pin = motorDE1_Pin|motorDE2_Pin|motorCE2_Pin|motorCE1_Pin
                          |motorALLE_Pin|motorAE1_Pin|motorAE2_Pin|motorBE2_Pin
                          |motorBE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : input8_Pin input7_Pin input5_Pin input4_Pin
                           input3_Pin input2_Pin */
  GPIO_InitStruct.Pin = input8_Pin|input7_Pin|input5_Pin|input4_Pin
                          |input3_Pin|input2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : input1_Pin input6_Pin */
  GPIO_InitStruct.Pin = input1_Pin|input6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
