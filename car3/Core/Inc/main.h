/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

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
#define motorDE1_Pin GPIO_PIN_0
#define motorDE1_GPIO_Port GPIOA
#define motorDE2_Pin GPIO_PIN_1
#define motorDE2_GPIO_Port GPIOA
#define motorCE2_Pin GPIO_PIN_2
#define motorCE2_GPIO_Port GPIOA
#define motorCE1_Pin GPIO_PIN_3
#define motorCE1_GPIO_Port GPIOA
#define motorALLE_Pin GPIO_PIN_4
#define motorALLE_GPIO_Port GPIOA
#define motorAE1_Pin GPIO_PIN_5
#define motorAE1_GPIO_Port GPIOA
#define motorAE2_Pin GPIO_PIN_6
#define motorAE2_GPIO_Port GPIOA
#define motorBE2_Pin GPIO_PIN_7
#define motorBE2_GPIO_Port GPIOA
#define input8_Pin GPIO_PIN_0
#define input8_GPIO_Port GPIOB
#define input7_Pin GPIO_PIN_1
#define input7_GPIO_Port GPIOB
#define motorCPWM_Pin GPIO_PIN_8
#define motorCPWM_GPIO_Port GPIOA
#define motorDPWM_Pin GPIO_PIN_9
#define motorDPWM_GPIO_Port GPIOA
#define motorBPWM_Pin GPIO_PIN_10
#define motorBPWM_GPIO_Port GPIOA
#define motorAPWM_Pin GPIO_PIN_11
#define motorAPWM_GPIO_Port GPIOA
#define motorBE1_Pin GPIO_PIN_12
#define motorBE1_GPIO_Port GPIOA
#define input5_Pin GPIO_PIN_3
#define input5_GPIO_Port GPIOB
#define input4_Pin GPIO_PIN_4
#define input4_GPIO_Port GPIOB
#define input3_Pin GPIO_PIN_5
#define input3_GPIO_Port GPIOB
#define input2_Pin GPIO_PIN_6
#define input2_GPIO_Port GPIOB
#define input1_Pin GPIO_PIN_7
#define input1_GPIO_Port GPIOB
#define input6_Pin GPIO_PIN_9
#define input6_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
