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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define M1_DIR_Pin GPIO_PIN_0
#define M1_DIR_GPIO_Port GPIOA
#define M2_DIR_Pin GPIO_PIN_1
#define M2_DIR_GPIO_Port GPIOA
#define M3_DIR_Pin GPIO_PIN_2
#define M3_DIR_GPIO_Port GPIOA
#define M1_NBRAKE_Pin GPIO_PIN_3
#define M1_NBRAKE_GPIO_Port GPIOA
#define M2_NBRAKE_Pin GPIO_PIN_4
#define M2_NBRAKE_GPIO_Port GPIOA
#define M3_NBRAKE_Pin GPIO_PIN_5
#define M3_NBRAKE_GPIO_Port GPIOA
#define M2_HB_Pin GPIO_PIN_10
#define M2_HB_GPIO_Port GPIOB
#define M2_HB_EXTI_IRQn EXTI15_10_IRQn
#define M2_HC_Pin GPIO_PIN_11
#define M2_HC_GPIO_Port GPIOB
#define M2_HC_EXTI_IRQn EXTI15_10_IRQn
#define M3_HA_Pin GPIO_PIN_12
#define M3_HA_GPIO_Port GPIOB
#define M3_HA_EXTI_IRQn EXTI15_10_IRQn
#define M3_HB_Pin GPIO_PIN_13
#define M3_HB_GPIO_Port GPIOB
#define M3_HB_EXTI_IRQn EXTI15_10_IRQn
#define M3_HC_Pin GPIO_PIN_14
#define M3_HC_GPIO_Port GPIOB
#define M3_HC_EXTI_IRQn EXTI15_10_IRQn
#define M1_HA_Pin GPIO_PIN_6
#define M1_HA_GPIO_Port GPIOB
#define M1_HA_EXTI_IRQn EXTI9_5_IRQn
#define M1_HB_Pin GPIO_PIN_7
#define M1_HB_GPIO_Port GPIOB
#define M1_HB_EXTI_IRQn EXTI9_5_IRQn
#define M1_HC_Pin GPIO_PIN_8
#define M1_HC_GPIO_Port GPIOB
#define M1_HC_EXTI_IRQn EXTI9_5_IRQn
#define M2_HA_Pin GPIO_PIN_9
#define M2_HA_GPIO_Port GPIOB
#define M2_HA_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
