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
#define K_6B_Pin GPIO_PIN_13
#define K_6B_GPIO_Port GPIOC
#define K_SW_Pin GPIO_PIN_0
#define K_SW_GPIO_Port GPIOA
#define K_5A_Pin GPIO_PIN_3
#define K_5A_GPIO_Port GPIOA
#define K_5B_Pin GPIO_PIN_4
#define K_5B_GPIO_Port GPIOA
#define K_1A_Pin GPIO_PIN_5
#define K_1A_GPIO_Port GPIOA
#define K_1B_Pin GPIO_PIN_6
#define K_1B_GPIO_Port GPIOA
#define K_2A_Pin GPIO_PIN_7
#define K_2A_GPIO_Port GPIOA
#define K_3A_Pin GPIO_PIN_0
#define K_3A_GPIO_Port GPIOB
#define K_6A_Pin GPIO_PIN_1
#define K_6A_GPIO_Port GPIOB
#define K_2B_Pin GPIO_PIN_10
#define K_2B_GPIO_Port GPIOB
#define NRF_IRQ_Pin GPIO_PIN_12
#define NRF_IRQ_GPIO_Port GPIOB
#define NRF_IRQ_EXTI_IRQn EXTI15_10_IRQn
#define K_4A_Pin GPIO_PIN_11
#define K_4A_GPIO_Port GPIOA
#define K_4B_Pin GPIO_PIN_12
#define K_4B_GPIO_Port GPIOA
#define K_3B_Pin GPIO_PIN_3
#define K_3B_GPIO_Port GPIOB
#define K_EB_Pin GPIO_PIN_5
#define K_EB_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_7
#define LED1_GPIO_Port GPIOB
#define NRF_CE_Pin GPIO_PIN_8
#define NRF_CE_GPIO_Port GPIOB
#define NRF_CSN_Pin GPIO_PIN_9
#define NRF_CSN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
