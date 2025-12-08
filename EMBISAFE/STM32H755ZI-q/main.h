
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
#include "stm32h7xx_hal.h"

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
#define BIT1_Pin GPIO_PIN_2
#define BIT1_GPIO_Port GPIOE
#define USER_BTN_Pin GPIO_PIN_13
#define USER_BTN_GPIO_Port GPIOC
#define USER_BTN_EXTI_IRQn EXTI15_10_IRQn
#define LED_VERDE_Pin GPIO_PIN_0
#define LED_VERDE_GPIO_Port GPIOB
#define ESP_Pin3_Pin GPIO_PIN_9
#define ESP_Pin3_GPIO_Port GPIOE
#define L8_Pin GPIO_PIN_11
#define L8_GPIO_Port GPIOE
#define L10_Pin GPIO_PIN_13
#define L10_GPIO_Port GPIOE
#define L9_Pin GPIO_PIN_14
#define L9_GPIO_Port GPIOE
#define MY_WAKE_Pin GPIO_PIN_15
#define MY_WAKE_GPIO_Port GPIOE
#define MY_WAKE_EXTI_IRQn EXTI15_10_IRQn
#define MODE_Pin GPIO_PIN_10
#define MODE_GPIO_Port GPIOB
#define WAKE_UP_Pin GPIO_PIN_11
#define WAKE_UP_GPIO_Port GPIOB
#define LED_ROJO_Pin GPIO_PIN_14
#define LED_ROJO_GPIO_Port GPIOB
#define L3_Pin GPIO_PIN_15
#define L3_GPIO_Port GPIOB
#define BIT2_Pin GPIO_PIN_11
#define BIT2_GPIO_Port GPIOD
#define BIT3_Pin GPIO_PIN_12
#define BIT3_GPIO_Port GPIOD
#define BIT4_Pin GPIO_PIN_13
#define BIT4_GPIO_Port GPIOD
#define L1_Pin GPIO_PIN_6
#define L1_GPIO_Port GPIOC
#define L6_Pin GPIO_PIN_7
#define L6_GPIO_Port GPIOC
#define L5_Pin GPIO_PIN_15
#define L5_GPIO_Port GPIOA
#define B4_Pin GPIO_PIN_10
#define B4_GPIO_Port GPIOC
#define B3_Pin GPIO_PIN_11
#define B3_GPIO_Port GPIOC
#define B2_Pin GPIO_PIN_12
#define B2_GPIO_Port GPIOC
#define ESP_Pin1_Pin GPIO_PIN_0
#define ESP_Pin1_GPIO_Port GPIOD
#define ESP_Pin2_Pin GPIO_PIN_1
#define ESP_Pin2_GPIO_Port GPIOD
#define B1_Pin GPIO_PIN_2
#define B1_GPIO_Port GPIOD
#define L7_Pin GPIO_PIN_12
#define L7_GPIO_Port GPIOG
#define L2_Pin GPIO_PIN_8
#define L2_GPIO_Port GPIOB
#define L4_Pin GPIO_PIN_9
#define L4_GPIO_Port GPIOB
#define LED_AMARILLO_Pin GPIO_PIN_1
#define LED_AMARILLO_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
