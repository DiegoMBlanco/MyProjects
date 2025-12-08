/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
/* USER CODE BEGIN Variables */
//extern ADC_HandleTypeDef hadc2;
extern UART_HandleTypeDef huart3;
extern SPI_HandleTypeDef hspi1;
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
void vApplicationIdleHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )

{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
	//HAL_ADC_Stop(&hadc2);
	//HAL_UART_DeInit(&huart3);
	//HAL_SPI_DeInit(&hspi1);   // si tu RFID usa SPI
	// Suspende el tick de FreeRTOS (usa TIM6 normalmente)
	HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, 0);
	HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, 0);
	HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, 0);
	HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, 0);
	HAL_SuspendTick();


	// Espera a una interrupción externa (por ejemplo, botón EXTI)
	__WFI();

	// Al despertar

	// Reactiva el tick
	HAL_ResumeTick();
	//SystemClock_Config();      // ★★★ RESTAURA LOS RELOJES ★★★
	HAL_GPIO_TogglePin(LED_AMARILLO_GPIO_Port, LED_AMARILLO_Pin);
}
/* USER CODE END 2 */

/* USER CODE BEGIN PREPOSTSLEEP */
__weak void PreSleepProcessing(uint32_t ulExpectedIdleTime)
{
/* place for user code */
}

__weak void PostSleepProcessing(uint32_t ulExpectedIdleTime)
{
/* place for user code */
}
/* USER CODE END PREPOSTSLEEP */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
