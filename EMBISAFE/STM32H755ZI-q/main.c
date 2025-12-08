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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rc522.h"
#include "string.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	volatile uint8_t size;
	volatile uint8_t M4toM7[10];
}shared_data;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
#define DUAL_CORE_BOOT_SYNC_SEQUENCE

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
osThreadId_t RFIDTaskHandle;
const osThreadAttr_t RFIDTask_attr = {
	.name = "RFIDTask",
	.stack_size = 1024 * 4,
	.priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t HuellaTaskHandle;
const osThreadAttr_t HuellaTask_attr = {
	.name = "HuellaTask",
	.stack_size = 1024 * 4,
	.priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t VozTaskHandle;
const osThreadAttr_t VozTask_attr = {
	.name = "VozTask",
	.stack_size = 1024 * 4,
	.priority = (osPriority_t) osPriorityNormal,
};

//CREACIÓN DEL HANDLER DE MI QUEUE!
osMessageQueueId_t queueSpeedHandler;
const osMessageQueueAttr_t queueSpeed_attr={
		.name = "queueSpeed",
};

//MUTEX PARA UART
osMutexId_t uartMutexHandle;
const osMutexAttr_t uartMutex_attr = {
  .name = "uartMutex",
};

uint8_t PersonaRFID = 0;
uint8_t PersonaHuella = 0;
uint8_t PersonaVoz = 0;
uint8_t RFID_Check = 0;
uint8_t Huella_Check = 0;
uint8_t Voz_Check = 0;
uint8_t errorCounter = 0;
uint32_t buffer = 0;
uint16_t adc_value16 = 0;
uint8_t msg[128];
uint8_t msg2[128];
volatile uint8_t woke_by_button = 0;

//RFID
uint8_t status;
uint8_t str[MAX_LEN]; // Max_LEN = 16
uint8_t sNum[5];


volatile shared_data * const dev = (struct shared_data *)0x38001000;
char msg_adc[] = "\r\n";
char buffer_serial[64];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void RFID_Task(void *argument);
void Huella_Task(void *argument);
void Voz_Task(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  int32_t timeout;
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
  /* Wait until CPU2 boots and enters in stop mode or timeout*/
  timeout = 0xFFFF;
  while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
  if ( timeout < 0 )
  {
  Error_Handler();
  }
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
HSEM notification */
/*HW semaphore Clock enable*/
__HAL_RCC_HSEM_CLK_ENABLE();
/*Take HSEM */
HAL_HSEM_FastTake(HSEM_ID_0);
/*Release HSEM in order to notify the CPU2(CM4)*/
HAL_HSEM_Release(HSEM_ID_0,0);
/* wait until CPU2 wakes up from stop mode */
timeout = 0xFFFF;
while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
if ( timeout < 0 )
{
Error_Handler();
}
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  MFRC522_Init();
  dev->size = 10;
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  uartMutexHandle = osMutexNew(&uartMutex_attr);
	if (uartMutexHandle == NULL) {
	  Error_Handler(); // o manejar el fallo
	}
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  queueSpeedHandler = osMessageQueueNew(5, sizeof(uint16_t), &queueSpeed_attr);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  RFIDTaskHandle = osThreadNew(RFID_Task, NULL, &RFIDTask_attr);
  HuellaTaskHandle = osThreadNew(Huella_Task, NULL, &HuellaTask_attr);
  VozTaskHandle = osThreadNew(Voz_Task, NULL, &VozTask_attr);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_VERDE_Pin|MODE_Pin|WAKE_UP_Pin|LED_ROJO_Pin
                          |L3_Pin|L2_Pin|L4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, ESP_Pin3_Pin|L8_Pin|L10_Pin|L9_Pin
                          |LED_AMARILLO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14|GPIO_PIN_15|ESP_Pin1_Pin|ESP_Pin2_Pin
                          |B1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, L1_Pin|L6_Pin|B4_Pin|B3_Pin
                          |B2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(L5_GPIO_Port, L5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(L7_GPIO_Port, L7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BIT1_Pin */
  GPIO_InitStruct.Pin = BIT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BIT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_BTN_Pin */
  GPIO_InitStruct.Pin = USER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_VERDE_Pin MODE_Pin WAKE_UP_Pin LED_ROJO_Pin
                           L3_Pin L2_Pin L4_Pin */
  GPIO_InitStruct.Pin = LED_VERDE_Pin|MODE_Pin|WAKE_UP_Pin|LED_ROJO_Pin
                          |L3_Pin|L2_Pin|L4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ESP_Pin3_Pin */
  GPIO_InitStruct.Pin = ESP_Pin3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ESP_Pin3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : L8_Pin L10_Pin L9_Pin LED_AMARILLO_Pin */
  GPIO_InitStruct.Pin = L8_Pin|L10_Pin|L9_Pin|LED_AMARILLO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : MY_WAKE_Pin */
  GPIO_InitStruct.Pin = MY_WAKE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MY_WAKE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BIT2_Pin BIT3_Pin BIT4_Pin */
  GPIO_InitStruct.Pin = BIT2_Pin|BIT3_Pin|BIT4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD14 PD15 B1_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15|B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : L1_Pin L6_Pin B4_Pin B3_Pin
                           B2_Pin */
  GPIO_InitStruct.Pin = L1_Pin|L6_Pin|B4_Pin|B3_Pin
                          |B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : L5_Pin */
  GPIO_InitStruct.Pin = L5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(L5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ESP_Pin1_Pin ESP_Pin2_Pin */
  GPIO_InitStruct.Pin = ESP_Pin1_Pin|ESP_Pin2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PG11 PG13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : L7_Pin */
  GPIO_InitStruct.Pin = L7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(L7_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(USER_BTN_EXTI_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USER_BTN_EXTI_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == MY_WAKE_Pin){
		woke_by_button = 1;
		HAL_HSEM_FastTake(HSEM_ID_0);
		//Acción del M7

		HAL_HSEM_Release(HSEM_ID_0, 0);
		PersonaHuella = 0;
		PersonaRFID = 0;
		PersonaVoz = 0;


		BaseType_t xHigherPriorityTaskWoken = pdFALSE;


		// Notificar también a la tarea del RFID
		vTaskNotifyGiveFromISR(RFIDTaskHandle, &xHigherPriorityTaskWoken);

		// Notificar a la tarea del ADC/UART
		vTaskNotifyGiveFromISR(defaultTaskHandle, &xHigherPriorityTaskWoken);





		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

	}
}

void RFID_Task(void *argument){
	HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, 1);
	HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, 1);
	HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin, 1);
	HAL_GPIO_WritePin(L4_GPIO_Port, L4_Pin, 1);
	HAL_GPIO_WritePin(L5_GPIO_Port, L5_Pin, 1);
	HAL_GPIO_WritePin(L6_GPIO_Port, L6_Pin, 1);
	HAL_GPIO_WritePin(L7_GPIO_Port, L7_Pin, 1);
	HAL_GPIO_WritePin(L8_GPIO_Port, L8_Pin, 1);
	HAL_GPIO_WritePin(L9_GPIO_Port, L9_Pin, 1);
	HAL_GPIO_WritePin(L10_GPIO_Port, L10_Pin, 1);
	uint8_t counter = 0;
	volatile uint32_t i;
	PersonaRFID = 0;
	RFID_Check = 0;
	errorCounter=0;
	for(;;){
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, 1);
		HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, 0);
		HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, 0);
		HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, 0);
		//for(int j = 0; j<3; j++){
			/* ahora usar el MFRC522 */
			for(i = 0; i<100; i++){
				status = MFRC522_Request(PICC_REQIDL, str);
				mfrc_short_delay();
				status = MFRC522_Anticoll(str);
				mfrc_short_delay();
				memcpy(sNum, str, 5);
				if((str[0]==67) && (str[1]==32) && (str[2]==70) && (str[3]==28) && (str[4]==239) ){
					HAL_GPIO_WritePin(LED_ROJO_GPIO_Port, LED_ROJO_Pin, 1);
					HAL_GPIO_WritePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin, 0);
					char nombre[32] = "RFID = Adrian\r\n";
					HAL_UART_Transmit(&huart3, (uint8_t*)nombre, strlen(nombre), HAL_MAX_DELAY);
					PersonaRFID = 1;
					RFID_Check = 1;
					break;


				}
				else if((str[0]==114) && (str[1]==110) && (str[2]==65) && (str[3]==6) && (str[4]==91) ){
					RFID_Check = 1;
					PersonaRFID = 2;
					char nombre[32] = "RFID = Alan\r\n";
					HAL_UART_Transmit(&huart3, (uint8_t*)nombre, strlen(nombre), HAL_MAX_DELAY);
					HAL_GPIO_WritePin(LED_ROJO_GPIO_Port, LED_ROJO_Pin, 1);
					HAL_GPIO_WritePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin, 0);
					break;
				}
				else if((str[0]==19) && (str[1]==168) && (str[2]==72) && (str[3]==6) && (str[4]==245) ){
					HAL_GPIO_WritePin(LED_ROJO_GPIO_Port, LED_ROJO_Pin, 1);
					HAL_GPIO_WritePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin, 0);
					char nombre[32] = "RFID = Diego\r\n";
					HAL_UART_Transmit(&huart3, (uint8_t*)nombre, strlen(nombre), HAL_MAX_DELAY);
					RFID_Check = 1;
					PersonaRFID = 3;
					break;

				}
				else{

					HAL_GPIO_WritePin(LED_ROJO_GPIO_Port, LED_ROJO_Pin, 1);
					HAL_GPIO_WritePin(LED_VERDE_GPIO_Port, LED_VERDE_Pin, 0);
					counter = 0;
				}
			}
			if(RFID_Check == 0){
				HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, 0);
				HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, 1);
				HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, 0);
				HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, 1);
				mfrc_extra_big_delay();
			}else{
				RFID_Check = 0;
				HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, 0);
				HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, 0);
				HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin, 0);
				HAL_GPIO_WritePin(L4_GPIO_Port, L4_Pin, 1);
				HAL_GPIO_WritePin(L5_GPIO_Port, L5_Pin, 1);
				HAL_GPIO_WritePin(L6_GPIO_Port, L6_Pin, 1);
				HAL_GPIO_WritePin(L7_GPIO_Port, L7_Pin, 1);
				HAL_GPIO_WritePin(L8_GPIO_Port, L8_Pin, 1);
				HAL_GPIO_WritePin(L9_GPIO_Port, L9_Pin, 1);
				HAL_GPIO_WritePin(L10_GPIO_Port, L10_Pin, 1);
				BaseType_t xHigherPriorityTaskWoken = pdFALSE;
				vTaskNotifyGiveFromISR(HuellaTaskHandle, &xHigherPriorityTaskWoken);
			}


			/*if(RFID_Check == 1){
				break;
			}
			errorCounter ++;
			ErrorDisplay();
		//}
		if(errorCounter==3){
			//Notificar pantalla de bloqueo y fin de programa
		}else{
			// Notificar a la tarea de HUELLA

		}*/


	}
}

void Huella_Task(void *argument){
	PersonaHuella = 0;
	Huella_Check = 0;
	for(;;){
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if (osMutexAcquire(uartMutexHandle, 500) == osOK){
			HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, 0);
				HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, 1);
				HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, 0);
				HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, 0);
			//for(int j = 0; j<3; j++){
				HAL_GPIO_WritePin(MODE_GPIO_Port, MODE_Pin, 1);
				HAL_GPIO_WritePin(WAKE_UP_GPIO_Port, WAKE_UP_Pin, 1);
				mfrc_short_delay();
				HAL_GPIO_WritePin(WAKE_UP_GPIO_Port, WAKE_UP_Pin, 0);
				for(int i = 0; i<6000; i++){
					if((HAL_GPIO_ReadPin(BIT1_GPIO_Port, BIT1_Pin)==GPIO_PIN_RESET)&&(HAL_GPIO_ReadPin(BIT2_GPIO_Port, BIT2_Pin)==GPIO_PIN_SET)){
						PersonaHuella = 1;
						Huella_Check = 1;
						char nombre[32] = "Huella = Adrian\r\n";
						HAL_UART_Transmit(&huart3, (uint8_t*)nombre, strlen(nombre), HAL_MAX_DELAY);
					}else if((HAL_GPIO_ReadPin(BIT1_GPIO_Port, BIT1_Pin)==GPIO_PIN_SET)&&(HAL_GPIO_ReadPin(BIT2_GPIO_Port, BIT2_Pin)==GPIO_PIN_RESET)){
						PersonaHuella = 2;
						Huella_Check = 1;
						char nombre[32] = "Huella = Alan\r\n";
						HAL_UART_Transmit(&huart3, (uint8_t*)nombre, strlen(nombre), HAL_MAX_DELAY);
					}else if((HAL_GPIO_ReadPin(BIT1_GPIO_Port, BIT1_Pin)==GPIO_PIN_SET)&&(HAL_GPIO_ReadPin(BIT2_GPIO_Port, BIT2_Pin)==GPIO_PIN_SET)){
						PersonaHuella = 3;
						Huella_Check = 1;
						char nombre[32] = "Huella = Diego\r\n";
						HAL_UART_Transmit(&huart3, (uint8_t*)nombre, strlen(nombre), HAL_MAX_DELAY);
					}else{
						/*char nombre[32] = " = No reconocida\r\n";
						HAL_UART_Transmit(&huart3, (uint8_t*)nombre, strlen(nombre), HAL_MAX_DELAY);*/
					}
					mfrc_short_delay();
				}
				if((Huella_Check == 0)){
					HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, 0);
					HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, 1);
					HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, 0);
					HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, 1);
					mfrc_extra_big_delay();
				}else if ((Huella_Check == 1)||(PersonaHuella == PersonaRFID)) {
					Huella_Check = 0;
					HAL_GPIO_WritePin(MODE_GPIO_Port, MODE_Pin, 0);
					HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, 0);
					HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, 0);
					HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin, 0);
					HAL_GPIO_WritePin(L4_GPIO_Port, L4_Pin, 0);
					HAL_GPIO_WritePin(L5_GPIO_Port, L5_Pin, 0);
					HAL_GPIO_WritePin(L6_GPIO_Port, L6_Pin, 0);
					HAL_GPIO_WritePin(L7_GPIO_Port, L7_Pin, 1);
					HAL_GPIO_WritePin(L8_GPIO_Port, L8_Pin, 1);
					HAL_GPIO_WritePin(L9_GPIO_Port, L9_Pin, 1);
					HAL_GPIO_WritePin(L10_GPIO_Port, L10_Pin, 1);
					osMutexRelease(uartMutexHandle);
					BaseType_t xHigherPriorityTaskWoken = pdFALSE;
					vTaskNotifyGiveFromISR(VozTaskHandle, &xHigherPriorityTaskWoken);
					portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
				}



				/*if((Huella_Check == 1)&&(PersonaHuella == PersonaRFID)){
					break;
				}
				errorCounter ++;
				ErrorDisplay();
		   // }

			if(errorCounter == 6){
				osMutexRelease(uartMutexHandle);
				//Notificar pantalla de bloqueo y fin de programa
			}else{

			}*/

		}

	}

}

void Voz_Task(void *argument){
	PersonaVoz = 0;
	Voz_Check = 0;
	for(;;){
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		    HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, 1);
			HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, 1);
			HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, 0);
			HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, 0);
		//for(int j = 0; j<4; j++){
			HAL_GPIO_WritePin(MODE_GPIO_Port, MODE_Pin, 0);
			HAL_GPIO_WritePin(WAKE_UP_GPIO_Port, WAKE_UP_Pin, 1);
			mfrc_short_delay();
			HAL_GPIO_WritePin(WAKE_UP_GPIO_Port, WAKE_UP_Pin, 0);
			mfrc_extra_big_delay();
			for(int i = 0; i<200; i++){
				if((HAL_GPIO_ReadPin(BIT3_GPIO_Port, BIT3_Pin)==GPIO_PIN_SET)&&(HAL_GPIO_ReadPin(BIT4_GPIO_Port, BIT4_Pin)==GPIO_PIN_RESET)){
					PersonaVoz = 1;
					Voz_Check = 1;
					char nombre[32] = " VOZ = Adrian\r\n";
					HAL_UART_Transmit(&huart3, (uint8_t*)nombre, strlen(nombre), HAL_MAX_DELAY);
				}else if((HAL_GPIO_ReadPin(BIT3_GPIO_Port, BIT3_Pin)==GPIO_PIN_RESET)&&(HAL_GPIO_ReadPin(BIT4_GPIO_Port, BIT4_Pin)==GPIO_PIN_SET)){
					PersonaVoz = 2;
					Voz_Check = 1;
					char nombre[32] = "VOZ = Alan\r\n";
					HAL_UART_Transmit(&huart3, (uint8_t*)nombre, strlen(nombre), HAL_MAX_DELAY);
				}else if((HAL_GPIO_ReadPin(BIT3_GPIO_Port, BIT3_Pin)==GPIO_PIN_SET)&&(HAL_GPIO_ReadPin(BIT4_GPIO_Port, BIT4_Pin)==GPIO_PIN_SET)){
					PersonaVoz = 3;
					Voz_Check = 1;

					char nombre[32] = " VOZ = Diego\r\n";
					HAL_UART_Transmit(&huart3, (uint8_t*)nombre, strlen(nombre), HAL_MAX_DELAY);
				}else{
					char nombre[32] = "VOZ  = No reconocida\r\n";

					HAL_UART_Transmit(&huart3, (uint8_t*)nombre, strlen(nombre), HAL_MAX_DELAY);
				}
				mfrc_big_delay();
			}
			if(Voz_Check == 0){
				HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, 0);
				HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, 1);
				HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, 0);
				HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, 1);

				mfrc_extra_big_delay();
			}
				else if ((PersonaHuella == 1)&&(PersonaVoz == 1)){

				Voz_Check = 0;
				PersonaVoz = 0;

				HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, 0);
				HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, 0);
				HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin, 0);
				HAL_GPIO_WritePin(L4_GPIO_Port, L4_Pin, 0);
				HAL_GPIO_WritePin(L5_GPIO_Port, L5_Pin, 0);
				HAL_GPIO_WritePin(L6_GPIO_Port, L6_Pin, 0);
				HAL_GPIO_WritePin(L7_GPIO_Port, L7_Pin, 0);
				HAL_GPIO_WritePin(L8_GPIO_Port, L8_Pin, 0);
				HAL_GPIO_WritePin(L9_GPIO_Port, L9_Pin, 0);
				HAL_GPIO_WritePin(L10_GPIO_Port, L10_Pin, 1);

//Adrian
				HAL_GPIO_WritePin(ESP_Pin1_GPIO_Port, ESP_Pin1_Pin, 1);
				HAL_GPIO_WritePin(ESP_Pin2_GPIO_Port, ESP_Pin2_Pin, 0);
				HAL_GPIO_WritePin(ESP_Pin2_GPIO_Port, ESP_Pin3_Pin, 0);

				HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, 1);
				HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, 0);
				HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, 0);
				HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, 1);
				mfrc_extra_big_delay();
			}else if ((PersonaHuella == 2)&&(PersonaVoz == 2)){
				Voz_Check = 0;

				HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, 0);
				HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, 0);
				HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin, 0);
				HAL_GPIO_WritePin(L4_GPIO_Port, L4_Pin, 0);
				HAL_GPIO_WritePin(L5_GPIO_Port, L5_Pin, 0);
				HAL_GPIO_WritePin(L6_GPIO_Port, L6_Pin, 0);
				HAL_GPIO_WritePin(L7_GPIO_Port, L7_Pin, 0);
				HAL_GPIO_WritePin(L8_GPIO_Port, L8_Pin, 0);
				HAL_GPIO_WritePin(L9_GPIO_Port, L9_Pin, 0);
				HAL_GPIO_WritePin(L10_GPIO_Port, L10_Pin, 1);

//Alsn

				HAL_GPIO_WritePin(ESP_Pin1_GPIO_Port, ESP_Pin1_Pin, 0);
				HAL_GPIO_WritePin(ESP_Pin2_GPIO_Port, ESP_Pin2_Pin, 1);
				HAL_GPIO_WritePin(ESP_Pin2_GPIO_Port, ESP_Pin3_Pin, 0);

				HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, 0);
				HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, 0);
				HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, 0);
				HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, 1);
				mfrc_extra_big_delay();
			}else if ((PersonaHuella == 3)&&(PersonaVoz == 3)){
				Voz_Check = 0;


				HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, 0);
				HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, 0);
				HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin, 0);
				HAL_GPIO_WritePin(L4_GPIO_Port, L4_Pin, 0);
				HAL_GPIO_WritePin(L5_GPIO_Port, L5_Pin, 0);
				HAL_GPIO_WritePin(L6_GPIO_Port, L6_Pin, 0);
				HAL_GPIO_WritePin(L7_GPIO_Port, L7_Pin, 0);
				HAL_GPIO_WritePin(L8_GPIO_Port, L8_Pin, 0);
				HAL_GPIO_WritePin(L9_GPIO_Port, L9_Pin, 0);
				HAL_GPIO_WritePin(L10_GPIO_Port, L10_Pin, 1);

//Diego

				HAL_GPIO_WritePin(ESP_Pin1_GPIO_Port, ESP_Pin1_Pin, 1);
				HAL_GPIO_WritePin(ESP_Pin2_GPIO_Port, ESP_Pin2_Pin, 1);
				HAL_GPIO_WritePin(ESP_Pin2_GPIO_Port, ESP_Pin3_Pin, 0);

				HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, 1);
				HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, 1);
				HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, 1);
				HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, 0);
				mfrc_extra_big_delay();
			}else{

				HAL_GPIO_WritePin(ESP_Pin1_GPIO_Port, ESP_Pin1_Pin, 1);
				HAL_GPIO_WritePin(ESP_Pin2_GPIO_Port, ESP_Pin2_Pin, 0);
				HAL_GPIO_WritePin(ESP_Pin2_GPIO_Port, ESP_Pin3_Pin, 1);
				HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, 0);
				HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, 1);
				HAL_GPIO_WritePin(B3_GPIO_Port, B3_Pin, 0);
				HAL_GPIO_WritePin(B4_GPIO_Port, B4_Pin, 1);
				mfrc_extra_big_delay();
			}
			/*if((Voz_Check == 1)&&(PersonaVoz == PersonaRFID)){
				break;
			}
			errorCounter ++;
			ErrorDisplay();
		//}
		if(errorCounter == 10){
			//Notificar pantalla de bloqueo y fin de programa
		}else{
			//Notificar pantalla de apertura, persona, y fin de programa
		}*/
	}
}
void mfrc_short_delay(void){
    volatile uint32_t i;
    for(i = 0; i < 30000; ++i) { __asm volatile("nop"); }
}

void mfrc_big_delay(void){
    volatile uint32_t i;
    for(i = 0; i < 300000; ++i) { __asm volatile("nop"); }
}

void mfrc_extra_big_delay(void){
    volatile uint32_t i;
    for(i = 0; i < 30000000; ++i) { __asm volatile("nop"); }
}

void ErrorDisplay(void){
	switch (errorCounter){
		case 0:
			HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, 0);
			HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, 0);
			HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin, 0);
			HAL_GPIO_WritePin(L4_GPIO_Port, L4_Pin, 0);
			HAL_GPIO_WritePin(L5_GPIO_Port, L5_Pin, 0);
			HAL_GPIO_WritePin(L6_GPIO_Port, L6_Pin, 0);
			HAL_GPIO_WritePin(L7_GPIO_Port, L7_Pin, 0);
			HAL_GPIO_WritePin(L8_GPIO_Port, L8_Pin, 0);
			HAL_GPIO_WritePin(L9_GPIO_Port, L9_Pin, 0);
			HAL_GPIO_WritePin(L10_GPIO_Port, L10_Pin, 0);
			break;
		case 1:
			HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, 1);
			HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, 0);
			HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin, 0);
			HAL_GPIO_WritePin(L4_GPIO_Port, L4_Pin, 0);
			HAL_GPIO_WritePin(L5_GPIO_Port, L5_Pin, 0);
			HAL_GPIO_WritePin(L6_GPIO_Port, L6_Pin, 0);
			HAL_GPIO_WritePin(L7_GPIO_Port, L7_Pin, 0);
			HAL_GPIO_WritePin(L8_GPIO_Port, L8_Pin, 0);
			HAL_GPIO_WritePin(L9_GPIO_Port, L9_Pin, 0);
			HAL_GPIO_WritePin(L10_GPIO_Port, L10_Pin, 0);
			break;
		case 2:
			HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, 1);
			HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, 1);
			HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin, 0);
			HAL_GPIO_WritePin(L4_GPIO_Port, L4_Pin, 0);
			HAL_GPIO_WritePin(L5_GPIO_Port, L5_Pin, 0);
			HAL_GPIO_WritePin(L6_GPIO_Port, L6_Pin, 0);
			HAL_GPIO_WritePin(L7_GPIO_Port, L7_Pin, 0);
			HAL_GPIO_WritePin(L8_GPIO_Port, L8_Pin, 0);
			HAL_GPIO_WritePin(L9_GPIO_Port, L9_Pin, 0);
			HAL_GPIO_WritePin(L10_GPIO_Port, L10_Pin, 0);
			break;
		case 3:
			HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, 1);
			HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, 1);
			HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin, 3);
			HAL_GPIO_WritePin(L4_GPIO_Port, L4_Pin, 0);
			HAL_GPIO_WritePin(L5_GPIO_Port, L5_Pin, 0);
			HAL_GPIO_WritePin(L6_GPIO_Port, L6_Pin, 0);
			HAL_GPIO_WritePin(L7_GPIO_Port, L7_Pin, 0);
			HAL_GPIO_WritePin(L8_GPIO_Port, L8_Pin, 0);
			HAL_GPIO_WritePin(L9_GPIO_Port, L9_Pin, 0);
			HAL_GPIO_WritePin(L10_GPIO_Port, L10_Pin, 0);
			break;
		case 4:
			HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, 1);
			HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, 1);
			HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin, 1);
			HAL_GPIO_WritePin(L4_GPIO_Port, L4_Pin, 1);
			HAL_GPIO_WritePin(L5_GPIO_Port, L5_Pin, 0);
			HAL_GPIO_WritePin(L6_GPIO_Port, L6_Pin, 0);
			HAL_GPIO_WritePin(L7_GPIO_Port, L7_Pin, 0);
			HAL_GPIO_WritePin(L8_GPIO_Port, L8_Pin, 0);
			HAL_GPIO_WritePin(L9_GPIO_Port, L9_Pin, 0);
			HAL_GPIO_WritePin(L10_GPIO_Port, L10_Pin, 0);
			break;
		case 5:
			HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, 1);
			HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, 1);
			HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin, 1);
			HAL_GPIO_WritePin(L4_GPIO_Port, L4_Pin, 1);
			HAL_GPIO_WritePin(L5_GPIO_Port, L5_Pin, 1);
			HAL_GPIO_WritePin(L6_GPIO_Port, L6_Pin, 0);
			HAL_GPIO_WritePin(L7_GPIO_Port, L7_Pin, 0);
			HAL_GPIO_WritePin(L8_GPIO_Port, L8_Pin, 0);
			HAL_GPIO_WritePin(L9_GPIO_Port, L9_Pin, 0);
			HAL_GPIO_WritePin(L10_GPIO_Port, L10_Pin, 0);
			break;
		case 6:
			HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, 1);
			HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, 1);
			HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin, 1);
			HAL_GPIO_WritePin(L4_GPIO_Port, L4_Pin, 1);
			HAL_GPIO_WritePin(L5_GPIO_Port, L5_Pin, 1);
			HAL_GPIO_WritePin(L6_GPIO_Port, L6_Pin, 1);
			HAL_GPIO_WritePin(L7_GPIO_Port, L7_Pin, 0);
			HAL_GPIO_WritePin(L8_GPIO_Port, L8_Pin, 0);
			HAL_GPIO_WritePin(L9_GPIO_Port, L9_Pin, 0);
			HAL_GPIO_WritePin(L10_GPIO_Port, L10_Pin, 0);
			break;
		case 7:
			HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, 1);
			HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, 1);
			HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin, 1);
			HAL_GPIO_WritePin(L4_GPIO_Port, L4_Pin, 1);
			HAL_GPIO_WritePin(L5_GPIO_Port, L5_Pin, 1);
			HAL_GPIO_WritePin(L6_GPIO_Port, L6_Pin, 1);
			HAL_GPIO_WritePin(L7_GPIO_Port, L7_Pin, 1);
			HAL_GPIO_WritePin(L8_GPIO_Port, L8_Pin, 0);
			HAL_GPIO_WritePin(L9_GPIO_Port, L9_Pin, 0);
			HAL_GPIO_WritePin(L10_GPIO_Port, L10_Pin, 0);
			break;
		case 8:
			HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, 1);
			HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, 1);
			HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin, 1);
			HAL_GPIO_WritePin(L4_GPIO_Port, L4_Pin, 1);
			HAL_GPIO_WritePin(L5_GPIO_Port, L5_Pin, 1);
			HAL_GPIO_WritePin(L6_GPIO_Port, L6_Pin, 1);
			HAL_GPIO_WritePin(L7_GPIO_Port, L7_Pin, 1);
			HAL_GPIO_WritePin(L8_GPIO_Port, L8_Pin, 1);
			HAL_GPIO_WritePin(L9_GPIO_Port, L9_Pin, 0);
			HAL_GPIO_WritePin(L10_GPIO_Port, L10_Pin, 0);
			break;
		case 9:
			HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, 1);
			HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, 1);
			HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin, 1);
			HAL_GPIO_WritePin(L4_GPIO_Port, L4_Pin, 1);
			HAL_GPIO_WritePin(L5_GPIO_Port, L5_Pin, 1);
			HAL_GPIO_WritePin(L6_GPIO_Port, L6_Pin, 1);
			HAL_GPIO_WritePin(L7_GPIO_Port, L7_Pin, 1);
			HAL_GPIO_WritePin(L8_GPIO_Port, L8_Pin, 1);
			HAL_GPIO_WritePin(L9_GPIO_Port, L9_Pin, 1);
			HAL_GPIO_WritePin(L10_GPIO_Port, L10_Pin, 0);
			break;
		case 10:
			HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, 1);
			HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, 1);
			HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin, 1);
			HAL_GPIO_WritePin(L4_GPIO_Port, L4_Pin, 1);
			HAL_GPIO_WritePin(L5_GPIO_Port, L5_Pin, 1);
			HAL_GPIO_WritePin(L6_GPIO_Port, L6_Pin, 1);
			HAL_GPIO_WritePin(L7_GPIO_Port, L7_Pin, 1);
			HAL_GPIO_WritePin(L8_GPIO_Port, L8_Pin, 1);
			HAL_GPIO_WritePin(L9_GPIO_Port, L9_Pin, 1);
			HAL_GPIO_WritePin(L10_GPIO_Port, L10_Pin, 1);

			errorCounter = 0;
			//Notificar pantalla de bloqueo de sistema
			break;
		default:
			break;
	}
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  /* Esperar notificación del botón (sin timeout) */
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	//HAL_UART_Init(&huart3);
	/*HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 1000);
	buffer = HAL_ADC_GetValue(&hadc2);
	adc_value16 = (uint16_t)buffer;
	osMessageQueuePut(queueSpeedHandler, &adc_value16, 0, 0);
	sprintf(msg, "ADC: %lu\r\n", buffer);
	if (osMutexAcquire(uartMutexHandle, 500) == osOK) {
		// Timeout 500ms: si falla el acquire, se omite la transmisión para no quedar bloqueado por mucho tiempo
		HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 1000);
		osMutexRelease(uartMutexHandle);
	}*/






  }


  /* USER CODE END 5 */
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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
