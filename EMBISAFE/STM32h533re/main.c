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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "stdio.h"
#include "arm_math.h"
#include "math.h"
#include <math.h>
#include <string.h>
#include "stdint.h"

#define ADC_VREF 3.3f
#define FS 16000.0f
#define FRAME 512
#define HOP (FRAME/2) //256
#define LEN_SIGNAL 20000
#define MAX_FRAMES ((LEN_SIGNAL-FRAME)/HOP+2) //122
#define PI 3.14159f

#define MUESTRAS 10
#define FRAMES 39
#define CARACTERISTICAS 4

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	IDLE,
	RUN,
	STOP,
}States;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
float zcrPersona;
float centroidPersona;
float bwPersona;
float rolloffPersona;

//Arreglo
float zcrPerfil[MUESTRAS];
float centroidPerfil[MUESTRAS];
float bwPerfil[MUESTRAS];
float rolloffPerfil[MUESTRAS];

float perfilPersona[4];

volatile float alan[CARACTERISTICAS]={0.0736,922.11,767.440,1715.9455};
volatile float diego[CARACTERISTICAS]={0.0782,947.1895,751.6553,1765.625};
volatile float adrian[CARACTERISTICAS]={0.0710,865.0595,703.313,1535.2564};

float numero=0;

//Array para guardar valores relativos respecto a nueva entrada
volatile float distAlan[CARACTERISTICAS]={};
volatile float distDiego[CARACTERISTICAS]={};
volatile float distAdrian[CARACTERISTICAS]={};

volatile float similitud=-1;

volatile States devState = STOP;
volatile uint16_t adc_buf[LEN_SIGNAL] = {0};   // DMA
float mic_f[LEN_SIGNAL]; // DMA pero en float
volatile bool adcReady=false;

uint8_t start=0,cont=0;

//Filtro

float array_filtrado[LEN_SIGNAL];
float sosCoeffs[20] = {
     1.0000f, -2.0000f,  1.0000f, 0.5529f,  -0.1248f,
     1.0000f,  2.0001f,  1.0001f, 0.6072f,  -0.5296f,
     1.0000f,  1.9999f,  0.9999f, 1.8065f,  -0.8179f,
     1.0000f, -2.0000f,  1.0000f, 1.9257f,  -0.9354f
};
float sosState[4*4];
float gain = 0.0288f;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef handle_GPDMA1_Channel0;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#include "fingerprint.h"
#include "VozAnalisis.h"
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_GPDMA1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ICACHE_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
arm_biquad_cascade_df2T_instance_f32 S;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_ADC1_Init();
  MX_ICACHE_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  arm_biquad_cascade_df1_init_f32(
    	&S,
    	4,            // número de secciones SOS
    	sosCoeffs,
    	sosState
    );
  init_fingerprint();
  iniciar();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch(devState){
		  case IDLE:

			  //devState=STOP;
			  break;
		  case RUN:


			  if(HAL_GPIO_ReadPin(MODE_GPIO_Port, MODE_Pin)==GPIO_PIN_SET){
				  for(int i = 0; i<10; i++){
					  uint16_t result = check_fingerprint();

					  if (result >= 0) {
						  char msg[32];
						  sprintf(msg, "Huella ID: %d\r", result);
						  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
						  if (result == 1){
							  char nombre[32] = " = Adrian\n";
							  HAL_UART_Transmit(&huart2, (uint8_t*)nombre, strlen(nombre), HAL_MAX_DELAY);
							  HAL_GPIO_WritePin(BIT1_GPIO_Port, BIT1_Pin, 1);
							  HAL_GPIO_WritePin(BIT2_GPIO_Port, BIT2_Pin, 0);

						  }
						  else if(result == 2){
							  char nombre[32] = " = Alan\n";
							  HAL_UART_Transmit(&huart2, (uint8_t*)nombre, strlen(nombre), HAL_MAX_DELAY);
							  HAL_GPIO_WritePin(BIT1_GPIO_Port, BIT1_Pin, 0);
							  HAL_GPIO_WritePin(BIT2_GPIO_Port, BIT2_Pin, 1);
						  }
						  else if(result == 3){
							  char nombre[32] = " = Diego\n";
							  HAL_UART_Transmit(&huart2, (uint8_t*)nombre, strlen(nombre), HAL_MAX_DELAY);
							  HAL_GPIO_WritePin(BIT1_GPIO_Port, BIT1_Pin, 1);
							  HAL_GPIO_WritePin(BIT2_GPIO_Port, BIT2_Pin, 1);
						  }
						  else{
							  char nombre[32] = " = Huella no registrada\n";
							  HAL_UART_Transmit(&huart2, (uint8_t*)nombre, strlen(nombre), HAL_MAX_DELAY);
							  HAL_GPIO_WritePin(BIT1_GPIO_Port, BIT1_Pin, 0);
							  HAL_GPIO_WritePin(BIT2_GPIO_Port, BIT2_Pin, 0);
						  }
					  }
					  else if (result == -1) {
						  char msg[32];
						  sprintf(msg, "Huella no registrada\r\n", result);
						  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
					  }
					  else if (result == -2) {
						  char msg[32];
						  sprintf(msg, "Error en lectura\r\n", result);
						  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
					  }
					  else if (result == -3) {
						  char msg[32];
						  sprintf(msg, "No hay dedo\r\n", result);
						  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
					  }
				  }
				  devState = STOP;
			  }	  else if(HAL_GPIO_ReadPin(MODE_GPIO_Port, MODE_Pin)==GPIO_PIN_RESET){
				  HAL_ADC_Stop_DMA(&hadc1);
				  memset((void*)adc_buf,0,sizeof(adc_buf));
				  HAL_TIM_Base_Start(&htim6);
				  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, 1);
				  HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, 0);
				  HAL_Delay(1000);
				  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, 20000);
				  HAL_Delay(6000);
				  HAL_TIM_Base_Stop(&htim6);
				  HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, 0);
				  HAL_Delay(1000);

				  //float mic_f[LEN_SIGNAL];
				  for (int k = 0; k < LEN_SIGNAL; k++) {
					 // si quieres convertir a signed: mic_f[k] = (float)((int16_t)adc_buf[k]);
					  mic_f[k] = (float)adc_buf[k];
				  }
				  // Filtro
				  Filtro_pasaBandas(mic_f, array_filtrado, 20000);
				  procesar(array_filtrado,20000, &zcrPersona,&centroidPersona, &bwPersona, &rolloffPersona);

				  perfilPersona[0]=zcrPersona;
				  perfilPersona[1]=centroidPersona;
				  perfilPersona[2]=bwPersona;
				  perfilPersona[3]=rolloffPersona;

				  distancia_relativa(adrian, perfilPersona, distAdrian);
				  distancia_relativa(diego, perfilPersona, distDiego);
				  distancia_relativa(alan, perfilPersona, distAlan);


				  similitud=definirPersona(distAdrian, distAlan, distDiego,&numero);

				  if(similitud==1){

					  HAL_GPIO_WritePin(BIT3_GPIO_Port,BIT3_Pin ,0);
					  HAL_GPIO_WritePin(BIT4_GPIO_Port,BIT4_Pin ,1);

				  }else if(similitud==2){
					  HAL_GPIO_WritePin(BIT3_GPIO_Port,BIT3_Pin ,1);
					  HAL_GPIO_WritePin(BIT4_GPIO_Port,BIT4_Pin ,0);

				  }else if(similitud==3){

					  HAL_GPIO_WritePin(BIT3_GPIO_Port,BIT3_Pin ,1);
					  HAL_GPIO_WritePin(BIT4_GPIO_Port,BIT4_Pin ,1);
				  }
				  else{

					  HAL_GPIO_WritePin(BIT3_GPIO_Port,BIT3_Pin ,0);
					  HAL_GPIO_WritePin(BIT4_GPIO_Port,BIT4_Pin ,0);
				  }

				  HAL_Delay(100);

				  devState = STOP;
			  }


				  break;

		  case STOP:
			  //HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, 0);
			  HAL_GPIO_WritePin(BIT3_GPIO_Port, BIT3_Pin, 0);
			  HAL_GPIO_WritePin(BIT4_GPIO_Port, BIT4_Pin, 0);
			  HAL_GPIO_WritePin(BIT1_GPIO_Port, BIT1_Pin, 0);
			  HAL_GPIO_WritePin(BIT2_GPIO_Port, BIT2_Pin, 0);
			  HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, 1);

			  //HAL_TIM_Base_Stop(&htim6);
			  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

			  break;
		  default:
			  break;
	  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_CSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 36;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_1);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPDMA1_Init(void)
{

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* GPDMA1 interrupt Init */
    HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 44;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, USER_LED_Pin|A_Pin|BIT1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BIT3_Pin|BIT4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BIT2_GPIO_Port, BIT2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BTN_Pin */
  GPIO_InitStruct.Pin = USER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USER_LED_Pin A_Pin BIT1_Pin */
  GPIO_InitStruct.Pin = USER_LED_Pin|A_Pin|BIT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 MODE_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : WAKE_UP_Pin */
  GPIO_InitStruct.Pin = WAKE_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(WAKE_UP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BIT3_Pin BIT4_Pin */
  GPIO_InitStruct.Pin = BIT3_Pin|BIT4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BIT2_Pin */
  GPIO_InitStruct.Pin = BIT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(BIT2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI10_IRQn);

  HAL_NVIC_SetPriority(EXTI13_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI13_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_pin){
	if (GPIO_pin == WAKE_UP_Pin){
		SystemClock_Config();
		devState = RUN;
		/*if(devState==IDLE){
			devState++;
		}

		if(devState == RUN && cont<10){
			start=1;

		}else {
			devState=devState; //Aqui va cambiando el estado de manera secuencial,0, 1, 2, 3... 0, 1,...
		}*/
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	adcReady=true;
	HAL_GPIO_WritePin(A_GPIO_Port, A_Pin, 0);
	// Enviar por UART
	  /*char buffer[20];
	  for (int i = 0; i < 20000; i++) {
		  int len = sprintf(buffer, "%hu\r\n", adc_buf[i]);
		  HAL_UART_Transmit(&huart2, (uint16_t*)buffer, len, HAL_MAX_DELAY);
	  }*/
	  adcReady=true;

}

void Filtro_pasaBandas(volatile float *array, float *array_filtrado, int length){


	// 3. Filtrar todo el arreglo
	arm_biquad_cascade_df1_f32(&S, array, array_filtrado, length);

	// 4. Aplicar ganancia final
	for(int i = 0; i < length; i++)
		array_filtrado[i] *= gain;
	/*char buffer[32];
	for(int i = 0; i < length; i++) {
		// Convertir cada float a string
		uint8_t len = snprintf(buffer, sizeof(buffer), "%.6f\r\n", array_filtrado[i]);
		HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);
	}*/
}

/* USER CODE END 4 */

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
