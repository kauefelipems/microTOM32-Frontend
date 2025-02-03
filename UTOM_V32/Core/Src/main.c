/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdlib.h>
#include "arm_const_structs.h"
#include "arm_math.h"
#include "state_machine.h"
#include "ring_buffer.h"

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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

DAC_HandleTypeDef hdac1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM8_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

// State Functions BEGIN
Actions_TypeDef comm_wait_state(void);
Actions_TypeDef g_sel_state(void);
Actions_TypeDef ch_sel_state(void);
Actions_TypeDef meas_state(void);
Actions_TypeDef error_state(void);
Actions_TypeDef calc_FFT_state(void);
Actions_TypeDef exc_state(void);
Actions_TypeDef current_p_state(void);
Actions_TypeDef current_n_state(void);
// State Functions END

// Auxiliary Functions BEGIN
Actions_TypeDef take_measurement(ADC_HandleTypeDef* hadc);
// Auxiliary Functions END

void APPEND_MEASUREMENT(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Interface Commands
uint8_t comm[COMM_BUFF_SIZE]; 									//command

//Voltage Measurements
uint16_t adc_buffer[ADC_BUFF_SIZE]; 									//ADC Measurements
uint16_t uart_ring_buffer[RING_BUFFER_SIZE];
ring_buf_t ring_buffer_struct;

//State Function Pointer (need to be synchronized with States_TypeDef)
State_FunctionsTypeDef state_func_ptr[] = {comm_wait_state, g_sel_state, ch_sel_state,
		meas_state, error_state, calc_FFT_state, exc_state, current_p_state, current_n_state};

//Flow Locks
HAL_LockTypeDef uart_cplt_lck = HAL_LOCKED; //UART Receiving Complete Transfer
HAL_LockTypeDef fft_lck       = HAL_UNLOCKED; //UART Receiving Complete Transfer

//MUX Ports and Pins
uint16_t mux_pins[5]  	     = {MUX_A0_Pin, MUX_A1_Pin, MUX_A2_Pin, MUX_A3_Pin, MUX_A4_Pin};
GPIO_TypeDef* mux_ports[5]   = {MUX_A0_GPIO_Port, MUX_A1_GPIO_Port, MUX_A2_GPIO_Port, MUX_A3_GPIO_Port, MUX_A4_GPIO_Port};
uint16_t cs_pins[4]          = {MUX_CS1_Pin, MUX_CS2_Pin, MUX_CS3_Pin, MUX_CS4_Pin};
GPIO_TypeDef* cs_ports[4]    = {MUX_CS1_GPIO_Port, MUX_CS2_GPIO_Port, MUX_CS3_GPIO_Port, MUX_CS4_GPIO_Port};

//Machine State for Diagnostics
uint32_t dac_DB = __DAC1_VOLTAGE2BIT(PGA_MIN_VOLTAGE);
uint32_t dac_LIN = __DAC2_VOLTAGE2BIT(PGA_MIN_VOLTAGE);
uint8_t channels_value[4] = {0,0,0,0};

//FFT parameters and functions
// Initialize FFT instance
float32_t fft_input[FFT_SIZE];   // Real input buffer
float32_t fft_output[FFT_SIZE]; // FFT output buffer (interleaved complex)
float32_t magnitudes[FFT_SIZE / 2];  // Magnitudes for positive frequencies
arm_rfft_fast_instance_f32 fft_instance;
float32_t mag_value; //desired output
float32_t hann_multiplier[ADC_BUFF_SIZE];

//Excitation active
uint8_t f_target = 1;

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
  MX_DMA_Init();
  MX_RTC_Init();
  MX_USART3_UART_Init();
  MX_DAC1_Init();
  MX_TIM8_Init();
  MX_ADC3_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */


  // Set State Machine Variables:
  States_TypeDef state = ENTRY_STATE;
  Actions_TypeDef action;
  State_FunctionsTypeDef state_func;

  //ADC Calibration Function
  if(HAL_ADCEx_Calibration_Start(&hadc3, ADC_DIFFERENTIAL_ENDED) != HAL_OK)
	  Error_Handler();

  //GAIN_DAC Start and set value to 0.1 V
	if((HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_DB) != HAL_OK) ||
		  (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK) != HAL_OK)
	  Error_Handler();
	if((HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_LIN) != HAL_OK) ||
		  (HAL_DAC_Start(&hdac1, DAC_CHANNEL_2) != HAL_OK) != HAL_OK)
	  Error_Handler();


  //Ring Buffer Initialization
	ring_buf_init(&ring_buffer_struct, uart_ring_buffer, RING_BUFFER_SIZE);

  //Initialize Hann Window for FFT
	for(int i=0; i < ADC_BUFF_SIZE; i++){
		hann_multiplier[i] = 0.5 * (1 - cos(2*PI*i/2047));
	}

  //UART Terminator
	//adc_buffer[ADC_BUFF_SIZE - 1] = TERMINATOR;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	state_func = state_func_ptr[state];
	action = state_func();
	state = ST_nextstate(state, action);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_RTC
                              |RCC_PERIPHCLK_TIM8|RCC_PERIPHCLK_ADC12
                              |RCC_PERIPHCLK_ADC34|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_SYSCLK;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.Tim8ClockSelection = RCC_TIM8CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
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
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc3.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_SOFTWARE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 35;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 99;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 71;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 35;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

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
  huart3.Init.WordLength = UART_WORDLENGTH_9B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_ODD;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
  /* DMA2_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MUX_A0_GPIO_Port, MUX_A0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MUX_A1_Pin|MUX_A4_Pin|MUX_A3_Pin|LD3_Pin
                          |LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, MUX_CS4_Pin|MUX_CS3_Pin|MUX_CS2_Pin|MUX_CS1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MUX_A2_GPIO_Port, MUX_A2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MUX_A0_Pin */
  GPIO_InitStruct.Pin = MUX_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MUX_A0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MUX_A1_Pin MUX_A4_Pin MUX_A3_Pin LD3_Pin
                           LD2_Pin */
  GPIO_InitStruct.Pin = MUX_A1_Pin|MUX_A4_Pin|MUX_A3_Pin|LD3_Pin
                          |LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MUX_CS4_Pin MUX_CS3_Pin MUX_CS2_Pin MUX_CS1_Pin
                           MUX_A2_Pin */
  GPIO_InitStruct.Pin = MUX_CS4_Pin|MUX_CS3_Pin|MUX_CS2_Pin|MUX_CS1_Pin
                          |MUX_A2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* Inline Functions BEGIN-----------------------------------------------------------------*/
/**
  * @brief Wait for Serial Command from the Interface
  * @retval Next Action: go_g, go_ch, go_meas, repeat, fail
  */
void inline __attribute__((always_inline)) delay(uint32_t n_clocks)
{
    while(n_clocks--) __asm__ volatile("");
}

/* Inline Functions END-----------------------------------------------------------------*/


/* State Functions BEGIN-----------------------------------------------------------------*/
/**
  * @brief Wait for Serial Command from the Interface
  * @retval Next Action: go_g, go_ch, go_meas, repeat, fail
  */
Actions_TypeDef comm_wait_state(void){

	HAL_StatusTypeDef uart_state = HAL_OK;

	if (uart_cplt_lck == HAL_UNLOCKED){	//DMA complete transfer

		uart_cplt_lck = HAL_LOCKED;

		switch (comm[0]){
			case 'G': return go_g;
			case 'S': return go_ch;
			case 'M': return go_meas;
			case 'F': return go_FFT;
			case 'E': return go_ex;
			case 'P': return go_ip;
			case 'N': return go_in;
			default: return repeat;
		}
	}

	else {

		uart_state = HAL_UART_Receive_DMA(&huart3, comm, COMM_BUFF_SIZE); //Start receiving

		if((uart_state == HAL_ERROR)){		//If UART ERROR
				Error_Handler();
		}

		return repeat;
	}
}

/**
  * @brief Change Gain of the PGA
  * @retval Next Action: OK, repeat, fail
  */
Actions_TypeDef g_sel_state(void){

	dac_DB = __DAC1_VOLTAGE2BIT(comm[1]);
	dac_LIN = __DAC2_VOLTAGE2BIT(comm[2]);

	//Set DAC to the desired GAIN value
	if((HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_DB) != HAL_OK) ||
		  (HAL_DAC_Start(&hdac1, DAC_CHANNEL_1) != HAL_OK) != HAL_OK)
		Error_Handler();

	//Set DAC to the desired GAIN value
	if((HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dac_LIN) != HAL_OK) ||
		  (HAL_DAC_Start(&hdac1, DAC_CHANNEL_2) != HAL_OK) != HAL_OK)
		Error_Handler();

	return ok;

}

/**
  * @brief Change Measurement Channel
  * @retval Next Action: OK, repeat, fail
  */
Actions_TypeDef ch_sel_state(void){

	//Channel selection initialization

	channels_value[3] = comm[1];
	channels_value[2] = comm[2];
	channels_value[1] = comm[3];
	channels_value[0] = comm[4];

	//Check if channel is valid
	if ((channels_value[0] >= N_CHANNELS) && (channels_value[1] >= N_CHANNELS) && (channels_value[2] >= N_CHANNELS)
			&&(channels_value[3] >= N_CHANNELS)){
		return fail;
	}

	//Set pins for channel selection

	for(uint8_t i = 0; i < 4; i++)
	{
		for(uint8_t j = 0; j < 5; j++)
		{
			GPIO_PinState pin_value;

			//Extract MUX Input Configuration from UART value
			pin_value = (channels_value[i] & (1 << j)) ? GPIO_PIN_SET : GPIO_PIN_RESET;

			//Set MUX selection input
			HAL_GPIO_WritePin(mux_ports[j], mux_pins[j], pin_value);

		}

		//Write channel configuration to MUX i using chip select pins
		HAL_GPIO_WritePin(cs_ports[i], cs_pins[i], GPIO_PIN_RESET);


		//Latch MUX i
		HAL_GPIO_WritePin(cs_ports[i], cs_pins[i], GPIO_PIN_SET);
	}
	//delay(500); //wait for channel selection stabilization

	return ok;
}


/**
  * @brief Starts/Stops Excitation Signal (Square Wave, 10 kHz)
  * @retval Next Action: OK, repeat, fail
  */
Actions_TypeDef exc_state(void){
	f_target= comm[1];

	switch(f_target){

		case 1: // 10kHz
			__HAL_TIM_SET_PRESCALER(&htim8, 99);
			HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2); //Starts PWM for Current Source (10 kHz)
			break;

		case 2: // 100kHz
			__HAL_TIM_SET_PRESCALER(&htim8, 9);
			HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2); //Starts PWM for Current Source (100 kHz)
			break;

		default: //TURN-OFF
			HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2); //Stops PWM for Current Source
	}

	return ok;
}


/**
  * @brief Starts and Manages One Voltage Measurement Run
  * @retval Next Action: OK, repeat, fail
  */
Actions_TypeDef meas_state(void){

	return take_measurement(&hadc3);
}


/**
  * @brief Starts and Manages One Current Measurement Leaving Terminal P of the Howland Source
  * @retval Next Action - OK, repeat, fail
  */
Actions_TypeDef current_p_state(void){

	return take_measurement(&hadc1);
}

/**
  * @brief Starts and Manages One Current Measurement Leaving Terminal N of the Howland Source
  * @retval Next Action - OK, repeat, fail
  */
Actions_TypeDef current_n_state(void){

	return take_measurement(&hadc2);
}


/**
  * @brief Measures the magnitude and phase of the trans-impedance at the current frequency
  * @retval Next Action - OK, repeat, fail
  */

Actions_TypeDef calc_FFT_state(void){

	//if (ch_sel_state() != ok){ //Change channel
		//return fail;
	//}

	fft_lck = HAL_LOCKED; //don't send data via uart after ADC completing

	if (take_measurement(&hadc3) != ok){ //Take measurement from ADC
		return repeat;
	}

	// Prepare input: fill with ADC data, zero-padding if necessary
	for (int i = 0; i < FFT_SIZE; i++) {
		if (i < ADC_BUFF_SIZE) {
			fft_input[i] = (float32_t)(adc_buffer[i] - 2048) * (1.0f / 2048.0f) ;  // Scale to [-1, 1]
			fft_input[i] *= hann_multiplier[i]; //windowing
		} else {
			fft_input[i] = 0.0f;  // Zero-padding
		}
	}

		// Initialize FFT
		arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);
		// Perform FFT
		arm_rfft_fast_f32(&fft_instance, fft_input, fft_output, 0);

		// Calculate magnitudes
		for (int i = 0; i < FFT_SIZE / 2; i++) {
			float32_t real = fft_output[2 * i];      // Real part
			float32_t imag = fft_output[2 * i + 1]; // Imaginary part
			magnitudes[i] = sqrtf(real * real + imag * imag);
		}


	switch (f_target) {
		case 1:
			mag_value = magnitudes[10];  // Bin 5 corresponds to 10kHz
			break;
		case 2:
			mag_value = magnitudes[102]; // Bin 51 corresponds to 100kHz
			break;
		default:
			mag_value = magnitudes[10];
			break;
	}

	HAL_UART_Transmit_DMA(&huart3, (uint8_t*) &mag_value, 4);

	fft_lck = HAL_UNLOCKED;
    return ok;
}



/**
  * @brief Handles and Informs Errors
  * @retval Next Action - OK, repeat
  */
Actions_TypeDef error_state(void){
	Error_Handler();
	return ok;
}


/* State Functions END-----------------------------------------------------------------*/

/* Auxiliary Functions BEGIN-----------------------------------------------------------------*/

/**
  * @brief Take measure of 1000 points for a certain ADC and store it in adc_buffer
  * @retval Next Action - OK, repeat, fail
  */
Actions_TypeDef take_measurement(ADC_HandleTypeDef* hadc){

	//ADC Start (TRANSFER TO volt_vector USING DMA)
	if(HAL_ADC_Start_DMA(hadc, (uint32_t*) adc_buffer, ADC_BUFF_SIZE) == HAL_OK)
		__HAL_TIM_SET_COUNTER(&htim4, 0);
		HAL_TIM_Base_Start(&htim4);					//If not busy, starts the timer

	if (__HAL_TIM_GET_COUNTER(&htim4) >= CHANNEL_STABILIZATION_TIME){ //Wait switching stabilization time

		//Start TIM3 to trigger the ADC
		if(HAL_TIM_Base_Start(&htim3) != HAL_OK)
		  Error_Handler();

		HAL_TIM_Base_Stop(&htim4); //stops counter
		return ok;
	}
	else
		return repeat;
}

/* Auxiliary Functions END-----------------------------------------------------------------*/

/* Callback Functions BEGIN-----------------------------------------------------------------*/


//Stops TIM3 once the DMA transfer is completed (this is called by the ADC DMA IRQ Handler)
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

	//Stops ADC Measurement and Timer
	if(HAL_TIM_Base_Stop(&htim3) != HAL_OK)
		Error_Handler();

	if(HAL_ADC_Stop_DMA(hadc) != HAL_OK)
		Error_Handler();

	//uart_cplt_lck = HAL_LOCKED;
	if(fft_lck != HAL_LOCKED)
		HAL_UART_Transmit_DMA(&huart3, (uint8_t*) adc_buffer, (ADC_BUFF_SIZE));

}

//UART Receive callback (called by the UART DMA IRQ Handler)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	uart_cplt_lck = HAL_UNLOCKED;
}

//UART Transmit callback (called by the UART DMA IRQ Handler)
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	//uart_cplt_lck = HAL_UNLOCKED;
}

/* Callback Functions END-----------------------------------------------------------------*/

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

#ifdef  USE_FULL_ASSERT
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
