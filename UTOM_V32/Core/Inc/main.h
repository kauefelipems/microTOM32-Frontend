/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef uint8_t (*State_FunctionsTypeDef)(void);

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define I_P_Pin GPIO_PIN_0
#define I_P_GPIO_Port GPIOC
#define I_N_Pin GPIO_PIN_3
#define I_N_GPIO_Port GPIOC
#define MUX_A0_Pin GPIO_PIN_0
#define MUX_A0_GPIO_Port GPIOA
#define PGA_DB_Pin GPIO_PIN_4
#define PGA_DB_GPIO_Port GPIOA
#define PGA_LIN_Pin GPIO_PIN_5
#define PGA_LIN_GPIO_Port GPIOA
#define MUX_A1_Pin GPIO_PIN_0
#define MUX_A1_GPIO_Port GPIOB
#define MUX_CS4_Pin GPIO_PIN_10
#define MUX_CS4_GPIO_Port GPIOE
#define MUX_CS3_Pin GPIO_PIN_12
#define MUX_CS3_GPIO_Port GPIOE
#define MUX_CS2_Pin GPIO_PIN_14
#define MUX_CS2_GPIO_Port GPIOE
#define MUX_CS1_Pin GPIO_PIN_15
#define MUX_CS1_GPIO_Port GPIOE
#define MUX_A4_Pin GPIO_PIN_10
#define MUX_A4_GPIO_Port GPIOB
#define MUX_A3_Pin GPIO_PIN_11
#define MUX_A3_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define VMEAS_N_Pin GPIO_PIN_12
#define VMEAS_N_GPIO_Port GPIOD
#define VMEAS_P_Pin GPIO_PIN_13
#define VMEAS_P_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define HWL_DIBS_Pin GPIO_PIN_7
#define HWL_DIBS_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define MUX_A2_Pin GPIO_PIN_0
#define MUX_A2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

//Machine Constants
#define N_CHANNELS 32
#define FREQUENCY 10000
#define	AMPLITUDE 1.5
#define MIN_GAIN 0
#define MAX_GAIN 1.1

//UART Constants
#define UART_TIMEOUT 1000 									//ms
#define ADC_BUFF_SIZE 2000     								//Data Buffer
#define COMM_BUFF_SIZE 5  									//Command Buffer
#define TERMINATOR 2573     								//CR/LF Terminator in 16 bits
#define RING_BUFFER_SIZE ADC_BUFF_SIZE						//Buffer to transmit data

// State Machine Constant
#define ENTRY_STATE 0 	    /*defines entry state (allows for further change without
								modifying the main())*/

// Channel Stabilization Time
#define CHANNEL_STABILIZATION_TIME 1 // [timer cycles] 1 cycle = 1µs (tim4)

// Multiplexer Configuration Stabilization Time
#define MUX_DELAY 5 // [timer cycles]  1 cycle = 14ns (sysclk)

// PGA
// Initial PGA Voltage (255 is 1.1V and 24 is 0.1035V)
#define PGA_MIN_VOLTAGE 24 // [bits]
#define PGA_MAX_VOLTAGE 255 // [bits]

//FFT
#define FFT_SIZE 2048  // FFT size
#define ADC_MAX 4095   // Maximum ADC value for normalization

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

//GET and SET the Timer Repetition Counter Register


//DAC Macros
#define __DAC1_VOLTAGE2BIT(__VOLTAGE__) (uint16_t)__VOLTAGE__*4095/(561)
#define __DAC2_VOLTAGE2BIT(__VOLTAGE__) (uint16_t)__VOLTAGE__*4095/(255)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
