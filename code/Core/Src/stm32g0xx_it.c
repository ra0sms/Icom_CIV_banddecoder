/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32g0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern uint8_t TRXData[15];
extern int flag_band;
extern char flag_mode;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM14 global interrupt.
  */
void TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM14_IRQn 0 */
	if (LL_TIM_IsActiveFlag_UPDATE(TIM14)) {
		LL_TIM_ClearFlag_UPDATE(TIM14);
		LL_GPIO_TogglePin(LINK_GPIO_Port, LINK_Pin);
	}

  /* USER CODE END TIM14_IRQn 0 */
  /* USER CODE BEGIN TIM14_IRQn 1 */

  /* USER CODE END TIM14_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void) {
	/* USER CODE BEGIN USART1_IRQn 0 */
	uint8_t letter;
	static uint8_t i = 0;
	if (LL_USART_IsActiveFlag_RXNE(USART1)) {
		letter = LL_USART_ReceiveData8(USART1);
		if (letter != 0xFD) {
			TRXData[i] = letter;
			i++;
			if (i == 15)
				i = 0;
		} else {
			TRXData[i] = 0xFD;
			i = 0;
			if ((TRXData[0] == 0xFE) && (TRXData[1] == 0xFE)
					&& ((TRXData[4] == 0x00)||(TRXData[4] == 0x03))) {
				if ((TRXData[8] == 0x01) || (TRXData[8] == 0x02))
					flag_band = 160; //160m 1000-2999 kHz
				if ((TRXData[8] == 0x03) || (TRXData[8] == 0x04))
					flag_band = 80; //80m 3000-4999 kHz
				if ((TRXData[8] == 0x05))
					flag_band = 60; //60m 5000-5999 kHz
				if ((TRXData[8] == 0x07))
					flag_band = 40; //40m 7000-7999 kHz
				if ((TRXData[8] == 0x10))
					flag_band = 30; //30m 10000-10999 kHz
				if ((TRXData[8] == 0x14))
					flag_band = 20; //20m 14000-14999 kHz
				if ((TRXData[8] == 0x18))
					flag_band = 17; //17m 18000-18999 kHz
				if ((TRXData[8] == 0x21))
					flag_band = 15; //15m 21000-21999 kHz
				if ((TRXData[8] == 0x24))
					flag_band = 12; //12m 24000-24999 kHz
				if ((TRXData[8] == 0x28) || (TRXData[8] == 0x29))
					flag_band = 10; //10m 28000-29999 kHz
				if ((TRXData[8] == 0x050))
					flag_band = 6; //6m 50000-50999 kHz
			}
			if ((TRXData[0] == 0xFE) && (TRXData[1] == 0xFE)
					&& ((TRXData[4] == 0x01)||(TRXData[4] == 0x04))) {
				if ((TRXData[5] == 0x03))
					flag_mode = 'C';		//CW
				if ((TRXData[5] == 0x00) || (TRXData[5] == 0x01)
						|| (TRXData[5] == 0x02) || (TRXData[5] == 0x05)
						|| (TRXData[5] == 0x06))
					flag_mode = 'P';		//Phone
			} /*else flag_mode=0;*/

		}

	}

	/* USER CODE END USART1_IRQn 0 */
	/* USER CODE BEGIN USART1_IRQn 1 */

	/* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
