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
extern int flag_band;
extern char flag_mode;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TRX_BUFFER_SIZE 32

void HF_band_decode(void) {
    static uint8_t i = 0;
    static uint8_t TRXData[TRX_BUFFER_SIZE];

    if (!LL_USART_IsActiveFlag_RXNE(USART1)) {
        return;
    }
    uint8_t letter = LL_USART_ReceiveData8(USART1);
    if (i == 0 && letter != 0xFE) {
        return;
    }
    if (i == 1 && letter != 0xFE) {
        i = 0;
        return;
    }
    if (i < TRX_BUFFER_SIZE - 1) {
        TRXData[i++] = letter;
    } else {
        i = 0;
        return;
    }
    if (letter == 0xFD) {
        if (i >= 9) {
            if ((TRXData[4] == 0x00 || TRXData[4] == 0x03)) {
                uint8_t band_code = TRXData[8];
                if (band_code == 0x01) flag_band = 160;
                else if (band_code == 0x03) flag_band = 80;
                else if (band_code == 0x05) flag_band = 60;
                else if (band_code == 0x07) flag_band = 40;
                else if (band_code == 0x10) flag_band = 30;
                else if (band_code == 0x14) flag_band = 20;
                else if (band_code == 0x18) flag_band = 17;
                else if (band_code == 0x21) flag_band = 15;
                else if (band_code == 0x24) flag_band = 12;
                else if (band_code == 0x28 || band_code == 0x29) flag_band = 10;
                else if (band_code == 0x50) flag_band = 6;
            }

            if ((TRXData[4] == 0x01 || TRXData[4] == 0x04)) {
                uint8_t mode_code = TRXData[5];
                if (mode_code == 0x03) {
                    flag_mode = 'C'; // CW
                } else if (mode_code == 0x00 || mode_code == 0x01 ||
                           mode_code == 0x02 || mode_code == 0x05 ||
                           mode_code == 0x06) {
                    flag_mode = 'P'; // Phone
                }
            }
        }
        i = 0;
    }
}

void VHF_band_decode() {
	uint8_t letter;
	static uint8_t TRXData[TRX_BUFFER_SIZE];
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
					&& ((TRXData[4] == 0x00) || (TRXData[4] == 0x03))) {
				if ((TRXData[9] == 0x01))
					flag_band = 160; // 100-199 MHz
				if ((TRXData[9] == 0x04))
					flag_band = 80; // 400-499MHz
				if ((TRXData[9] == 0x12))
					flag_band = 40; //1200-1299MHz
			}
			if ((TRXData[0] == 0xFE) && (TRXData[1] == 0xFE)
					&& ((TRXData[4] == 0x01) || (TRXData[4] == 0x04))) {
				if ((TRXData[5] == 0x03))
					flag_mode = 'C';		//CW
				if ((TRXData[5] == 0x00) || (TRXData[5] == 0x01)
						|| (TRXData[5] == 0x02) || (TRXData[5] == 0x05)
						|| (TRXData[5] == 0x06))
					flag_mode = 'P';		//Phone
			} /*else flag_mode=0;*/

		}

	}
}

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
   HF_band_decode();
   //VHF_band_decode();
	/* USER CODE END USART1_IRQn 0 */
	/* USER CODE BEGIN USART1_IRQn 1 */

	/* USER CODE END USART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
