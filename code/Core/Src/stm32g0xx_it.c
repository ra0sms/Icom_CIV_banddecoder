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

#include "band_rules.h"

volatile uint8_t cmd_buffer[256];
volatile uint16_t cmd_index = 0;
volatile uint8_t cmd_ready = 0;

extern int flag_band;
extern char flag_mode;
extern band_rule_t band_rules[];
extern uint8_t rules_count;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TRX_BUFFER_SIZE 32

void uart_rx_command_handler_byte(uint8_t b)
{
    if (cmd_ready) return;
    if (cmd_index == 0) {
        if (b != 0xAA) return;
    }
    if (cmd_index < sizeof(cmd_buffer)) {
        cmd_buffer[cmd_index++] = b;
    } else {
        cmd_index = 0;
        return;
    }
    if (cmd_index == 2) {
        uint8_t count = cmd_buffer[1];
        if (count == 0 || count > MAX_RULES) {
            cmd_index = 0;
            return;
        }
    }
    if (cmd_index >= 2) {
        uint8_t count = cmd_buffer[1];
        uint16_t expected_size = 2 + count * 10 + 2;
        if (expected_size > sizeof(cmd_buffer)) {
            cmd_index = 0;
            return;
        }
        if (cmd_index == expected_size) {
            if (cmd_buffer[cmd_index - 1] == 0x55) {
                cmd_ready = 1;
            } else {
                cmd_index = 0;
            }
        }
    }
}

uint32_t get_frequency_khz(uint8_t *data)
{
    uint32_t freq = 0;

    for (int i = 9; i >= 5; i--) {
        uint8_t b = data[i];
        freq *= 100;
        freq += ((b >> 4) * 10) + (b & 0x0F);
    }

    return freq / 1000;
}

void decode_by_freq(uint32_t freq_khz)
{
    flag_band = 0;
    flag_mode = 0;

    for (uint8_t i = 0; i < rules_count; i++) {

        if (freq_khz >= band_rules[i].start_khz &&
            freq_khz <  band_rules[i].end_khz) {

            flag_band = band_rules[i].band;
            flag_mode = band_rules[i].mode;
            return;
        }
    }
}


void HF_band_decode_byte(uint8_t letter)
{
    static uint8_t i = 0;
    static uint8_t TRXData[TRX_BUFFER_SIZE];

    if (i == 0 && letter != 0xFE) return;

    if (i == 1 && letter != 0xFE) {
        i = 0;
        return;
    }
    if (i < TRX_BUFFER_SIZE) {
        TRXData[i++] = letter;
    } else {
        i = 0;
        return;
    }
    if (letter == 0xFD) {
        if (i >= 10) {
            if ((TRXData[4] == 0x00 || TRXData[4] == 0x03)) {
                uint32_t freq_khz = get_frequency_khz(TRXData);
                decode_by_freq(freq_khz);
            }
        }

        i = 0;
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

void USART1_IRQHandler(void)
{
    if (!LL_USART_IsActiveFlag_RXNE(USART1)) return;

    uint8_t b = LL_USART_ReceiveData8(USART1);
    uart_rx_command_handler_byte(b);
    HF_band_decode_byte(b);
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
