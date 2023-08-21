/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"
#include "stm32g0xx_ll_tim.h"
#include "stm32g0xx_ll_usart.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_exti.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"
#include "stm32g0xx_ll_dma.h"

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
#define LED10_Pin LL_GPIO_PIN_9
#define LED10_GPIO_Port GPIOB
#define LED_CW_Pin LL_GPIO_PIN_14
#define LED_CW_GPIO_Port GPIOC
#define LINK_Pin LL_GPIO_PIN_15
#define LINK_GPIO_Port GPIOC
#define BAND1_Pin LL_GPIO_PIN_0
#define BAND1_GPIO_Port GPIOA
#define BAND2_Pin LL_GPIO_PIN_1
#define BAND2_GPIO_Port GPIOA
#define BAND3_Pin LL_GPIO_PIN_2
#define BAND3_GPIO_Port GPIOA
#define BAND4_Pin LL_GPIO_PIN_3
#define BAND4_GPIO_Port GPIOA
#define BAND5_Pin LL_GPIO_PIN_4
#define BAND5_GPIO_Port GPIOA
#define BAND6_Pin LL_GPIO_PIN_5
#define BAND6_GPIO_Port GPIOA
#define BAND7_Pin LL_GPIO_PIN_6
#define BAND7_GPIO_Port GPIOA
#define BAND8_Pin LL_GPIO_PIN_7
#define BAND8_GPIO_Port GPIOA
#define BAND9_Pin LL_GPIO_PIN_0
#define BAND9_GPIO_Port GPIOB
#define BAND10_Pin LL_GPIO_PIN_1
#define BAND10_GPIO_Port GPIOB
#define OUT_CW_Pin LL_GPIO_PIN_2
#define OUT_CW_GPIO_Port GPIOB
#define OUT_SSB_Pin LL_GPIO_PIN_8
#define OUT_SSB_GPIO_Port GPIOA
#define BTN_STOP_Pin LL_GPIO_PIN_6
#define BTN_STOP_GPIO_Port GPIOC
#define LED1_Pin LL_GPIO_PIN_11
#define LED1_GPIO_Port GPIOA
#define LED2_Pin LL_GPIO_PIN_12
#define LED2_GPIO_Port GPIOA
#define LED3_Pin LL_GPIO_PIN_15
#define LED3_GPIO_Port GPIOA
#define LED4_Pin LL_GPIO_PIN_3
#define LED4_GPIO_Port GPIOB
#define LED5_Pin LL_GPIO_PIN_4
#define LED5_GPIO_Port GPIOB
#define LED6_Pin LL_GPIO_PIN_5
#define LED6_GPIO_Port GPIOB
#define LED7_Pin LL_GPIO_PIN_6
#define LED7_GPIO_Port GPIOB
#define LED8_Pin LL_GPIO_PIN_7
#define LED8_GPIO_Port GPIOB
#define LED9_Pin LL_GPIO_PIN_8
#define LED9_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
