/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
int flag_band=0;
char flag_mode=0;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

void ResetOuts() {
	LL_GPIO_ResetOutputPin(BAND1_GPIO_Port, BAND1_Pin);
	LL_GPIO_ResetOutputPin(BAND2_GPIO_Port, BAND2_Pin);
	LL_GPIO_ResetOutputPin(BAND3_GPIO_Port, BAND3_Pin);
	LL_GPIO_ResetOutputPin(BAND4_GPIO_Port, BAND4_Pin);
	LL_GPIO_ResetOutputPin(BAND5_GPIO_Port, BAND5_Pin);
	LL_GPIO_ResetOutputPin(BAND6_GPIO_Port, BAND6_Pin);
	LL_GPIO_ResetOutputPin(BAND7_GPIO_Port, BAND7_Pin);
	LL_GPIO_ResetOutputPin(BAND8_GPIO_Port, BAND8_Pin);
	LL_GPIO_ResetOutputPin(BAND9_GPIO_Port, BAND9_Pin);
	LL_GPIO_ResetOutputPin(BAND10_GPIO_Port, BAND10_Pin);
}


void ResetLeds() {
	LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
	LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);
	LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);
	LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);
	LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);
	LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin);
	LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);
	LL_GPIO_ResetOutputPin(LED8_GPIO_Port, LED8_Pin);
	LL_GPIO_ResetOutputPin(LED9_GPIO_Port, LED9_Pin);
	LL_GPIO_ResetOutputPin(LED10_GPIO_Port, LED10_Pin);
}

void SetBand(int x){
	ResetOuts();
	switch (x){
	case 1: LL_GPIO_SetOutputPin(BAND1_GPIO_Port, BAND1_Pin); break;
	case 2: LL_GPIO_SetOutputPin(BAND2_GPIO_Port, BAND2_Pin); break;
	case 3: LL_GPIO_SetOutputPin(BAND3_GPIO_Port, BAND3_Pin); break;
	case 4: LL_GPIO_SetOutputPin(BAND4_GPIO_Port, BAND4_Pin); break;
	case 5: LL_GPIO_SetOutputPin(BAND5_GPIO_Port, BAND5_Pin); break;
	case 6: LL_GPIO_SetOutputPin(BAND6_GPIO_Port, BAND6_Pin); break;
	case 7: LL_GPIO_SetOutputPin(BAND7_GPIO_Port, BAND7_Pin); break;
	case 8: LL_GPIO_SetOutputPin(BAND8_GPIO_Port, BAND8_Pin); break;
	case 9: LL_GPIO_SetOutputPin(BAND9_GPIO_Port, BAND9_Pin); break;
	case 10: LL_GPIO_SetOutputPin(BAND10_GPIO_Port, BAND10_Pin); break;
	}
}

void SetLed(int x){
	ResetLeds();
	switch (x){
	case 1: LL_GPIO_SetOutputPin(LED1_GPIO_Port, LED1_Pin); break;
	case 2: LL_GPIO_SetOutputPin(LED2_GPIO_Port, LED2_Pin); break;
	case 3: LL_GPIO_SetOutputPin(LED3_GPIO_Port, LED3_Pin); break;
	case 4: LL_GPIO_SetOutputPin(LED4_GPIO_Port, LED4_Pin); break;
	case 5: LL_GPIO_SetOutputPin(LED5_GPIO_Port, LED5_Pin); break;
	case 6: LL_GPIO_SetOutputPin(LED6_GPIO_Port, LED6_Pin); break;
	case 7: LL_GPIO_SetOutputPin(LED7_GPIO_Port, LED7_Pin); break;
	case 8: LL_GPIO_SetOutputPin(LED8_GPIO_Port, LED8_Pin); break;
	case 9: LL_GPIO_SetOutputPin(LED9_GPIO_Port, LED9_Pin); break;
	case 10: LL_GPIO_SetOutputPin(LED10_GPIO_Port, LED10_Pin); break;
	}
}

void SetMode() {
	if (flag_mode == 'C') {
		LL_GPIO_SetOutputPin(OUT_CW_GPIO_Port, OUT_CW_Pin);
		LL_GPIO_ResetOutputPin(OUT_SSB_GPIO_Port, OUT_SSB_Pin);
		LL_GPIO_ResetOutputPin(LED_CW_GPIO_Port, LED_CW_Pin);
	}
	if (flag_mode == 'P') {
		LL_GPIO_ResetOutputPin(OUT_CW_GPIO_Port, OUT_CW_Pin);
		LL_GPIO_SetOutputPin(OUT_SSB_GPIO_Port, OUT_SSB_Pin);
		LL_GPIO_SetOutputPin(LED_CW_GPIO_Port, LED_CW_Pin);
	}
	if (flag_mode == 0) {
		LL_GPIO_ResetOutputPin(OUT_CW_GPIO_Port, OUT_CW_Pin);
		LL_GPIO_ResetOutputPin(OUT_SSB_GPIO_Port, OUT_SSB_Pin);
		LL_GPIO_ResetOutputPin(LED_CW_GPIO_Port, LED_CW_Pin);
	}
}

void SetOut(void) {
	if (flag_band == 160) {
		SetLed(1);
		SetBand(1);
	}
	if (flag_band == 80) {
		SetLed(2);
		SetBand(2);
	}
	if (flag_band == 40) {
		SetLed(3);
		SetBand(3);
	}
	if (flag_band == 30) {
		SetLed(4);
		SetBand(4);
	}
	if (flag_band == 20) {
		SetLed(5);
		SetBand(5);
	}
	if (flag_band == 17) {
		SetLed(6);
		SetBand(6);
	}
	if (flag_band == 15) {
		SetLed(7);
		SetBand(7);
	}
	if (flag_band == 12) {
		SetLed(8);
		SetBand(8);
	}
	if (flag_band == 10) {
		SetLed(9);
		SetBand(9);
	}
	if (flag_band == 6) {
		SetLed(10);
		SetBand(10);
	}
	if (flag_band == 0) {
		ResetLeds();
		ResetOuts();
	}

}

void SetOut_UN3M(void) {
	if (flag_band == 160) {
		SetLed(1);
		SetBand(1);
	}
	if (flag_band == 80) {
		SetLed(2);
		SetBand(2);
	}
	if (flag_band == 40) {
		SetLed(3);
		SetBand(3);
	}
	if (flag_band == 30) {
		SetLed(4);
		SetBand(4);
	}
	if (flag_band == 20) {
		SetLed(5);
		SetBand(5);
	}
	if (flag_band == 17) {
		SetLed(6);
		SetBand(6);
	}
	if (flag_band == 15) {
		SetLed(7);
		SetBand(7);
	}
	if (flag_band == 12) {
		SetLed(8);
		SetBand(8);
	}
	if (flag_band == 10) {
		SetLed(7);
		SetBand(7);
	}
	if (flag_band == 6) {
		SetLed(10);
		SetBand(10);
	}
	if (flag_band == 60) {
			SetLed(9);
			SetBand(9);
		}
	if (flag_band == 0) {
		ResetLeds();
		ResetOuts();
	}

}

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_USART1_UART_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  LL_TIM_EnableCounter(TIM14);
  LL_TIM_EnableIT_UPDATE(TIM14);
  LL_USART_EnableIT_RXNE(USART1);
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
	while (1) {
		if (LL_GPIO_IsInputPinSet(BTN_STOP_GPIO_Port, BTN_STOP_Pin) == 1) {
			SetOut();
			//SetOut_UN3M();
			SetMode();
		}
		LL_mDelay(400);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM14);

  /* TIM14 interrupt Init */
  NVIC_SetPriority(TIM14_IRQn, 0);
  NVIC_EnableIRQ(TIM14_IRQn);

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  TIM_InitStruct.Prescaler = 31999;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 499;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM14, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM14);
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, 0);
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate = 19200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
  LL_USART_DisableFIFO(USART1);
  LL_USART_ConfigHalfDuplexMode(USART1);

  /* USER CODE BEGIN WKUPType USART1 */

  /* USER CODE END WKUPType USART1 */

  LL_USART_Enable(USART1);

  /* Polling USART1 initialisation */
  while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
  {
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOC);
  LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

  /**/
  LL_GPIO_ResetOutputPin(LED10_GPIO_Port, LED10_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED_CW_GPIO_Port, LED_CW_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LINK_GPIO_Port, LINK_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BAND1_GPIO_Port, BAND1_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BAND2_GPIO_Port, BAND2_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BAND3_GPIO_Port, BAND3_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BAND4_GPIO_Port, BAND4_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BAND5_GPIO_Port, BAND5_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BAND6_GPIO_Port, BAND6_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BAND7_GPIO_Port, BAND7_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BAND8_GPIO_Port, BAND8_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BAND9_GPIO_Port, BAND9_Pin);

  /**/
  LL_GPIO_ResetOutputPin(BAND10_GPIO_Port, BAND10_Pin);

  /**/
  LL_GPIO_ResetOutputPin(OUT_CW_GPIO_Port, OUT_CW_Pin);

  /**/
  LL_GPIO_ResetOutputPin(OUT_SSB_GPIO_Port, OUT_SSB_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED8_GPIO_Port, LED8_Pin);

  /**/
  LL_GPIO_ResetOutputPin(LED9_GPIO_Port, LED9_Pin);

  /**/
  GPIO_InitStruct.Pin = LED10_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED10_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED_CW_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED_CW_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LINK_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LINK_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BAND1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BAND1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BAND2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BAND2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BAND3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BAND3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BAND4_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BAND4_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BAND5_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BAND5_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BAND6_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BAND6_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BAND7_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BAND7_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BAND8_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BAND8_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BAND9_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BAND9_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BAND10_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(BAND10_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = OUT_CW_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(OUT_CW_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = OUT_SSB_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(OUT_SSB_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = BTN_STOP_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(BTN_STOP_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED4_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED5_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED5_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED6_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED6_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED7_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED7_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED8_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED8_GPIO_Port, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LED9_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(LED9_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
