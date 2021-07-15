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
#include <string.h>
#include "math.h"
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
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t ADCin = 0;
uint64_t _micro = 0;
uint64_t timestamp = 0;

char TxDataBuffer[100] =
{ 0 };
char RxDataBuffer[32] =
{ 0 };

uint32_t Period = 1000000;
uint8_t Hz = 10;
uint8_t V_Max = 33;
uint8_t V_Min = 0;
int8_t Slope = 1;
uint8_t Duty_Cycle = 50;

typedef enum
{
	Mode_Menu,
	Mode_Wait,
	Saw_Menu,
	Saw_Wait,
	Sine_Menu,
	Sine_Wait,
	Square_Menu,
	Square_Wait
}Menu_State;
Menu_State State = Mode_Menu;

typedef enum
{
	None,
	Sawtooth,
	Sine,
	Square
}Gen_Mode;
Gen_Mode Mode = None;

uint16_t dataOut = 0;

uint8_t DACConfig = 0b0011;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */
void MCP4922SetOutput(uint8_t Config, uint16_t DACOutput);
uint64_t micros();
uint16_t UARTRecieveIT();
void Menu(int16_t input);
void Function_Gen();
void Calc();

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_SPI3_Init();
  MX_TIM3_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start_IT(&htim11);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &ADCin, 1);

	HAL_GPIO_WritePin(LOAD_GPIO_Port, LOAD_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		HAL_UART_Receive_IT(&huart2,  (uint8_t*)RxDataBuffer, 32);

		int16_t inputchar = UARTRecieveIT();
		Menu(inputchar);
		Function_Gen();
		if (micros() - timestamp >= Period)
		{
			timestamp = micros();

			if (hspi3.State == HAL_SPI_STATE_READY
					&& HAL_GPIO_ReadPin(SPI_SS_GPIO_Port, SPI_SS_Pin)
							== GPIO_PIN_SET)
			{
				MCP4922SetOutput(DACConfig, dataOut);
			}
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100;
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
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 99;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SHDN_GPIO_Port, SHDN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LOAD_GPIO_Port, LOAD_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LOAD_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LOAD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_SS_Pin */
  GPIO_InitStruct.Pin = SPI_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_SS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SHDN_Pin */
  GPIO_InitStruct.Pin = SHDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SHDN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint16_t UARTRecieveIT()
{
	static uint32_t dataPos =0;
	int16_t data=-1;
	if(huart2.RxXferSize - huart2.RxXferCount!=dataPos)
	{
		data=RxDataBuffer[dataPos];
		dataPos= (dataPos+1)%huart2.RxXferSize;
	}
	return data;
}

void Menu(int16_t input)
{
	if ((input <= 90) && (input >= 65))
	{
		input += 32;
	}

	switch (State)
	{
	case Mode_Menu:
		sprintf(TxDataBuffer, "--------------\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "Mode Select\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "  1 : Sawtooth\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "  2 : Sine\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "  3 : Square\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "--------------\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		State = Mode_Wait;
		break;
	case Mode_Wait:
		switch (input)
		{
		case -1:
			break;
		case 49:
			State = Saw_Menu;
			Mode = Sawtooth;
			break;
		case 50:
			State = Sine_Menu;
			Mode = Sine;
			break;
		case 51:
			State = Square_Menu;
			Mode = Square;
			break;
		default:
			sprintf(TxDataBuffer, "Wrong Input\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
			break;
		}
		break;
	case Saw_Menu:
		sprintf(TxDataBuffer, "--------------\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "Sawtooth\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "  1 : Frequency +\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "  2 : Frequency -\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "  3 : V_Max +\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "  4 : V_Max -\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "  5 : V_Min +\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "  6 : V_Min -\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "  7 : Slope Up/Down\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "  9 : back\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "--------------\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		State = Saw_Wait;
		break;
	case Saw_Wait:
		switch (input)
		{
		case -1:
			break;
		case 49:  //1 F+
			if (Hz < 100)
			{
				Hz += 1;
				sprintf(TxDataBuffer, "Frequency = %d.%d Hz\r\n", Hz/10, Hz%10);
			}
			else if (Hz == 100)
			{
				sprintf(TxDataBuffer, "Frequency upper limit is 10 Hz\r\n");
			}

			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
			break;
		case 50:  //2 F-
			if (Hz > 0)
			{
				Hz -= 1;
				sprintf(TxDataBuffer, "Frequency = %d.%d Hz\r\n", Hz/10, Hz%10);
			}
			else if (Hz == 0)
			{
				sprintf(TxDataBuffer, "Frequency lower limit is 0 Hz\r\n");
			}

			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
			break;
		case 51:  //3 V_Max+
			if (V_Max < 33)
			{
				V_Max += 1;
				data_MAX += 4096/3.3
				sprintf(TxDataBuffer, "V_Max = %d.%d V\r\n", V_Max/10, V_Max%10);
			}
			else if (V_Max == 33)
			{
				sprintf(TxDataBuffer, "V_Max upper limit is 3.3 V\r\n");
			}

			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
			break;
		case 52:  //4 V_Max-
			if (V_Max == V_Min+1)
			{
				sprintf(TxDataBuffer, "V_Max must be more than V_Min\r\n");
			}
			else if (V_Max > V_Min+1)
			{
				V_Max -= 1;
				sprintf(TxDataBuffer, "V_Max = %d.%d V\r\n", V_Max/10, V_Max%10);
			}

			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
			break;
		case 53:  //5 V_Min+
			if (V_Min == V_Max-1)
			{
				sprintf(TxDataBuffer, "V_Min must be less than to V_Max\r\n");
			}
			else if (V_Min < V_Max-1)
			{
				V_Min += 1;
				sprintf(TxDataBuffer, "V_Min = %d.%d V\r\n", V_Min/10, V_Min%10);
			}

			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
			break;
		case 54:  //6 V_Min-
			if (V_Min > 0)
			{
				V_Min -= 1;
				sprintf(TxDataBuffer, "V_Min = %d.%d V\r\n", V_Min/10, V_Min%10);
			}
			else if (V_Min == 0)
			{
				sprintf(TxDataBuffer, "V_Min lower limit is 0 V\r\n");
			}

			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
			break;
		case 55:  //7 S
			Slope = -Slope;
			sprintf(TxDataBuffer, "Slope = %d \r\n", Slope);
			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
			break;
		case 57:  //9 back
			State = Mode_Menu;
			Mode = None;
			break;
		default:
			sprintf(TxDataBuffer, "Wrong Input\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
			break;
		}
		break;
	case Sine_Menu:
		sprintf(TxDataBuffer, "--------------\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "Sine\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "  1 : Frequency +\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "  2 : Frequency -\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "  3 : V_Max +\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "  4 : V_Max -\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "  5 : V_Min +\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "  6 : V_Min -\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "  9 : back\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "--------------\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		State = Sine_Wait;
		break;
	case Sine_Wait:
		switch (input)
		{
		case -1:
			break;
		case 49:  //1 F+
			if (Hz < 100)
			{
				Hz += 1;
				sprintf(TxDataBuffer, "Frequency = %d.%d Hz\r\n", Hz/10, Hz%10);
			}
			else if (Hz == 100)
			{
				sprintf(TxDataBuffer, "Frequency upper limit is 10 Hz\r\n");
			}

			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
			break;
		case 50:  //2 F-
			if (Hz > 0)
			{
				Hz -= 1;
				sprintf(TxDataBuffer, "Frequency = %d.%d Hz\r\n", Hz/10, Hz%10);
			}
			else if (Hz == 0)
			{
				sprintf(TxDataBuffer, "Frequency lower limit is 0 Hz\r\n");
			}

			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
			break;
		case 51:  //3 V_Max+
			if (V_Max < 33)
			{
				V_Max += 1;
				sprintf(TxDataBuffer, "V_Max = %d.%d V\r\n",V_Max/10, V_Max%10);
			}
			else if (V_Max == 33)
			{
				sprintf(TxDataBuffer, "V_Max upper limit is 3.3 V\r\n");
			}

			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
			break;
		case 52:  //4 V_Max-
			if (V_Max == V_Min+1)
			{
				sprintf(TxDataBuffer, "V_Max must be more than V_Min\r\n");
			}
			else if (V_Max > V_Min+1)
			{
				V_Max -= 1;
				sprintf(TxDataBuffer, "V_Max = %d.%d V\r\n", V_Max/10, V_Max%10);
			}

			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
			break;
		case 53:  //5 V_Min+
			if (V_Min == V_Max-1)
			{
				sprintf(TxDataBuffer, "V_Min must be less than to V_Max\r\n");
			}
			else if (V_Min < V_Max-1)
			{
				V_Min += 1;
				sprintf(TxDataBuffer, "V_Min = %d.%d V\r\n", V_Min/10, V_Min%10);
			}

			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
			break;
		case 54:  //6 V_Min-
			if (V_Min > 0)
			{
				V_Min -= 1;
				sprintf(TxDataBuffer, "V_Min = %d.%d V\r\n", V_Min/10, V_Min%10);
			}
			else if (V_Min == 0)
			{
				sprintf(TxDataBuffer, "V_Min lower limit is 0 V\r\n");
			}

			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
			break;
		case 57:  //back
			State = Mode_Menu;
			Mode = None;
			break;
		default:
			sprintf(TxDataBuffer, "Wrong Input\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
			break;
		}
		break;
	case Square_Menu:
		sprintf(TxDataBuffer, "--------------\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "Square\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "  1 : Frequency +\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "  2 : Frequency -\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "  3 : V_Max +\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "  4 : V_Max -\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "  5 : V_Min +\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "  6 : V_Min -\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "  7 : Duty Cycle +\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "  8 : Duty Cycle -\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "  9 : back\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		sprintf(TxDataBuffer, "--------------\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
		State = Square_Wait;
		break;
	case Square_Wait:
		switch (input)
		{
		case -1:
			break;
		case 49:  //1 F+
			if (Hz < 100)
			{
				Hz += 1;
				sprintf(TxDataBuffer, "Frequency = %d.%d Hz\r\n", Hz/10, Hz%10);
			}
			else if (Hz == 100)
			{
				sprintf(TxDataBuffer, "Frequency upper limit is 10 Hz\r\n");
			}

			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
			break;
		case 50:  //2 F-
			if (Hz > 0)
			{
				Hz -= 1;
				sprintf(TxDataBuffer, "Frequency = %d.%d Hz\r\n", Hz/10, Hz%10);
			}
			else if (Hz == 0)
			{
				sprintf(TxDataBuffer, "Frequency lower limit is 0 Hz\r\n");
			}

			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
			break;
		case 51:  //3 V_Max+
			if (V_Max < 33)
			{
				V_Max += 1;
				sprintf(TxDataBuffer, "V_Max = %d.%d V\r\n", V_Max/10, V_Max%10);
			}
			else if (V_Max == 33)
			{
				sprintf(TxDataBuffer, "V_Max upper limit is 3.3 V\r\n");
			}

			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
			break;
		case 52:  //4 V_Max-
			if (V_Max == V_Min+1)
			{
				sprintf(TxDataBuffer, "V_Max must be more than V_Min\r\n");
			}
			else if (V_Max > V_Min+1)
			{
				V_Max -= 1;
				sprintf(TxDataBuffer, "V_Max = %d.%d V\r\n", V_Max/10, V_Max%10);
			}

			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
			break;
		case 53:  //5 V_Min+
			if (V_Min == V_Max-1)
			{
				sprintf(TxDataBuffer, "V_Min must be less than to V_Max\r\n");
			}
			else if (V_Min < V_Max-1)
			{
				V_Min += 1;
				sprintf(TxDataBuffer, "V_Min = %d.%d V\r\n",V_Min/10, V_Min%10);
			}

			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
			break;
		case 54:  //6 V_Min-
			if (V_Min > 0)
			{
				V_Min -= 1;
				sprintf(TxDataBuffer, "V_Min = %d.%d V\r\n",V_Max/10, V_Max%10);
			}
			else if (V_Min == 0)
			{
				sprintf(TxDataBuffer, "V_Min lower limit is 0 V\r\n");
			}

			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
			break;
		case 55:  //7 D+
			if ( Duty_Cycle < 100)
			{
				Duty_Cycle += 1;
				sprintf(TxDataBuffer, "Duty Cycle = %d \r\n", Duty_Cycle);
			}
			else if (Duty_Cycle == 100)
			{
				sprintf(TxDataBuffer, "Duty cycle upper limit is 100\r\n");
			}

			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
			break;
		case 56:  //8 D-
			if (Duty_Cycle > 0)
			{
				Duty_Cycle -= 1;
				sprintf(TxDataBuffer, "Duty Cycle = %d \r\n", Duty_Cycle);
			}
			else if (V_Min == 0)
			{
				sprintf(TxDataBuffer, "Duty Cycle lower limit is 0\r\n");
			}

			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 100);
			break;
		case 57:  //back
			State = Mode_Menu;
			Mode = None;
			break;
		default:
			sprintf(TxDataBuffer, "Wrong Input\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*)TxDataBuffer, strlen(TxDataBuffer), 1000);
			break;
		}
		break;
	}
}

void Function_Gen()
{
	static float rad = 0;
	static uint8_t V_Out = 0;
	if (Hz > 0)
	{
		switch (Mode)
		{
		case None:
			dataOut = 0;
			break;
		case Sawtooth:
			Period = 1000000/(Hz*(V_Max-V_Min)*4096/330);
			if (Slope == 1)
			{
				if (dataOut >= (V_Max*4096/33))
				{
					dataOut = V_Min*4096/33;
				}
				dataOut += 1;
			}
			else if (Slope == -1)
			{
				if (dataOut <= (V_Min*4096/33))
				{
					dataOut = V_Max*4096/33;
				}
				dataOut -= 1;
			}

			break;
		case Sine:
			dataOut = (V_Max-V_Min)*(sin(rad));
			rad+=0.01;
			break;
		case Square:
			V_Out += 1;
			if (V_Out >= Duty_Cycle)
			{
				V_Out = V_Max;
			}
			else if (V_Out < Duty_Cycle)
			{
				V_Out = V_Min;
			}

			if (V_Out >= 100)
			{
				V_Out = 0;
			}
			break;
		}
	}
	else if (Hz == 0)
	{
		dataOut = V_Min*4096/33;
	}

}

void MCP4922SetOutput(uint8_t Config, uint16_t DACOutput)
{
	uint32_t OutputPacket = (DACOutput & 0x0fff) | ((Config & 0xf) << 12);
	HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit_IT(&hspi3, &OutputPacket, 1);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi == &hspi3)
	{
		HAL_GPIO_WritePin(SPI_SS_GPIO_Port, SPI_SS_Pin, GPIO_PIN_SET);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim11)
	{
		_micro += 65535;
	}
}

uint64_t micros()
{
	return htim11.Instance->CNT + _micro;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
