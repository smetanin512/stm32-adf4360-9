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
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



typedef enum
{
	BAND_SELECT_CLOCK_DIVIDER_1=0,
	BAND_SELECT_CLOCK_DIVIDER_2=1,
	BAND_SELECT_CLOCK_DIVIDER_4=2,
	BAND_SELECT_CLOCK_DIVIDER_8=3
} BAND_SELECT_CLOCK_DIVIDER;

typedef enum
{
	THREE_CONSECUTIVE_CYCLES=0,
	FIVE_CONSECUTIVE_CYCLES=1
} LOCK_DETECT_PRECISION;


typedef enum
{
	PULSE_WIDTH_1_3ns=1,
	PULSE_WIDTH_3ns=0,
	PULSE_WIDTH_6=2,
} ANTIBACKLASH_PULSE_WIDTH;

typedef enum
{
	NORMAL_OPERATION=4,
	ASYNCHRONOUS_POWER_DOWN=5,
	SYNCHRONOUS_POWER_DOWN=7
} MODE;

typedef enum
{
	ICP_mA_0_31=0,
	ICP_mA_0_62=1,
	ICP_mA_0_93=2,
	ICP_mA_1_25=3,
	ICP_mA_1_56=4,
	ICP_mA_1_87=5,
	ICP_mA_2_18=6,
	ICP_mA_2_50=7
} ICP_mA;

typedef enum
{
	POWER_MINUS_9dbm=0,
	POWER_MINUS_6dbm=1,
	POWER_MINUS_3dbm=2,
	POWER_0dbm=3
} OUTPUT_POWER_LEVEL;

typedef enum
{
	 MUTE_DISABLE=0,
	 MUTE_ENABLE=1
} MUTE_TIL_LOCK_DETECT;

typedef enum
{
	NORMAL=0,
	THREE_STATE=1
} CHARGE_PUMP_OUTPUT;


typedef enum
{
	NEGATIVE=0,
	POSITIVE=1
} PHASE_DETECTOR_POLARITY;

typedef enum
{
	MUX_DVDD=0,
	MUX_DIGITAL_LOCK_DETECT=1,
	MUX_N_DIVIDER_OUTPUT=2,
	MUX_R_DIVIDER_OUTPUT=4,
	MUX_A_CNTR_2_OUT=5,
	MUX_A_CNTR_OUT=6,
	MUX_DGND=7
} MUX_OUT;

typedef enum
{
	OP_NORMAL=0,
	OP_R_A_B_COUNTERS_HELD_IN_RESET=1
} COUNTER_OPERATION;


typedef enum
{
	CORE_POWER_LEVEL_2_5mA=0,
	CORE_POWER_LEVEL_5_0mA=1,
	CORE_POWER_LEVEL_7_5mA=2,
	CORE_POWER_LEVEL_10_0mA=3
} CORE_POWER_LEVEL;



typedef struct
{
	// R counter Latch
	BAND_SELECT_CLOCK_DIVIDER band_select_clock_devider;
	LOCK_DETECT_PRECISION lock_detect_precision;
	ANTIBACKLASH_PULSE_WIDTH antibacklash_pulse_width;
	uint16_t R_div;
	// N counter latch
	uint16_t B_div;
	uint8_t A_div;
	// Control Latch
	MODE mode;
	ICP_mA Icp_mA;
	OUTPUT_POWER_LEVEL output_power_level;
	MUTE_TIL_LOCK_DETECT mute_til_lock_detect;
	CHARGE_PUMP_OUTPUT charge_pump_output;
	PHASE_DETECTOR_POLARITY phase_detector_polarity;
	MUX_OUT mux_out;
	COUNTER_OPERATION counter_operation;
	CORE_POWER_LEVEL core_power_level;
} ADF4360_9_settings;







/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint8_t Rcounter_Latch[3]={0,0,0};
  uint8_t Control_Latch[3]={0,0,0};
  uint8_t Ncounter_Latch[3]={0,0,0};
  ADF4360_9_settings settings;
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
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Заполняем настройки ( выбирать из предложенного )

  /*
	BAND_SELECT_CLOCK_DIVIDER_1
	BAND_SELECT_CLOCK_DIVIDER_2
	BAND_SELECT_CLOCK_DIVIDER_4
	BAND_SELECT_CLOCK_DIVIDER_8
   */
  	settings.band_select_clock_devider 		=     BAND_SELECT_CLOCK_DIVIDER_8 ;
  /*
   	THREE_CONSECUTIVE_CYCLES
	FIVE_CONSECUTIVE_CYCLES
  */
	settings.lock_detect_precision 		=         FIVE_CONSECUTIVE_CYCLES ;
  /*
   	PULSE_WIDTH_1_3ns
	PULSE_WIDTH_3ns
	PULSE_WIDTH_6
   */
	settings.antibacklash_pulse_width 	 =      PULSE_WIDTH_3ns ;
	//  В регистре R counter latch - R делитель (R14-R1) ( число от 1 до 16383 )
    settings.R_div						 =        	 2 ;


	//  В регистре N counter latch - B делитель (B13-B1) ( число от 3 до 8191 )
	settings.B_div                       =        10 ;
	//  В регистре N counter latch - A делитель (A5-A1)  (число от 2 до 31)
	settings.A_div                       =       8 ;

	/*
	NORMAL_OPERATION
	ASYNCHRONOUS_POWER_DOWN
	SYNCHRONOUS_POWER_DOWN
	 */
	settings.mode						= 		NORMAL_OPERATION ;

	/*
	ICP_mA_0_31
	ICP_mA_0_62
	ICP_mA_0_93
	ICP_mA_1_25
	ICP_mA_1_56
	ICP_mA_1_87
	ICP_mA_2_18
	ICP_mA_2_50
	 */
	settings.Icp_mA                     =     	ICP_mA_1_25 ;

	/*

	POWER_MINUS_9dbm
	POWER_MINUS_6dbm
	POWER_MINUS_3dbm
	POWER_0dbm
	 */
	settings.output_power_level			=		POWER_MINUS_9dbm ;

	/*
	 MUTE_DISABLE
	 MUTE_ENABLE
	 */
	settings.mute_til_lock_detect		=		MUTE_DISABLE ;

	/*
	NORMAL
	THREE_STATE
	 */
	settings.charge_pump_output 		=		NORMAL ;


	/*
	NEGATIVE
	POSITIVE
	 */
	settings.phase_detector_polarity	=		NEGATIVE ;


	settings.mux_out 					=		MUX_DVDD ;

	/*
	OP_NORMAL
	OP_R_A_B_COUNTERS_HELD_IN_RESET
	 */
	settings.counter_operation			=		OP_NORMAL ;

	/*
	CORE_POWER_LEVEL_2_5mA
	CORE_POWER_LEVEL_5_0mA
	CORE_POWER_LEVEL_7_5mA
	CORE_POWER_LEVEL_10_0mA
	 */
	settings.core_power_level			=		CORE_POWER_LEVEL_5_0mA ;


////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

  // Ждем пока микруха включится
  HAL_Delay(100); //100 мс

  // Заполняем массивы на отправку
  //R counter Latch
  Rcounter_Latch[0]= (uint8_t)(settings.band_select_clock_devider<<4) | (uint8_t)(settings.lock_detect_precision<<2) | settings.antibacklash_pulse_width;
  Rcounter_Latch[1]= (uint8_t)(settings.R_div>>6);
  Rcounter_Latch[2]= (uint8_t)(settings.R_div<<2) | 0x01;
  //Control Latch
  Control_Latch[0]=(settings.mode<<4) | (settings.Icp_mA>>2);
  Control_Latch[1]= (settings.Icp_mA<<6) | (settings.output_power_level<<4)| (settings.mute_til_lock_detect<<3) | (settings.charge_pump_output<<1) | settings.phase_detector_polarity;
  Control_Latch[2]= (settings.mux_out<<5) | (settings.counter_operation<<4) | (settings.core_power_level<<2);
  //N counter Latch

  Ncounter_Latch[0]= (settings.charge_pump_output<<5) | (uint8_t)(settings.B_div >>8);
  Ncounter_Latch[1]= (uint8_t)settings.B_div;
  Ncounter_Latch[2]= (uint8_t)(settings.A_div<<2) | 0x02; ;




  // Заполняем регистры ADF4360
  HAL_GPIO_WritePin(ADF4360_LE_GPIO_Port, ADF4360_LE_Pin, GPIO_PIN_RESET); // LE on
  // Отправляем по SPI R counter Latch
  HAL_SPI_Transmit(&hspi2,Rcounter_Latch,3,100);
  HAL_GPIO_WritePin(ADF4360_LE_GPIO_Port, ADF4360_LE_Pin, GPIO_PIN_SET); // LE off
  HAL_Delay(15);


  HAL_GPIO_WritePin(ADF4360_LE_GPIO_Port, ADF4360_LE_Pin, GPIO_PIN_RESET); // LE on
  // Отправляем по SPI Control Latch
  HAL_SPI_Transmit(&hspi2,Control_Latch,3,100);
  HAL_GPIO_WritePin(ADF4360_LE_GPIO_Port, ADF4360_LE_Pin, GPIO_PIN_SET); // LE off
  HAL_Delay(15);


  HAL_GPIO_WritePin(ADF4360_LE_GPIO_Port, ADF4360_LE_Pin, GPIO_PIN_RESET); // LE on
  // Отправляем по SPI N counter Latch
  HAL_SPI_Transmit(&hspi2,Ncounter_Latch,3,100);
  HAL_GPIO_WritePin(ADF4360_LE_GPIO_Port, ADF4360_LE_Pin, GPIO_PIN_SET); // LE off
  HAL_Delay(15);

  /* USER CODE END 2 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_4);
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADF4360_LE_GPIO_Port, ADF4360_LE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ADF4360_LE_Pin */
  GPIO_InitStruct.Pin = ADF4360_LE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ADF4360_LE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
