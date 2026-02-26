	/* USER CODE BEGIN Header */
	/**
	  ******************************************************************************
	  * @file           : main.c
	  * @brief          : Main program body
	  ******************************************************************************
	  * @attention
	  *
	  * Copyright (c) 2026 STMicroelectronics.
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
	#include <stdio.h>
	#include <string.h>
	#include <math.h>
	#include "bmi088.h"
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

	SPI_HandleTypeDef hspi4;

	UART_HandleTypeDef huart1;

	/* USER CODE BEGIN PV */

	BMI088_t bmi088;
	BMI088_FloatData_t floatDataG;
	BMI088_FloatData_t floatDataA;
	uint8_t id_acc;
	uint8_t id_gyro;
	float global_z;
	float g_constant = 9.80665;
	/* USER CODE END PV */

	/* Private function prototypes -----------------------------------------------*/
	void SystemClock_Config(void);
	static void MX_GPIO_Init(void);
	static void MX_SPI4_Init(void);
	static void MX_USART1_UART_Init(void);
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
	  MX_SPI4_Init();
	  MX_USART1_UART_Init();
	  /* USER CODE BEGIN 2 */
	  BMI088_InitAccel(&bmi088, &hspi4, CSB_ACCEL_GPIO_Port, CSB_ACCEL_Pin);
	  BMI088_InitGyro(&bmi088, &hspi4, CSB_GYRO_GPIO_Port, CSB_GYRO_Pin);

	  id_acc = BMI088_ReadChipID_Accel(&bmi088);
	  id_gyro = BMI088_ReadChipID_Gyro(&bmi088);

	  /* USER CODE END 2 */

	  /* Infinite loop */
	  /* USER CODE BEGIN WHILE */
	  while (1)
	  {
		  BMI088_ReadAccelXYZ_MG(&bmi088, &floatDataA);
		  BMI088_ReadGyroXYZ_DPS(&bmi088, &floatDataG);
		  global_z = sqrt(powf(floatDataA.x * g_constant, 2) + powf(floatDataA.y * g_constant, 2) + powf(floatDataA.z*g_constant, 2));
		  HAL_Delay(100);
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
	  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

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
	  RCC_OscInitStruct.PLL.PLLN = 125;
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

	  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	  {
		Error_Handler();
	  }

	  /** Configure the programming delay
	  */
	  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
	}

	/**
	  * @brief SPI4 Initialization Function
	  * @param None
	  * @retval None
	  */
	static void MX_SPI4_Init(void)
	{

	  /* USER CODE BEGIN SPI4_Init 0 */

	  /* USER CODE END SPI4_Init 0 */

	  /* USER CODE BEGIN SPI4_Init 1 */

	  /* USER CODE END SPI4_Init 1 */
	  /* SPI4 parameter configuration*/
	  hspi4.Instance = SPI4;
	  hspi4.Init.Mode = SPI_MODE_MASTER;
	  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
	  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
	  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
	  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
	  hspi4.Init.NSS = SPI_NSS_SOFT;
	  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
	  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
	  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	  hspi4.Init.CRCPolynomial = 0x7;
	  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
	  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
	  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
	  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
	  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
	  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
	  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
	  hspi4.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
	  hspi4.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
	  if (HAL_SPI_Init(&hspi4) != HAL_OK)
	  {
		Error_Handler();
	  }
	  /* USER CODE BEGIN SPI4_Init 2 */

	  /* USER CODE END SPI4_Init 2 */

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
	  huart1.Init.BaudRate = 115200;
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
	  __HAL_RCC_GPIOE_CLK_ENABLE();
	  __HAL_RCC_GPIOB_CLK_ENABLE();
	  __HAL_RCC_GPIOD_CLK_ENABLE();
	  __HAL_RCC_GPIOG_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(GPIOD, CSB_ACCEL_Pin|CSB_GYRO_Pin, GPIO_PIN_SET);

	  /*Configure GPIO pin Output Level */
	  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

	  /*Configure GPIO pin : INT2_ACCEL_Pin */
	  GPIO_InitStruct.Pin = INT2_ACCEL_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(INT2_ACCEL_GPIO_Port, &GPIO_InitStruct);

	  /*Configure GPIO pins : INT1_ACCEL_Pin INT4_GYRO_Pin INT3_GYRO_Pin */
	  GPIO_InitStruct.Pin = INT1_ACCEL_Pin|INT4_GYRO_Pin|INT3_GYRO_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	  /*Configure GPIO pins : CSB_ACCEL_Pin CSB_GYRO_Pin */
	  GPIO_InitStruct.Pin = CSB_ACCEL_Pin|CSB_GYRO_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	  /*Configure GPIO pin : LED1_Pin */
	  GPIO_InitStruct.Pin = LED1_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

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
