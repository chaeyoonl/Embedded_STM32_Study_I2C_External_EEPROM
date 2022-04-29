/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "rb.h"
#include "calculateFunction.h"

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
 I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
RingFifo_t gtUart2Fifo;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
	if (ch == '\n')
		HAL_UART_Transmit(&huart1, (uint8_t*) "\r", 1, 0xFFFF);
		HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, 0xFFFF);

	return ch;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

    uint16_t AT24C256C_addr = 0xA0;
    uint8_t T_buffer[65];
    uint8_t R_buffer[65];

    uint8_t testsss_write[] = {0x3};
    uint8_t readBuffer[16];
    uint8_t writteBuffer[16] = {0x05, 0x06, 0x07, 0x08, 0x09, 0x05, 0x06, 0x07, 0x08, 0x09, 0x05, 0x06, 0x07, 0x08, 0x09, 0x05};
//    uint8_t writteBuffer[2];


    T_buffer[64] = 0x00;
    R_buffer[64] = 0x00;

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
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
	if (RB_init(&gtUart2Fifo, 16)) {

	}


//
//    // Byte Write
//    T_buffer[0] = 0x49;
//    HAL_I2C_Mem_Write(&hi2c2, AT24C256C_addr, 0x00, 2, T_buffer, 1, 1000);	//write to EEPROM, write to adress 0X0001
//    HAL_Delay(10);
//
//    // Byte Read
//    HAL_I2C_Mem_Read(&hi2c2, AT24C256C_addr, 0x00, 2, R_buffer, 1, 1000);
//    HAL_Delay(1);
//
//    printf("\n\n\n");
//
//	for(int i = 0 ; i < sizeof(R_buffer) ; i++) {
//		printf("%X ", R_buffer[i]);
//	}
//	printf("\n");





//
//
//	// Page Write
//    for (int i = 0; i < 64; i++)  {
//	   T_buffer[i] = i;
//    }
//
//    // Byte Write
//    HAL_I2C_Mem_Write(&hi2c2, AT24C256C_addr, 0x01, 2, T_buffer, 64, 1000);
//    HAL_Delay(10);
//
//
//    // Page Read
//    HAL_I2C_Mem_Read(&hi2c2, AT24C256C_addr, 0x01, 2, R_buffer, 64, 1000);
//    HAL_Delay(1);
//
//
//    printf("\n\n\n");
//
//	for(int i = 0 ; i < sizeof(R_buffer) ; i++) {
//		printf("%X ", R_buffer[i]);
//	}
//	printf("\n");


	printf("\n\n\n\n");
//	__HAL_RCC_I2C2_FORCE_RESET();
//	HAL_Delay(200);
//	__HAL_RCC_I2C2_RELEASE_RESET();

	// Byte Write
//	while (HAL_I2C_Mem_Write(&hi2c2, AT24C256C_addr, 0x00, 1, testsss_write, 1, 1000)!= HAL_OK) {
//		HAL_StatusTypeDef rets = HAL_I2C_Mem_Write(&hi2c2, AT24C256C_addr, 0x00, 1, testsss_write, 1, 1000);
//	}
//	HAL_Delay(1000);
//	if (rets == HAL_OK) {
//		//printf("success!! \n");
//		printf("%X  %X \n", rets, testsss_write);
//	}else {
//		printf(" %ld : Fail !!!\n",hi2c2.ErrorCode );
//	}

//	HAL_Delay(10000);




//	HAL_StatusTypeDef testRet = HAL_I2C_IsDeviceReady(&hi2c2, AT24C256C_addr, 0x05, 1000);


	writteBuffer[0] = 0x39;

//	HAL_Delay(1000);

	HAL_StatusTypeDef rets = HAL_I2C_Mem_Write(&hi2c2, AT24C256C_addr, 0x00, 1, writteBuffer, 16, 1000);
	HAL_Delay(1000);
	if (rets == HAL_OK) {
		printf("success to write!! --> %X \n", writteBuffer[0]);
	}




	// Byte Read
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c2, AT24C256C_addr, 0x00, 1, readBuffer, 16, 1000);
	if (ret == HAL_OK) {
		printf("success!! \n");
	}else {
		printf(" ret=%d, %ld : Fail !!!\n", ret, hi2c2.ErrorCode );
	}

	printf("%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X \n",
			readBuffer[0], readBuffer[1], readBuffer[2], readBuffer[3], readBuffer[4],
			readBuffer[5], readBuffer[6], readBuffer[7], readBuffer[8], readBuffer[9],
			readBuffer[10], readBuffer[11], readBuffer[12], readBuffer[13], readBuffer[14],
			readBuffer[15]);

	HAL_Delay(1000);


















  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	uint8_t ch;
	int compareTempData = 0;

	while (1) {

//		writteBuffer[0] = 0x39;
//
//		HAL_StatusTypeDef rets = HAL_I2C_Mem_Write(&hi2c2, AT24C256C_addr, 0x00, 1, writteBuffer, 16, 1000);
//		HAL_Delay(10);
//		if (rets == HAL_OK) {
//			printf("success to write!! \n");
//		}
//
//		// Byte Read
//		HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c2, AT24C256C_addr, 0x00, 1, readBuffer, 16, 1000);
//		if (ret == HAL_OK) {
//			printf("success!! \n");
//		}else {
//			printf(" ret=%d, %ld : Fail !!!\n", ret, hi2c2.ErrorCode );
//		}
//
//		printf("%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X \n",
//				readBuffer[0], readBuffer[1], readBuffer[2], readBuffer[3], readBuffer[4],
//				readBuffer[5], readBuffer[6], readBuffer[7], readBuffer[8], readBuffer[9],
//				readBuffer[10], readBuffer[11], readBuffer[12], readBuffer[13], readBuffer[14],
//				readBuffer[15]);
//
//		HAL_Delay(1000);




//#if 0
		if (!RB_isempty(&gtUart2Fifo)) {	//RB_isempty -> ringbuffer empty
			ch = RB_read(&gtUart2Fifo);
			printf("%X \n", ch);
//		  HAL_UART_Transmit (&huart2, &ch, 1, 0xFF);

			if (compare(ch, compareTempData))	compareTempData++;

			if (compareTempData == 5) {

				for (int i = 0; i < 503; i++) {
					uint8_t transmitDatas = returnTransmitData(i);
					printf("--> %X \n", transmitDatas);
					HAL_UART_Transmit(&huart2, (uint8_t*) &transmitDatas, 1, 0xFFFF);
				}


//				char * transmitDatas = returnTransmitData2();
//				for (int i = 0 ; i < 503 ; i++) {
//					printf("--> %X \n", (uint8_t*) transmitDatas[i]);
//				}
//
//				HAL_UART_Transmit(&huart2, (uint8_t*) transmitDatas, 503, 0xFFFF);


				compareTempData = 0;
			}

		}

		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

//		printf("======== \n");
//		HAL_Delay(100);
//#endif

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 160;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 799;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 40;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 99;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SYS_ON_GPIO_Port, SYS_ON_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ACC_ON_GPIO_Port, ACC_ON_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BOOT_MODE_GPIO_Port, BOOT_MODE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MAIN_ON_GPIO_Port, MAIN_ON_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SYS_ON_Pin */
  GPIO_InitStruct.Pin = SYS_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SYS_ON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ACC_ON_Pin BOOT_MODE_Pin */
  GPIO_InitStruct.Pin = ACC_ON_Pin|BOOT_MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : MAIN_ON_Pin */
  GPIO_InitStruct.Pin = MAIN_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MAIN_ON_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
	uint8_t rx;

	if (UartHandle->Instance == USART2) {
		rx = (uint8_t) (UartHandle->Instance->DR & (uint8_t) 0x00FF);
		RB_write(&gtUart2Fifo, rx);
	}
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
	while (1) {
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
