/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t g_rx_flag = 0; // 0=нічого, 1=FIFO0, 2=FIFO1
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

/**
  * @brief Колбек для FIFO0 (Команди 0x040)
  * Встановлюємо прапорець для main()
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	if (RxHeader.StdId == 0x040)
	{
		g_rx_flag = 1; // Сигнал для main()
	}
}

/**
  * @brief Колбек для FIFO1 (Heartbeats 0x081)
  */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData);
	if (RxHeader.StdId == 0x081)
	{
		g_rx_flag = 2; // Сигнал для main()
	}
}
//CAN_TxHeaderTypeDef TxHeader;
//CAN_RxHeaderTypeDef RxHeader;
//
//uint32_t TxMailbox[4];
//
//uint8_t TxData[8];
//uint8_t RxData[8];
//
//uint8_t count = 0;
//
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
//	count++;
//    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//}
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
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
//  __HAL_CAN_CLEAR_FLAG(&hcan1, CAN_FLAG_WKU);
  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
      Error_Handler();
  }
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);

//  TxHeader.DLC = 8; // Згідно з док. 8 байт (CmdID|...|CRC8) [cite: 22, 25]
//    TxHeader.ExtId = 0;
//    TxHeader.IDE = CAN_ID_STD;
//    TxHeader.RTR = CAN_RTR_DATA;
//    TxHeader.StdId = 0x040; // ID для CMD (Команди)
//    TxHeader.TransmitGlobalTime = DISABLE;
//
//    // "Фейкові" дані команди [cite: 25]
//    TxData[0] = 0x01; // CmdID
//    TxData[1] = 0x0A; // Arg0
//    TxData[2] = 0x0B; // Arg1
//    TxData[3] = 0x0C; // Arg2
//    TxData[4] = 0x44; // Token
//    TxData[5] = 0x01; // Seq
//    TxData[6] = 0xAB; // CRC8
//    TxData[7] = 0x01; // Ver
//  TxHeader.DLC = 1;
//  TxHeader.ExtId = 0;
//  TxHeader.IDE = CAN_ID_STD;
//  TxHeader.RTR = CAN_RTR_DATA;
//  TxHeader.StdId = 0x111;
//  TxHeader.TransmitGlobalTime = DISABLE;
//
//  TxData[0] = 0x01;
//  TxData[1] = 0x02;
//  TxData[2] = 0x03;
//  TxData[3] = 0x04;
//
//  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, &TxData[0], &TxMailbox[0]) != HAL_OK)
//  {
//	  Error_Handler();
//  }
//  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, &TxData[1], &TxMailbox[1]) != HAL_OK)
//  {
//	  Error_Handler();
//  }
//  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, &TxData[2], &TxMailbox[2]) != HAL_OK)
//  {
//	  Error_Handler();
//  }
//  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, &TxData[3], &TxMailbox[3]) != HAL_OK)
//  {
//	  Error_Handler();
//  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox[0]) != HAL_OK)
//	  {
//		  Error_Handler();
//	  }
//	  HAL_Delay(100);
	  if (g_rx_flag == 1) // Отримали повідомлення у FIFO0 (0x040)
	  {
		  // Виконуємо подвійний блимк тут, у main()
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		  HAL_Delay(50);
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		  HAL_Delay(50);
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		  HAL_Delay(50);
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

		  g_rx_flag = 0; // Скидаємо прапорець
	  }
	  else if (g_rx_flag == 2) // Отримали повідомлення у FIFO1 (0x081)
	  {
		  // Виконуємо Toggle тут, у main()
		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

		  g_rx_flag = 0; // Скидаємо прапорець
	  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  // --- Налаштування Фільтра 0 (для FIFO0) ---
    CAN_FilterTypeDef canfilterconfig_fifo0;

    canfilterconfig_fifo0.FilterActivation = CAN_FILTER_ENABLE;
    canfilterconfig_fifo0.FilterBank = 0; // Використовуємо банк 0
    canfilterconfig_fifo0.FilterFIFOAssignment = CAN_RX_FIFO0; // Направляємо у FIFO0 [cite: 46]
    canfilterconfig_fifo0.FilterMode = CAN_FILTERMODE_IDLIST; // Режим "Точний список ID"
    canfilterconfig_fifo0.FilterScale = CAN_FILTERSCALE_32BIT; // 32-бітний масштаб

    // Дозволяємо ID 0x040
    // Використовуємо зсув << 5, як вимагає bxCAN HAL [cite: 44]
    canfilterconfig_fifo0.FilterIdHigh = (0x040 << 5);
    canfilterconfig_fifo0.FilterIdLow = 0x0000;
    // Маски не використовуються в режимі IDLIST
    canfilterconfig_fifo0.FilterMaskIdHigh = 0x0000;
    canfilterconfig_fifo0.FilterMaskIdLow = 0x0000;

    canfilterconfig_fifo0.SlaveStartFilterBank = 14; // Розділ банків (для F4)

    if (HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig_fifo0) != HAL_OK)
    {
        Error_Handler();
    }


    // --- Налаштування Фільтра 1 (для FIFO1) ---
    CAN_FilterTypeDef canfilterconfig_fifo1;

    canfilterconfig_fifo1.FilterActivation = CAN_FILTER_ENABLE;
    canfilterconfig_fifo1.FilterBank = 1; // Використовуємо банк 1
    canfilterconfig_fifo1.FilterFIFOAssignment = CAN_RX_FIFO1; // Направляємо у FIFO1 [cite: 47]
    canfilterconfig_fifo1.FilterMode = CAN_FILTERMODE_IDLIST;
    canfilterconfig_fifo1.FilterScale = CAN_FILTERSCALE_32BIT;

    // Дозволяємо ID 0x081 (Heartbeat від Node A)
    canfilterconfig_fifo1.FilterIdHigh = (0x081 << 5); // Зсув << 5 [cite: 44]
    canfilterconfig_fifo1.FilterIdLow = 0x0000;
    canfilterconfig_fifo1.FilterMaskIdHigh = 0x0000;
    canfilterconfig_fifo1.FilterMaskIdLow = 0x0000;

    // SlaveStartFilterBank ігнорується для банків > 0, але встановимо для ясності
    canfilterconfig_fifo1.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig_fifo1) != HAL_OK)
    {
        Error_Handler();
    }
  //  CAN_FilterTypeDef canfilterconfig;
//
//  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
//  canfilterconfig.FilterBank = 10;  // anything between 0 to SlaveStartFilterBank
//  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
////  canfilterconfig.FilterIdHigh = 0x103<<5;
//  canfilterconfig.FilterIdHigh = 0x0000;
//  canfilterconfig.FilterIdLow = 0x0000;
////  canfilterconfig.FilterMaskIdHigh = 0x7FF<<5;
//  canfilterconfig.FilterMaskIdHigh = 0x0000;
//  canfilterconfig.FilterMaskIdLow = 0x0000;
//  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
//  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
//  canfilterconfig.SlaveStartFilterBank = 14;  // 13 to 27 are assigned to slave CAN (CAN 2) OR 0 to 12 are assgned to CAN1
//
//  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
  /* USER CODE END CAN1_Init 2 */

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
#ifdef USE_FULL_ASSERT
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
