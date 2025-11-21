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
#include <string.h>
#include <stdbool.h>

#define CMD_TOKEN 0x42
#define CMD_VER   0x01

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DMA_RX_BUFFER_SIZE 256
#define GPS_PROCESS_BUFFER_SIZE 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
uint8_t g_dma_rx_buffer[DMA_RX_BUFFER_SIZE];
uint8_t g_gps_process_buffer[GPS_PROCESS_BUFFER_SIZE];
volatile uint8_t g_gps_data_ready = 0;
volatile uint16_t g_gps_data_size = 0;

int32_t lat_e5 = 0; // Широта * 100000
int32_t lon_e5 = 0; // Довгота * 100000
uint8_t sats_view = 0;
uint8_t fix_status = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//CAN_TxHeaderTypeDef TxHeader;
//CAN_RxHeaderTypeDef RxHeader;
//
//uint32_t TxMailbox[4];
//
//uint8_t TxData[8];
//uint8_t RxData[8];
//
uint8_t count = 0;

// TX буфери
CAN_TxHeaderTypeDef A_TxHeader;
CAN_RxHeaderTypeDef A_RxHeader;
uint32_t A_TxMailbox;
uint8_t  A_TxData[8];
uint8_t A_RxData[8];

// “GPS”

uint32_t tick_tlm  = 0;
uint32_t tick_hb   = 0;
uint32_t tick_cmd  = 0;

static uint8_t cmd_seq = 0;


uint8_t crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    const uint8_t poly = 0x07; // x^8 + x^2 + x + 1

    for (uint8_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++)
        {
            if (crc & 0x80)
                crc = (crc << 1) ^ poly;
            else
                crc <<= 1;
        }
    }
    return crc;
}

void A_Send_TLM120(void)
{
    uint32_t lat24 = ((uint32_t)lat_e5) & 0xFFFFFF;
    uint32_t lon24 = ((uint32_t)lon_e5) & 0xFFFFFF;

    A_TxHeader.StdId = 0x120;
    A_TxHeader.ExtId = 0;
    A_TxHeader.IDE   = CAN_ID_STD;
    A_TxHeader.RTR   = CAN_RTR_DATA;
    A_TxHeader.DLC   = 8;
    A_TxHeader.TransmitGlobalTime = DISABLE;

    A_TxData[0] = (lat24 >> 16) & 0xFF;
    A_TxData[1] = (lat24 >> 8) & 0xFF;
    A_TxData[2] = (lat24) & 0xFF;

    A_TxData[3] = (lon24 >> 16) & 0xFF;
    A_TxData[4] = (lon24 >> 8) & 0xFF;
    A_TxData[5] = (lon24) & 0xFF;

    A_TxData[6] = sats_view;
    A_TxData[7] = fix_status;

    HAL_CAN_AddTxMessage(&hcan1, &A_TxHeader, A_TxData, &A_TxMailbox);
}

// HB A: 0x081 (1 байт статусу)
void A_Send_HB081(void)
{
    A_TxHeader.StdId = 0x081;
    A_TxHeader.ExtId = 0;
    A_TxHeader.IDE   = CAN_ID_STD;
    A_TxHeader.RTR   = CAN_RTR_DATA;
    A_TxHeader.DLC   = 1;
    A_TxHeader.TransmitGlobalTime = DISABLE;

    A_TxData[0] = 0xA1;

    HAL_CAN_AddTxMessage(&hcan1, &A_TxHeader, A_TxData, &A_TxMailbox);
}

// НОРМАЛЬНА команда 0x040 з валідним CRC/Token
void A_Send_CMD_Normal(uint8_t cmd_id, uint8_t arg0, uint8_t arg1, uint8_t arg2)
{
    A_TxHeader.StdId = 0x040;
    A_TxHeader.ExtId = 0;
    A_TxHeader.IDE   = CAN_ID_STD;
    A_TxHeader.RTR   = CAN_RTR_DATA;
    A_TxHeader.DLC   = 8;
    A_TxHeader.TransmitGlobalTime = DISABLE;

    A_TxData[0] = cmd_id;
    A_TxData[1] = arg0;
    A_TxData[2] = arg1;
    A_TxData[3] = arg2;
    A_TxData[4] = CMD_TOKEN;
    A_TxData[5] = cmd_seq;
    A_TxData[7] = CMD_VER;

    uint8_t tmp[7];
    memcpy(tmp, A_TxData, 6); // 0..5
    tmp[6] = A_TxData[7];     // Ver
    A_TxData[6] = crc8(tmp, 7);

    HAL_CAN_AddTxMessage(&hcan1, &A_TxHeader, A_TxData, &A_TxMailbox);

    cmd_seq++;
}

// ФАЛЬШИВА команда (поганий Token/CRC) – для Red-team тесту
void A_Send_CMD_Fake(void)
{
    A_TxHeader.StdId = 0x040;
    A_TxHeader.ExtId = 0;
    A_TxHeader.IDE   = CAN_ID_STD;
    A_TxHeader.RTR   = CAN_RTR_DATA;
    A_TxHeader.DLC   = 8;
    A_TxHeader.TransmitGlobalTime = DISABLE;

    A_TxData[0] = 0x10;
    A_TxData[1] = 0xAA;
    A_TxData[2] = 0xBB;
    A_TxData[3] = 0xCC;
    A_TxData[4] = 0x99;    // неправильний Token
    A_TxData[5] = cmd_seq;
    A_TxData[7] = CMD_VER;
    A_TxData[6] = 0x00;    // спеціально кривий CRC

    HAL_CAN_AddTxMessage(&hcan1, &A_TxHeader, A_TxData, &A_TxMailbox);

    cmd_seq++;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &A_RxHeader, A_RxData);
	count++;
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}
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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  if(HAL_UARTEx_ReceiveToIdle_DMA(&huart2, g_dma_rx_buffer, DMA_RX_BUFFER_SIZE) != HAL_OK) {
	  Error_Handler();
  }
  __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
//  __HAL_CAN_CLEAR_FLAG(&hcan1, CAN_FLAG_WKU);
  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
      Error_Handler();
  }
//  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
//  	TxHeader.DLC = 1; // 1 байт даних
//    TxHeader.ExtId = 0;
//    TxHeader.IDE = CAN_ID_STD;
//    TxHeader.RTR = CAN_RTR_DATA;
//    TxHeader.TransmitGlobalTime = DISABLE;
//
//    TxData[0] = 0xAA; // Тестові дані

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
	  if (g_gps_data_ready) {
	          char* line = strtok((char*)g_gps_process_buffer, "\r\n");

	          while (line != NULL)
	          {
	              // Шукаємо рядки GNGGA або GPGGA
	              if (strncmp(line, "$GNGGA", 6) == 0 || strncmp(line, "$GPGGA", 6) == 0) {
	                  float nmea_time, nmea_lat, nmea_lon, altitude;
	                  char lat_dir, lon_dir;
	                  int fix_quality, num_sats;

	                  // Знаходимо початок даних (після коми)
	                  const char* data_start = strchr(line, ',');
	                  if (data_start) {
	                      // Парсимо рядок
	                      int items = sscanf(data_start + 1, "%f,%f,%c,%f,%c,%d,%d,%*f,%f,",
	                                         &nmea_time, &nmea_lat, &lat_dir, &nmea_lon, &lon_dir,
	                                         &fix_quality, &num_sats, &altitude);

	                      if (items >= 7 && fix_quality > 0) {
	                          // 1. Конвертуємо координати з формату NMEA (ddmm.mmmm) в десяткові градуси (dd.ddddd)
	                          int lat_deg_int = (int)(nmea_lat / 100);
	                          float lat_min = nmea_lat - (lat_deg_int * 100);
	                          float lat_decimal = lat_deg_int + (lat_min / 60.0f);
	                          if (lat_dir == 'S') lat_decimal = -lat_decimal;

	                          int lon_deg_int = (int)(nmea_lon / 100);
	                          float lon_min = nmea_lon - (lon_deg_int * 100);
	                          float lon_decimal = lon_deg_int + (lon_min / 60.0f);
	                          if (lon_dir == 'W') lon_decimal = -lon_decimal;

	                          // 2. Оновлюємо глобальні змінні для CAN (формат: * 100000)
	                          lat_e5 = (int32_t)(lat_decimal * 100000.0f);
	                          lon_e5 = (int32_t)(lon_decimal * 100000.0f);
	                          sats_view = (uint8_t)num_sats;
	                          fix_status = (uint8_t)fix_quality; // 1 = GPS fix, 2 = DGPS fix
	                      } else {
	                           // Якщо фіксу немає, можна слати нулі або статус помилки
	                           fix_status = 0;
	                      }
	                  }
	              }
	              line = strtok(NULL, "\r\n"); // Наступний рядок
	          }
	          g_gps_data_ready = 0;
	      }
	  uint32_t now = HAL_GetTick();

	  // 10 Гц – 0x120
	  if (now - tick_tlm >= 10)
	  {
	      tick_tlm += 100;          // стабільніший період, ніж tick_tlm = now;
	      A_Send_TLM120();
	  }

	  // 1 Гц – HB 0x081
	  if (now - tick_hb >= 1000)
	  {
	      tick_hb += 1000;
	      A_Send_HB081();
	  }

	  // ~раз на 2.7 с – CMD (не кратно 1000, щоб фази “плавали”)
	  if (now - tick_cmd >= 2700)
	  {
	      tick_cmd += 2700;
//	      A_Send_CMD_Normal(0x10, 1, 2, 3);
	      A_Send_CMD_Fake();
	      // або A_Send_CMD_Fake();
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 90;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
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
  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 0;
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;

  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;

  canfilterconfig.FilterIdHigh = (0x0000 << 5);
  canfilterconfig.FilterIdLow = 0x0000;

  canfilterconfig.FilterMaskIdHigh = (0x0700 << 5);
  canfilterconfig.FilterMaskIdLow = 0x0000;

  canfilterconfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig) != HAL_OK)
  {
      Error_Handler();
  }
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
  huart2.Init.BaudRate = 9600;
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART2) {
        // Копіюємо отримані дані в буфер обробки
        if (Size > 0 && Size < GPS_PROCESS_BUFFER_SIZE) {
            memcpy(g_gps_process_buffer, g_dma_rx_buffer, Size);
            g_gps_process_buffer[Size] = '\0'; // Завершуємо рядок
            g_gps_data_size = Size;
            g_gps_data_ready = 1; // Прапорець: дані готові до парсингу
        }

        // Перезапускаємо DMA на прийом
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, g_dma_rx_buffer, DMA_RX_BUFFER_SIZE);
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT); // Вимикаємо переривання половини буфера
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
