/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (Node C - CTRL)
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

#define CMD_TOKEN 0x42   // спільний секретний токен для CMD
#define CMD_VER   0x01
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
volatile uint8_t g_rx_flag = 0; // 0 = нічого, 1 = CMD (FIFO0), 2 = HB (FIFO1)

// Прапорці для відкладеного ACK
volatile uint8_t g_ack_pending = 0;
volatile uint8_t g_ack_cmd_id = 0;
volatile uint8_t g_ack_seq    = 0;
volatile uint8_t g_ack_result = 0;

CAN_TxHeaderTypeDef C_TxHeader;
CAN_RxHeaderTypeDef C_RxHeader;
uint8_t C_TxData[8];
uint8_t C_RxData[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
static void CAN_ConfigFilters_NodeC(void);
static void C_Send_ACK(uint8_t cmd_id, uint8_t seq, uint8_t result);
uint8_t crc8(const uint8_t *data, uint8_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// CRC8 для CmdID|Arg0|Arg1|Arg2|Token|Seq|Ver
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

// Налаштування фільтрів: 0x040 → FIFO0, 0x081/0x082 → FIFO1
static void CAN_ConfigFilters_NodeC(void)
{
    CAN_FilterTypeDef filter = {0};

    // Bank 0: точний збіг 0x040 (CMD) → FIFO0
    filter.FilterActivation     = CAN_FILTER_ENABLE;
    filter.FilterBank           = 0;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = (0x040 << 5);
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = (0x7FF << 5); // всі 11 біт важливі
    filter.FilterMaskIdLow      = 0x0000;
    filter.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &filter);

    // Bank 1: точний збіг 0x081 (HB A) → FIFO1
    filter.FilterBank           = 1;
    filter.FilterFIFOAssignment = CAN_RX_FIFO1;
    filter.FilterIdHigh         = (0x081 << 5);
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = (0x7FF << 5);
    filter.FilterMaskIdLow      = 0x0000;
    HAL_CAN_ConfigFilter(&hcan1, &filter);

    // Bank 2: точний збіг 0x082 (HB B) → FIFO1
    filter.FilterBank           = 2;
    filter.FilterFIFOAssignment = CAN_RX_FIFO1;
    filter.FilterIdHigh         = (0x082 << 5);
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = (0x7FF << 5);
    filter.FilterMaskIdLow      = 0x0000;
    HAL_CAN_ConfigFilter(&hcan1, &filter);
}

// Відправка ACK/ERR (0x041)
static void C_Send_ACK(uint8_t cmd_id, uint8_t seq, uint8_t result)
{
	uint32_t txMailbox;
	uint32_t freeLevel;

    C_TxHeader.StdId = 0x041;
    C_TxHeader.ExtId = 0;
    C_TxHeader.IDE   = CAN_ID_STD;
    C_TxHeader.RTR   = CAN_RTR_DATA;
    C_TxHeader.DLC   = 8;
    C_TxHeader.TransmitGlobalTime = DISABLE;

    memset(C_TxData, 0, sizeof(C_TxData));
    C_TxData[0] = cmd_id;
    C_TxData[1] = seq;
    C_TxData[2] = result;   // 0=OK, 1=CRC fail, 2=Token fail, 3=Ver fail

    freeLevel = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
	if (freeLevel == 0)
	{
		// ERROR: All mailboxes full. This means previous messages weren't ACKed.
		// Optional: Clear mailboxes if stuck (HAL_CAN_AbortTxRequest)
		return; // Keep g_ack_pending = 1 to retry?
	}

	if (HAL_CAN_AddTxMessage(&hcan1, &C_TxHeader, C_TxData, &txMailbox) != HAL_OK)
	{
		// Capture Error Code
		uint32_t error = HAL_CAN_GetError(&hcan1);
	}
}

/**
  * @brief Колбек для FIFO0 (Команди 0x040)
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN1) return;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &C_RxHeader, C_RxData) != HAL_OK)
        return;

    if (C_RxHeader.StdId != 0x040) return;

    uint8_t cmd_id = C_RxData[0];
    uint8_t token  = C_RxData[4];
    uint8_t seq    = C_RxData[5];
    uint8_t crc_rx = C_RxData[6];
    uint8_t ver    = C_RxData[7];

    // перерахунок CRC8 по byte[0..5] + ver (byte7)
    uint8_t tmp[7];
    memcpy(tmp, C_RxData, 6); // CmdID..Seq
    tmp[6] = ver;             // Ver
    uint8_t crc_calc = crc8(tmp, 7);

    uint8_t result;
    if (token != CMD_TOKEN) {
        result = 2;  // Token fail
    } else if (crc_rx != crc_calc) {
        result = 1;  // CRC fail
    } else if (ver != CMD_VER) {
        result = 3;  // Ver fail
    } else {
        result = 0;  // OK
    }

    // Зберігаємо ACK для відправки у main()
    g_ack_cmd_id = cmd_id;
    g_ack_seq    = seq;
    g_ack_result = result;
    g_ack_pending = 1;

    g_rx_flag = 1; // сигнал для LED-патерну (CMD)
}

/**
  * @brief Колбек для FIFO1 (Heartbeats 0x081/0x082)
  */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN1) return;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &C_RxHeader, C_RxData) != HAL_OK)
        return;

    if (C_RxHeader.StdId == 0x081 || C_RxHeader.StdId == 0x082)
    {
        g_rx_flag = 2; // сигнал для LED (HB)
    }
}

/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();

  /* USER CODE BEGIN 2 */

  // Налаштовуємо фільтри ноди C
  CAN_ConfigFilters_NodeC();

  // Стартуємо CAN
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
      Error_Handler();
  }

  // Вмикаємо IRQ по FIFO0/FIFO1
  HAL_CAN_ActivateNotification(&hcan1,
      CAN_IT_RX_FIFO0_MSG_PENDING |
      CAN_IT_RX_FIFO1_MSG_PENDING);

  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN 3 */

    // 1) Якщо є ACK — спочатку його шлемо
    if (g_ack_pending)
    {
        C_Send_ACK(g_ack_cmd_id, g_ack_seq, g_ack_result);
        g_ack_pending = 0;
    }

    // 2) Потім – LED-патерни
    if (g_rx_flag == 1) // Отримали команду 0x040 (FIFO0)
    {
        // Тут залишила твій "довгий" блим-патерн :)
        for (int i = 0; i < 8; i++)
        {
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
            HAL_Delay(50);
            HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
            HAL_Delay(50);
        }
        g_rx_flag = 0;
    }
    else if (g_rx_flag == 2) // Отримали HB у FIFO1 (0x081/0x082)
    {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        g_rx_flag = 0;
    }

    // можна додати ще якийсь watchdog/diag тут, якщо треба

    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;  // можна тимчасово DISABLE для дебага
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN CAN1_Init 2 */
  // Фільтри налаштовуємо окремо в CAN_ConfigFilters_NodeC()
  /* USER CODE END CAN1_Init 2 */
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart2.Init.OverSampling = USART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
