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
#include <stdbool.h>
#include <stddef.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
} GpioOutput_t;

typedef struct
{
  GpioOutput_t output;
  uint32_t releaseTick;
  bool active;
} RelayChannel_t;

typedef enum
{
  ESP_DEVICE_1 = 0,
  ESP_DEVICE_2,
  ESP_DEVICE_COUNT
} EspDevice_t;

typedef enum
{
  ESP_KEY_INCOME = 0,
  ESP_KEY_ANSWER,
  ESP_KEY_HANGUP,
  ESP_KEY_DIAL1,
  ESP_KEY_DIAL2,
  ESP_KEY_COUNT
} EspKey_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_CMD_STD_ID              0x321U
#define CAN_STATUS_STD_ID           0x322U
#define DEFAULT_KEY_PRESS_MS        200U
#define MIN_KEY_PRESS_MS            50U
#define MAX_KEY_PRESS_MS            5000U
#define HEARTBEAT_INTERVAL_MS       500U
#define CAN_CMD_DEVICE_ALL          0U
#define CAN_CMD_DEVICE_ESP1         1U
#define CAN_CMD_DEVICE_ESP2         2U

#define CAN_CMD_POWER_OFF           0x00U
#define CAN_CMD_POWER_ON            0x01U
#define CAN_CMD_KEY_INCOME          0x10U
#define CAN_CMD_KEY_ANSWER          0x11U
#define CAN_CMD_KEY_HANGUP          0x12U
#define CAN_CMD_KEY_DIAL1           0x13U
#define CAN_CMD_KEY_DIAL2           0x14U
#define CAN_CMD_ALL_RELAYS_OFF      0x20U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
CAN_HandleTypeDef hcan;

static RelayChannel_t relayChannels[ESP_DEVICE_COUNT][ESP_KEY_COUNT] = {
  [ESP_DEVICE_1] = {
    [ESP_KEY_INCOME] = {{K1INCOME_GPIO_Port, K1INCOME_Pin}, 0U, false},
    [ESP_KEY_ANSWER] = {{K1ANSWER_GPIO_Port, K1ANSWER_Pin}, 0U, false},
    [ESP_KEY_HANGUP] = {{K1HANGUP_GPIO_Port, K1HANGUP_Pin}, 0U, false},
    [ESP_KEY_DIAL1] = {{K1DIAL1_GPIO_Port, K1DIAL1_Pin}, 0U, false},
    [ESP_KEY_DIAL2] = {{K1DIAL2_GPIO_Port, K1DIAL2_Pin}, 0U, false},
  },
  [ESP_DEVICE_2] = {
    [ESP_KEY_INCOME] = {{K2INCOME_GPIO_Port, K2INCOME_Pin}, 0U, false},
    [ESP_KEY_ANSWER] = {{K2ANSWER_GPIO_Port, K2ANSWER_Pin}, 0U, false},
    [ESP_KEY_HANGUP] = {{K2HANGUP_GPIO_Port, K2HANGUP_Pin}, 0U, false},
    [ESP_KEY_DIAL1] = {{K2DIAL1_GPIO_Port, K2DIAL1_Pin}, 0U, false},
    [ESP_KEY_DIAL2] = {{K2DIAL2_GPIO_Port, K2DIAL2_Pin}, 0U, false},
  },
};

static const GpioOutput_t espPowerOutputs[ESP_DEVICE_COUNT] = {
  [ESP_DEVICE_1] = {MOSFET1_GPIO_Port, MOSFET1_Pin},
  [ESP_DEVICE_2] = {MOSFET2_GPIO_Port, MOSFET2_Pin},
};

static uint8_t canRxData[8];
static uint32_t lastHeartbeatTick;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
static void App_Init(void);
static void App_Task(void);
static void Heartbeat_Task(void);
static void Relay_Task(void);
static void Relay_Set(RelayChannel_t *relay, GPIO_PinState state);
static void Relay_Press(EspDevice_t device, EspKey_t key, uint32_t durationMs);
static void All_Relays_Off(void);
static void Esp_SetPower(EspDevice_t device, bool enabled);
static void Can_Start(void);
static void Can_ProcessRx(void);
static void Can_HandleCommand(const uint8_t *data, uint8_t len);
static void Can_SendStatus(uint8_t code, uint8_t detail);
static uint32_t DecodeDurationMs(const uint8_t *data, uint8_t len);
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
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  App_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    App_Task();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_CAN1_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 18;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, MOSFET1_Pin|MOSFET2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, K1INCOME_Pin|K1ANSWER_Pin|K1HANGUP_Pin|K1DIAL1_Pin
                          |K1DIAL2_Pin|K2INCOME_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, K2ANSWER_Pin|K2HANGUP_Pin|K2DIAL1_Pin|K2DIAL2_Pin
                          |OLED_SDA_Pin|OLED_SCL_Pin|LED_B_Pin|LED_G_Pin
                          |LED_R_Pin|K2CHECK_Pin|K1CHECK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : MOSFET1_Pin MOSFET2_Pin K1INCOME_Pin K1ANSWER_Pin
                           K1HANGUP_Pin K1DIAL1_Pin K1DIAL2_Pin K2INCOME_Pin */
  GPIO_InitStruct.Pin = MOSFET1_Pin|MOSFET2_Pin|K1INCOME_Pin|K1ANSWER_Pin
                          |K1HANGUP_Pin|K1DIAL1_Pin|K1DIAL2_Pin|K2INCOME_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : K2ANSWER_Pin K2HANGUP_Pin K2DIAL1_Pin K2DIAL2_Pin
                           LED_B_Pin LED_G_Pin LED_R_Pin */
  GPIO_InitStruct.Pin = K2ANSWER_Pin|K2HANGUP_Pin|K2DIAL1_Pin|K2DIAL2_Pin
                          |LED_B_Pin|LED_G_Pin|LED_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED_SDA_Pin OLED_SCL_Pin */
  GPIO_InitStruct.Pin = OLED_SDA_Pin|OLED_SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : K2CHECK_Pin K1CHECK_Pin */
  GPIO_InitStruct.Pin = K2CHECK_Pin|K1CHECK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void App_Init(void)
{
  All_Relays_Off();
  Esp_SetPower(ESP_DEVICE_1, false);
  Esp_SetPower(ESP_DEVICE_2, false);
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
  lastHeartbeatTick = HAL_GetTick();
  Can_Start();
}

static void App_Task(void)
{
  Heartbeat_Task();
  Relay_Task();
  Can_ProcessRx();
}

static void Heartbeat_Task(void)
{
  const uint32_t now = HAL_GetTick();

  if ((now - lastHeartbeatTick) >= HEARTBEAT_INTERVAL_MS)
  {
    lastHeartbeatTick = now;
    HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
  }
}

static void Relay_Task(void)
{
  const uint32_t now = HAL_GetTick();

  for (size_t device = 0U; device < ESP_DEVICE_COUNT; device++)
  {
    for (size_t key = 0U; key < ESP_KEY_COUNT; key++)
    {
      RelayChannel_t *relay = &relayChannels[device][key];
      if (relay->active && ((int32_t)(now - relay->releaseTick) >= 0))
      {
        Relay_Set(relay, GPIO_PIN_RESET);
      }
    }
  }
}

static void Relay_Set(RelayChannel_t *relay, GPIO_PinState state)
{
  HAL_GPIO_WritePin(relay->output.port, relay->output.pin, state);
  relay->active = (state == GPIO_PIN_SET);
  if (!relay->active)
  {
    relay->releaseTick = 0U;
  }
}

static void Relay_Press(EspDevice_t device, EspKey_t key, uint32_t durationMs)
{
  if ((device >= ESP_DEVICE_COUNT) || (key >= ESP_KEY_COUNT))
  {
    return;
  }

  if (durationMs < MIN_KEY_PRESS_MS)
  {
    durationMs = MIN_KEY_PRESS_MS;
  }
  else if (durationMs > MAX_KEY_PRESS_MS)
  {
    durationMs = MAX_KEY_PRESS_MS;
  }

  RelayChannel_t *relay = &relayChannels[device][key];
  relay->releaseTick = HAL_GetTick() + durationMs;
  Relay_Set(relay, GPIO_PIN_SET);
}

static void All_Relays_Off(void)
{
  for (size_t device = 0U; device < ESP_DEVICE_COUNT; device++)
  {
    for (size_t key = 0U; key < ESP_KEY_COUNT; key++)
    {
      Relay_Set(&relayChannels[device][key], GPIO_PIN_RESET);
    }
  }
}

static void Esp_SetPower(EspDevice_t device, bool enabled)
{
  if (device >= ESP_DEVICE_COUNT)
  {
    return;
  }

  /* AO3401A P-MOS high-side switch: gate low = ESP32 powered, gate high = off. */
  HAL_GPIO_WritePin(espPowerOutputs[device].port,
                    espPowerOutputs[device].pin,
                    enabled ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void Can_Start(void)
{
  CAN_FilterTypeDef filter = {0};

  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = (uint16_t)(CAN_CMD_STD_ID << 5);
  filter.FilterIdLow = 0x0000U;
  filter.FilterMaskIdHigh = 0xFFE0U;
  filter.FilterMaskIdLow = 0x0000U;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
}

static void Can_ProcessRx(void)
{
  CAN_RxHeaderTypeDef rxHeader;

  while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0U)
  {
    if (HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &rxHeader, canRxData) == HAL_OK)
    {
      if ((rxHeader.IDE == CAN_ID_STD) && (rxHeader.StdId == CAN_CMD_STD_ID))
      {
        Can_HandleCommand(canRxData, rxHeader.DLC);
      }
    }
  }
}

static void Can_HandleCommand(const uint8_t *data, uint8_t len)
{
  if (len < 2U)
  {
    Can_SendStatus(0xE1U, len);
    return;
  }

  const uint8_t target = data[0];
  const uint8_t command = data[1];
  const uint32_t durationMs = DecodeDurationMs(data, len);

  if (target > CAN_CMD_DEVICE_ESP2)
  {
    Can_SendStatus(0xE2U, target);
    return;
  }

  for (uint8_t deviceIndex = 0U; deviceIndex < ESP_DEVICE_COUNT; deviceIndex++)
  {
    if ((target != CAN_CMD_DEVICE_ALL) && (target != (deviceIndex + 1U)))
    {
      continue;
    }

    switch (command)
    {
      case CAN_CMD_POWER_OFF:
        Esp_SetPower((EspDevice_t)deviceIndex, false);
        break;

      case CAN_CMD_POWER_ON:
        Esp_SetPower((EspDevice_t)deviceIndex, true);
        break;

      case CAN_CMD_KEY_INCOME:
        Relay_Press((EspDevice_t)deviceIndex, ESP_KEY_INCOME, durationMs);
        break;

      case CAN_CMD_KEY_ANSWER:
        Relay_Press((EspDevice_t)deviceIndex, ESP_KEY_ANSWER, durationMs);
        break;

      case CAN_CMD_KEY_HANGUP:
        Relay_Press((EspDevice_t)deviceIndex, ESP_KEY_HANGUP, durationMs);
        break;

      case CAN_CMD_KEY_DIAL1:
        Relay_Press((EspDevice_t)deviceIndex, ESP_KEY_DIAL1, durationMs);
        break;

      case CAN_CMD_KEY_DIAL2:
        Relay_Press((EspDevice_t)deviceIndex, ESP_KEY_DIAL2, durationMs);
        break;

      case CAN_CMD_ALL_RELAYS_OFF:
        All_Relays_Off();
        break;

      default:
        Can_SendStatus(0xE3U, command);
        return;
    }
  }

  Can_SendStatus(0x00U, command);
}

static void Can_SendStatus(uint8_t code, uint8_t detail)
{
  CAN_TxHeaderTypeDef txHeader = {0};
  uint8_t txData[8] = {0};
  uint32_t txMailbox;

  txHeader.StdId = CAN_STATUS_STD_ID;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.DLC = 3U;
  txHeader.TransmitGlobalTime = DISABLE;

  txData[0] = code;
  txData[1] = detail;
  txData[2] = (uint8_t)(HAL_GetTick() & 0xFFU);

  (void)HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox);
}

static uint32_t DecodeDurationMs(const uint8_t *data, uint8_t len)
{
  uint32_t durationMs = DEFAULT_KEY_PRESS_MS;

  if (len >= 4U)
  {
    durationMs = (uint32_t)data[2] | ((uint32_t)data[3] << 8);
  }
  else if (len >= 3U)
  {
    durationMs = (uint32_t)data[2] * 10U;
  }

  return durationMs;
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
