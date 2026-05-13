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
  GPIO_TypeDef *port;
  uint16_t pin;
} GpioInput_t;

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
#define OLED_REFRESH_INTERVAL_MS    500U
#define OLED_I2C_ADDR               0x3CU
#define OLED_WIDTH                  128U
#define OLED_HEIGHT                 64U
#define OLED_PAGES                  (OLED_HEIGHT / 8U)
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

static const GpioInput_t espCheckInputs[ESP_DEVICE_COUNT] = {
  [ESP_DEVICE_1] = {K1CHECK_GPIO_Port, K1CHECK_Pin},
  [ESP_DEVICE_2] = {K2CHECK_GPIO_Port, K2CHECK_Pin},
};

static uint8_t canRxData[8];
static uint8_t oledBuffer[OLED_WIDTH * OLED_PAGES];
static bool espPowerEnabled[ESP_DEVICE_COUNT];
static bool espCheckActive[ESP_DEVICE_COUNT];
static uint32_t lastHeartbeatTick;
static uint32_t lastOledRefreshTick;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
static void App_Init(void);
static void App_Task(void);
static void Heartbeat_Task(void);
static void EspStatus_Task(void);
static void Oled_Task(void);
static void Relay_Task(void);
static void Relay_Set(RelayChannel_t *relay, GPIO_PinState state);
static void Relay_Press(EspDevice_t device, EspKey_t key, uint32_t durationMs);
static void All_Relays_Off(void);
static void Esp_SetPower(EspDevice_t device, bool enabled);
static bool Esp_IsCheckActive(EspDevice_t device);
static void Oled_Init(void);
static void Oled_RenderStatus(void);
static void Oled_ClearBuffer(void);
static void Oled_DrawText(uint8_t page, uint8_t column, const char *text);
static void Oled_DrawChar(uint8_t page, uint8_t column, char c);
static const uint8_t *Oled_GetFont(char c);
static void Oled_Update(void);
static void Oled_WriteCommand(uint8_t command);
static void Oled_WriteData(const uint8_t *data, size_t length);
static void SoftI2c_Start(void);
static void SoftI2c_Stop(void);
static bool SoftI2c_WriteByte(uint8_t data);
static void SoftI2c_SetSda(GPIO_PinState state);
static void SoftI2c_SetScl(GPIO_PinState state);
static void SoftI2c_Delay(void);
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
                          |LED_R_Pin, GPIO_PIN_RESET);

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
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
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
  EspStatus_Task();
  lastHeartbeatTick = HAL_GetTick();
  lastOledRefreshTick = HAL_GetTick();
  Oled_Init();
  Oled_RenderStatus();
  Oled_Update();
  Can_Start();
}

static void App_Task(void)
{
  Heartbeat_Task();
  EspStatus_Task();
  Relay_Task();
  Can_ProcessRx();
  Oled_Task();
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
  espPowerEnabled[device] = enabled;
}

static bool Esp_IsCheckActive(EspDevice_t device)
{
  if (device >= ESP_DEVICE_COUNT)
  {
    return false;
  }

  return (HAL_GPIO_ReadPin(espCheckInputs[device].port,
                           espCheckInputs[device].pin) == GPIO_PIN_SET);
}

static void EspStatus_Task(void)
{
  for (uint8_t device = 0U; device < ESP_DEVICE_COUNT; device++)
  {
    espCheckActive[device] = Esp_IsCheckActive((EspDevice_t)device);
  }
}

static void Oled_Task(void)
{
  const uint32_t now = HAL_GetTick();

  if ((now - lastOledRefreshTick) >= OLED_REFRESH_INTERVAL_MS)
  {
    lastOledRefreshTick = now;
    Oled_RenderStatus();
    Oled_Update();
  }
}

static void Oled_Init(void)
{
  static const uint8_t initCommands[] = {
    0xAE, 0x20, 0x00, 0xB0, 0xC8, 0x00, 0x10, 0x40,
    0x81, 0x7F, 0xA1, 0xA6, 0xA8, 0x3F, 0xA4, 0xD3,
    0x00, 0xD5, 0x80, 0xD9, 0xF1, 0xDA, 0x12, 0xDB,
    0x40, 0x8D, 0x14, 0xAF,
  };

  SoftI2c_SetSda(GPIO_PIN_SET);
  SoftI2c_SetScl(GPIO_PIN_SET);
  HAL_Delay(20U);

  for (size_t i = 0U; i < sizeof(initCommands); i++)
  {
    Oled_WriteCommand(initCommands[i]);
  }
}

static void Oled_RenderStatus(void)
{
  Oled_ClearBuffer();
  Oled_DrawText(0U, 0U, "ESP32 CONTROL");
  Oled_DrawText(2U, 0U, espPowerEnabled[ESP_DEVICE_1] ? "E1 PWR:ON" : "E1 PWR:OFF");
  Oled_DrawText(3U, 0U, espCheckActive[ESP_DEVICE_1] ? "E1 CHECK:HI" : "E1 CHECK:LO");
  Oled_DrawText(5U, 0U, espPowerEnabled[ESP_DEVICE_2] ? "E2 PWR:ON" : "E2 PWR:OFF");
  Oled_DrawText(6U, 0U, espCheckActive[ESP_DEVICE_2] ? "E2 CHECK:HI" : "E2 CHECK:LO");
}

static void Oled_ClearBuffer(void)
{
  for (size_t i = 0U; i < sizeof(oledBuffer); i++)
  {
    oledBuffer[i] = 0x00U;
  }
}

static void Oled_DrawText(uint8_t page, uint8_t column, const char *text)
{
  while ((*text != '\0') && (column < (OLED_WIDTH - 5U)))
  {
    Oled_DrawChar(page, column, *text);
    column = (uint8_t)(column + 6U);
    text++;
  }
}

static void Oled_DrawChar(uint8_t page, uint8_t column, char c)
{
  const uint8_t *font = Oled_GetFont(c);
  const size_t offset = ((size_t)page * OLED_WIDTH) + column;

  if ((page >= OLED_PAGES) || ((column + 5U) > OLED_WIDTH))
  {
    return;
  }

  for (uint8_t i = 0U; i < 5U; i++)
  {
    oledBuffer[offset + i] = font[i];
  }
}

static const uint8_t *Oled_GetFont(char c)
{
  static const uint8_t space[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
  static const uint8_t colon[5] = {0x00, 0x36, 0x36, 0x00, 0x00};
  static const uint8_t zero[5] = {0x3E, 0x51, 0x49, 0x45, 0x3E};
  static const uint8_t one[5] = {0x00, 0x42, 0x7F, 0x40, 0x00};
  static const uint8_t two[5] = {0x42, 0x61, 0x51, 0x49, 0x46};
  static const uint8_t three[5] = {0x21, 0x41, 0x45, 0x4B, 0x31};
  static const uint8_t a[5] = {0x7E, 0x11, 0x11, 0x11, 0x7E};
  static const uint8_t c[5] = {0x3E, 0x41, 0x41, 0x41, 0x22};
  static const uint8_t e[5] = {0x7F, 0x49, 0x49, 0x49, 0x41};
  static const uint8_t f[5] = {0x7F, 0x09, 0x09, 0x09, 0x01};
  static const uint8_t h[5] = {0x7F, 0x08, 0x08, 0x08, 0x7F};
  static const uint8_t i[5] = {0x00, 0x41, 0x7F, 0x41, 0x00};
  static const uint8_t k[5] = {0x7F, 0x08, 0x14, 0x22, 0x41};
  static const uint8_t l[5] = {0x7F, 0x40, 0x40, 0x40, 0x40};
  static const uint8_t n[5] = {0x7F, 0x04, 0x08, 0x10, 0x7F};
  static const uint8_t o[5] = {0x3E, 0x41, 0x41, 0x41, 0x3E};
  static const uint8_t p[5] = {0x7F, 0x09, 0x09, 0x09, 0x06};
  static const uint8_t r[5] = {0x7F, 0x09, 0x19, 0x29, 0x46};
  static const uint8_t s[5] = {0x46, 0x49, 0x49, 0x49, 0x31};
  static const uint8_t t[5] = {0x01, 0x01, 0x7F, 0x01, 0x01};
  static const uint8_t w[5] = {0x7F, 0x20, 0x18, 0x20, 0x7F};

  switch (c)
  {
    case '0': return zero;
    case '1': return one;
    case '2': return two;
    case '3': return three;
    case ':': return colon;
    case 'A': return a;
    case 'C': return c;
    case 'E': return e;
    case 'F': return f;
    case 'H': return h;
    case 'I': return i;
    case 'K': return k;
    case 'L': return l;
    case 'N': return n;
    case 'O': return o;
    case 'P': return p;
    case 'R': return r;
    case 'S': return s;
    case 'T': return t;
    case 'W': return w;
    default: return space;
  }
}

static void Oled_Update(void)
{
  for (uint8_t page = 0U; page < OLED_PAGES; page++)
  {
    Oled_WriteCommand((uint8_t)(0xB0U + page));
    Oled_WriteCommand(0x00U);
    Oled_WriteCommand(0x10U);
    Oled_WriteData(&oledBuffer[page * OLED_WIDTH], OLED_WIDTH);
  }
}

static void Oled_WriteCommand(uint8_t command)
{
  SoftI2c_Start();
  (void)SoftI2c_WriteByte((uint8_t)(OLED_I2C_ADDR << 1));
  (void)SoftI2c_WriteByte(0x00U);
  (void)SoftI2c_WriteByte(command);
  SoftI2c_Stop();
}

static void Oled_WriteData(const uint8_t *data, size_t length)
{
  SoftI2c_Start();
  (void)SoftI2c_WriteByte((uint8_t)(OLED_I2C_ADDR << 1));
  (void)SoftI2c_WriteByte(0x40U);
  for (size_t i = 0U; i < length; i++)
  {
    (void)SoftI2c_WriteByte(data[i]);
  }
  SoftI2c_Stop();
}

static void SoftI2c_Start(void)
{
  SoftI2c_SetSda(GPIO_PIN_SET);
  SoftI2c_SetScl(GPIO_PIN_SET);
  SoftI2c_SetSda(GPIO_PIN_RESET);
  SoftI2c_SetScl(GPIO_PIN_RESET);
}

static void SoftI2c_Stop(void)
{
  SoftI2c_SetSda(GPIO_PIN_RESET);
  SoftI2c_SetScl(GPIO_PIN_SET);
  SoftI2c_SetSda(GPIO_PIN_SET);
}

static bool SoftI2c_WriteByte(uint8_t data)
{
  bool ack;

  for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1U)
  {
    SoftI2c_SetSda((data & mask) != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
    SoftI2c_SetScl(GPIO_PIN_SET);
    SoftI2c_SetScl(GPIO_PIN_RESET);
  }

  SoftI2c_SetSda(GPIO_PIN_SET);
  SoftI2c_SetScl(GPIO_PIN_SET);
  ack = (HAL_GPIO_ReadPin(OLED_SDA_GPIO_Port, OLED_SDA_Pin) == GPIO_PIN_RESET);
  SoftI2c_SetScl(GPIO_PIN_RESET);
  return ack;
}

static void SoftI2c_SetSda(GPIO_PinState state)
{
  HAL_GPIO_WritePin(OLED_SDA_GPIO_Port, OLED_SDA_Pin, state);
  SoftI2c_Delay();
}

static void SoftI2c_SetScl(GPIO_PinState state)
{
  HAL_GPIO_WritePin(OLED_SCL_GPIO_Port, OLED_SCL_Pin, state);
  SoftI2c_Delay();
}

static void SoftI2c_Delay(void)
{
  for (volatile uint8_t i = 0U; i < 20U; i++)
  {
  }
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
