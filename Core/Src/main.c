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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// NFC Module (PN532) I2C Address
#define PN532_I2C_ADDRESS         0x48  // 7-bit address (0x24 << 1)

// LCD I2C Address (PCF8574 based)
#define LCD_I2C_ADDRESS           0x4E  // Adjust based on your LCD module (0x27 << 1)

// PN532 Commands
#define PN532_COMMAND_GETFIRMWAREVERSION    0x02
#define PN532_COMMAND_SAMCONFIGURATION      0x14
#define PN532_COMMAND_RFCONFIGURATION       0x32
#define PN532_COMMAND_INLISTPASSIVETARGET   0x4A
#define PN532_COMMAND_INDATAEXCHANGE        0x40
#define PN532_COMMAND_INDESELECT            0x44

// MIFARE Commands
#define MIFARE_READ                 0x30
#define MIFARE_WRITE                0xA0
#define MIFARE_ULTRALIGHT_WRITE     0xA2

// LCD Commands
#define LCD_CLEAR_DISPLAY           0x01
#define LCD_RETURN_HOME             0x02
#define LCD_ENTRY_MODE_SET          0x04
#define LCD_DISPLAY_CONTROL         0x08
#define LCD_CURSOR_SHIFT            0x10
#define LCD_FUNCTION_SET            0x20
#define LCD_SET_CGRAM_ADDR          0x40
#define LCD_SET_DDRAM_ADDR          0x80

// LCD Control bits
#define LCD_ENABLE_BIT              0x04
#define LCD_BACKLIGHT               0x08
#define LCD_RS_BIT                  0x01

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
uint8_t nfc_uid[10];
uint8_t nfc_uid_length = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
// NFC Functions
HAL_StatusTypeDef PN532_Init(void);
HAL_StatusTypeDef PN532_GetFirmwareVersion(uint32_t *version);
HAL_StatusTypeDef PN532_SAMConfig(void);
HAL_StatusTypeDef PN532_ReadPassiveTarget(uint8_t *uid, uint8_t *uid_len);
HAL_StatusTypeDef PN532_ReadBlock(uint8_t block, uint8_t *data);
HAL_StatusTypeDef PN532_WriteBlock(uint8_t block, uint8_t *data);

// LCD Functions
void LCD_Init(void);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_Print(char *str);
void LCD_SendCommand(uint8_t cmd);
void LCD_SendData(uint8_t data);
void LCD_SendNibble(uint8_t nibble, uint8_t rs);

// Utility Functions
void PN532_SendCommand(uint8_t *cmd, uint8_t cmd_len);
HAL_StatusTypeDef PN532_ReadResponse(uint8_t *response, uint8_t *response_len);
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

	  char lcd_buffer[32];
	  uint8_t card_data[16];
	  uint8_t write_data[16] = "Hello NFC World!";
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
    // Initialize LCD
    HAL_Delay(100);
    LCD_Init();
    LCD_Clear();
    LCD_Print("NFC Reader Ready");

    // Initialize NFC module
    if (PN532_Init() == HAL_OK) {
      LCD_SetCursor(1, 0);
      LCD_Print("PN532 OK!");
      HAL_Delay(1000);
    } else {
      LCD_SetCursor(1, 0);
      LCD_Print("PN532 Error!");
      while(1);
    }

    LCD_Clear();
    LCD_Print("Place NFC card...");
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* Turn on the LED (active low, so set PC13 to RESSET) */
	      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);




	      // Try to read NFC card
	      if (PN532_ReadPassiveTarget(nfc_uid, &nfc_uid_length) == HAL_OK) {
	        // Card detected
	        LCD_Clear();
	        LCD_Print("Card Found!");

	        // Display UID
	        LCD_SetCursor(1, 0);
	        sprintf(lcd_buffer, "UID: %02X%02X%02X%02X",
	                nfc_uid[0], nfc_uid[1], nfc_uid[2], nfc_uid[3]);
	        LCD_Print(lcd_buffer);

	        HAL_Delay(2000);

	        // Try to read block 4 (first user data block in MIFARE Classic)
	        if (PN532_ReadBlock(4, card_data) == HAL_OK) {
	          LCD_Clear();
	          LCD_Print("Read Success:");
	          LCD_SetCursor(1, 0);
	          // Display first 16 characters
	          card_data[15] = '\0';  // Ensure null termination
	          LCD_Print((char*)card_data);
	          HAL_Delay(3000);

	          // Write new data to the card
	          if (PN532_WriteBlock(4, write_data) == HAL_OK) {
	            LCD_Clear();
	            LCD_Print("Write Success!");
	            HAL_Delay(2000);
	          } else {
	            LCD_Clear();
	            LCD_Print("Write Failed!");
	            HAL_Delay(2000);
	          }
	        } else {
	          LCD_Clear();
	          LCD_Print("Read Failed!");
	          HAL_Delay(2000);
	        }

	        LCD_Clear();
	        LCD_Print("Place NFC card...");
	      }

	      HAL_Delay(500);  // Check for cards every 500ms
	    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hi2c2.Init.ClockSpeed = 100000;
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
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  hspi2.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// ===== NFC Functions =====

HAL_StatusTypeDef PN532_Init(void) {
  uint32_t version;

  // Get firmware version to test communication
  if (PN532_GetFirmwareVersion(&version) != HAL_OK) {
    return HAL_ERROR;
  }

  // Configure SAM (Security Access Module)
  if (PN532_SAMConfig() != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef PN532_GetFirmwareVersion(uint32_t *version) {
  uint8_t cmd[] = {0x00, 0x00, 0xFF, 0x02, 0xFE, 0xD4, 0x02, 0x2A, 0x00};
  uint8_t response[32];
  uint8_t response_len = 0;

  if (HAL_I2C_Master_Transmit(&hi2c1, PN532_I2C_ADDRESS, cmd, sizeof(cmd), 1000) != HAL_OK) {
    return HAL_ERROR;
  }

  HAL_Delay(50);

  if (PN532_ReadResponse(response, &response_len) != HAL_OK) {
    return HAL_ERROR;
  }

  if (response_len >= 12 && response[7] == 0xD5 && response[8] == 0x03) {
    *version = (response[9] << 24) | (response[10] << 16) | (response[11] << 8) | response[12];
    return HAL_OK;
  }

  return HAL_ERROR;
}

HAL_StatusTypeDef PN532_SAMConfig(void) {
  uint8_t cmd[] = {0x00, 0x00, 0xFF, 0x05, 0xFB, 0xD4, 0x14, 0x01, 0x14, 0x01, 0x02, 0x00};
  uint8_t response[32];
  uint8_t response_len = 0;

  if (HAL_I2C_Master_Transmit(&hi2c1, PN532_I2C_ADDRESS, cmd, sizeof(cmd), 1000) != HAL_OK) {
    return HAL_ERROR;
  }

  HAL_Delay(50);

  if (PN532_ReadResponse(response, &response_len) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef PN532_ReadPassiveTarget(uint8_t *uid, uint8_t *uid_len) {
  uint8_t cmd[] = {0x00, 0x00, 0xFF, 0x04, 0xFC, 0xD4, 0x4A, 0x01, 0x00, 0xE1, 0x00};
  uint8_t response[64];
  uint8_t response_len = 0;

  if (HAL_I2C_Master_Transmit(&hi2c1, PN532_I2C_ADDRESS, cmd, sizeof(cmd), 1000) != HAL_OK) {
    return HAL_ERROR;
  }

  HAL_Delay(100);

  if (PN532_ReadResponse(response, &response_len) != HAL_OK) {
    return HAL_ERROR;
  }

  if (response_len >= 19 && response[7] == 0xD5 && response[8] == 0x4B && response[9] == 0x01) {
    *uid_len = response[12];
    memcpy(uid, &response[13], *uid_len);
    return HAL_OK;
  }

  return HAL_ERROR;
}

HAL_StatusTypeDef PN532_ReadBlock(uint8_t block, uint8_t *data) {
  uint8_t cmd[] = {0x00, 0x00, 0xFF, 0x05, 0xFB, 0xD4, 0x40, 0x01, 0x30, block, 0x00};
  uint8_t response[64];
  uint8_t response_len = 0;

  // Calculate checksum
  uint8_t checksum = 0xD4 + 0x40 + 0x01 + 0x30 + block;
  cmd[9] = block;
  cmd[10] = (~checksum + 1) & 0xFF;

  if (HAL_I2C_Master_Transmit(&hi2c1, PN532_I2C_ADDRESS, cmd, sizeof(cmd), 1000) != HAL_OK) {
    return HAL_ERROR;
  }

  HAL_Delay(100);

  if (PN532_ReadResponse(response, &response_len) != HAL_OK) {
    return HAL_ERROR;
  }

  if (response_len >= 26 && response[7] == 0xD5 && response[8] == 0x41 && response[9] == 0x00) {
    memcpy(data, &response[10], 16);
    return HAL_OK;
  }

  return HAL_ERROR;
}

HAL_StatusTypeDef PN532_WriteBlock(uint8_t block, uint8_t *data) {
  uint8_t cmd[27] = {0x00, 0x00, 0xFF, 0x15, 0xEB, 0xD4, 0x40, 0x01, 0xA0, block};
  uint8_t response[32];
  uint8_t response_len = 0;

  // Copy data to command
  memcpy(&cmd[10], data, 16);

  // Calculate checksum
  uint8_t checksum = 0xD4 + 0x40 + 0x01 + 0xA0 + block;
  for (int i = 0; i < 16; i++) {
    checksum += data[i];
  }
  cmd[26] = (~checksum + 1) & 0xFF;

  if (HAL_I2C_Master_Transmit(&hi2c1, PN532_I2C_ADDRESS, cmd, sizeof(cmd), 1000) != HAL_OK) {
    return HAL_ERROR;
  }

  HAL_Delay(100);

  if (PN532_ReadResponse(response, &response_len) != HAL_OK) {
    return HAL_ERROR;
  }

  if (response_len >= 10 && response[7] == 0xD5 && response[8] == 0x41 && response[9] == 0x00) {
    return HAL_OK;
  }

  return HAL_ERROR;
}

HAL_StatusTypeDef PN532_ReadResponse(uint8_t *response, uint8_t *response_len) {
  uint8_t ready_byte;

  // Check if PN532 is ready
  if (HAL_I2C_Master_Receive(&hi2c1, PN532_I2C_ADDRESS, &ready_byte, 1, 100) != HAL_OK) {
    return HAL_ERROR;
  }

  if (ready_byte != 0x01) {
    return HAL_ERROR;
  }

  // Read response
  if (HAL_I2C_Master_Receive(&hi2c1, PN532_I2C_ADDRESS, response, 64, 1000) != HAL_OK) {
    return HAL_ERROR;
  }

  *response_len = 64;  // Simplified - should parse actual length
  return HAL_OK;
}

// ===== LCD Functions =====

void LCD_Init(void) {
  HAL_Delay(50);

  // Initialize in 4-bit mode
  LCD_SendNibble(0x03, 0);
  HAL_Delay(5);
  LCD_SendNibble(0x03, 0);
  HAL_Delay(1);
  LCD_SendNibble(0x03, 0);
  HAL_Delay(1);
  LCD_SendNibble(0x02, 0);  // Switch to 4-bit mode
  HAL_Delay(1);

  // Function set: 4-bit, 2 lines, 5x8 dots
  LCD_SendCommand(LCD_FUNCTION_SET | 0x08);
  // Display control: display on, cursor off, blink off
  LCD_SendCommand(LCD_DISPLAY_CONTROL | 0x04);
  // Clear display
  LCD_SendCommand(LCD_CLEAR_DISPLAY);
  HAL_Delay(2);
  // Entry mode: increment cursor, no shift
  LCD_SendCommand(LCD_ENTRY_MODE_SET | 0x02);
}

void LCD_Clear(void) {
  LCD_SendCommand(LCD_CLEAR_DISPLAY);
  HAL_Delay(2);
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
  uint8_t address = (row == 0) ? 0x00 : 0x40;
  address += col;
  LCD_SendCommand(LCD_SET_DDRAM_ADDR | address);
}

void LCD_Print(char *str) {
  while (*str) {
    LCD_SendData(*str++);
  }
}

void LCD_SendCommand(uint8_t cmd) {
  uint8_t upper_nibble = (cmd & 0xF0);
  uint8_t lower_nibble = ((cmd << 4) & 0xF0);

  LCD_SendNibble(upper_nibble, 0);
  LCD_SendNibble(lower_nibble, 0);
}

void LCD_SendData(uint8_t data) {
  uint8_t upper_nibble = (data & 0xF0);
  uint8_t lower_nibble = ((data << 4) & 0xF0);

  LCD_SendNibble(upper_nibble, LCD_RS_BIT);
  LCD_SendNibble(lower_nibble, LCD_RS_BIT);
}

void LCD_SendNibble(uint8_t nibble, uint8_t rs) {
  uint8_t data = nibble | rs | LCD_BACKLIGHT;

  // Send with enable high
  data |= LCD_ENABLE_BIT;
  HAL_I2C_Master_Transmit(&hi2c2, LCD_I2C_ADDRESS, &data, 1, 100);
  HAL_Delay(1);

  // Send with enable low
  data &= ~LCD_ENABLE_BIT;
  HAL_I2C_Master_Transmit(&hi2c2, LCD_I2C_ADDRESS, &data, 1, 100);
  HAL_Delay(1);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

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
