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
  *
  * Hardware Connections (STM32F103C8T6 Blue Pill):
  * ================================================
  * PN532 Module -> STM32 Blue Pill
  * VCC  -> 3.3V
  * GND  -> GND
  * SCK  -> PA5 (SPI1_SCK)
  * MISO -> PA6 (SPI1_MISO)
  * MOSI -> PA7 (SPI1_MOSI)
  * NSS  -> PA4 (GPIO Output, Software CS)
  *
  * Serial Debug:
  * TX -> PA9 (USART1_TX)
  * RX -> PA10 (USART1_RX)
  * Baudrate: 115200
  *
  * PN532 Module Setup:
  * Set I0I1 jumpers: I0=LOW, I1=HIGH (SPI Mode)
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "pn532.h"
#include "pn532_stm32f1.h"
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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
PN532 pn532;
char uart_buffer[200];
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

/* USER CODE BEGIN PFP */
/* Diagnostic Functions */
void Test_SPI_Connection(void);
void Test_PN532_Wakeup(void);
void Test_Raw_SPI_Communication(void);
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // Send startup message
  sprintf(uart_buffer, "\r\n=== STM32 Blue Pill + PN532 NFC Reader ===\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

  sprintf(uart_buffer, "Initializing PN532...\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

  // Step 1: Test basic SPI
  sprintf(uart_buffer, "Step 1: Testing SPI connection...\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
  Test_SPI_Connection();

  HAL_Delay(1000);

  // Step 2: Test raw SPI communication
  sprintf(uart_buffer, "Step 2: Testing raw SPI communication...\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
  Test_Raw_SPI_Communication();

  HAL_Delay(1000);

  // Step 3: Test PN532 wakeup
  sprintf(uart_buffer, "Step 3: Testing PN532 wakeup sequence...\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
  Test_PN532_Wakeup();

  HAL_Delay(1000);

  // Initialize PN532
  PN532_Init(&pn532);

  HAL_Delay(1000);

  // Get firmware version
  uint8_t version[4];
  if (PN532_GetFirmwareVersion(&pn532, version) == PN532_STATUS_OK) {
    sprintf(uart_buffer, "PN532 Firmware Version: %d.%d\r\n", version[1], version[2]);
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
    sprintf(uart_buffer, "IC: PN5%02X, Support: 0x%02X\r\n", version[0], version[3]);
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
  } else {
    sprintf(uart_buffer, "ERROR: Failed to get firmware version!\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
    sprintf(uart_buffer, "Check connections and PN532 module!\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
    // You might want to continue anyway for debugging
    // while(1); // Uncomment to stop on error
  }

  // Configure SAM
  if (PN532_SamConfiguration(&pn532) == PN532_STATUS_OK) {
    sprintf(uart_buffer, "SAM configuration successful!\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
  } else {
    sprintf(uart_buffer, "Warning: SAM configuration failed!\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
  }

  sprintf(uart_buffer, "\r\nReady! Place an NFC card near the reader...\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
  sprintf(uart_buffer, "==========================================\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Turn on the LED (active low, so set PC13 to RESET) */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint8_t uid[10]; // Buffer for UID (max 10 bytes for triple size UID)

    // Try to read a card
    int uid_length = PN532_ReadPassiveTarget(&pn532, uid, PN532_MIFARE_ISO14443A, 1000);

    if (uid_length > 0) {
      // Card found!
      sprintf(uart_buffer, "*** NFC CARD DETECTED! ***\r\n");
      HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

      sprintf(uart_buffer, "UID Length: %d bytes\r\n", uid_length);
      HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

      sprintf(uart_buffer, "UID (Hex): ");
      HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

      // Print UID in hexadecimal
      for (int i = 0; i < uid_length; i++) {
        sprintf(uart_buffer, "%02X", uid[i]);
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
        if (i < uid_length - 1) {
          sprintf(uart_buffer, " ");
          HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
        }
      }
      sprintf(uart_buffer, "\r\n");
      HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

      // Print UID in decimal
      sprintf(uart_buffer, "UID (Dec): ");
      HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

      for (int i = 0; i < uid_length; i++) {
        sprintf(uart_buffer, "%d", uid[i]);
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
        if (i < uid_length - 1) {
          sprintf(uart_buffer, " ");
          HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
        }
      }
      sprintf(uart_buffer, "\r\n");
      HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

      // Calculate and display UID as a single number (for shorter UIDs)
      if (uid_length <= 4) {
        uint32_t uid_number = 0;
        for (int i = 0; i < uid_length; i++) {
          uid_number = (uid_number << 8) | uid[i];
        }
        sprintf(uart_buffer, "UID (Number): %lu\r\n", uid_number);
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
      }

      sprintf(uart_buffer, "==========================================\r\n");
      HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

      // Wait before next read to avoid spamming
      HAL_Delay(2000);

      sprintf(uart_buffer, "Remove card and place another one...\r\n");
      HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

    } else {
      // No card found or error
      // Only print error message occasionally to avoid spam
      static uint32_t last_no_card_message = 0;
      if (HAL_GetTick() - last_no_card_message > 5000) { // Every 5 seconds
        sprintf(uart_buffer, "Scanning for NFC cards...\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
        last_no_card_message = HAL_GetTick();
      }
    }

    // Small delay between scans
    HAL_Delay(500);
  }
  /* USER CODE END 3 */
}

/* USER CODE BEGIN 4 */
/* Diagnostic Functions */

void Test_SPI_Connection(void) {
  // Test basic SPI functionality
  uint8_t test_data = 0xAA;
  uint8_t received_data = 0x00;

  sprintf(uart_buffer, "  - CS pin test (PA4)...\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  sprintf(uart_buffer, "  - SPI loopback test...\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, &test_data, &received_data, 1, 1000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  sprintf(uart_buffer, "  - Sent: 0x%02X, Received: 0x%02X\r\n", test_data, received_data);
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
}

void Test_Raw_SPI_Communication(void) {
  // Test raw PN532 SPI commands
  uint8_t status_cmd[2] = {0x02, 0x00}; // Status read command

  sprintf(uart_buffer, "  - Sending status read command...\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_SPI_TransmitReceive(&hspi1, status_cmd, status_cmd, 2, 1000);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  sprintf(uart_buffer, "  - Status response: 0x%02X 0x%02X\r\n", status_cmd[0], status_cmd[1]);
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

  if (status_cmd[1] == 0x01) {
    sprintf(uart_buffer, "  - PN532 appears to be ready!\r\n");
  } else {
    sprintf(uart_buffer, "  - PN532 not ready or not connected\r\n");
  }
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
}

void Test_PN532_Wakeup(void) {
  sprintf(uart_buffer, "  - Enhanced wakeup sequence...\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

  // Step 1: Long reset pulse
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_Delay(100);  // Hold CS low longer
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_Delay(100);  // Wait for stabilization

  // Step 2: Send proper wakeup command
  uint8_t wakeup_data = 0x55;  // PN532 wakeup command

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_Delay(10);  // Longer setup time
  HAL_SPI_Transmit(&hspi1, &wakeup_data, 1, 1000);
  HAL_Delay(10);  // Longer hold time
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  HAL_Delay(1000);  // Wait much longer for PN532 to wake up

  // Step 3: Multiple status checks
  for (int attempt = 1; attempt <= 5; attempt++) {
    uint8_t status_cmd[2] = {0x02, 0x00};

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_Delay(5);  // Longer setup
    HAL_SPI_TransmitReceive(&hspi1, status_cmd, status_cmd, 2, 1000);
    HAL_Delay(5);  // Longer hold
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    sprintf(uart_buffer, "  - Attempt %d - Status: 0x%02X 0x%02X\r\n", attempt, status_cmd[0], status_cmd[1]);
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

    if (status_cmd[1] == 0x01) {
      sprintf(uart_buffer, "  - PN532 is READY!\r\n");
      HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
      return;
    }

    HAL_Delay(200);  // Wait between attempts
  }

  sprintf(uart_buffer, "  - PN532 did not become ready after 5 attempts\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
}
/* USER CODE END 4 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
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
