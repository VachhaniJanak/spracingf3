/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

#define BMP180_ADDRESS (0x77 << 1) // BMP180 I2C address
#define TEMP_PRESSURE_REG 0xF6     // Register for raw data
#define CONTROL_REG 0xF4           // Control register
#define READ_TEMP_CMD 0x2E         // Command to read temperature
#define READ_PRESS_CMD 0x34        // Command to read pressure

#define MPU6050_ADDR 0x68

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c2_tx;
DMA_HandleTypeDef hdma_i2c2_rx;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

// BMP180 calibration data
int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
uint16_t AC4, AC5, AC6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void readBMP180CalibrationData(void);
int32_t readRawTemperature(void);
int32_t readRawPressure(void);
float calculateTemperature(int32_t UT);
float calculatePressure(int32_t UP);
int32_t BMP180_GetAlt(int32_t UP);

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  // Read and print calibration data for debugging
  // Read calibration data
  // readBMP180CalibrationData();

  // while (1)
  // {
  // int32_t UT = readRawTemperature();
  // int32_t UP = readRawPressure();

  // Calculate temperature and pressure
  // uint32_t temperature = calculateTemperature(UT);
  // uint32_t pressure = calculatePressure(UP);
  // uint32_t altitue = BMP180_GetAlt(UP);

  // Format and transmit the final message
  // char msg[128];
  // snprintf(msg, sizeof(msg), "Temperature: %ld Pressure: %ld ALT: %ld\r\n", temperature, pressure, altitue);
  // HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 100);

  // HAL_Delay(1000); // Delay for readability in output
  // }
  MPU6050_Init();
  while (1)
  {
    // read the Accelerometer and Gyro values
    MPU6050_Read_Accel();
    MPU6050_Read_Gyro();
    HAL_Delay(500); // Add delay to control data rate
  }
}
void MPU6050_Init(void)
{
  uint8_t check;
  uint8_t Data;

  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 1000); // read WHO_AM_I
  if (check == 0x68)                                                // 0x68 will be returned by the sensor if everything goes well
  {
    // power management register 0X6B we should write all 0's to wake the sensor up
    Data = 0;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &Data, 1, 1000);
    // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
    Data = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x19, 1, &Data, 1, 1000);
    // Set accelerometer configuration in ACCEL_CONFIG Register
    Data = 0x00; // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x43, 1, &Data, 1, 1000);

    // Set Gyroscopic configuration in GYRO_CONFIG Register
    Data = 0x00; // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 ̐/s
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x43, 1, &Data, 1, 1000);
  }
}
void MPU6050_Read_Accel(void)
{
  uint8_t Rec_Data[6];

  // Read 6 BYTES of data starting from ACCEL_XOUT_H (0x3B) register
  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, 1, Rec_Data, 6, 1000);
  int16_t Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
  int16_t Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
  int16_t Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
  int Ax = (float)Accel_X_RAW / 16384.0;
  int Ay = (float)Accel_Y_RAW / 16384.0;
  int Az = (float)Accel_Z_RAW / 16384.0;
  // Use UART or another method to send data for debugging or further processing
  char msg[128];
  snprintf(msg, sizeof(msg), "Ax: %d, Ay: %d, Az: %d\r\n",
           Ax, Ay, Az);
  HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

void MPU6050_Read_Gyro(void)
{
  uint8_t Rec_Data[6];

  // Read 6 BYTES of data starting from GYRO_XOUT_H register
  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43, 1, Rec_Data, 6, 1000);

  int16_t Gyro_X_RAW = (Rec_Data[0] << 8 | Rec_Data[1]);
  int16_t Gyro_Y_RAW = (Rec_Data[2] << 8 | Rec_Data[3]);
  int16_t Gyro_Z_RAW = (Rec_Data[4] << 8 | Rec_Data[5]);
  // Use UART or another method to send data for debugging or further processing
  // char msg[128];
  // snprintf(msg, sizeof(msg), "Gx: %d, Gy: %d, Gz: %d\r\n",Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW);
  // HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
  // int Gx = (float)Gyro_X_RAW/131.0;
  // int Gy = (float)Gyro_Y_RAW/131.0;
  // int Gz = (float)Gyro_Z_RAW/131.0;
}

// Read BMP180 calibration data
void readBMP180CalibrationData(void)
{
  uint8_t calib_data[22];
  HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDRESS, 0xAA, I2C_MEMADD_SIZE_8BIT, calib_data, 22, HAL_MAX_DELAY);

  AC1 = (calib_data[0] << 8) | calib_data[1];
  AC2 = (calib_data[2] << 8) | calib_data[3];
  AC3 = (calib_data[4] << 8) | calib_data[5];
  AC4 = (calib_data[6] << 8) | calib_data[7];
  AC5 = (calib_data[8] << 8) | calib_data[9];
  AC6 = (calib_data[10] << 8) | calib_data[11];
  B1 = (calib_data[12] << 8) | calib_data[13];
  B2 = (calib_data[14] << 8) | calib_data[15];
  MB = (calib_data[16] << 8) | calib_data[17];
  MC = (calib_data[18] << 8) | calib_data[19];
  MD = (calib_data[20] << 8) | calib_data[21];
}

// Read raw temperature
int32_t readRawTemperature(void)
{
  uint8_t temp_cmd = READ_TEMP_CMD;
  uint8_t data[2];
  HAL_I2C_Mem_Write(&hi2c1, BMP180_ADDRESS, CONTROL_REG, I2C_MEMADD_SIZE_8BIT, &temp_cmd, 1, HAL_MAX_DELAY);
  HAL_Delay(5); // Wait for conversion
  HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDRESS, TEMP_PRESSURE_REG, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
  return (data[0] << 8) | data[1];
}

// Read raw pressure
int32_t readRawPressure(void)
{
  uint8_t press_cmd = READ_PRESS_CMD + (3 << 6);
  HAL_I2C_Mem_Write(&hi2c1, BMP180_ADDRESS, CONTROL_REG, I2C_MEMADD_SIZE_8BIT, &press_cmd, 1, HAL_MAX_DELAY);
  HAL_Delay(26); // Wait for conversion
  uint8_t data[3];
  HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDRESS, TEMP_PRESSURE_REG, I2C_MEMADD_SIZE_8BIT, data, 3, HAL_MAX_DELAY);
  return ((data[0] << 16) | (data[1] << 8) | data[2]) >> (8 - 3);
}

float calculateTemperature(int32_t UT)
{
  int32_t X1 = ((UT - AC6) * AC5) >> 15;
  int32_t X2 = (MC << 11) / (X1 + MD);
  int32_t B5 = X1 + X2;
  return ((B5 + 8) >> 4) / 10.0;
}

// Calculate true pressure in Pascals
float calculatePressure(int32_t UP)
{
  int32_t X1, X2, X3, B3, B5, B6, p;
  uint32_t B4, B7;

  // Assuming previous calculation of B5 in calculateTemperature()
  int32_t UT = readRawTemperature();
  X1 = ((UT - AC6) * AC5) >> 15;
  X2 = (MC << 11) / (X1 + MD);
  B5 = X1 + X2;

  B6 = B5 - 4000;
  X1 = (B2 * (B6 * B6 >> 12)) >> 11;
  X2 = AC2 * B6 >> 11;
  X3 = X1 + X2;
  B3 = (((AC1 * 4 + X3) << 3) + 2) >> 2;
  X1 = AC3 * B6 >> 13;
  X2 = (B1 * (B6 * B6 >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = AC4 * (uint32_t)(X3 + 32768) >> 15;
  B7 = ((uint32_t)UP - B3) * (50000 >> 3);

  if (B7 < 0x80000000)
    p = (B7 * 2) / B4;
  else
    p = (B7 / B4) * 2;

  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;

  return p + ((X1 + X2 + 3791) >> 4);
}

#define atmPress 101325 // Pa

int32_t BMP180_GetAlt(int32_t UP)
{
  return 44330 * (1 - pow(calculatePressure(UP) / atmPress, 0.19029495718));
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_USART3 | RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_I2C2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
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
  hi2c2.Init.Timing = 0x00201D2B;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  huart2.Init.BaudRate = 38400;
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
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 38400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER____GPIO_OUTPUT_GPIO_Port, BUZZER____GPIO_OUTPUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RSSI____GPIO_OUTPUT_Pin | GPIO_OUTPUT____LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GYRO____GPIO_EXTI13_Pin MAG____GPIO_EXTI14_Pin */
  GPIO_InitStruct.Pin = GYRO____GPIO_EXTI13_Pin | MAG____GPIO_EXTI14_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER____GPIO_OUTPUT_Pin */
  GPIO_InitStruct.Pin = BUZZER____GPIO_OUTPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER____GPIO_OUTPUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RC_CH1____TIM2_CH1_Pin RC_CH2____TIM2_CH2_Pin */
  GPIO_InitStruct.Pin = RC_CH1____TIM2_CH1_Pin | RC_CH2____TIM2_CH2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_7____TIM15_CH1_Pin MOTOR_8____TIM15_CH2_Pin */
  GPIO_InitStruct.Pin = MOTOR_7____TIM15_CH1_Pin | MOTOR_8____TIM15_CH2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_TIM15;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : VBAT____GPIO_ANALOG_Pin CURRENT____GPIO_ANALOG_Pin */
  GPIO_InitStruct.Pin = VBAT____GPIO_ANALOG_Pin | CURRENT____GPIO_ANALOG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR_1____TIM16_CH1_Pin */
  GPIO_InitStruct.Pin = MOTOR_1____TIM16_CH1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM16;
  HAL_GPIO_Init(MOTOR_1____TIM16_CH1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR_2____TIM17_CH1_Pin */
  GPIO_InitStruct.Pin = MOTOR_2____TIM17_CH1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM17;
  HAL_GPIO_Init(MOTOR_2____TIM17_CH1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RC_CH7___SONAR_TRI____TIM3_CH3_Pin RC_CH8___SONAR_ECHO____TIM3_CH4_Pin RC_CH5____TIM3_CH1_Pin RC_CH6____TIM3_CH2_Pin */
  GPIO_InitStruct.Pin = RC_CH7___SONAR_TRI____TIM3_CH3_Pin | RC_CH8___SONAR_ECHO____TIM3_CH4_Pin | RC_CH5____TIM3_CH1_Pin | RC_CH6____TIM3_CH2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RSSI____GPIO_OUTPUT_Pin GPIO_OUTPUT____LED_Pin */
  GPIO_InitStruct.Pin = RSSI____GPIO_OUTPUT_Pin | GPIO_OUTPUT____LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_STRIP____TIM1_CH1_Pin */
  GPIO_InitStruct.Pin = LED_STRIP____TIM1_CH1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
  HAL_GPIO_Init(LED_STRIP____TIM1_CH1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_3____TIM4_CH1_Pin MOTOR_4____TIM4_CH2_Pin */
  GPIO_InitStruct.Pin = MOTOR_3____TIM4_CH1_Pin | MOTOR_4____TIM4_CH2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_TIM4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_5____TIM4_CH3_Pin MOTOR_6____TIM4_CH4_Pin */
  GPIO_InitStruct.Pin = MOTOR_5____TIM4_CH3_Pin | MOTOR_6____TIM4_CH4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
