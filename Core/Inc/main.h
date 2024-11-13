/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GYRO____GPIO_EXTI13_Pin GPIO_PIN_13
#define GYRO____GPIO_EXTI13_GPIO_Port GPIOC
#define MAG____GPIO_EXTI14_Pin GPIO_PIN_14
#define MAG____GPIO_EXTI14_GPIO_Port GPIOC
#define BUZZER____GPIO_OUTPUT_Pin GPIO_PIN_15
#define BUZZER____GPIO_OUTPUT_GPIO_Port GPIOC
#define GYRO____I2C2_SDA_Pin GPIO_PIN_0
#define GYRO____I2C2_SDA_GPIO_Port GPIOF
#define GYRO____I2C2_SCL_Pin GPIO_PIN_1
#define GYRO____I2C2_SCL_GPIO_Port GPIOF
#define RC_CH1____TIM2_CH1_Pin GPIO_PIN_0
#define RC_CH1____TIM2_CH1_GPIO_Port GPIOA
#define RC_CH2____TIM2_CH2_Pin GPIO_PIN_1
#define RC_CH2____TIM2_CH2_GPIO_Port GPIOA
#define MOTOR_7____TIM15_CH1_Pin GPIO_PIN_2
#define MOTOR_7____TIM15_CH1_GPIO_Port GPIOA
#define MOTOR_8____TIM15_CH2_Pin GPIO_PIN_3
#define MOTOR_8____TIM15_CH2_GPIO_Port GPIOA
#define VBAT____GPIO_ANALOG_Pin GPIO_PIN_4
#define VBAT____GPIO_ANALOG_GPIO_Port GPIOA
#define CURRENT____GPIO_ANALOG_Pin GPIO_PIN_5
#define CURRENT____GPIO_ANALOG_GPIO_Port GPIOA
#define MOTOR_1____TIM16_CH1_Pin GPIO_PIN_6
#define MOTOR_1____TIM16_CH1_GPIO_Port GPIOA
#define MOTOR_2____TIM17_CH1_Pin GPIO_PIN_7
#define MOTOR_2____TIM17_CH1_GPIO_Port GPIOA
#define RC_CH7___SONAR_TRI____TIM3_CH3_Pin GPIO_PIN_0
#define RC_CH7___SONAR_TRI____TIM3_CH3_GPIO_Port GPIOB
#define RC_CH8___SONAR_ECHO____TIM3_CH4_Pin GPIO_PIN_1
#define RC_CH8___SONAR_ECHO____TIM3_CH4_GPIO_Port GPIOB
#define RSSI____GPIO_OUTPUT_Pin GPIO_PIN_2
#define RSSI____GPIO_OUTPUT_GPIO_Port GPIOB
#define USART3_TX_PINS_Pin GPIO_PIN_10
#define USART3_TX_PINS_GPIO_Port GPIOB
#define USART3_RX_PINS_Pin GPIO_PIN_11
#define USART3_RX_PINS_GPIO_Port GPIOB
#define LED_STRIP____TIM1_CH1_Pin GPIO_PIN_8
#define LED_STRIP____TIM1_CH1_GPIO_Port GPIOA
#define USART1_TX_PINS_Pin GPIO_PIN_9
#define USART1_TX_PINS_GPIO_Port GPIOA
#define USART1_RX_PINS_Pin GPIO_PIN_10
#define USART1_RX_PINS_GPIO_Port GPIOA
#define MOTOR_3____TIM4_CH1_Pin GPIO_PIN_11
#define MOTOR_3____TIM4_CH1_GPIO_Port GPIOA
#define MOTOR_4____TIM4_CH2_Pin GPIO_PIN_12
#define MOTOR_4____TIM4_CH2_GPIO_Port GPIOA
#define USART2_TX_PINS_Pin GPIO_PIN_14
#define USART2_TX_PINS_GPIO_Port GPIOA
#define USART2_RX_PINS_Pin GPIO_PIN_15
#define USART2_RX_PINS_GPIO_Port GPIOA
#define GPIO_OUTPUT____LED_Pin GPIO_PIN_3
#define GPIO_OUTPUT____LED_GPIO_Port GPIOB
#define RC_CH5____TIM3_CH1_Pin GPIO_PIN_4
#define RC_CH5____TIM3_CH1_GPIO_Port GPIOB
#define RC_CH6____TIM3_CH2_Pin GPIO_PIN_5
#define RC_CH6____TIM3_CH2_GPIO_Port GPIOB
#define BARO____I2C2_SCL_Pin GPIO_PIN_6
#define BARO____I2C2_SCL_GPIO_Port GPIOB
#define BARO____I2C1_SDA_Pin GPIO_PIN_7
#define BARO____I2C1_SDA_GPIO_Port GPIOB
#define MOTOR_5____TIM4_CH3_Pin GPIO_PIN_8
#define MOTOR_5____TIM4_CH3_GPIO_Port GPIOB
#define MOTOR_6____TIM4_CH4_Pin GPIO_PIN_9
#define MOTOR_6____TIM4_CH4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
