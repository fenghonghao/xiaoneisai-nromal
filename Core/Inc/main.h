/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
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
#define M3_Pin GPIO_PIN_5
#define M3_GPIO_Port GPIOE
#define M3E6_Pin GPIO_PIN_6
#define M3E6_GPIO_Port GPIOE
#define ENM1_Pin GPIO_PIN_0
#define ENM1_GPIO_Port GPIOA
#define ENM1A1_Pin GPIO_PIN_1
#define ENM1A1_GPIO_Port GPIOA
#define M2_Pin GPIO_PIN_9
#define M2_GPIO_Port GPIOE
#define LED_Pin GPIO_PIN_10
#define LED_GPIO_Port GPIOE
#define M2E11_Pin GPIO_PIN_11
#define M2E11_GPIO_Port GPIOE
#define M1_Pin GPIO_PIN_13
#define M1_GPIO_Port GPIOE
#define M1E14_Pin GPIO_PIN_14
#define M1E14_GPIO_Port GPIOE
#define IMU_SCL_Pin GPIO_PIN_10
#define IMU_SCL_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_11
#define IMU_SDA_GPIO_Port GPIOB
#define IMU_ITR_Pin GPIO_PIN_12
#define IMU_ITR_GPIO_Port GPIOB
#define IMU_ITR_EXTI_IRQn EXTI15_10_IRQn
#define RasPI_TX_Pin GPIO_PIN_8
#define RasPI_TX_GPIO_Port GPIOD
#define RasPI_RX_Pin GPIO_PIN_9
#define RasPI_RX_GPIO_Port GPIOD
#define SERVO3_Pin GPIO_PIN_8
#define SERVO3_GPIO_Port GPIOC
#define SERVO4_Pin GPIO_PIN_9
#define SERVO4_GPIO_Port GPIOC
#define DBG_TX_Pin GPIO_PIN_9
#define DBG_TX_GPIO_Port GPIOA
#define DBG_RX_Pin GPIO_PIN_10
#define DBG_RX_GPIO_Port GPIOA
#define SERVO1_Pin GPIO_PIN_11
#define SERVO1_GPIO_Port GPIOA
#define SERVO2_Pin GPIO_PIN_12
#define SERVO2_GPIO_Port GPIOA
#define ENM2_Pin GPIO_PIN_15
#define ENM2_GPIO_Port GPIOA
#define BLE_Pin GPIO_PIN_5
#define BLE_GPIO_Port GPIOD
#define BLED6_Pin GPIO_PIN_6
#define BLED6_GPIO_Port GPIOD
#define ENM2B3_Pin GPIO_PIN_3
#define ENM2B3_GPIO_Port GPIOB
#define ENM4_Pin GPIO_PIN_4
#define ENM4_GPIO_Port GPIOB
#define ENM4B5_Pin GPIO_PIN_5
#define ENM4B5_GPIO_Port GPIOB
#define ENM3_Pin GPIO_PIN_6
#define ENM3_GPIO_Port GPIOB
#define ENM3B7_Pin GPIO_PIN_7
#define ENM3B7_GPIO_Port GPIOB
#define M4_Pin GPIO_PIN_8
#define M4_GPIO_Port GPIOB
#define M4B9_Pin GPIO_PIN_9
#define M4B9_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
