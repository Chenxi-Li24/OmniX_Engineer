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
#include "stm32h7xx_hal.h"

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
#define MCP1_INT1_Pin GPIO_PIN_14
#define MCP1_INT1_GPIO_Port GPIOC
#define MCP1_INT1_EXTI_IRQn EXTI15_10_IRQn
#define MCP1_INT0_Pin GPIO_PIN_13
#define MCP1_INT0_GPIO_Port GPIOC
#define MCP1_INT0_EXTI_IRQn EXTI15_10_IRQn
#define MCP_CS_Pin GPIO_PIN_9
#define MCP_CS_GPIO_Port GPIOB
#define SD_CD_Pin GPIO_PIN_15
#define SD_CD_GPIO_Port GPIOA
#define MCP1_INT_Pin GPIO_PIN_15
#define MCP1_INT_GPIO_Port GPIOC
#define MCP1_INT_EXTI_IRQn EXTI15_10_IRQn
#define BMI_ACC_CS_Pin GPIO_PIN_4
#define BMI_ACC_CS_GPIO_Port GPIOE
#define ETH_NRST_Pin GPIO_PIN_2
#define ETH_NRST_GPIO_Port GPIOC
#define PWR_I_Pin GPIO_PIN_0
#define PWR_I_GPIO_Port GPIOC
#define BMI_CS_GYRO_Pin GPIO_PIN_0
#define BMI_CS_GYRO_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_13
#define BUZZER_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
