/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define SR_nOE_IMU_Pin GPIO_PIN_13
#define SR_nOE_IMU_GPIO_Port GPIOC
#define SR_Latch_IMU_Pin GPIO_PIN_1
#define SR_Latch_IMU_GPIO_Port GPIOC
#define RF_IRQ_Pin GPIO_PIN_2
#define RF_IRQ_GPIO_Port GPIOC
#define one_wire_Pin GPIO_PIN_1
#define one_wire_GPIO_Port GPIOA
#define SR_Latch_Pin GPIO_PIN_4
#define SR_Latch_GPIO_Port GPIOC
#define SR_nOE_RF_Pin GPIO_PIN_5
#define SR_nOE_RF_GPIO_Port GPIOC
#define PPS_Pin GPIO_PIN_10
#define PPS_GPIO_Port GPIOB
#define BME_CS_Pin GPIO_PIN_11
#define BME_CS_GPIO_Port GPIOA
#define switch_Pin GPIO_PIN_3
#define switch_GPIO_Port GPIOB
#define buzzer_Pin GPIO_PIN_4
#define buzzer_GPIO_Port GPIOB
#define burner_Pin GPIO_PIN_5
#define burner_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
