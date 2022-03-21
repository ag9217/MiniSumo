/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AI1_Pin GPIO_PIN_13
#define AI1_GPIO_Port GPIOB
#define STBY_Pin GPIO_PIN_14
#define STBY_GPIO_Port GPIOB
#define BI1_Pin GPIO_PIN_15
#define BI1_GPIO_Port GPIOB
#define BI2_Pin GPIO_PIN_8
#define BI2_GPIO_Port GPIOA
#define AI2_Pin GPIO_PIN_15
#define AI2_GPIO_Port GPIOA
#define LINE2_Pin GPIO_PIN_2
#define LINE2_GPIO_Port GPIOD
#define LINE2_EXTI_IRQn EXTI2_3_IRQn
#define LINE1_Pin GPIO_PIN_3
#define LINE1_GPIO_Port GPIOD
#define LINE1_EXTI_IRQn EXTI2_3_IRQn
#define DIST3_Pin GPIO_PIN_4
#define DIST3_GPIO_Port GPIOB
#define DIST3_EXTI_IRQn EXTI4_15_IRQn
#define DIST2_Pin GPIO_PIN_5
#define DIST2_GPIO_Port GPIOB
#define DIST2_EXTI_IRQn EXTI4_15_IRQn
#define DIST1_Pin GPIO_PIN_6
#define DIST1_GPIO_Port GPIOB
#define DIST1_EXTI_IRQn EXTI4_15_IRQn
#define LED_STATUS_Pin GPIO_PIN_9
#define LED_STATUS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
