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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "stdio.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define TAG 10

#define AnchorIndex 2 //CHANGE ANCHOR INDEX HERE. BOARDS LABELLED A-E
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DW_RESET_Pin GPIO_PIN_0
#define DW_RESET_GPIO_Port GPIOC
#define DW_NSS1_WAKEUP_Pin GPIO_PIN_1
#define DW_NSS1_WAKEUP_GPIO_Port GPIOC
#define DW_NSS_Pin GPIO_PIN_4
#define DW_NSS_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_5
#define LED3_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_0
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOB
#define DW_IRQ2n_Pin GPIO_PIN_9
#define DW_IRQ2n_GPIO_Port GPIOA
#define DW_IRQ2n_EXTI_IRQn EXTI9_5_IRQn
#define DW_IRQn_Pin GPIO_PIN_10
#define DW_IRQn_GPIO_Port GPIOA
#define DW_IRQn_EXTI_IRQn EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
