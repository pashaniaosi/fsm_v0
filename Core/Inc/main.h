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
#include "stm32l1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fsm.h"
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
#define BUFFER_SIZE 8
#define DIO0_Pin GPIO_PIN_13
#define DIO0_GPIO_Port GPIOC
#define DIO0_EXTI_IRQn EXTI15_10_IRQn
#define DIO1_Pin GPIO_PIN_0
#define DIO1_GPIO_Port GPIOB
#define DIO1_EXTI_IRQn EXTI0_IRQn
#define DIO2_Pin GPIO_PIN_1
#define DIO2_GPIO_Port GPIOB
#define DIO2_EXTI_IRQn EXTI1_IRQn
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOB
#define RXTX_Pin GPIO_PIN_8
#define RXTX_GPIO_Port GPIOA
#define LORA_NSS_Pin GPIO_PIN_15
#define LORA_NSS_GPIO_Port GPIOA
#define DIO3_Pin GPIO_PIN_6
#define DIO3_GPIO_Port GPIOB
#define DIO3_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */
#define LED_CONTROL(val) HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, (val))
#define LED_ON() LED_CONTROL(1)
#define LED_OFF() LED_CONTROL(0)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
