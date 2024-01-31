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
#include "stm32l4xx_hal.h"

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
#define G2_Pin GPIO_PIN_0
#define G2_GPIO_Port GPIOC
#define R2_Pin GPIO_PIN_1
#define R2_GPIO_Port GPIOC
#define G1_Pin GPIO_PIN_2
#define G1_GPIO_Port GPIOC
#define R1_Pin GPIO_PIN_3
#define R1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define TEST_Pin GPIO_PIN_4
#define TEST_GPIO_Port GPIOA
#define C_Pin GPIO_PIN_0
#define C_GPIO_Port GPIOB
#define A_Pin GPIO_PIN_1
#define A_GPIO_Port GPIOB
#define STROBE_Pin GPIO_PIN_2
#define STROBE_GPIO_Port GPIOB
#define D2_Pin GPIO_PIN_10
#define D2_GPIO_Port GPIOB
#define D3_Pin GPIO_PIN_11
#define D3_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_12
#define D4_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_13
#define D5_GPIO_Port GPIOB
#define D6_Pin GPIO_PIN_14
#define D6_GPIO_Port GPIOB
#define BLANKING_Pin GPIO_PIN_15
#define BLANKING_GPIO_Port GPIOB
#define B_Pin GPIO_PIN_6
#define B_GPIO_Port GPIOC
#define D_Pin GPIO_PIN_7
#define D_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_8
#define B1_GPIO_Port GPIOA
#define CLK_Pin GPIO_PIN_9
#define CLK_GPIO_Port GPIOA
#define OE_Pin GPIO_PIN_10
#define OE_GPIO_Port GPIOA
#define LAT_Pin GPIO_PIN_11
#define LAT_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define BCD3_Pin GPIO_PIN_15
#define BCD3_GPIO_Port GPIOA
#define BCD2_Pin GPIO_PIN_10
#define BCD2_GPIO_Port GPIOC
#define BCD1_Pin GPIO_PIN_11
#define BCD1_GPIO_Port GPIOC
#define BCD0_Pin GPIO_PIN_12
#define BCD0_GPIO_Port GPIOC
#define B2_Pin GPIO_PIN_2
#define B2_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define D1_Pin GPIO_PIN_9
#define D1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
