/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "motor.h"
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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define USER_BTN_Pin GPIO_PIN_0
#define USER_BTN_GPIO_Port GPIOA
#define USER_BTN_EXTI_IRQn EXTI0_IRQn
#define L_MOT_EN_Pin GPIO_PIN_2
#define L_MOT_EN_GPIO_Port GPIOA
#define CC1101_CS_Pin GPIO_PIN_4
#define CC1101_CS_GPIO_Port GPIOA
#define CC1101_SCK_Pin GPIO_PIN_5
#define CC1101_SCK_GPIO_Port GPIOA
#define CC1101_MISO_Pin GPIO_PIN_6
#define CC1101_MISO_GPIO_Port GPIOA
#define CC1101_MOSI_Pin GPIO_PIN_7
#define CC1101_MOSI_GPIO_Port GPIOA
#define R_MOT_EN_Pin GPIO_PIN_0
#define R_MOT_EN_GPIO_Port GPIOB
#define GDO0_Pin GPIO_PIN_1
#define GDO0_GPIO_Port GPIOB
#define GDO0_EXTI_IRQn EXTI1_IRQn
#define GDO2_Pin GPIO_PIN_2
#define GDO2_GPIO_Port GPIOB
#define GDO2_EXTI_IRQn EXTI2_IRQn
#define R_ENC1_Pin GPIO_PIN_8
#define R_ENC1_GPIO_Port GPIOA
#define R_ENC2_Pin GPIO_PIN_9
#define R_ENC2_GPIO_Port GPIOA
#define L_MOT_FWD_Pin GPIO_PIN_15
#define L_MOT_FWD_GPIO_Port GPIOA
#define L_MOT_BWD_Pin GPIO_PIN_3
#define L_MOT_BWD_GPIO_Port GPIOB
#define R_MOT_FWD_Pin GPIO_PIN_4
#define R_MOT_FWD_GPIO_Port GPIOB
#define R_MOT_BWD_Pin GPIO_PIN_5
#define R_MOT_BWD_GPIO_Port GPIOB
#define L_ENC1_Pin GPIO_PIN_6
#define L_ENC1_GPIO_Port GPIOB
#define L_ENC2_Pin GPIO_PIN_7
#define L_ENC2_GPIO_Port GPIOB
#define BNO055_SCL_Pin GPIO_PIN_8
#define BNO055_SCL_GPIO_Port GPIOB
#define BNO055_SDA_Pin GPIO_PIN_9
#define BNO055_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
