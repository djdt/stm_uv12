/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f0xx_hal.h"

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
#define Enable_Pin GPIO_PIN_0
#define Enable_GPIO_Port GPIOA
#define DB7_Pin GPIO_PIN_1
#define DB7_GPIO_Port GPIOA
#define DB6_Pin GPIO_PIN_2
#define DB6_GPIO_Port GPIOA
#define DB5_Pin GPIO_PIN_3
#define DB5_GPIO_Port GPIOA
#define DB4_Pin GPIO_PIN_4
#define DB4_GPIO_Port GPIOA
#define E_Pin GPIO_PIN_5
#define E_GPIO_Port GPIOA
#define RW_Pin GPIO_PIN_6
#define RW_GPIO_Port GPIOA
#define RS_Pin GPIO_PIN_7
#define RS_GPIO_Port GPIOA
#define S1_Pin GPIO_PIN_8
#define S1_GPIO_Port GPIOA
#define S1_EXTI_IRQn EXTI4_15_IRQn
#define S2_Pin GPIO_PIN_9
#define S2_GPIO_Port GPIOA
#define S2_EXTI_IRQn EXTI4_15_IRQn
#define S3_Pin GPIO_PIN_10
#define S3_GPIO_Port GPIOA
#define S3_EXTI_IRQn EXTI4_15_IRQn
#define LAT0_Pin GPIO_PIN_6
#define LAT0_GPIO_Port GPIOB
#define CS_DAC_Pin GPIO_PIN_7
#define CS_DAC_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
enum DISPLAY_STATE {
    STATE_SPLASH,
    STATE_MAIN,
    STATE_CONFIRM,
};
typedef struct {
    enum DISPLAY_STATE display;
    uint8_t dac_value;
    uint8_t uv_enabled;
    uint8_t ticks_enabled;
} state_t;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
