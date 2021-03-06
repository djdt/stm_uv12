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
enum DISPLAY_STATE {
    STATE_SPLASH,

    STATE_UV_SELECT,
    STATE_DOSE_SELECT,
    STATE_RATE_SELECT,

    STATE_INIT,
    STATE_PAUSED,
    STATE_RUNNING,
    STATE_FINISHED,
};
enum UVMODE {
    UVMODE_A,
    UVMODE_B,
    UVMODE_C
};
enum UPDATE {
    UPDATE_NONE = 0,
    UPDATE_DISPLAY = 1,
    UPDATE_DAC = 2,
    UPDATE_ENABLE = 4,
};
typedef struct {
    enum DISPLAY_STATE display;
    enum UVMODE mode;
    enum UPDATE update;

    uint8_t enabled;
    uint8_t dac;

    uint16_t step;
    uint32_t rate;
    uint32_t dose;
    uint32_t delivered;

    uint32_t remaining;
    uint32_t frame;
} state_t;
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

// General Settings
#define DOSE_MAX 10 * 1000 * 1000
#define DOSE_STEP_SIZE 100 * 1000

// DAC
#define DAC_MIN 16 // 30.0 mA
#define DAC_MAX 252 // 0.0 mA
#define DAC_STEPS (DAC_MAX - DAC_MIN)
#define DAC_STEP_SIZE 1

// Calulated from datasheet power, angle of half intensity

// UV Rates
#define UVA_RATE_MAX 11763 // mJ/m2/s
#define UVB_RATE_MAX 0 // mJ/m2/s
#define UVC_RATE_MAX 921 // mJ/m2/s
#define UVA_RATE_STEP (UVA_RATE_MAX / DAC_STEPS)
#define UVB_RATE_STEP (UVB_RATE_MAX / DAC_STEPS)
#define UVC_RATE_STEP (UVC_RATE_MAX / DAC_STEPS)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
