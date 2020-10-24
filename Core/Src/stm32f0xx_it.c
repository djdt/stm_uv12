/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_it.h"
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SHORT_PRESS 10 // ms
#define LONG_PRESS 1000 // ms
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t last_pin_pressed;
uint8_t button_held = 0;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim14;
/* USER CODE BEGIN EV */
extern state_t state;
extern TIM_HandleTypeDef htim14;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
    /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

    /* USER CODE END NonMaskableInt_IRQn 0 */
    /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

    /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
    /* USER CODE BEGIN HardFault_IRQn 0 */

    /* USER CODE END HardFault_IRQn 0 */
    while (1) {
        /* USER CODE BEGIN W1_HardFault_IRQn 0 */
        /* USER CODE END W1_HardFault_IRQn 0 */
    }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
    /* USER CODE BEGIN SVC_IRQn 0 */

    /* USER CODE END SVC_IRQn 0 */
    /* USER CODE BEGIN SVC_IRQn 1 */

    /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
    /* USER CODE BEGIN PendSV_IRQn 0 */

    /* USER CODE END PendSV_IRQn 0 */
    /* USER CODE BEGIN PendSV_IRQn 1 */

    /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
    /* USER CODE BEGIN SysTick_IRQn 0 */

    /* USER CODE END SysTick_IRQn 0 */
    HAL_IncTick();
    /* USER CODE BEGIN SysTick_IRQn 1 */

    /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles RTC interrupt through EXTI lines 17, 19 and 20.
  */
void RTC_IRQHandler(void)
{
    /* USER CODE BEGIN RTC_IRQn 0 */

    /* USER CODE END RTC_IRQn 0 */
    HAL_RTC_AlarmIRQHandler(&hrtc);
    /* USER CODE BEGIN RTC_IRQn 1 */
    /* USER CODE END RTC_IRQn 1 */
}

/**
  * @brief This function handles EXTI line 4 to 15 interrupts.
  */
void EXTI4_15_IRQHandler(void)
{
    /* USER CODE BEGIN EXTI4_15_IRQn 0 */
    /* USER CODE END EXTI4_15_IRQn 0 */
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
    /* USER CODE BEGIN EXTI4_15_IRQn 1 */
    /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
  * @brief This function handles TIM14 global interrupt.
  */
void TIM14_IRQHandler(void)
{
    /* USER CODE BEGIN TIM14_IRQn 0 */
    if (!button_held) {
        // Fast timer
        htim14.Init.Period = 100;
        HAL_TIM_Base_Init(&htim14);
        __HAL_TIM_SET_COUNTER(&htim14, 0);
        __HAL_TIM_CLEAR_IT(&htim14, TIM_IT_UPDATE);
        HAL_TIM_Base_Start_IT(&htim14);
        button_held = 1;
    }
    /* USER CODE END TIM14_IRQn 0 */
    HAL_TIM_IRQHandler(&htim14);
    /* USER CODE BEGIN TIM14_IRQn 1 */
    switch (last_pin_pressed) {
    case S1_Pin:
        if (state.dac > DAC_MIN) {
            state.dac -= 1;
            state.update = 1;
        }
        break;
    case S2_Pin:
        if (state.dac < DAC_MAX) {
            state.dac += 1;
            state.update = 1;
        }
        break;
    }
    /* USER CODE END TIM14_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    uint8_t pin_state = HAL_GPIO_ReadPin(S1_GPIO_Port, pin);
    if (pin_state == GPIO_PIN_RESET) {
        last_pin_pressed = pin;
        // Reset the timer
        htim14.Init.Period = 1000;
        HAL_TIM_Base_Init(&htim14);
        __HAL_TIM_SET_COUNTER(&htim14, 0);
        __HAL_TIM_CLEAR_IT(&htim14, TIM_IT_UPDATE);
        HAL_TIM_Base_Start_IT(&htim14);
    } else {
        uint16_t len = __HAL_TIM_GET_COUNTER(&htim14);
        if (!button_held && len > SHORT_PRESS) {
            switch (pin) {
            case S1_Pin:
                if (state.dac > DAC_MIN) {
                    state.dac -= 1;
                    state.update = 0x01;
                }
                break;
            case S2_Pin:
                if (state.dac < DAC_MAX) {
                    state.dac += 1;
                    state.update = 0x01;
                }
                break;
            case S3_Pin:
                state.update = 0x02; // Toggle enabled
                state.enabled ^= 1;
                HAL_GPIO_TogglePin(Enable_GPIO_Port, Enable_Pin);
                break;
            }
        }
        // Long presses handled in TIM14_IRQHandler
        HAL_TIM_Base_Stop_IT(&htim14);
    }
    button_held = 0;
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
