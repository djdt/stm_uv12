/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mcp48fvb.h"
#include "oled0010.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */
mcp48fvb_t mcp = {
    &hspi1,
    CS_DAC_GPIO_Port, CS_DAC_Pin,
    LAT0_GPIO_Port, LAT0_Pin
};

uint16_t db_pins[8] = { 0, 0, 0, 0, DB4_Pin, DB5_Pin, DB6_Pin, DB7_Pin };
oled0010_t oled = {
    RS_GPIO_Port, RS_Pin,
    RW_GPIO_Port, RW_Pin,
    E_GPIO_Port, E_Pin,
    DB4_GPIO_Port, db_pins,
    0
};

state_t state = { STATE_SPLASH, UVMODE_A, UPDATE_NONE,
    0, DAC_MAX - 1,
    0, 0, 1000, 0 };

/* char time_string[] = "00:00"; */
char strbuf[16];
const char dose_units[] = " J/m"
                          "\x1e";
const char rate_units[] = " J/m"
                          "\x1e"
                          "/s";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM14_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void get_time_string(char* str, RTC_TimeTypeDef* t)
{
    str[0] = (t->Minutes >> 4) + '0';
    str[1] = (t->Minutes & 0x0f) + '0';
    str[2] = ':';
    str[3] = (t->Seconds >> 4) + '0';
    str[4] = (t->Seconds & 0x0f) + '0';
}

void get_seconds_string(char* str, uint16_t seconds)
{
    uint8_t m = seconds / 60;
    uint8_t s = seconds % 60;

    str[0] = m / 10 + '0';
    str[1] = m % 10 + '0';
    str[2] = ':';
    str[3] = s / 10 + '0';
    str[4] = s % 10 + '0';
}

uint8_t get_int_string(char* str, uint32_t n)
{
    uint8_t i = 0;

    // Get string last digit first
    do {
        str[i++] = n % 10 + '0';
    } while (((n /= 10) > 0));

    uint8_t l = i;
    i -= 1;

    // Reverse the string
    char t;
    for (uint8_t j = 0; j < i; ++j, --i) {
        t = str[j];
        str[j] = str[i];
        str[i] = t;
    }
    str[l] = '\0';

    return l;
}

uint8_t get_decimal_string(char* str, uint32_t n, uint8_t decimals)
{
    uint16_t d = 1;
    for (uint8_t i = 0; i < decimals; ++i) {
        d *= 10;
    }

    uint8_t l = get_int_string(str, n / d);
    str[l++] = '.';

    uint8_t l2 = get_int_string(str + l, n % d);
    for (uint8_t i = 0; i < decimals - l2; ++i) {
        for (uint8_t j = l + decimals; j > l; --j) {
            str[j] = str[j - 1];
        }
        str[l + i] = '0';
    }
    // Terminate overlong strings
    str[l + decimals] = '\0';

    return l + decimals;
}

void oled_pad(oled0010_t* oled, uint8_t n)
{
    for (; n > 0; n--) {
        oled_print_char(oled, ' ');
    }
}

void print_splash(oled0010_t* oled)
{
    oled_move_cursor(oled, 0, 0);
    oled_print(oled, "\x1d"
                     "Cell Destroyer"
                     "\x1d");
    oled_move_cursor(oled, 10, 1);
    oled_print(oled, "v0.2.0");
}

void print_uv_select(oled0010_t* oled, enum UVMODE mode)
{
    oled_move_cursor(oled, 0, 0);
    oled_print(oled, "Select Module");
    oled_move_cursor(oled, 15, 0);
    oled_print_char(oled, '\xc5');

    oled_move_cursor(oled, 0, 1);
    oled_print(oled, mode == UVMODE_A ? "UVA" : (mode == UVMODE_B ? "UVB" : "UVC"));
    oled_print(oled, " Module");
    oled_move_cursor(oled, 15, 1);
    oled_print_char(oled, '\xc6');
}

void print_dose_select(oled0010_t* oled, uint32_t dose)
{
    uint8_t len = get_int_string(strbuf, dose / 1000);

    oled_move_cursor(oled, 0, 0);
    oled_print(oled, "Total Dose");
    oled_move_cursor(oled, 15, 0);
    oled_print_char(oled, '\xc5');

    oled_move_cursor(oled, 0, 1);
    oled_pad(oled, 14 - 5 - len);
    oled_print(oled, strbuf);
    oled_print(oled, (char*)dose_units);

    oled_move_cursor(oled, 15, 1);
    oled_print_char(oled, '\xc6');
}

void print_rate_select(oled0010_t* oled, uint32_t rate)
{
    uint32_t len = get_decimal_string(strbuf, rate, 3);

    oled_move_cursor(oled, 0, 0);
    oled_print(oled, "Dose Rate");
    oled_move_cursor(oled, 15, 0);
    oled_print_char(oled, '\xc5');

    oled_move_cursor(oled, 0, 1);
    oled_pad(oled, 14 - 7 - len);
    oled_print(oled, strbuf);
    oled_print(oled, (char*)rate_units);

    oled_move_cursor(oled, 15, 1);
    oled_print_char(oled, '\xc6');
}

void print_main(oled0010_t* oled, char sym, enum UVMODE mode, uint32_t seconds, uint32_t delivered)
{
    get_seconds_string(strbuf, seconds);
    oled_move_cursor(oled, 0, 0);
    oled_print(oled, mode == UVMODE_A ? "UVA" : (mode == UVMODE_B ? "UVB" : "UVC"));
    oled_move_cursor(oled, 4, 0);
    oled_print_char(oled, sym);
    oled_move_cursor(oled, 16 - 5, 0);
    oled_print(oled, strbuf);

    uint32_t len = get_decimal_string(strbuf, delivered, 3);
    oled_move_cursor(oled, 0, 1);
    oled_pad(oled, 16 - 5 - len);
    oled_print(oled, strbuf);
    oled_print(oled, (char*)dose_units);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */
    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_TIM14_Init();
    MX_RTC_Init();
    /* USER CODE BEGIN 2 */
    // Disable interrupts
    HAL_NVIC_DisableIRQ(RTC_IRQn);
    HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);

    // Ensure that the UV is not enabled
    HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, GPIO_PIN_RESET);

    // Init the DAC
    HAL_Delay(50);
    mcp_init(&mcp, MCP_VREF_GAP, MCP_PWRD_NORM, MCP_GAIN_1X);
    mcp_set_dac(&mcp, DAC_MAX);

    // Start the display
    oled_init(&oled,
        OLED_FS_4BIT | OLED_FS_2LINES | OLED_FS_FONT_SMALL | OLED_FS_FONT_WEST_EUR2,
        OLED_DC_BLINK_OFF | OLED_DC_CURSOR_OFF,
        OLED_EM_INC | OLED_EM_DISP_SHIFT_OFF);

    // Print welcome message
    print_splash(&oled);

    // Reenable the button interrupts
    HAL_NVIC_EnableIRQ(RTC_IRQn);
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {

        /* HAL_NVIC_DisableIRQ(RTC_IRQn); */
        if (state.update & UPDATE_DISPLAY) {
            oled_clear_display(&oled);
            state.update &= ~UPDATE_DISPLAY;
        }
        if (state.update & UPDATE_DAC) {
            mcp_set_dac(&mcp, state.dac);
            state.update &= ~UPDATE_DAC;
        }
        if (state.update & UPDATE_ENABLE) {
            HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin,
                state.enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
            state.update &= ~UPDATE_ENABLE;
        }

        switch (state.display) {
        case STATE_UV_SELECT:
            print_uv_select(&oled, state.mode);
            break;
        case STATE_DOSE_SELECT:
            print_dose_select(&oled, state.dose);
            break;
        case STATE_RATE_SELECT:
            print_rate_select(&oled, state.rate);
            break;
        case STATE_PAUSED:
            print_main(&oled, '=', state.mode, state.remaining, state.delivered);
            break;
        case STATE_RUNNING:
            print_main(&oled, '\xf6', state.mode, state.remaining, state.delivered);
            break;
        case STATE_FINISHED:
            print_main(&oled, '\xfa', state.mode, state.remaining, state.delivered);
            break;
        default:
            break;
        }

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        /* HAL_NVIC_EnableIRQ(RTC_IRQn); */
        HAL_SuspendTick();
        HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFE);
        HAL_ResumeTick();
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
    RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

    /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
  */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

    /* USER CODE BEGIN RTC_Init 0 */

    /* USER CODE END RTC_Init 0 */

    RTC_TimeTypeDef sTime = { 0 };
    RTC_DateTypeDef sDate = { 0 };
    RTC_AlarmTypeDef sAlarm = { 0 };

    /* USER CODE BEGIN RTC_Init 1 */
    // Predivs calced from a 15 min run
    // 124
    // 307.17
    /* USER CODE END RTC_Init 1 */
    /** Initialize RTC Only
  */
    hrtc.Instance = RTC;
    hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv = 128 - 1;
    hrtc.Init.SynchPrediv = 312 - 1;
    hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    if (HAL_RTC_Init(&hrtc) != HAL_OK) {
        Error_Handler();
    }

    /* USER CODE BEGIN Check_RTC_BKUP */

    /* USER CODE END Check_RTC_BKUP */

    /** Initialize RTC and set the Time and Date
  */
    sTime.Hours = 0;
    sTime.Minutes = 0;
    sTime.Seconds = 0;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
        Error_Handler();
    }
    sDate.WeekDay = RTC_WEEKDAY_MONDAY;
    sDate.Month = RTC_MONTH_JANUARY;
    sDate.Date = 1;
    sDate.Year = 0;

    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
        Error_Handler();
    }
    /** Enable the Alarm A
  */
    sAlarm.AlarmTime.Hours = 0;
    sAlarm.AlarmTime.Minutes = 0;
    sAlarm.AlarmTime.Seconds = 0;
    sAlarm.AlarmTime.SubSeconds = 0;
    sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
    sAlarm.AlarmMask = RTC_ALARMMASK_ALL;
    sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
    sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
    sAlarm.AlarmDateWeekDay = 1;
    sAlarm.Alarm = RTC_ALARM_A;
    if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN RTC_Init 2 */
    /* USER CODE END RTC_Init 2 */
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

    /* USER CODE BEGIN SPI1_Init 0 */

    /* USER CODE END SPI1_Init 0 */

    /* USER CODE BEGIN SPI1_Init 1 */

    /* USER CODE END SPI1_Init 1 */
    /* SPI1 parameter configuration*/
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */
}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

    /* USER CODE BEGIN TIM14_Init 0 */

    /* USER CODE END TIM14_Init 0 */

    /* USER CODE BEGIN TIM14_Init 1 */

    /* USER CODE END TIM14_Init 1 */
    htim14.Instance = TIM14;
    htim14.Init.Prescaler = 8000 - 1;
    htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim14.Init.Period = 1000;
    htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim14) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM14_Init 2 */

    /* USER CODE END TIM14_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, Enable_Pin | DB7_Pin | DB6_Pin | DB5_Pin | DB4_Pin | E_Pin | RW_Pin | RS_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, LAT0_Pin | CS_DAC_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : Enable_Pin DB7_Pin DB6_Pin DB5_Pin
                           DB4_Pin E_Pin RW_Pin RS_Pin */
    GPIO_InitStruct.Pin = Enable_Pin | DB7_Pin | DB6_Pin | DB5_Pin
        | DB4_Pin | E_Pin | RW_Pin | RS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : S1_Pin S3_Pin */
    GPIO_InitStruct.Pin = S1_Pin | S3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : S2_Pin */
    GPIO_InitStruct.Pin = S2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(S2_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : LAT0_Pin CS_DAC_Pin */
    GPIO_InitStruct.Pin = LAT0_Pin | CS_DAC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
