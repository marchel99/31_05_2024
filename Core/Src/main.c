

/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed pod terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "epd4in2b.h"
#include "epdpaint.h"
#include "fonts.h"
#include "imagedata.h"
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int counter = 1;
int batteryLevel = 0; // Przykładowy poziom naładowania (0-3)
int updateBattery = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void UpdateIcons(Epd *epd, Paint icon_paints[], int selected_icon, int blink);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// POD PRINTF
int __io_putchar(int ch)
{
    if (ch == '\n')
    {
        uint8_t ch2 = '\r';
        HAL_UART_Transmit(&huart2, &ch2, 1, HAL_MAX_DELAY);
    }

    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return 1;
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
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    MX_ADC1_Init();
    MX_SPI1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    /* USER CODE BEGIN 2 */

    /* E-paper display setup */

    HAL_TIM_Base_Start_IT(&htim3); // Uruchom TIM3 w trybie przerwań

    Epd epd;
    Paint paint;
    Paint paint_top;

    unsigned char top_menu[400 * 28 / 8] = {0}; // Bufor dla całego paska

    if (Epd_Init(&epd) != 0)
    {
        printf("e-Paper init failed\n");
        return 1;
    }
    Epd_Clear(&epd);

    // Inicjalizacja obrazu dla górnego paska
    Paint_Init(&paint_top, top_menu, 400, 28);
    Paint_Clear(&paint_top, UNCOLORED);



    // Początkowe wypełnienie ekranu
    unsigned char full_image[(400 * 300) / 8] = {0}; // Cały ekran 400x300
    Paint_Init(&paint, full_image, 400, 300);
    Paint_Clear(&paint, UNCOLORED);

    // interfejs
    Paint_DrawStringAt(&paint, 10, 35, "", &Font16, COLORED);
    Paint_DrawRoundedRectangle(&paint, 10, 50, 190, 170, 10, COLORED);
    Paint_DrawRoundedRectangle(&paint, 210, 50, 390, 170, 10, COLORED);

    // Tekst w lewym prostokącie
    Paint_DrawStringAt(&paint, 20, 60, "AQI: 1", &Font16, COLORED);
    Paint_DrawStringAt(&paint, 20, 80, "TVOC: 44ppm", &Font16, COLORED);
    Paint_DrawStringAt(&paint, 20, 100, "HCHO: 0.04ppm", &Font16, COLORED);
    Paint_DrawStringAt(&paint, 20, 120, "CO: <5ppm", &Font16, COLORED);
    Paint_DrawStringAt(&paint, 20, 140, "CO2: 412ppm", &Font16, COLORED);

    // Tekst w prawym prostokącie
    Paint_DrawStringAt(&paint, 220, 60, "TEMP: 22 C", &Font16, COLORED);
    Paint_DrawStringAt(&paint, 220, 80, "HUM: 51%", &Font16, COLORED);
    Paint_DrawStringAt(&paint, 220, 100, "PRESS: 941 hPa", &Font16, COLORED);
    Paint_DrawStringAt(&paint, 220, 120, "DP: 12 C", &Font16, COLORED);
    Paint_DrawStringAt(&paint, 10, 275, "Created by Marchel99", &Font16, COLORED);

    // IKONY
    Paint_DrawBitmap(&paint, icon_temp, 5, 200, 48, 48, COLORED);
    Paint_DrawBitmap(&paint, icon_humi, 55, 200, 48, 48, COLORED);
    Paint_DrawBitmap(&paint, icon_sun, 105, 200, 48, 48, COLORED);
    Paint_DrawBitmap(&paint, icon_leaf, 155, 200, 48, 48, COLORED);
    Paint_DrawBitmap(&paint, icon_sunset, 205, 200, 48, 48, COLORED);
    Paint_DrawBitmap(&paint, icon_sunrise, 255, 200, 48, 48, COLORED);
    Paint_DrawBitmap(&paint, icon_wind, 305, 200, 48, 48, COLORED);
    Paint_DrawBitmap(&paint, icon_settings, 355, 200, 48, 48, COLORED);

    Epd_DisplayFull(&epd, Paint_GetImage(&paint));

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    /* USER CODE END 2 */

    /* Infinite loop */

    int impulse_counter = 0; // Licznik impulsów

    /* USER CODE BEGIN WHILE */
    while (1)
    {
      


 impulse_counter++;

        if (impulse_counter >= 60)
        {
            // Pełne odświeżanie wyświetlacza co 6 impulsów

            // Czyszczenie ekranu
            Paint_Clear(&paint, UNCOLORED);
            Paint_Clear(&paint_top, UNCOLORED);

            // Aktualizacja górnego paska
            char buffer_top[100];
            snprintf(buffer_top, sizeof(buffer_top), "%d", counter++);
            Paint_DrawStringAt(&paint_top, 10, 5, buffer_top, &Font20, COLORED);
            Paint_DrawStringAt(&paint_top, 150, 5, "SD: Error", &Font20, COLORED);

            // Aktualizacja baterii na górnym pasku
            DrawBattery(&paint_top, 350, 2, 32, 24, COLORED);
            DrawBatteryLevel(&paint_top, 350, 2, 30, 24, batteryLevel, COLORED);
            batteryLevel = (batteryLevel + 1) % 4;

            // Kopiowanie górnego paska do pełnego obrazu
            memcpy(full_image, top_menu, sizeof(top_menu));

            // Rysowanie interfejsu na pełnym obrazie
            Paint_DrawStringAt(&paint, 10, 35, "", &Font16, COLORED);
            Paint_DrawRoundedRectangle(&paint, 10, 50, 190, 170, 10, COLORED);
            Paint_DrawRoundedRectangle(&paint, 210, 50, 390, 170, 10, COLORED);

            Paint_DrawStringAt(&paint, 20, 60, "AQI: 1", &Font16, COLORED);
            Paint_DrawStringAt(&paint, 20, 80, "TVOC: 44ppm", &Font16, COLORED);
            Paint_DrawStringAt(&paint, 20, 100, "HCHO: 0.04ppm", &Font16, COLORED);
            Paint_DrawStringAt(&paint, 20, 120, "CO: <5ppm", &Font16, COLORED);
            Paint_DrawStringAt(&paint, 20, 140, "CO2: 412ppm", &Font16, COLORED);

            Paint_DrawStringAt(&paint, 220, 60, "TEMP: 22 C", &Font16, COLORED);
            Paint_DrawStringAt(&paint, 220, 80, "HUM: 51%", &Font16, COLORED);
            Paint_DrawStringAt(&paint, 220, 100, "PRESS: 941 hPa", &Font16, COLORED);
            Paint_DrawStringAt(&paint, 220, 120, "DP: 12 C", &Font16, COLORED);
            Paint_DrawStringAt(&paint, 10, 275, "Created by Marchel99", &Font16, COLORED);

            Paint_DrawBitmap(&paint, icon_temp, 5, 200, 48, 48, COLORED);
            Paint_DrawBitmap(&paint, icon_humi, 55, 200, 48, 48, COLORED);
            Paint_DrawBitmap(&paint, icon_sun, 105, 200, 48, 48, COLORED);
            Paint_DrawBitmap(&paint, icon_leaf, 155, 200, 48, 48, COLORED);
            Paint_DrawBitmap(&paint, icon_sunset, 205, 200, 48, 48, COLORED);
            Paint_DrawBitmap(&paint, icon_sunrise, 255, 200, 48, 48, COLORED);
            Paint_DrawBitmap(&paint, icon_wind, 305, 200, 48, 48, COLORED);
            Paint_DrawBitmap(&paint, icon_settings, 355, 200, 48, 48, COLORED);

            Epd_DisplayFull(&epd, Paint_GetImage(&paint));

            impulse_counter = 0; // Reset licznika impulsów
        }
        else
        {
            // Aktualizacja górnego paska bez pełnego odświeżania
            Paint_Clear(&paint_top, UNCOLORED);

            // Aktualizacja licznika
            char buffer_top[100];
            snprintf(buffer_top, sizeof(buffer_top), "%d", counter++);
            Paint_DrawStringAt(&paint_top, 10, 5, buffer_top, &Font20, COLORED);
            Paint_DrawStringAt(&paint_top, 150, 5, "SD: Error", &Font20, COLORED);

            // Aktualizacja baterii na górnym pasku
            DrawBattery(&paint_top, 350, 2, 32, 24, COLORED);
            DrawBatteryLevel(&paint_top, 350, 2, 30, 24, batteryLevel, COLORED);
            batteryLevel = (batteryLevel + 1) % 4;

            // Wyświetlanie górnego paska
            Epd_Display_Partial(&epd, Paint_GetImage(&paint_top), 0, 0, 400, 28);
        }

        HAL_Delay(300); // Opóźnienie 300 ms







        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }

    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 40;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */

    /** Common config
     */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 2;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure the ADC multi-mode
     */
    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_2;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x10909CEC;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Analogue filter
     */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Digital filter
     */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */
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
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

    /* USER CODE BEGIN TIM2_Init 0 */

    /* USER CODE END TIM2_Init 0 */

    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 29;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;
    if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */

    /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

    /* USER CODE BEGIN TIM3_Init 0 */

    /* USER CODE END TIM3_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 999;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, DC_Pin | RST_Pin | CS_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : DC_Pin RST_Pin CS_Pin */
    GPIO_InitStruct.Pin = DC_Pin | RST_Pin | CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : BUSY_Pin */
    GPIO_InitStruct.Pin = BUSY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BUSY_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
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
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
