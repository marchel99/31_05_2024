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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "epd4in2b.h"
#include "epdpaint.h"
#include "fonts.h"
#include "imagedata.h"
#include "user_interface.h"
#include "globals.h"
#include "ens160.h"
#include "max.h"
#include "bme280.h"

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
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//volatile uint8_t buttonState_SW = 0; 
static struct bme280_t bme280;
static struct bme280_t *p_bme280 = &bme280;

volatile int currentIconIndex = 1; // Globalna zmienna do śledzenia wybranej ikony


DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
extern DFRobot_ENS160_I2C ens160;

int counter = 1;
float batteryLevel = 0;



Epd epd;
Paint paint;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void print_sensor_data_bme280(struct bme280_t *bme280);
void init_ens160(void);
void read_and_print_ens160_data(void);

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint8_t len);
int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint8_t len);

float read_soc(I2C_HandleTypeDef *hi2c);

// void DisplayIcon(int iconIndex);


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin); 

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t loopCounter = 0; // Licznik impulsów pętli while

#define BME280_OK 0
#define BME280_I2C_ADDR 0x76

// Redirect r to UART
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


 void init_ens160(void)
  {
    DFRobot_ENS160_I2C_Init(&ens160, &hi2c1, 0x53);

    while (DFRobot_ENS160_I2C_Begin(&ens160) != NO_ERR)
    {
      printf("ENS160 initialization failed!\r\n");
      HAL_Delay(3000);
    }
    printf("ENS160 initialized successfully!\r\n");

    DFRobot_ENS160_SetPWRMode(&ens160, ENS160_STANDARD_MODE);
    DFRobot_ENS160_SetTempAndHum(&ens160, 25.0, 50.0);
  }

  // I2C read function for BME280
  int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t * reg_data, uint8_t len)
  {
    HAL_StatusTypeDef status;

    if (HAL_I2C_IsDeviceReady(&hi2c1, dev_id << 1, 10, 1000) != HAL_OK)
    {
      printf("I2C device not ready!\r\n");
      return -1;
    }

    status = HAL_I2C_Mem_Read(&hi2c1, dev_id << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, len, 1000);
    if (status != HAL_OK)
    {
      printf("I2C read error: %d\r\n", status);
      return -1;
    }
    return 0;
  }



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  // Initialize the ENS160 sensor
 

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  /* E-paper display setup */
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  

  if (Epd_Init(&epd) != 0)
  {
    printf("e-Paper init failed\n");
    return 1;
  }

  Epd_Clear(&epd);
  printf("e-Paper init succeed!\n");

  unsigned char top_menu[(400 * 300) / 8 + 100] = {0}; // Bufor dla całego gornego  paska

  // width should be the multiple of 8
  Paint_Init(&paint, top_menu, 400, 300);
  Paint_Clear(&paint, UNCOLORED);

  // Początkowe wypełnienie ekranu

  Epd_DisplayFull(&epd, Paint_GetImage(&paint));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  int thickness = 2; // Grubość obiektów interfejsu

  // Parametry obiektów
  int width = 120;
  int height = 90;

  // Obliczenia przesunięć
  int offset_x = 260; // Przesunięcie poziome
  int offset_y = 100; // Przesunięcie pionowe

  // Obliczanie środka ekranu
  int screen_center_x = EPD_WIDTH / 2;
  int screen_center_y = EPD_HEIGHT / 2;

  int vertical_gap = 10; // Odległość pionowa między ćwiartkami

  // Obliczanie pozycji ćwiartek względem środka ekranu
  int x0_left = screen_center_x - offset_x / 2 - width / 2;
  int x0_right = screen_center_x + offset_x / 2 - width / 2;
  int y0_top = screen_center_y - offset_y / 2 - height / 2 - 10 - vertical_gap / 2;
  int y0_bottom = screen_center_y + offset_y / 2 - height / 2 - 10 - vertical_gap / 2;

  init_ens160();
  while (1)
  {
    read_and_print_ens160_data();

    uint32_t encoderValue = __HAL_TIM_GET_COUNTER(&htim2);
    int iconIndex = getIconIndex(encoderValue);
      currentIconIndex = getIconIndex(encoderValue); 
    // Użycie zmiennej do śledzenia ostatniej wartości iconIndex

    Paint_Clear(&paint, UNCOLORED);

    DisplayTopSection(&paint, iconIndex, encoderValue, counter++, batteryLevel);

    uint8_t bat_percentage = read_soc(&hi2c1);

    char buffer_bat_percentage[100];

    Paint_DrawStringAt(&paint, 327, 10, "%", &Font20, COLORED);

    snprintf(buffer_bat_percentage, sizeof(buffer_bat_percentage), "%d", bat_percentage);
    Paint_DrawStringAt(&paint, 300, 10, buffer_bat_percentage, &Font20, COLORED);
    batteryLevel = bat_percentage;

    // Paint_DrawStringAt(&paint, 340, 102, "ppm",&Font16, COLORED);

    // Rysowanie linii poziomej
    // Paint_DrawLineWithThickness(&paint, x0_left, y0_top - vertical_gap, x0_right + width, y0_top - vertical_gap, thickness, COLORED);

    // Rysowanie obiektów
    Paint_Universal_Ring(&paint, x0_left, y0_top, width, height, thickness, COLORED, 1); // kolor: COLORED, Ćwiartka: 1

    Paint_Universal_Ring(&paint, x0_right, y0_top, width, height, thickness, COLORED, 2); // kolor: COLORED, Ćwiartka: 2

    uint8_t tvoc = DFRobot_ENS160_GetTVOC(&ens160);

    char buffer_tvoc[20]; // Adjust the buffer size as needed

    Paint_DrawStringAt(&paint, 306, 80, "TVOC", &Font20, COLORED);
    snprintf(buffer_tvoc, sizeof(buffer_tvoc), "%d", tvoc);
    Paint_DrawStringAt(&paint, 310, 100, buffer_tvoc, &Font20, COLORED);
    Paint_DrawStringAt(&paint, 340, 102, "ppm", &Font16, COLORED);

    Paint_Universal_Ring(&paint, x0_left, y0_bottom, width, height, thickness, COLORED, 3); // kolor: COLORED, Ćwiartka: 3

    uint8_t co2 = DFRobot_ENS160_GetECO2(&ens160);
    char buffer_CO2[300];
    snprintf(buffer_CO2, sizeof(buffer_CO2), "%d", co2);
    Paint_DrawStringAt(&paint, 20, 100, buffer_CO2, &Font20, COLORED);
    Paint_DrawStringAt(&paint, 18, 80, "CO2", &Font20, COLORED);
    Paint_DrawStringAt(&paint, 64, 103, "ppm", &Font16, COLORED);

    Paint_Universal_Ring(&paint, x0_right, y0_bottom, width, height, thickness, COLORED, 4); // kolor: COLORED, Ćwiartka: 4

    // Obliczanie środka geometrycznego czterech obiektów
    int center_x = (x0_left + width / 2 + x0_right + width / 2) / 2;
    int center_y = (y0_top + height / 2 + y0_bottom + height / 2) / 2;

    // Ustawienia dla pierścienia
    int outer_radius = screen_center_x - x0_left - height - vertical_gap - thickness;

    // Rysowanie centralnego pierścienia
    Paint_DrawRing(&paint, center_x, center_y, outer_radius, thickness, COLORED); // Jasnoszary pierścień

    uint8_t aqi = DFRobot_ENS160_GetAQI(&ens160);
    char buffer_AQI[100];
    snprintf(buffer_AQI, sizeof(buffer_AQI), "%d", aqi);
    Paint_DrawStringAtCenter(&paint, center_y, buffer_AQI, &Font20, 400);
    Paint_DrawStringAtCenter(&paint, center_y - 20, "AQI:", &Font20, 400);

    /*


    int32_t temp_raw, pressure_raw, humidity_raw;
    bme280_set_power_mode(BME280_FORCED_MODE);

     int32_t v_comp_temp_s32 = bme280_compensate_temperature_int32(temp_raw);
            int32_t  v_comp_press_u32 = bme280_compensate_pressure_int32(pressure_raw);
            int32_t  v_comp_humidity_u32 = bme280_compensate_humidity_int32(humidity_raw);



       char buffer_temp[100];
     char buffer_press[100];
     char buffer_hum[100];

    snprintf(buffer_temp, sizeof(buffer_temp), "%d", v_comp_temp_s32);
    snprintf(buffer_press, sizeof(buffer_press), "%d", v_comp_press_u32);

    snprintf(buffer_hum, sizeof(buffer_hum), "%d", v_comp_humidity_u32);




    Paint_DrawStringAtCenter(&paint, center_y, v_comp_humidity_u32, &Font20, 300);
    Paint_DrawStringAtCenter(&paint, center_y-20, "Tem:", &Font20, 300);





     */

    // int r_height = 30;
    // Paint_DrawLineWithThickness(&paint, x0_left, y0_bottom + height + 16 + r_height + vertical_gap, x0_left + height, y0_bottom + height + 16 + r_height + vertical_gap, thickness, COLORED);

    DisplayBottomSection(&paint, iconIndex);

    Epd_Display_Partial_DMA(&epd, Paint_GetImage(&paint), 0, 0, 400, 300);

    HAL_Delay(10); // Opóźnienie 1 ms

    // Zwiększenie licznika impulsów
    loopCounter++;

    // Sprawdzanie, czy licznik osiągnął 180
    if (loopCounter >= 180)
    {
      Epd_Clear(&epd); // Pełny reset ekranu
      Paint_Clear(&paint, UNCOLORED);
      Epd_DisplayFull(&epd, Paint_GetImage(&paint));

      loopCounter = 0; // Zresetowanie licznika impulsów
    }

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, DC_Pin|RST_Pin|CS_Pin|CS_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DC_Pin RST_Pin CS_Pin CS_2_Pin */
  GPIO_InitStruct.Pin = DC_Pin|RST_Pin|CS_Pin|CS_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUSY_Pin */
  GPIO_InitStruct.Pin = BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZ_Pin */
  GPIO_InitStruct.Pin = BUZZ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EN_SW_Pin */
  GPIO_InitStruct.Pin = EN_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EN_SW_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == EN_SW_Pin)
    {
        printf("Przycisk enkodera wciśnięty! Aktualna ikona: %d\n", currentIconIndex);
        switch (currentIconIndex)
        {
            case 1:
                printf("Przechodzę do menu 1\n");
                // Funkcja do obsługi menu 1
                break;
            case 2:
                printf("Przechodzę do menu 2\n");
                // Funkcja do obsługi menu 2
                break;
            case 3:
                printf("Przechodzę do menu 3\n");
                // Funkcja do obsługi menu 3
                break;
            case 4:
                printf("Przechodzę do menu 4\n");
                // Funkcja do obsługi menu 4
                break;
            case 5:
                printf("Przechodzę do menu 5\n");
                // Funkcja do obsługi menu 5
                break;
            case 6:
                printf("Przechodzę do menu 6\n");
                // Funkcja do obsługi menu 6
                break;
            case 7:
                printf("Przechodzę do menu 7\n");
                // Funkcja do obsługi menu 7
                break;
            case 8:
                printf("Przechodzę do menu 8\n");
                // Funkcja do obsługi menu 8
                break;
            default:
                printf("Nieznane menu\n");
                break;
        }
    }
}

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

#ifdef  USE_FULL_ASSERT
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
