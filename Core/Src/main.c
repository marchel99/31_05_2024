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
 * This software is licensed under terms that can be found in the LICENSE file
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
#include <string.h>
#include <math.h>
#include "epd4in2b.h"
#include "imagedata.h"
#include "epdpaint.h"
#include "fonts.h"
#include "bme280.h"
#include "ens160.h"
#include "max.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BME280_OK 0
#define BME280_I2C_ADDR 0x76
#define I2C_DEFAULT_ADDRESS 0x36

#define BUZZER_Pin GPIO_PIN_8
#define BUZZER_GPIO_Port GPIOC
#define BUZZER_TOGGLE_INTERVAL 15 // Buzzer will toggle every 5 measurements
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
extern DFRobot_ENS160_I2C ens160;
extern const unsigned char temperature_icon[];
static struct bme280_t bme280;
static struct bme280_t *p_bme280 = &bme280;
static uint32_t measurement_number = 1;

static uint32_t buzzer_counter = 0; // Counter for buzzer activation
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */


void process_SD_card(const char *data);
void I2C_Scan(void);
void init_ens160(void);
void read_and_print_ens160_data(void);
void print_sensor_data(void);
void user_delay_ms(uint32_t period);
int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint8_t len);
int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint8_t len);
float read_voltage(I2C_HandleTypeDef *hi2c);
float read_soc(I2C_HandleTypeDef *hi2c);
void print_sensor_data_bme280(struct bme280_t *bme280);
void read_adc_values(void);
void toggle_buzzer(void); 
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Redirect printf to UART
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

// Delay function for BME280
void user_delay_ms(uint32_t period)
{
    HAL_Delay(period);
}

// I2C read function for BME280
int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint8_t len)
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

// I2C write function for BME280
int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint8_t len)
{
    HAL_StatusTypeDef status;

    if (HAL_I2C_IsDeviceReady(&hi2c1, dev_id << 1, 10, 1000) != HAL_OK)
    {
        printf("I2C device not ready!\r\n");
        return -1;
    }

    status = HAL_I2C_Mem_Write(&hi2c1, dev_id << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, len, 1000);
    if (status != HAL_OK)
    {
        printf("I2C write error: %d\r\n", status);
        return -1;
    }
    return 0;
}


// Function to toggle the buzzer
void toggle_buzzer(void)
{
    HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
}



// Function to read and print BME280 sensor data
void print_sensor_data_bme280(struct bme280_t *bme280)
{
    int32_t temp_raw, pressure_raw, humidity_raw;

    // Set the sensor to FORCED_MODE for a single measurement
    bme280_set_power_mode(BME280_FORCED_MODE);
    user_delay_ms(100); // Add delay after setting the mode

    if (bme280_read_uncomp_pressure_temperature_humidity(&pressure_raw, &temp_raw, &humidity_raw) == BME280_OK)
    {
        int32_t v_comp_temp_s32;
        uint32_t v_comp_press_u32;
        uint32_t v_comp_humidity_u32;

        v_comp_temp_s32 = bme280_compensate_temperature_int32(temp_raw);
        v_comp_press_u32 = bme280_compensate_pressure_int32(pressure_raw);
        v_comp_humidity_u32 = bme280_compensate_humidity_int32(humidity_raw);

        float imp_temp = ((float)v_comp_temp_s32 / 100);
        float imp_press = ((float)v_comp_press_u32 / 100);
        float imp_humi = ((float)v_comp_humidity_u32 / 1024);
        float dewpt = imp_temp - ((100 - imp_humi) / 5.0);

        char raw_buffer[200];
        const char separator[] = "________________________________________________________________________\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)separator, strlen(separator), HAL_MAX_DELAY);
        int length = snprintf(raw_buffer, sizeof(raw_buffer), "ENS160_Status: %d, Raw Temp: %ld, Raw Pressure: %ld, Raw Humidity: %ld\r\n",
                              DFRobot_ENS160_GetStatus(&ens160), temp_raw, pressure_raw, humidity_raw);
        HAL_UART_Transmit(&huart2, (uint8_t *)raw_buffer, length, HAL_MAX_DELAY);

        HAL_UART_Transmit(&huart2, (uint8_t *)separator, strlen(separator), HAL_MAX_DELAY);

        char display_buffer[200];
        length = snprintf(display_buffer, sizeof(display_buffer), "Temp: %.2f °C, Press: %.2f hPa, Hum: %.2f%% rH, Dew Point: %.2f °C\r\n",
                          imp_temp, imp_press, imp_humi, dewpt);
        HAL_UART_Transmit(&huart2, (uint8_t *)display_buffer, length, HAL_MAX_DELAY);
    }
    else
    {
        const char error_message[] = "Sensor read error!\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)error_message, strlen(error_message), HAL_MAX_DELAY);
    }
}

// Function to read and print voltage and SoC
void print_sensor_data(void)
{
    float voltage_battery = read_voltage(&hi2c1);
    float soc = read_soc(&hi2c1);

    printf("Voltage: %.3f V, SoC: %.2f %%\r\n", voltage_battery, soc);

    // Prepare CSV data
    char data_buffer[256];
    snprintf(data_buffer, sizeof(data_buffer), "%lu,%.3f,%.2f\r\n", measurement_number++, voltage_battery, soc);

    // Save data to SD card
    process_SD_card(data_buffer);
}

// Initialize the ENS160 sensor
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

// Function to scan for I2C devices
void I2C_Scan(void)
{
    char buffer[25];
    uint8_t i2c_devices = 0;

    if (i2c_devices == 0)
    {
        printf("No I2C devices found\r\n");
    }
    else
    {
        printf("I2C scan completed\r\n");
    }
}

// Save data to the SD card
void process_SD_card(const char *data)
{
    FATFS FatFs;  // Fatfs handle
    FIL fil;      // File handle
    FRESULT fres; // Result after operations

    // Mount the SD card
    fres = f_mount(&FatFs, "", 1); // 1=mount now
    if (fres != FR_OK)
    {
        printf("Failed to mount SD card: (%i)\r\n", fres);
        return;
    }
    printf("SD card mounted successfully!!!\r\n");

    // Open the file for writing
    printf("Opening file 'sensor_data.csv' for writing\n");
    fres = f_open(&fil, "sensor_data.csv", FA_WRITE | FA_OPEN_APPEND);
    if (fres != FR_OK)
    {
        printf("Error opening file (FA_WRITE | FA_OPEN_APPEND): (%i)\r\n", fres);
        f_mount(NULL, "", 0);
        return;
    }

    printf("Writing data to the file...\r\n");
    // Write data
    UINT bytes_written;
    fres = f_write(&fil, data, strlen(data), &bytes_written);
    if (fres != FR_OK || bytes_written != strlen(data))
    {
        printf("File write error, written %u bytes\n", bytes_written);
        f_close(&fil);
        f_mount(NULL, "", 0);
        return;
    }
    printf("Data saved: %s\n", data);

    // Close the file after writing
    fres = f_close(&fil);
    if (fres != HAL_OK)
    {
        printf("File close error after writing: (%i)\r\n", fres);
        f_mount(NULL, "", 0);
        return;
    }

    // Unmount the card
    f_mount(NULL, "", 0);
    printf("SD card unmounted successfully!!!\r\n");
}

// Function to read ADC values and print them
void read_adc_values(void)
{
    uint32_t value[2];
    float voltage[2];

    HAL_ADC_Start(&hadc1);

    // Conversion for channel 0
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    value[0] = HAL_ADC_GetValue(&hadc1);

    // Conversion for channel 1
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    value[1] = HAL_ADC_GetValue(&hadc1);

    // Convert ADC values to voltage
    voltage[0] = 3.3f * value[0] / 4095.0f;
    voltage[1] = 3.3f * value[1] / 4095.0f;

    printf("CO value=%lu (%.3f V), \nHCHO value=%lu (%.3f V)\n", value[0], voltage[0], value[1], voltage[1]);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
    printf("Start!\n\n");
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
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);  // Calibrate ADC

    I2C_Scan();
    init_ens160();

    p_bme280->bus_write = user_i2c_write;
    p_bme280->bus_read = user_i2c_read;
    p_bme280->delay_msec = user_delay_ms;
    p_bme280->dev_addr = BME280_I2C_ADDR;

    if (bme280_init(p_bme280) != BME280_OK)
    {
        printf("BME280 initialization failed!\r\n");
        Error_Handler();
    }
    else
    {
        printf("BME280 initialized successfully!\r\n");
    }

    // Configure BME280 sensor
    bme280_set_oversamp_humidity(BME280_OVERSAMP_1X);
    bme280_set_oversamp_pressure(BME280_OVERSAMP_16X);
    bme280_set_oversamp_temperature(BME280_OVERSAMP_2X);
    bme280_set_filter(BME280_FILTER_COEFF_16);
    bme280_set_standby_durn(BME280_STANDBY_TIME_63_MS);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        // Read and print data from sensors
        print_sensor_data_bme280(p_bme280);
        print_sensor_data();
        read_and_print_ens160_data();

   // Check if it's time to toggle the buzzer
        if (measurement_number % BUZZER_TOGGLE_INTERVAL == 0)
        {
            toggle_buzzer();
        }


        HAL_Delay(100);  // Delay for 100 milliseconds

        // Encoder read and ADC conversion for additional sensors
        unsigned short en_count;
        en_count = __HAL_TIM_GET_COUNTER(&htim2);

        printf("Encoder read: %d\n", en_count);

        // Read and print ADC values
        read_adc_values();

        HAL_Delay(1500);  // Delay for 250 milliseconds
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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

  /*Configure GPIO pins : DC_Pin RST_Pin CS_Pin CS2_Pin */
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

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZ_Pin */
  GPIO_InitStruct.Pin = BUZZ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_Pin */
  GPIO_InitStruct.Pin = EN_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EN_SW_GPIO_Port, &GPIO_InitStruct);

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
  __disable_irq();
  while (1)
  {
  }
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
