#include "main.h"
#include "stm32l4xx_hal.h"
#include "bme280.h"
#include "ens160.h"
#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

static struct bme280_t bme280;
static struct bme280_t *p_bme280 = &bme280;

DFRobot_ENS160_I2C ens160;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);

void user_delay_ms(uint32_t period);
int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint8_t len);
int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint8_t len);
void print_sensor_data(struct bme280_t *bme280);
void read_and_print_ens160_data(void);
void init_ens160(void);
void I2C_Scan(void);

#define BME280_OK 0
#define BME280_I2C_ADDR 0x76

void user_delay_ms(uint32_t period)
{
    HAL_Delay(period);
}

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint8_t len)
{
    HAL_StatusTypeDef status;

    if (HAL_I2C_IsDeviceReady(&hi2c1, dev_id << 1, 10, 1000) != HAL_OK)
    {
        const char error_message[] = "I2C device not ready!\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)error_message, strlen(error_message), HAL_MAX_DELAY);
        return -1;
    }

    status = HAL_I2C_Mem_Read(&hi2c1, dev_id << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, len, 1000);
    if (status != HAL_OK)
    {
        char buffer[50];
        snprintf(buffer, sizeof(buffer), "I2C read error: %d\r\n", status);
        HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
        return -1;
    }
    return 0;
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint8_t len)
{
    HAL_StatusTypeDef status;

    if (HAL_I2C_IsDeviceReady(&hi2c1, dev_id << 1, 10, 1000) != HAL_OK)
    {
        const char error_message[] = "I2C device not ready!\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)error_message, strlen(error_message), HAL_MAX_DELAY);
        return -1;
    }

    status = HAL_I2C_Mem_Write(&hi2c1, dev_id << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, len, 1000);
    if (status != HAL_OK)
    {
        char buffer[50];
        snprintf(buffer, sizeof(buffer), "I2C write error: %d\r\n", status);
        HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
        return -1;
    }
    return 0;
}

void print_sensor_data(struct bme280_t *bme280)
{
    bme280_set_power_mode(BME280_FORCED_MODE);

    int32_t temp_raw, pressure_raw, humidity_raw;

    if (bme280_read_pressure_temperature_humidity((uint32_t *)&pressure_raw, &temp_raw, (uint32_t *)&humidity_raw) == BME280_OK)
    {
        char raw_buffer[100];
        int length = snprintf(raw_buffer, sizeof(raw_buffer), "Raw Temp: %ld, Raw Pressure: %ld, Raw Humidity: %ld\r\n", temp_raw, pressure_raw, humidity_raw);
        HAL_UART_Transmit(&huart2, (uint8_t *)raw_buffer, length, HAL_MAX_DELAY);

        int32_t v_comp_temp_s32[1];
        int32_t v_comp_press_u32[1];
        int32_t v_comp_humidity_u32[1];
        int com_rslt = 0;

        com_rslt += bme280_read_pressure_temperature_humidity(
            (uint32_t *)&v_comp_press_u32[1], (uint32_t *)&v_comp_temp_s32[1], (uint32_t *)&v_comp_humidity_u32[1]);

        float imp_temp = ((float)(v_comp_temp_s32[1]) / 100);
        float imp_press = ((float)(v_comp_press_u32[1]) / 100);
        float imp_humi = ((float)(v_comp_humidity_u32[1]) / 1024);
        float dewpt = ((float)v_comp_temp_s32[1] / 100) - ((100 - imp_humi) / 5.);

        char display_buffer[120];
        int lengthd = snprintf(display_buffer, sizeof(display_buffer), "Temp: %.2f °C, Ciśn: %.2f hPa, Wilg: %.2f%% rH, Punkt Rosy: %.2f °C\r\n",
                               imp_temp,
                               imp_press,
                               imp_humi,
                               dewpt);

        HAL_UART_Transmit(&huart2, (uint8_t *)display_buffer, lengthd, HAL_MAX_DELAY);
    }
    else
    {
        const char error_message[] = "Sensor read error!\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)error_message, strlen(error_message), HAL_MAX_DELAY);
    }

    const char separator[] = "________________________________________________________________________\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)separator, strlen(separator), HAL_MAX_DELAY);
}

void read_and_print_ens160_data(void) {
    uint8_t status = DFRobot_ENS160_GetStatus(&ens160);
    char status_buffer[50];
    snprintf(status_buffer, sizeof(status_buffer), "Sensor operating status: %d\r\n", status);
    HAL_UART_Transmit(&huart2, (uint8_t *)status_buffer, strlen(status_buffer), HAL_MAX_DELAY);

    uint8_t aqi = DFRobot_ENS160_GetAQI(&ens160);
    char aqi_buffer[50];
    snprintf(aqi_buffer, sizeof(aqi_buffer), "Air quality index: %d\r\n", aqi);
    HAL_UART_Transmit(&huart2, (uint8_t *)aqi_buffer, strlen(aqi_buffer), HAL_MAX_DELAY);

    uint16_t tvoc = DFRobot_ENS160_GetTVOC(&ens160);
    char tvoc_buffer[100];
    snprintf(tvoc_buffer, sizeof(tvoc_buffer), "Concentration of total volatile organic compounds: %d ppb\r\n", tvoc);
    HAL_UART_Transmit(&huart2, (uint8_t *)tvoc_buffer, strlen(tvoc_buffer), HAL_MAX_DELAY);

    uint16_t eco2 = DFRobot_ENS160_GetECO2(&ens160);
    char eco2_buffer[100];
    snprintf(eco2_buffer, sizeof(eco2_buffer), "Carbon dioxide equivalent concentration: %d ppm\r\n", eco2);
    HAL_UART_Transmit(&huart2, (uint8_t *)eco2_buffer, strlen(eco2_buffer), HAL_MAX_DELAY);

    const char separator[] = "________________________________________________________________________\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)separator, strlen(separator), HAL_MAX_DELAY);
}

void init_ens160(void) {
    DFRobot_ENS160_I2C_Init(&ens160, &hi2c1, 0x53);

    while (DFRobot_ENS160_I2C_Begin(&ens160) != NO_ERR) {
        const char init_fail[] = "ENS160 initialization failed!\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)init_fail, strlen(init_fail), HAL_MAX_DELAY);
        HAL_Delay(3000);
    }
    const char init_success[] = "ENS160 initialized successfully!\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)init_success, strlen(init_success), HAL_MAX_DELAY);

    DFRobot_ENS160_SetPWRMode(&ens160, ENS160_STANDARD_MODE);
    DFRobot_ENS160_SetTempAndHum(&ens160, 25.0, 50.0);
}

void I2C_Scan() {
    char buffer[25];
    uint8_t i2c_devices = 0;

    HAL_UART_Transmit(&huart2, (uint8_t *)"Scanning I2C bus...\r\n", 21, HAL_MAX_DELAY);
    for (uint8_t i = 1; i < 128; i++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 1, 10) == HAL_OK) {
            snprintf(buffer, sizeof(buffer), "Found device at 0x%02X\r\n", i);
            HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
            i2c_devices++;
        }
    }

    if (i2c_devices == 0) {
        HAL_UART_Transmit(&huart2, (uint8_t *)"No I2C devices found\r\n", 22, HAL_MAX_DELAY);
    } else {
        HAL_UART_Transmit(&huart2, (uint8_t *)"I2C scan completed\r\n", 20, HAL_MAX_DELAY);
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();

    I2C_Scan();  // Skanowanie magistrali I2C przed inicjalizacją czujników

    p_bme280->bus_write = user_i2c_write;
    p_bme280->bus_read = user_i2c_read;
    p_bme280->delay_msec = user_delay_ms;
    p_bme280->dev_addr = BME280_I2C_ADDR;

    if (bme280_init(p_bme280) != BME280_OK)
    {
        const char init_fail[] = "BME280 initialization failed!\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)init_fail, strlen(init_fail), HAL_MAX_DELAY);
        Error_Handler();
    }
    else
    {
        const char init_success[] = "BME280 initialized successfully!\r\n";
        HAL_UART_Transmit(&huart2, (uint8_t *)init_success, strlen(init_success), HAL_MAX_DELAY);
    }

    init_ens160();

    while (1)
    {
        print_sensor_data(p_bme280);
        read_and_print_ens160_data();

        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay(2500);
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x00000E14;
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

    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
    {
        Error_Handler();
    }
}

static void MX_USART2_UART_Init(void)
{
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
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
