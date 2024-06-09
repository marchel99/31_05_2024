#include "epdif.h"
#include "main.h"
#include "stm32l4xx_hal.h"

extern SPI_HandleTypeDef hspi1;

// Declaration of the external function to initialize SPI1
extern void MX_SPI1_Init(void);

void DigitalWrite(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState value) {
    HAL_GPIO_WritePin(port, pin, value);
}

GPIO_PinState DigitalRead(GPIO_TypeDef* port, uint16_t pin) {
    return HAL_GPIO_ReadPin(port, pin);
}

void SpiTransfer(uint8_t data) {
    HAL_SPI_Transmit(&hspi1, &data, 1, HAL_MAX_DELAY);
}

void DelayMs(uint32_t delay) {
    HAL_Delay(delay);
}

void EpdIf_IfInit(void) {
    // GPIO initialization for EPD
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, DC_Pin | RST_Pin | CS_Pin, GPIO_PIN_RESET);

    /* Configure GPIO pins : DC_Pin RST_Pin CS_Pin */
    GPIO_InitStruct.Pin = DC_Pin | RST_Pin | CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Configure GPIO pin : BUSY_Pin */
    GPIO_InitStruct.Pin = BUSY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(BUSY_GPIO_Port, &GPIO_InitStruct);

    MX_SPI1_Init(); // Call the SPI initialization function
}
