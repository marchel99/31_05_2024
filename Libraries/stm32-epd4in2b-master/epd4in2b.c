#include "epd4in2b.h"
#include "stm32l4xx_hal.h"

// Przypisanie pinów
#define DC_PIN     DC_Pin
#define DC_PORT    DC_GPIO_Port
#define RST_PIN    RST_Pin
#define RST_PORT   RST_GPIO_Port
#define CS_PIN     CS_Pin
#define CS_PORT    CS_GPIO_Port
#define BUSY_PIN   BUSY_Pin
#define BUSY_PORT  BUSY_GPIO_Port


extern const unsigned char IMAGE_BLACK[];
extern const unsigned char IMAGE_RED[];
extern SPI_HandleTypeDef hspi1; 

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

   // HAL_SPI_Init(); // Call the SPI initialization function
}




int Epd_Init(Epd* epd) {
    epd->width = EPD_WIDTH;
    epd->height = EPD_HEIGHT;
    epd->reset_port = RST_PORT;
    epd->reset_pin = RST_PIN;
    epd->dc_port = DC_PORT;
    epd->dc_pin = DC_PIN;
    epd->cs_port = CS_PORT;
    epd->cs_pin = CS_PIN;
    epd->busy_port = BUSY_PORT;
    epd->busy_pin = BUSY_PIN;
    epd->flag = 0;

    DigitalWrite(epd->dc_port, epd->dc_pin, GPIO_PIN_RESET);
    SpiTransfer(0x2F);
    DelayMs(50);

    DigitalWrite(epd->dc_port, epd->dc_pin, GPIO_PIN_SET);
    SpiTransfer(0x00); // Dummy read

    if (DigitalRead(epd->busy_port, epd->busy_pin) == GPIO_PIN_SET) {
        epd->flag = 0;
        return Epd_Init_new(epd);
    } else {
        epd->flag = 1;
        return Epd_Init_old(epd);
    }
}

int Epd_Init_new(Epd* epd) {
    Epd_Reset(epd);
    Epd_ReadBusy(epd);
    Epd_SendCommand(epd, 0x12);
    Epd_ReadBusy(epd);
    Epd_SendCommand(epd, 0x3C);
    Epd_SendData(epd, 0x05);
    Epd_SendCommand(epd, 0x18);
    Epd_SendData(epd, 0x80);
    Epd_SendCommand(epd, 0x11);
    Epd_SendData(epd, 0x03);
    Epd_SendCommand(epd, 0x44);
    Epd_SendData(epd, 0x00);
    Epd_SendData(epd, epd->width / 8 - 1);
    Epd_SendCommand(epd, 0x45);
    Epd_SendData(epd, 0x00);
    Epd_SendData(epd, 0x00);
    Epd_SendData(epd, (epd->height - 1) % 256);
    Epd_SendData(epd, (epd->height - 1) / 256);
    Epd_SendCommand(epd, 0x4E);
    Epd_SendData(epd, 0x00);
    Epd_SendCommand(epd, 0x4F);
    Epd_SendData(epd, 0x00);
    Epd_SendData(epd, 0x00);
    Epd_ReadBusy(epd);
    return 0;
}

int Epd_Init_old(Epd* epd) {
    Epd_Reset(epd);
    Epd_SendCommand(epd, 0x04);
    Epd_ReadBusy(epd);
    Epd_SendCommand(epd, 0x00);
    Epd_SendData(epd, 0x0F);
    return 0;
}

void Epd_SendCommand(Epd* epd, unsigned char command) {
    DigitalWrite(epd->dc_port, epd->dc_pin, GPIO_PIN_RESET);
    SpiTransfer(command);
}

void Epd_SendData(Epd* epd, unsigned char data) {
    DigitalWrite(epd->dc_port, epd->dc_pin, GPIO_PIN_SET);
    SpiTransfer(data);
}

void Epd_ReadBusy(Epd* epd) {
    if (epd->flag == 0) {
        while (DigitalRead(epd->busy_port, epd->busy_pin) == GPIO_PIN_SET) {
            DelayMs(100);
        }
    } else {
        while (DigitalRead(epd->busy_port, epd->busy_pin) == GPIO_PIN_RESET) {
            DelayMs(100);
        }
    }
}

void Epd_Reset(Epd* epd) {
    DigitalWrite(epd->reset_port, epd->reset_pin, GPIO_PIN_SET);
    DelayMs(200);
    DigitalWrite(epd->reset_port, epd->reset_pin, GPIO_PIN_RESET);
    DelayMs(2);
    DigitalWrite(epd->reset_port, epd->reset_pin, GPIO_PIN_SET);
    DelayMs(200);
}


void Epd_Display_Window_Black(Epd* epd, const UBYTE* image, UBYTE count) {
    if (count == 0 && epd->flag == 0) {
        Epd_SendCommand(epd, 0x24); // full screen (0)
    } else if (count == 0) {
        Epd_SendCommand(epd, 0x10); // alt display (1)
    }

    
    for (UWORD j = 0; j < epd->height; j++) {
        for (UWORD i = 0; i < epd->width / 8; i++) {
            Epd_SendData(epd, image[i + (j * (epd->width / 8))]); // Wysyłanie danych obrazu
        }
    }
}



void Epd_Display_Window_Red(Epd* epd, const UBYTE* image, UBYTE count) {
    if (count == 0 && epd->flag == 0) {
        Epd_SendCommand(epd, 0x26); // full screen (0)
    } else if (count == 0) {
        Epd_SendCommand(epd, 0x13); // alt display (1)
    }

    for (UWORD j = 0; j < epd->height; j++) {
        for (UWORD i = 0; i < epd->width / 8; i++) {
            Epd_SendData(epd, image[i + (j * (epd->width / 8))]); // correct image data for red
        }
    }
}



void Epd_Display(Epd* epd, const UBYTE* blackimage, const UBYTE* ryimage) {
    if (epd->flag == 0) {
        Epd_SendCommand(epd, 0x24);
        for (UWORD j = 0; j < epd->height; j++) {
            for (UWORD i = 0; i < epd->width / 8; i++) {
                Epd_SendData(epd, blackimage[i + (j * epd->width / 8)]);
            }
        }
        Epd_SendCommand(epd, 0x26);
        for (UWORD j = 0; j < epd->height; j++) {
            for (UWORD i = 0; i < epd->width / 8; i++) {
                Epd_SendData(epd, ~ryimage[i + (j * epd->width / 8)]);
            }
        }
        Epd_SendCommand(epd, 0x22);
        Epd_SendData(epd, 0xF7);
        Epd_SendCommand(epd, 0x20);
        Epd_ReadBusy(epd);
    } else {
        Epd_SendCommand(epd, 0x10);
        for (UWORD j = 0; j < epd->height; j++) {
            for (UWORD i = 0; i < epd->width / 8; i++) {
                Epd_SendData(epd, blackimage[i + (j * epd->width / 8)]);
            }
        }
        Epd_SendCommand(epd, 0x13);
        for (UWORD j = 0; j < epd->height; j++) {
            for (UWORD i = 0; i < epd->width / 8; i++) {
                Epd_SendData(epd, ryimage[i + (j * epd->width / 8)]);
            }
        }
        Epd_SendCommand(epd, 0x12);
        DelayMs(100);
        Epd_ReadBusy(epd);
    }
}

void Epd_DisplayFrame(Epd* epd) {
    if (epd->flag == 0) {
        Epd_SendCommand(epd, 0x22);
        Epd_SendData(epd, 0xF7);
        Epd_SendCommand(epd, 0x20);
        Epd_ReadBusy(epd);
    } else {
        Epd_SendCommand(epd, 0x12);
        DelayMs(100);
        Epd_ReadBusy(epd);
    }
}


void Epd_Clear(Epd* epd) { //funnkcja przyjmujaca wskaznik na strukture
    if (epd->flag == 0) {
        Epd_SendCommand(epd, 0x24);
        for (UWORD j = 0; j < epd->height; j++) {
            for (UWORD i = 0; i < epd->width / 8; i++) {
                Epd_SendData(epd, 0xFF);
            }
        }
        Epd_SendCommand(epd, 0x26);
        for (UWORD j = 0; j < epd->height; j++) {
            for (UWORD i = 0; i < epd->width / 8; i++) {
                Epd_SendData(epd, 0x00);
            }
        }
        Epd_SendCommand(epd, 0x22);
        Epd_SendData(epd, 0xF7);
        Epd_SendCommand(epd, 0x20);
        Epd_ReadBusy(epd);
    } else {
        Epd_SendCommand(epd, 0x10);
        for (UWORD j = 0; j < epd->height; j++) {
            for (UWORD i = 0; i < epd->width / 8; i++) {
                Epd_SendData(epd, 0xFF);
            }
        }
        Epd_SendCommand(epd, 0x13);
        for (UWORD j = 0; j < epd->height; j++) {
            for (UWORD i = 0; i < epd->width / 8; i++) {
                Epd_SendData(epd, 0xFF);
            }
        }
        Epd_SendCommand(epd, 0x12);
        DelayMs(100);
        Epd_ReadBusy(epd);
    }
}

void Epd_Sleep(Epd* epd) {
    if (epd->flag == 0) {
        Epd_SendCommand(epd, 0x10);
        Epd_SendData(epd, 0x01);
    } else {
        Epd_SendCommand(epd, 0x50);
        Epd_SendData(epd, 0xF7);
        Epd_SendCommand(epd, 0x02);
        Epd_ReadBusy(epd);
        Epd_SendCommand(epd, 0x07);
        Epd_SendData(epd, 0xA5);
    }
}


