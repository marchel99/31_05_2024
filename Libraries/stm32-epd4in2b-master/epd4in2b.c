


#include "epd4in2b.h"
#include "stm32l4xx_hal.h"
#include <stdio.h>
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


void Epd_ReadBusy(Epd* epd) {
    while (HAL_GPIO_ReadPin(epd->busy_port, epd->busy_pin) == GPIO_PIN_RESET) {  // 0: busy, 1: idle
        HAL_Delay(1);  // Short delay to avoid busy-waiting
    }
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



/* 
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
 */
/* 


void Epd_DisplayFrame_Partial(Epd* epd, const unsigned char* image, int x, int y, int width, int height) {

    printf("Partial Window: (%d, %d), %dx%d\n", x, y, width, height);


// Debug: Sprawdzenie danych obrazu przed przesłaniem
    printf("Data to be sent:\n");
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < (width / 8); i++) {
            printf("%02X ", image[i + j * (width / 8)]);
        }
        printf("\n");
    }




    // Enter partial mode
    Epd_SendCommand(epd, 0x91); // Partial In (PTIN)

    // Set partial window
    Epd_SendCommand(epd, 0x90); // Partial Window (PTL)
    Epd_SendData(epd, (x >> 8) & 0xFF);               // HRST[8:3]
    Epd_SendData(epd, x & 0xFF);                      // HRST[2:0]
    Epd_SendData(epd, ((x + width - 1) >> 8) & 0xFF); // HRED[8:3]
    Epd_SendData(epd, (x + width - 1) & 0xFF);        // HRED[2:0]
    Epd_SendData(epd, (y >> 8) & 0xFF);               // VRST[8:0]
    Epd_SendData(epd, y & 0xFF);
    Epd_SendData(epd, ((y + height - 1) >> 8) & 0xFF); // VRED[8:0]
    Epd_SendData(epd, (y + height - 1) & 0xFF);
    Epd_SendData(epd, 0x01); // PT_SCAN (1: Gates scan both inside and outside of the partial window)

    // Write RAM for black/white data
    Epd_SendCommand(epd, 0x10); // Data Start Transmission 1
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < (width / 8); i++) {
            Epd_SendData(epd, image[i + j * (width / 8)]);
        }
    }

    // Debug: Sprawdzenie danych obrazu, które są przesyłane
    printf("Data being sent:\n");
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < (width / 8); i++) {
            printf("%02X ", image[i + j * (width / 8)]);
        }
        printf("\n");
    }

    // Refresh display
    Epd_SendCommand(epd, 0x12); // Display Refresh (DRF)
    Epd_ReadBusy(epd); // Wait until the busy signal goes LOW

    // Exit partial mode
    Epd_SendCommand(epd, 0x92); // Partial Out (PTOUT)
}


 */



















 /* this calls the peripheral hardware interface*/
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








 /**
 *  @brief: basic function for sending commands
 */
void Epd_SendCommand(Epd* epd, unsigned char command) {
    DigitalWrite(epd->dc_port, epd->dc_pin, GPIO_PIN_RESET);
    SpiTransfer(command);
}


 /**
 *  @brief: basic function for sending data
 */
void Epd_SendData(Epd* epd, unsigned char data) {
    DigitalWrite(epd->dc_port, epd->dc_pin, GPIO_PIN_SET);
    SpiTransfer(data);
}




 /**
 *  @brief: Wait until the busy_pin goes HIGH
 */

void Epd_WaitUntilIdle(Epd* epd) {
    while (DigitalRead(epd->busy_port, epd->busy_pin) == GPIO_PIN_RESET) { // 0: busy, 1: idle
        DelayMs(100);
    }
}





 /**
 *  @brief: module reset. 
 *          often used to awaken the module in deep sleep, 
 *          see Epd::Sleep();
 */
void Epd_Reset(Epd* epd) {
    DigitalWrite(epd->reset_port, epd->reset_pin, GPIO_PIN_SET);
    DelayMs(200);
    DigitalWrite(epd->reset_port, epd->reset_pin, GPIO_PIN_RESET);
    DelayMs(2);
    DigitalWrite(epd->reset_port, epd->reset_pin, GPIO_PIN_SET);
    DelayMs(200);
}








/**
 *  @brief: transmit partial data to the SRAM. The final parameter chooses between dtm=1 and dtm=2
 */
void Epd_SetPartialWindow(Epd* epd, const unsigned char* buffer_black, int x, int y, int w, int l, int dtm) {
    Epd_SendCommand(epd, PARTIAL_IN);
    Epd_SendCommand(epd, PARTIAL_WINDOW);
    Epd_SendData(epd, x >> 8);
    Epd_SendData(epd, x & 0xf8);     // x should be the multiple of 8, the last 3 bit will always be ignored
    Epd_SendData(epd, ((x & 0xf8) + w - 1) >> 8);
    Epd_SendData(epd, ((x & 0xf8) + w - 1) | 0x07);
    Epd_SendData(epd, y >> 8);
    Epd_SendData(epd, y & 0xff);
    Epd_SendData(epd, (y + l - 1) >> 8);
    Epd_SendData(epd, (y + l - 1) & 0xff);
    Epd_SendData(epd, 0x01);  // Gates scan both inside and outside of the partial window (default)
    // DelayMs(2);
    Epd_SendCommand(epd, (dtm == 1) ? DATA_START_TRANSMISSION_1 : DATA_START_TRANSMISSION_2);
    if (buffer_black != NULL) {
        for (int i = 0; i < w / 8 * l; i++) {
            Epd_SendData(epd, buffer_black[i]);
        }
    } else {
        for (int i = 0; i < w / 8 * l; i++) {
            Epd_SendData(epd, 0x00);
        }
    }
    // DelayMs(2);
    Epd_SendCommand(epd, PARTIAL_OUT);
}



/**
 *  @brief: transmit partial black data to the SRAM
 */
void SetPartialWindowBlack(Epd *epd, const unsigned char *buffer_black, int x, int y, int w, int l) {
    Epd_SendCommand(epd, PARTIAL_IN);
    Epd_SendCommand(epd, PARTIAL_WINDOW);
    Epd_SendData(epd, x >> 8);
    Epd_SendData(epd, x & 0xf8);     // x should be the multiple of 8, the last 3 bit will always be ignored
    Epd_SendData(epd, ((x & 0xf8) + w  - 1) >> 8);
    Epd_SendData(epd, ((x & 0xf8) + w  - 1) | 0x07);
    Epd_SendData(epd, y >> 8);        
    Epd_SendData(epd, y & 0xff);
    Epd_SendData(epd, (y + l - 1) >> 8);        
    Epd_SendData(epd, (y + l - 1) & 0xff);
    Epd_SendData(epd, 0x01);         // Gates scan both inside and outside of the partial window. (default) 
    DelayMs(2);
    Epd_SendCommand(epd, DATA_START_TRANSMISSION_1);
    if (buffer_black != NULL) {
        for(int i = 0; i < w  / 8 * l; i++) {
            Epd_SendData(epd, buffer_black[i]);  
        }  
    } else {
        for(int i = 0; i < w  / 8 * l; i++) {
            Epd_SendData(epd, 0x00);  
        }  
    }
    DelayMs(2);
    Epd_SendCommand(epd, PARTIAL_OUT);  
}

/**
 *  @brief: transmit partial red data to the SRAM
 */
void SetPartialWindowRed(Epd *epd, const unsigned char *buffer_red, int x, int y, int w, int l) {
    Epd_SendCommand(epd, PARTIAL_IN);
    Epd_SendCommand(epd, PARTIAL_WINDOW);
    Epd_SendData(epd, x >> 8);
    Epd_SendData(epd, x & 0xf8);     // x should be the multiple of 8, the last 3 bit will always be ignored
    Epd_SendData(epd, ((x & 0xf8) + w  - 1) >> 8);
    Epd_SendData(epd, ((x & 0xf8) + w  - 1) | 0x07);
    Epd_SendData(epd, y >> 8);        
    Epd_SendData(epd, y  & 0xff);
    Epd_SendData(epd, (y + l - 1) >> 8);        
    Epd_SendData(epd, (y + l - 1) & 0xff);
    Epd_SendData(epd, 0x01);         // Gates scan both inside and outside of the partial window. (default) 
    DelayMs(2);
    Epd_SendCommand(epd, DATA_START_TRANSMISSION_2);
    if (buffer_red != NULL) {
        for(int i = 0; i < w  / 8 * l; i++) {
            Epd_SendData(epd, buffer_red[i]);  
        }  
    } else {
        for(int i = 0; i < w  / 8 * l; i++) {
            Epd_SendData(epd, 0x00);  
        }  
    }
    DelayMs(2);
    Epd_SendCommand(epd, PARTIAL_OUT);  
}




























/**
 *  @brief: set the look-up table
*/
void SetLut(Epd* epd) {
    unsigned int count;
    Epd_SendCommand(epd, LUT_FOR_VCOM); //vcom
    for(count = 0; count < 44; count++) {
        Epd_SendData(epd, lut_vcom0[count]);
    }

    Epd_SendCommand(epd, LUT_WHITE_TO_WHITE); //ww --
    for(count = 0; count < 42; count++) {
        Epd_SendData(epd, lut_ww[count]);
    }

    Epd_SendCommand(epd, LUT_BLACK_TO_WHITE); //bw r
    for(count = 0; count < 42; count++) {
        Epd_SendData(epd, lut_bw[count]);
    }

    Epd_SendCommand(epd, LUT_WHITE_TO_BLACK); //wb w
    for(count = 0; count < 42; count++) {
        Epd_SendData(epd, lut_bb[count]);
    }

    Epd_SendCommand(epd, LUT_BLACK_TO_BLACK); //bb b
    for(count = 0; count < 42; count++) {
        Epd_SendData(epd, lut_wb[count]);
    }
}





void SetLutQuick(Epd *epd) {
    unsigned int count;
    Epd_SendCommand(epd, LUT_FOR_VCOM); // vcom
    for (count = 0; count < 44; count++) {
        Epd_SendData(epd, lut_vcom0_quick[count]);
    }

    Epd_SendCommand(epd, LUT_WHITE_TO_WHITE); // ww
    for (count = 0; count < 42; count++) {
        Epd_SendData(epd, lut_ww_quick[count]);
    }

    Epd_SendCommand(epd, LUT_BLACK_TO_WHITE); // bw
    for (count = 0; count < 42; count++) {
        Epd_SendData(epd, lut_bw_quick[count]);
    }

    Epd_SendCommand(epd, LUT_WHITE_TO_BLACK); // wb
    for (count = 0; count < 42; count++) {
        Epd_SendData(epd, lut_wb_quick[count]);
    }

    Epd_SendCommand(epd, LUT_BLACK_TO_BLACK); // bb
    for (count = 0; count < 42; count++) {
        Epd_SendData(epd, lut_bb_quick[count]);
    }
}














/**
 * @brief: refresh and displays the frame
*/
void Epd_DisplayFrame(Epd* epd, const unsigned char* frame_buffer) {
    Epd_SendCommand(epd, RESOLUTION_SETTING);
    Epd_SendData(epd, epd->width >> 8);        
    Epd_SendData(epd, epd->width & 0xff);
    Epd_SendData(epd, epd->height >> 8);
    Epd_SendData(epd, epd->height & 0xff);

    Epd_SendCommand(epd, VCM_DC_SETTING);
    Epd_SendData(epd, 0x12);                   

    Epd_SendCommand(epd, VCOM_AND_DATA_INTERVAL_SETTING);
    Epd_SendData(epd, 0x97);    // VBDF 17|D7 VBDW 97  VBDB 57  VBDF F7  VBDW 77  VBDB 37  VBDR B7

    if (frame_buffer != NULL) {
        Epd_SendCommand(epd, DATA_START_TRANSMISSION_1);
        for (unsigned int i = 0; i < epd->width / 8 * epd->height; i++) {
            Epd_SendData(epd, 0xFF);  // bit set: white, bit reset: black
        }
        DelayMs(2);
        Epd_SendCommand(epd, DATA_START_TRANSMISSION_2); 
        for (unsigned int i = 0; i < epd->width / 8 * epd->height; i++) {
            Epd_SendData(epd, frame_buffer[i]);
        }  
        DelayMs(2);                  
    }

    SetLut(epd);

    Epd_SendCommand(epd, DISPLAY_REFRESH); 
    DelayMs(100);
    Epd_ReadBusy(epd);
}












/**
 * @brief: clear the frame data from the SRAM, this won't refresh the display
 */

void Epd_ClearFrame(Epd* epd) {
    Epd_SendCommand(epd, RESOLUTION_SETTING);
    Epd_SendData(epd, epd->width >> 8);
    Epd_SendData(epd, epd->width & 0xff);
    Epd_SendData(epd, epd->height >> 8);
    Epd_SendData(epd, epd->height & 0xff);

    Epd_SendCommand(epd, DATA_START_TRANSMISSION_1);
    DelayMs(2);
    for (unsigned int i = 0; i < epd->width / 8 * epd->height; i++) {
        Epd_SendData(epd, 0xFF);
    }
    DelayMs(2);
    Epd_SendCommand(epd, DATA_START_TRANSMISSION_2);
    DelayMs(2);
    for (unsigned int i = 0; i < epd->width / 8 * epd->height; i++) {
        Epd_SendData(epd, 0xFF);
    }
    DelayMs(2);
}


/**
 * @brief: This displays the frame data from SRAM
 */
void Epd_DisplayFrameSRAM(Epd* epd) {
  SetLut(epd);
    Epd_SendCommand(epd, DISPLAY_REFRESH); 
    DelayMs(100);
    Epd_ReadBusy(epd);
    }

void Epd_DisplayFrameQuick(Epd *epd) {
    SetLutQuick(epd);
    Epd_SendCommand(epd, DISPLAY_REFRESH);
    // DelayMs(100);
    // Epd_ReadBusy(epd);
}







/**
 * @brief: After this command is transmitted, the chip would enter the deep-sleep mode to save power. 
 *         The deep sleep mode would return to standby by hardware reset. The only one parameter is a 
 *         check code, the command would be executed if check code = 0xA5. 
 *         You can use Epd::Reset() to awaken and use Epd::Init() to initialize.
 */

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




const unsigned char lut_vcom0[] = {
    0x00, 0x17, 0x00, 0x00, 0x00, 0x02,        
0x00, 0x17, 0x17, 0x00, 0x00, 0x02,        
0x00, 0x0A, 0x01, 0x00, 0x00, 0x01,        
0x00, 0x0E, 0x0E, 0x00, 0x00, 0x02,        
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,        
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,        
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
const unsigned char lut_ww[] = {
    0x40, 0x17, 0x00, 0x00, 0x00, 0x02,
0x90, 0x17, 0x17, 0x00, 0x00, 0x02,
0x40, 0x0A, 0x01, 0x00, 0x00, 0x01,
0xA0, 0x0E, 0x0E, 0x00, 0x00, 0x02,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
const unsigned char lut_bw[] = {
   0x40, 0x17, 0x00, 0x00, 0x00, 0x02,
0x90, 0x17, 0x17, 0x00, 0x00, 0x02,
0x40, 0x0A, 0x01, 0x00, 0x00, 0x01,
0xA0, 0x0E, 0x0E, 0x00, 0x00, 0x02,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
const unsigned char lut_bb[] = {
  0x80, 0x17, 0x00, 0x00, 0x00, 0x02,
0x90, 0x17, 0x17, 0x00, 0x00, 0x02,
0x80, 0x0A, 0x01, 0x00, 0x00, 0x01,
0x50, 0x0E, 0x0E, 0x00, 0x00, 0x02,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
const unsigned char lut_wb[] = {
   0x80, 0x17, 0x00, 0x00, 0x00, 0x02,
0x90, 0x17, 0x17, 0x00, 0x00, 0x02,
0x80, 0x0A, 0x01, 0x00, 0x00, 0x01,
0x50, 0x0E, 0x0E, 0x00, 0x00, 0x02,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            
};




const unsigned char lut_vcom0_quick[] = {
    0x00, 0x0E, 0x00, 0x00, 0x00, 0x01,        
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,        
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,        
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,        
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,        
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,        
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

const unsigned char lut_ww_quick[] = {
    0xA0, 0x0E, 0x00, 0x00, 0x00, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};


const unsigned char lut_bw_quick[] = {
    0xA0, 0x0E, 0x00, 0x00, 0x00, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

const unsigned char lut_bb_quick[] = {
    0x50, 0x0E, 0x00, 0x00, 0x00, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

const unsigned char lut_wb_quick[] = {
    0x50, 0x0E, 0x00, 0x00, 0x00, 0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};