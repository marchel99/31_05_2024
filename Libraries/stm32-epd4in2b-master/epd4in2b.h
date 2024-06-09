#ifndef EPD4IN2B_H
#define EPD4IN2B_H

#include "stm32l4xx_hal.h"
#include "main.h"

// Display resolution
#define EPD_WIDTH       400
#define EPD_HEIGHT      300

#define UWORD  unsigned int
#define UBYTE  unsigned char

typedef struct {
    unsigned int width;
    unsigned int height;
    GPIO_TypeDef* reset_port;
    uint16_t reset_pin;
    GPIO_TypeDef* dc_port;
    uint16_t dc_pin;
    GPIO_TypeDef* cs_port;
    uint16_t cs_pin;
    GPIO_TypeDef* busy_port;
    uint16_t busy_pin;
    unsigned char flag;
} Epd;

int Epd_Init(Epd* epd);
void Epd_DisplayFrame(Epd* epd);
int Epd_Init_new(Epd* epd);
int Epd_Init_old(Epd* epd);
void Epd_SendCommand(Epd* epd, unsigned char command);
void Epd_SendData(Epd* epd, unsigned char data);
void Epd_ReadBusy(Epd* epd);
void Epd_Reset(Epd* epd);
void Epd_Display_Window_Black(Epd* epd, const UBYTE *image, UBYTE count);
void Epd_Display_Window_Red(Epd* epd, const UBYTE *image, UBYTE count);
void Epd_Display(Epd* epd, const UBYTE *blackimage, const UBYTE *ryimage);
void Epd_Clear(Epd* epd);
void Epd_Sleep(Epd* epd);

#endif /* EPD4IN2B_H */
