/**
 *  @filename   :   epd4in2b.h
 *  @brief      :   Declarations of bitmap icons for the e-ink display project.
 *  @author     :   Created by Marchel99 for the purposes of a master's thesis at the Electronics and
 *                  Telecommunication Department, Lodz University of Technology.
 *  @note       :   Feel free to copy, modify, and distribute this file as you see fit.
 *                  Marchel99 is not responsible for any issues that arise from using this file.
 *                  For improvements or problem reports, please visit: github.com/marchel99
 *  @license    :   This software is licensed under the "Do What the F*ck You Want to Public License" (WTFPL).
 */

#ifndef EPD4IN2B_H
#define EPD4IN2B_H

#include "stm32l4xx_hal.h"
#include "main.h"

// Display resolution
#define EPD_WIDTH 400
#define EPD_HEIGHT 300

#define COLORED      1
#define UNCOLORED    0

#define UWORD unsigned int
#define UBYTE unsigned char

// EPD4IN2 commands
#define PANEL_SETTING 0x00
#define POWER_SETTING 0x01
#define POWER_OFF 0x02
#define POWER_OFF_SEQUENCE_SETTING 0x03
#define POWER_ON 0x04
#define POWER_ON_MEASURE 0x05
#define BOOSTER_SOFT_START 0x06
#define DEEP_SLEEP 0x07
#define DATA_START_TRANSMISSION_1 0x10
#define DATA_STOP 0x11
#define DISPLAY_REFRESH 0x12
#define DATA_START_TRANSMISSION_2 0x13
#define LUT_FOR_VCOM 0x20
#define LUT_WHITE_TO_WHITE 0x21
#define LUT_BLACK_TO_WHITE 0x22
#define LUT_WHITE_TO_BLACK 0x23
#define LUT_BLACK_TO_BLACK 0x24
#define PLL_CONTROL 0x30
#define TEMPERATURE_SENSOR_COMMAND 0x40
#define TEMPERATURE_SENSOR_SELECTION 0x41
#define TEMPERATURE_SENSOR_WRITE 0x42
#define TEMPERATURE_SENSOR_READ 0x43
#define VCOM_AND_DATA_INTERVAL_SETTING 0x50
#define LOW_POWER_DETECTION 0x51
#define TCON_SETTING 0x60
#define RESOLUTION_SETTING 0x61
#define GSST_SETTING 0x65
#define GET_STATUS 0x71
#define AUTO_MEASUREMENT_VCOM 0x80
#define READ_VCOM_VALUE 0x81
#define VCM_DC_SETTING 0x82
#define PARTIAL_WINDOW 0x90
#define PARTIAL_IN 0x91
#define PARTIAL_OUT 0x92
#define PROGRAM_MODE 0xA0
#define ACTIVE_PROGRAMMING 0xA1
#define READ_OTP 0xA2
#define POWER_SAVING 0xE3

extern const unsigned char lut_vcom0[];
extern const unsigned char lut_ww[];
extern const unsigned char lut_bw[];
extern const unsigned char lut_bb[];
extern const unsigned char lut_wb[];

extern const unsigned char lut_vcom0_quick[];
extern const unsigned char lut_ww_quick[];
extern const unsigned char lut_bw_quick[];
extern const unsigned char lut_bb_quick[];
extern const unsigned char lut_wb_quick[];




typedef struct
{
    unsigned int width;
    unsigned int height;
    GPIO_TypeDef *reset_port;
    uint16_t reset_pin;
    GPIO_TypeDef *dc_port;
    uint16_t dc_pin;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    GPIO_TypeDef *busy_port;
    uint16_t busy_pin;
    unsigned char flag;
} Epd;

/* int Epd_Init(Epd* epd);
void Epd_SendCommand(Epd* epd, unsigned char command);
void Epd_SendData(Epd* epd, unsigned char data);
void Epd_WaitUntilIdle(Epd* epd);
void Epd_Reset(Epd* epd);
void Epd_SetPartialWindow(Epd* epd, const unsigned char* buffer, int x, int y, int width, int height);
void SetPartialWindowBlack(Epd* epd, const unsigned char* buffer_black, int x, int y, int w, int l);


void Epd_DisplayFrame(Epd* epd);
void Epd_Clear(Epd* epd);
void Epd_Sleep(Epd* epd);
 */

// SYSTEM
int Epd_Init(Epd *epd);
void Epd_SendCommand(Epd *epd, UBYTE command);
void Epd_SendData(Epd *epd, UBYTE data);
void Epd_WaitUntilIdle(Epd *epd);
void Epd_Reset(Epd *epd);

// PARTIAL
void Epd_SetPartialWindow(Epd* epd, const unsigned char* buffer_black, int x, int y, int w, int l, int dtm);
void SetPartialWindowBlack(Epd *epd, const unsigned char *buffer_black, int x, int y, int w, int l);
void SetPartialWindowRed(Epd *epd, const unsigned char *buffer_red, int x, int y, int w, int l);

void SetLut(Epd* epd);
void SetLutQuick(Epd* epd);
void Epd_Display(Epd *epd, const UBYTE *blackimage, const UBYTE *ryimage);
void Epd_DisplayFrame(Epd* epd, const unsigned char* frame_buffer);
void Epd_DisplayFrameSRAM(Epd *epd);
void Epd_DisplayFrameQuick(Epd *epd);
void Epd_ClearFrame(Epd *epd);
void Epd_Sleep(Epd *epd);

// STM32
void Epd_ReadBusy(Epd *epd);
void DigitalWrite(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
GPIO_PinState DigitalRead(GPIO_TypeDef *port, uint16_t pin);
void SpiTransfer(uint8_t data);
void DelayMs(uint32_t ms);
void EpdIf_IfInit(void);

void Epd_DisplayFrame_Partial(Epd *epd, const unsigned char *image, int x, int y, int width, int height);

#endif /* EPD4IN2B_H */
