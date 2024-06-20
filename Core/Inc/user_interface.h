#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

#include "main.h"
#include "epdpaint.h"
#define COLORED     0
#define UNCOLORED   1

void UI_HandleButtonPress_1(void); 
void UI_HandleButtonPress_2(void); 
void UI_HandleButtonPress_3(void); 
void UI_HandleButtonPress_4(void); 
void UI_HandleButtonPress_5(void); 
void UI_HandleButtonPress_6(void); 
void UI_HandleButtonPress_7(void); 
void UI_HandleButtonPress_8(void); 

void DisplayTopSection(Paint* paint_top, int iconIndex, uint32_t encoderValue, int counter, uint8_t batteryLevel);
void DisplayMiddleSection(Paint* paint_top);

void DisplayBottomSection(Paint* paint_top, int iconIndex);
void UpdateBatteryLevel(uint8_t* batteryLevel);

int getIconIndex(uint32_t encoderValue);

extern int buttonState;
extern int currentDisplayMode;

#endif // USER_INTERFACE_H
