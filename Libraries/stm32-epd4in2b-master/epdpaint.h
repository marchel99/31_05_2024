#ifndef _EPDPAINT_H_
#define _EPDPAINT_H_

#define COLORED     1
#define UNCOLORED   0



#include "fonts.h" // Dodanie tego nagłówka, aby użyć sFONT z tego pliku



typedef struct {
    unsigned char* image;
    int width;
    int height;
    int rotate;
} Paint;




void Paint_Init(Paint* paint, unsigned char* image, int width, int height);
void Paint_Clear(Paint* paint, int colored);
void Paint_DrawPixel(Paint* paint, int x, int y, int colored);
void Paint_DrawCharAt(Paint* paint, int x, int y, char ascii_char, sFONT* font, int colored);
void Paint_DrawStringAt(Paint* paint, int x, int y, const char* text, sFONT* font, int colored);
void Paint_DrawHorizontalLine(Paint* paint, int x, int y, int width, int colored);
void Paint_DrawVerticalLine(Paint* paint, int x, int y, int height, int colored);
void Paint_DrawRectangle(Paint* paint, int x0, int y0, int x1, int y1, int colored);
void Paint_DrawRoundedRectangle(Paint* paint, int x0, int y0, int x1, int y1, int r, int colored);
void Paint_DrawFilledRectangle(Paint* paint, int x0, int y0, int x1, int y1, int colored);
void Paint_DrawLine(Paint* paint, int x0, int y0, int x1, int y1, int colored);
void Paint_DrawImage(Paint* paint, const unsigned char* image_buffer, int x, int y, int image_width, int image_height);
void Paint_DrawCircle(Paint* paint, int x, int y, int radius, int colored);
void Paint_DrawFilledCircle(Paint* paint, int x, int y, int radius, int colored);

void Paint_DrawCircleQuarter(Paint* paint, int x, int y, int r, int quarter, int colored);

void DrawBattery(Paint* paint, int x, int y, int width, int height, int colored);
void DrawBatteryLevel(Paint* paint, int x, int y, int width, int height, int level, int colored);


void Paint_DrawBitmap(Paint* paint, const unsigned char* bitmap, int x, int y, int width, int height, int colored);




void DrawIcon(Paint *paint, const unsigned char *icon, int x, int y, int width, int height, int selected);










void Paint_SetWidth(Paint* paint, int width);
void Paint_SetHeight(Paint* paint, int height);
int Paint_GetWidth(Paint* paint);
int Paint_GetHeight(Paint* paint);
void Paint_SetRotate(Paint* paint, int rotate);
int Paint_GetRotate(Paint* paint);
unsigned char* Paint_GetImage(Paint* paint);

#endif /* _EPDPAINT_H_ */
