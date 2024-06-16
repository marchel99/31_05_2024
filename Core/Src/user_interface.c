#include "user_interface.h"
#include "epd4in2b.h"
#include "fonts.h"
#include "imagedata.h"
#include <stdio.h>

#include "epdpaint.h"

int buttonState = 0;
int lastButtonState = 0;
uint32_t lastDebounceTime = 0;

int currentDisplayMode = COLORED;

extern Paint paint_top;
extern Epd epd;

void UI_HandleButtonPress_1(void)
{
    printf("1 przycisk jest wcisniety!\n");
}

void UI_HandleButtonPress_2(void)
{
    printf("2 przycisk jest wcisniety!\n");
}

void UI_HandleButtonPress_3(void)
{
    printf("3 przycisk jest wcisniety!\n");
}

void UI_HandleButtonPress_4(void)
{
    printf("4 przycisk jest wcisniety!\n");
}

void UI_HandleButtonPress_5(void)
{
    printf("5 przycisk jest wcisniety!\n");
}

void UI_HandleButtonPress_6(void)
{
    printf("6 przycisk jest wcisniety!\n");
}

void UI_HandleButtonPress_7(void)
{
    printf("7 przycisk jest wcisniety!\n");
    Epd_Clear(&epd);
    Epd_DisplayFull(&epd, Paint_GetImage(&paint_top));
    HAL_Delay(100);
}

void UI_HandleButtonPress_8(void)
{
    printf("8 przycisk jest wcisniety!\n");
}

void DisplayTopSection(Paint *paint_top, int iconIndex, uint32_t encoderValue, int counter, uint8_t batteryLevel)
{
    char buffer_top[100];
    snprintf(buffer_top, sizeof(buffer_top), "I%d, E:%d", iconIndex, encoderValue);
    Paint_DrawStringAt(paint_top, 150, 5, buffer_top, &Font20, COLORED);

    snprintf(buffer_top, sizeof(buffer_top), "%d", counter);
    Paint_DrawStringAt(paint_top, 10, 5, buffer_top, &Font20, COLORED);

    DrawBattery(paint_top, 350, 2, 32, 24, COLORED);
    DrawBatteryLevel(paint_top, 350, 2, 30, 24, batteryLevel, COLORED);
}

void DisplayMiddleSection(Paint *paint_top)
{
    // interfejs
    Paint_DrawStringAt(paint_top, 10, 35, "", &Font16, COLORED);
    Paint_DrawRoundedRectangle(paint_top, 10, 50, 190, 170, 10, COLORED);
    Paint_DrawRoundedRectangle(paint_top, 210, 50, 390, 170, 10, COLORED);

    // Tekst w lewym prostokącie
    Paint_DrawStringAt(paint_top, 20, 60, "AQI: 1", &Font16, COLORED);
    Paint_DrawStringAt(paint_top, 20, 80, "TVOC: 44ppm", &Font16, COLORED);
    Paint_DrawStringAt(paint_top, 20, 100, "HCHO: 0.04ppm", &Font16, COLORED);
    Paint_DrawStringAt(paint_top, 20, 120, "CO: <5ppm", &Font16, COLORED);
    Paint_DrawStringAt(paint_top, 20, 140, "CO2: 412ppm", &Font16, COLORED);

    // Tekst w prawym prostokącie
    Paint_DrawStringAt(paint_top, 220, 60, "TEMP: 22 C", &Font16, COLORED);
    Paint_DrawStringAt(paint_top, 220, 80, "HUM: 51%", &Font16, COLORED);
    Paint_DrawStringAt(paint_top, 220, 100, "PRESS: 941 hPa", &Font16, COLORED);
    Paint_DrawStringAt(paint_top, 220, 120, "DP: 12 C", &Font16, COLORED);
}



void DisplayBottomSection(Paint *paint_top, int iconIndex)
{
    int icon_height = 230;
    int desc_offset = 20;
    const char *iconDescriptions[] = {
        "Wykresy",    // Opis dla ikony 1
        "Wilgotnosc", // Opis dla ikony 2
        "Slonce",     // Opis dla ikony 3
        "Lisc",       // Opis dla ikony 4
        "Pomiary",     // Opis dla ikony 5
        "Tryb ciemny",     // Opis dla ikony 6
        "Wiatr",      // Opis dla ikony 7
        "Ustawienia"  // Opis dla ikony 8
    };
    // Upewnij się, że indeks jest w zakresie
    if (iconIndex < 1 || iconIndex > 8)
    {
        return;
    }

    // Wyświetlanie opisu ikony nad bitmapą
    Paint_DrawStringAtCenter(paint_top, icon_height - desc_offset, iconDescriptions[iconIndex - 1], &Font20, 400);
    switch (iconIndex)
    {

    case 1:
        Paint_DrawBitmap(paint_top, icon_temp, 5, icon_height, 48, 48, COLORED);
        Paint_DrawStringAtCenter(paint_top, icon_height - desc_offset, "Wykresy", &Font20, 400);
        break;
    case 2:
        Paint_DrawBitmap(paint_top, icon_humi, 55, icon_height, 48, 48, COLORED);
        break;
    case 3:
        Paint_DrawBitmap(paint_top, icon_sun, 105, icon_height, 48, 48, COLORED);
        break;
    case 4:
        Paint_DrawBitmap(paint_top, icon_leaf, 155, icon_height, 48, 48, COLORED);
        break;
    case 5:
        Paint_DrawBitmap(paint_top, icon_sunset, 205, icon_height, 48, 48, COLORED);
        break;
    case 6:
        Paint_DrawBitmap(paint_top, icon_sunrise, 255, icon_height, 48, 48, COLORED);
        break;
    case 7:
        Paint_DrawBitmap(paint_top, icon_wind, 305, icon_height, 48, 48, COLORED);
        break;
    case 8:
        Paint_DrawBitmap(paint_top, icon_settings, 355, icon_height, 48, 48, COLORED);
        break;
    default:
        break;
    }
}
int getIconIndex(uint32_t encoderValue)
{
    static int lastEncoderValue = 0; // Przechowuje poprzednią wartość enkodera
    static int iconIndex = 1;        // Zakładam, że startujesz od ikony 1

    // Oblicz różnicę wartości enkodera
    int delta = (int)(encoderValue / 2) - lastEncoderValue;

    // Aktualizacja wartości enkodera tylko jeśli zmiana jest w rozsądnym zakresie
    if (abs(delta) > 4)
    {                                               // Ignorowanie dużych zmian (np. teleportacji)
        lastEncoderValue = (int)(encoderValue / 2); // Mimo to aktualizujemy lastEncoderValue
        return iconIndex;
    }

    lastEncoderValue = (int)(encoderValue / 2);

    // Obsługa ruchów
    if (delta > 0)
    { // Ruch w prawo
        iconIndex += delta;
        if (iconIndex > 8)
        {
            iconIndex = 8; // Ogranicz do maksymalnej wartości
        }
    }
    else if (delta < 0)
    { // Ruch w lewo
        iconIndex += delta;
        if (iconIndex < 1)
        {
            iconIndex = 1; // Ogranicz do minimalnej wartości
        }
    }

    return iconIndex;
}

void UpdateBatteryLevel(uint8_t *batteryLevel)
{
    *batteryLevel = (*batteryLevel + 1) % 4;
}
