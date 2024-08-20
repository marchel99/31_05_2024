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

extern Paint paint;
extern Epd epd;

void ShowMenu1(void)
{
    printf("1 przycisk jest wcisniety!\n");
    // Wyświetlenie treści menu 1
    Paint_Clear(&paint, UNCOLORED);
    Paint_DrawStringAtCenter(&paint, EPD_HEIGHT/2, "Hello from 1 menu!", &Font20, 400);
    Epd_Display_Partial_DMA(&epd, Paint_GetImage(&paint), 0, 0, 400, 300);
}

void ShowMenu2(void)
{
    printf("Wyświetlanie Menu 2\n");
    // Wyświetlenie treści menu 2
    Paint_Clear(&paint, UNCOLORED);
    Paint_DrawStringAtCenter(&paint, EPD_HEIGHT/2, "Hello from 2 menu!", &Font20, 400);
    Epd_Display_Partial_DMA(&epd, Paint_GetImage(&paint), 0, 0, 400, 300);
}

void ShowMenu3(void)
{
    printf("Wyświetlanie Menu 3\n");
    // Wyświetlenie treści menu 3
    Paint_Clear(&paint, UNCOLORED);
    Paint_DrawStringAtCenter(&paint, EPD_HEIGHT/2, "Hello from 3 menu!", &Font20, 400);
    Epd_Display_Partial_DMA(&epd, Paint_GetImage(&paint), 0, 0, 400, 300);
}

void ShowMenu4(void)
{
    printf("Wyświetlanie Menu 4\n");
    // Wyświetlenie treści menu 4
    Paint_Clear(&paint, UNCOLORED);
    Paint_DrawStringAtCenter(&paint, EPD_HEIGHT/2, "Hello from 4 menu!", &Font20, 400);
    Epd_Display_Partial_DMA(&epd, Paint_GetImage(&paint), 0, 0, 400, 300);
}

void ShowMenu5(void)
{
    printf("Wyświetlanie Menu 5\n");
    // Wyświetlenie treści menu 5
    Paint_Clear(&paint, UNCOLORED);
    Paint_DrawStringAtCenter(&paint, EPD_HEIGHT/2, "Hello from 5 menu!", &Font20, 400);
    Epd_Display_Partial_DMA(&epd, Paint_GetImage(&paint), 0, 0, 400, 300);
}

void ShowMenu6(void)
{
    printf("Wyświetlanie Menu 6\n");
    // Wyświetlenie treści menu 6
    Paint_Clear(&paint, UNCOLORED);
    Paint_DrawStringAtCenter(&paint, EPD_HEIGHT/2, "Hello from 6 menu!", &Font20, 400);
    Epd_Display_Partial_DMA(&epd, Paint_GetImage(&paint), 0, 0, 400, 300);
}

void ShowMenu7(void)
{
    printf("Wyświetlanie Menu 7\n");
    // Wyświetlenie treści menu 7
    Paint_Clear(&paint, UNCOLORED);
    Paint_DrawStringAtCenter(&paint, EPD_HEIGHT/2, "Hello from 7 menu!", &Font20, 400);
    Epd_Display_Partial_DMA(&epd, Paint_GetImage(&paint), 0, 0, 400, 300);
}

void ShowMenu8(void)
{
    printf("Wyświetlanie Menu 8\n");
    // Wyświetlenie treści menu 8
    Paint_Clear(&paint, UNCOLORED);
    Paint_DrawStringAtCenter(&paint, EPD_HEIGHT/2, "Hello from 8 menu!", &Font20, 400);
    Epd_Display_Partial_DMA(&epd, Paint_GetImage(&paint), 0, 0, 400, 300);
}












void DisplayTopSection(Paint *paint, int iconIndex, uint32_t encoderValue, int counter, uint8_t batteryLevel)
{
    char buffer_top[100];
    snprintf(buffer_top, sizeof(buffer_top), "I%d, E:%d", iconIndex, encoderValue);
    Paint_DrawStringAt(paint, 150, 5, buffer_top, &Font20, COLORED);

    snprintf(buffer_top, sizeof(buffer_top), "%d", counter);
    Paint_DrawStringAt(paint, 10, 5, buffer_top, &Font20, COLORED);

    DrawBattery(paint, 350, 2, 32, 24, batteryLevel,COLORED);
  //  DrawBatteryLevel(paint, 350, 2, 30, 24, batteryLevel, COLORED);
}

void DisplayMiddleSection(Paint *paint)
{




// Wywołanie funkcji 10 jednostek poniżej y0_bottom
//Paint_Draw3RectanglesCenter(&paint, y0_bottom + height + 10, r_height, vertical_gap, thickness, COLORED, x0_left, x0_right + width);

// Rysowanie linii poziomej
//Paint_DrawLineWithThickness(&paint, x0_left, y0_bottom + height + 10 + r_height + vertical_gap, x0_right + width, y0_bottom + height + 10 + r_height + vertical_gap, thickness, COLORED);

// Rysowanie pionowej linii




}



void DisplayBottomSection(Paint *paint, int iconIndex)
{
    int icon_height = 245;
    int desc_offset = 11;
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
    Paint_DrawStringAtCenter(paint, icon_height - desc_offset, iconDescriptions[iconIndex - 1], &Font20, 400);
    switch (iconIndex)
    {

    case 1:
        Paint_DrawBitmap(paint, icon_temp, 5, icon_height, 48, 48, COLORED);
        Paint_DrawStringAtCenter(paint, icon_height - desc_offset, "Wykresy", &Font20, 400);
        break;
    case 2:
        Paint_DrawBitmap(paint, icon_humi, 55, icon_height, 48, 48, COLORED);
        break;
    case 3:
        Paint_DrawBitmap(paint, icon_sun, 105, icon_height, 48, 48, COLORED);
        break;
    case 4:
        Paint_DrawBitmap(paint, icon_leaf, 155, icon_height, 48, 48, COLORED);
        break;
    case 5:
        Paint_DrawBitmap(paint, icon_sunset, 205, icon_height, 48, 48, COLORED);
        break;
    case 6:
        Paint_DrawBitmap(paint, icon_sunrise, 255, icon_height, 48, 48, COLORED);
        break;
    case 7:
        Paint_DrawBitmap(paint, icon_wind, 305, icon_height, 48, 48, COLORED);
        break;
    case 8:
        Paint_DrawBitmap(paint, icon_settings, 355, icon_height, 48, 48, COLORED);
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
    int delta = (int)(encoderValue ) - lastEncoderValue;

    // Aktualizacja wartości enkodera tylko jeśli zmiana jest w rozsądnym zakresie
    if (abs(delta) > 12)
    {                                               // Ignorowanie dużych zmian (np. teleportacji)
        lastEncoderValue = (int)(encoderValue ); // Mimo to aktualizujemy lastEncoderValue
        return iconIndex;
    }

    lastEncoderValue = (int)(encoderValue );

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