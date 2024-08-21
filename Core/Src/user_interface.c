#include "user_interface.h"
#include "epd4in2b.h"
#include "fonts.h"
#include "imagedata.h"
#include <stdio.h>
#include "globals.h"
#include <string.h>

#include "epdpaint.h"

int buttonState = 0;
int lastButtonState = 0;
uint32_t lastDebounceTime = 0;

int currentDisplayMode = COLORED;

extern RTC_HandleTypeDef hrtc;
extern RTC_TimeTypeDef time;
extern RTC_DateTypeDef date;

extern Paint paint;
extern Epd epd;

extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim2;

void ShowMenu1(void)
{
    printf("1 przycisk jest wcisniety!\n");

    // Inicjalizacja licznika
    int counter = 1;
    char buffer[50]; // Bufor na tekst do wyświetlenia

    // Pętla while, która będzie działać dopóki jesteśmy w menu
    while (inMenu)
    {
        // Czyszczenie ekranu
        Paint_Clear(&paint, UNCOLORED);

        // Generowanie tekstu z aktualną wartością licznika
        snprintf(buffer, sizeof(buffer), "Counter: %d", counter);

        // Wyświetlanie licznika na ekranie
        Paint_DrawStringAtCenter(&paint, 50, buffer, &Font20, 400);

        Epd_Display_Partial_DMA(&epd, Paint_GetImage(&paint), 0, 0, 400, 300);

        // Zwiększenie licznika
        counter++;

        // Opóźnienie, aby zmiany były widoczne
        HAL_Delay(1000);
    }
}

void ShowMenu2(void)
{
    while (inMenu)
    {

        HAL_Delay(200);
    }
}



int encoderPosition = 0; // Zmienna globalna, inicjalizowana przy starcie

void ShowMenu3(void)
{
    printf("3 przycisk jest wcisniety!\n");

    encoderPosition = 0; // Ustaw wartość na 0 przy wejściu do funkcji

    uint32_t previousEncoderValue = __HAL_TIM_GET_COUNTER(&htim2); // Inicjalizacja zmiennej

    while (inMenu)
    {
        Paint_Clear(&paint, UNCOLORED);

        uint32_t encoderValue = __HAL_TIM_GET_COUNTER(&htim2);
        int encoderDirection = encoderValue > previousEncoderValue ? 1 : (encoderValue < previousEncoderValue ? -1 : 0);
        previousEncoderValue = encoderValue;

        // Aktualizuj encoderPosition z zawijaniem
        if (encoderDirection != 0) {
            encoderPosition += encoderDirection;

            if (encoderPosition < 1) {
                encoderPosition = 5; // Zawijanie z 1 na 5
            } else if (encoderPosition > 5) {
                encoderPosition = 1; // Zawijanie z 5 na 1
            }
        }

        char buffer_top[100];
        snprintf(buffer_top, sizeof(buffer_top), "E:%d",  encoderPosition);
        Paint_DrawStringAt(&paint, 150, 5, buffer_top, &Font20, COLORED);

        Paint_DrawStringAtCenter(&paint, 50, "USTAW GODZINE", &Font20, 400);

        Paint_DrawStringAt(&paint, 170, 100, "00:", &Font20, COLORED);
        Paint_DrawStringAt(&paint, 215, 100, "54", &Font20, COLORED);

        Paint_DrawStringAtCenter(&paint, 150, "USTAW DATE", &Font20, 400);

        Paint_DrawStringAt(&paint, 130, 200, "21", &Font20, COLORED);
        Paint_DrawStringAt(&paint, 150, 200, " sie", &Font20, COLORED);
        Paint_DrawStringAt(&paint, 200, 200, " 2024", &Font20, COLORED);

        Epd_Display_Partial_DMA(&epd, Paint_GetImage(&paint), 0, 0, 400, 300);



HAL_Delay(50);
  switch (encoderPosition)
        {
                 case 0:
                 break;
            case 1:
                Paint_DrawStringAt(&paint, 170, 100, "00:", &Font20, UNCOLORED);
                break;
            case 2:
                Paint_DrawStringAt(&paint, 215, 100, "54", &Font20, UNCOLORED);
                break;
            case 3:
                Paint_DrawStringAt(&paint, 130, 200, "21", &Font20, UNCOLORED);
                break;
            case 4:
                Paint_DrawStringAt(&paint, 150, 200, " sie", &Font20, UNCOLORED);
                break;
            case 5:
                Paint_DrawStringAt(&paint, 200, 200, " 2024", &Font20, UNCOLORED);
                break;
            default:
                break;
        }


      Epd_Display_Partial_DMA(&epd, Paint_GetImage(&paint), 0, 0, 400, 300);





    }
}





void ShowMenu4(void)
{
    printf("Wyświetlanie Menu 4\n");
    // Wyświetlenie treści menu 4
    Paint_Clear(&paint, UNCOLORED);
    Paint_DrawStringAtCenter(&paint, EPD_HEIGHT / 2, "Hello from 4 menu!", &Font20, 400);
    Epd_Display_Partial_DMA(&epd, Paint_GetImage(&paint), 0, 0, 400, 300);
}

void ShowMenu5(void)
{
    printf("Wyświetlanie Menu 5\n");
    // Wyświetlenie treści menu 5
    Paint_Clear(&paint, UNCOLORED);
    Paint_DrawStringAtCenter(&paint, EPD_HEIGHT / 2, "Hello from 5 menu!", &Font20, 400);
    Epd_Display_Partial_DMA(&epd, Paint_GetImage(&paint), 0, 0, 400, 300);
}

void ShowMenu6(void)
{
    printf("Wyświetlanie Menu 6\n");
    // Wyświetlenie treści menu 6
    Paint_Clear(&paint, UNCOLORED);
    Paint_DrawStringAtCenter(&paint, EPD_HEIGHT / 2, "Hello from 6 menu!", &Font20, 400);
    Epd_Display_Partial_DMA(&epd, Paint_GetImage(&paint), 0, 0, 400, 300);
}

void ShowMenu7(void)
{
    printf("Wyświetlanie Menu 7\n");
    // Wyświetlenie treści menu 7
    Paint_Clear(&paint, UNCOLORED);
    Paint_DrawStringAtCenter(&paint, EPD_HEIGHT / 2, "Hello from 7 menu!", &Font20, 400);
    Epd_Display_Partial_DMA(&epd, Paint_GetImage(&paint), 0, 0, 400, 300);
}

void ShowMenu8(void)
{
    printf("Wyświetlanie Menu 8\n");
    // Wyświetlenie treści menu 8
    Paint_Clear(&paint, UNCOLORED);
    Paint_DrawStringAtCenter(&paint, EPD_HEIGHT / 2, "Hello from 8 menu!", &Font20, 400);
    Epd_Display_Partial_DMA(&epd, Paint_GetImage(&paint), 0, 0, 400, 300);
}

void DisplayTopSection(Paint *paint, int iconIndex, uint32_t encoderValue, int counter, uint8_t batteryLevel)
{
    // Pobierz czas i datę
    HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);

    // Formatowanie daty i czasu
    char dateStr[30];
    snprintf(dateStr, sizeof(dateStr), "%02d %s", date.Date, getMonthStr(date.Month));

    char timeStr[30];
    snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", time.Hours, time.Minutes, time.Seconds);

    char yearStr[10];
    snprintf(yearStr, sizeof(yearStr), "%04d", 2000 + date.Year);

    // Wyświetl datę po lewej stronie
    Paint_DrawStringAt(paint, 10, 2, dateStr, &Font20, COLORED);

    // Wyświetl czas na środku na górze
    Paint_DrawStringAtCenter(paint, 10, timeStr, &Font20, 400);

    Paint_DrawStringAt(paint, 10, 20, yearStr, &Font20, COLORED);

    // Narysuj ikonę baterii na górze po prawej stronie
    DrawBattery(paint, 350, 7, 32, 19, batteryLevel, COLORED);
}

const char *getMonthStr(uint8_t month)
{
    switch (month)
    {
    case 1:
        return "sty";
    case 2:
        return "lut";
    case 3:
        return "mar";
    case 4:
        return "kwi";
    case 5:
        return "maj";
    case 6:
        return "cze";
    case 7:
        return "lip";
    case 8:
        return "sie";
    case 9:
        return "wrz";
    case 10:
        return "paź";
    case 11:
        return "lis";
    case 12:
        return "gru";
    default:
        return "???";
    }
}

void DisplayMiddleSection(Paint *paint)
{

    // Wywołanie funkcji 10 jednostek poniżej y0_bottom
    // Paint_Draw3RectanglesCenter(&paint, y0_bottom + height + 10, r_height, vertical_gap, thickness, COLORED, x0_left, x0_right + width);

    // Rysowanie linii poziomej
    // Paint_DrawLineWithThickness(&paint, x0_left, y0_bottom + height + 10 + r_height + vertical_gap, x0_right + width, y0_bottom + height + 10 + r_height + vertical_gap, thickness, COLORED);

    // Rysowanie pionowej linii
}

void DisplayBottomSection(Paint *paint, int iconIndex)
{
    int icon_height = 245;
    int desc_offset = 11;
    const char *iconDescriptions[] = {
        "Wykresy",     // Opis dla ikony 1
        "Wilgotnosc",  // Opis dla ikony 2
        "Slonce",      // Opis dla ikony 3
        "Lisc",        // Opis dla ikony 4
        "Pomiary",     // Opis dla ikony 5
        "Tryb ciemny", // Opis dla ikony 6
        "Wiatr",       // Opis dla ikony 7
        "Ustawienia"   // Opis dla ikony 8
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
    int delta = (int)(encoderValue)-lastEncoderValue;

    // Aktualizacja wartości enkodera tylko jeśli zmiana jest w rozsądnym zakresie
    if (abs(delta) > 12)
    {                                           // Ignorowanie dużych zmian (np. teleportacji)
        lastEncoderValue = (int)(encoderValue); // Mimo to aktualizujemy lastEncoderValue
        return iconIndex;
    }

    lastEncoderValue = (int)(encoderValue);

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