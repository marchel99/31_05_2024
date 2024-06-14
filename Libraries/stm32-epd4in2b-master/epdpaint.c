#include "epdpaint.h"
#include <stdlib.h>
#include <string.h>



void Paint_Init(Paint* paint, unsigned char* image, int width, int height) {
    paint->image = image;
    paint->width = (width % 8) ? (width + 8 - (width % 8)) : width;
    paint->height = height;
    paint->rotate = 0;
}

void Paint_Clear(Paint* paint, int colored) {
    for (int x = 0; x < paint->width; x++) {
        for (int y = 0; y < paint->height; y++) {
            Paint_DrawPixel(paint, x, y, colored);
        }
    }
}

void Paint_DrawPixel(Paint* paint, int x, int y, int colored) {
    if (x < 0 || x >= paint->width || y < 0 || y >= paint->height) {
        return;
    }

    int new_x = x;
    int new_y = y;

    switch (paint->rotate) {
        case 90:
            new_x = paint->width - 1 - y;
            new_y = x;
            break;
        case 180:
            new_x = paint->width - 1 - x;
            new_y = paint->height - 1 - y;
            break;
        case 270:
            new_x = y;
            new_y = paint->height - 1 - x;
            break;
    }

    if (colored) {
        paint->image[(new_x + new_y * paint->width) / 8] &= ~(0x80 >> (new_x % 8));
    } else {
        paint->image[(new_x + new_y * paint->width) / 8] |= 0x80 >> (new_x % 8);
    }
}

void Paint_DrawCharAt(Paint* paint, int x, int y, char ascii_char, sFONT* font, int colored) {
    int i, j;
    unsigned int offset = (ascii_char - ' ') * font->Height * ((font->Width + 7) / 8);
    const unsigned char* ptr = &font->table[offset];

    for (j = 0; j < font->Height; j++) {
        for (i = 0; i < font->Width; i++) {
            if (*ptr & (0x80 >> (i % 8))) {
                Paint_DrawPixel(paint, x + i, y + j, colored);
            }
            if (i % 8 == 7) {
                ptr++;
            }
        }
        if (font->Width % 8 != 0) {
            ptr++;
        }
    }
}
void Paint_DrawStringAt(Paint* paint, int x, int y, const char* text, sFONT* font, int colored) {
    const char* p_text = text;
    unsigned int refcolumn = x;

    while (*p_text != 0) {
        Paint_DrawCharAt(paint, refcolumn, y, *p_text, font, colored);
        refcolumn += font->Width;
        p_text++;
    }
}


void Paint_DrawHorizontalLine(Paint* paint, int x, int y, int width, int colored) {
    for (int i = x; i < x + width; i++) {
        Paint_DrawPixel(paint, i, y, colored);
    }
}

void Paint_DrawVerticalLine(Paint* paint, int x, int y, int height, int colored) {
    for (int i = y; i < y + height; i++) {
        Paint_DrawPixel(paint, x, i, colored);
    }
}

void Paint_DrawRectangle(Paint* paint, int x0, int y0, int x1, int y1, int colored) {
    int min_x = (x0 < x1) ? x0 : x1;
    int max_x = (x0 > x1) ? x0 : x1;
    int min_y = (y0 < y1) ? y0 : y1;
    int max_y = (y0 > y1) ? y0 : y1;

    Paint_DrawHorizontalLine(paint, min_x, min_y, max_x - min_x + 1, colored);
    Paint_DrawHorizontalLine(paint, min_x, max_y, max_x - min_x + 1, colored);
    Paint_DrawVerticalLine(paint, min_x, min_y, max_y - min_y + 1, colored);
    Paint_DrawVerticalLine(paint, max_x, min_y, max_y - min_y + 1, colored);
}

// rectangle with padding

void Paint_DrawRoundedRectangle(Paint* paint, int x0, int y0, int x1, int y1, int r, int colored) {
    // Rysowanie prostych odcinków
    Paint_DrawHorizontalLine(paint, x0 + r, y0, x1 - x0 - 2 * r + 1, colored);
    Paint_DrawHorizontalLine(paint, x0 + r, y1, x1 - x0 - 2 * r + 1, colored);
    Paint_DrawVerticalLine(paint, x0, y0 + r, y1 - y0 - 2 * r + 1, colored);
    Paint_DrawVerticalLine(paint, x1, y0 + r, y1 - y0 - 2 * r + 1, colored);

    // Rysowanie zaokrąglonych rogów
    Paint_DrawCircleQuarter(paint, x0 + r, y0 + r, r, 1, colored); // Lewy górny róg
    Paint_DrawCircleQuarter(paint, x1 - r, y0 + r, r, 2, colored); // Prawy górny róg
    Paint_DrawCircleQuarter(paint, x0 + r, y1 - r, r, 3, colored); // Lewy dolny róg
    Paint_DrawCircleQuarter(paint, x1 - r, y1 - r, r, 4, colored); // Prawy dolny róg
}

// Funkcja rysująca ćwiartkę okręgu
void Paint_DrawCircleQuarter(Paint* paint, int x, int y, int r, int quarter, int colored) {
    int x_pos = r, y_pos = 0;
    int err = 0;

    while (x_pos >= y_pos) {
        if (quarter == 1) {
            Paint_DrawPixel(paint, x - x_pos, y - y_pos, colored);
            Paint_DrawPixel(paint, x - y_pos, y - x_pos, colored);
        } else if (quarter == 2) {
            Paint_DrawPixel(paint, x + x_pos, y - y_pos, colored);
            Paint_DrawPixel(paint, x + y_pos, y - x_pos, colored);
        } else if (quarter == 3) {
            Paint_DrawPixel(paint, x - x_pos, y + y_pos, colored);
            Paint_DrawPixel(paint, x - y_pos, y + x_pos, colored);
        } else if (quarter == 4) {
            Paint_DrawPixel(paint, x + x_pos, y + y_pos, colored);
            Paint_DrawPixel(paint, x + y_pos, y + x_pos, colored);
        }

        if (err <= 0) {
            y_pos += 1;
            err += 2 * y_pos + 1;
        }

        if (err > 0) {
            x_pos -= 1;
            err -= 2 * x_pos + 1;
        }
    }
}



void Paint_DrawFilledRectangle(Paint* paint, int x0, int y0, int x1, int y1, int colored) {
    int min_x = (x0 < x1) ? x0 : x1;
    int max_x = (x0 > x1) ? x0 : x1;
    int min_y = (y0 < y1) ? y0 : y1;
    int max_y = (y0 > y1) ? y0 : y1;

    for (int i = min_x; i <= max_x; i++) {
        Paint_DrawVerticalLine(paint, i, min_y, max_y - min_y + 1, colored);
    }
}

void Paint_DrawLine(Paint* paint, int x0, int y0, int x1, int y1, int colored) {
    int dx = abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    int e2;

    while (1) {
        Paint_DrawPixel(paint, x0, y0, colored);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void Paint_DrawImage(Paint* paint, const unsigned char* image_buffer, int x, int y, int image_width, int image_height) {
    for (int j = 0; j < image_height; j++) {
        for (int i = 0; i < image_width; i++) {
            if (image_buffer[i + j * image_width] == 0) {
                Paint_DrawPixel(paint, x + i, y + j, COLORED);
            } else {
                Paint_DrawPixel(paint, x + i, y + j, UNCOLORED);
            }
        }
    }
}

void Paint_DrawCircle(Paint* paint, int x, int y, int radius, int colored) {
    int xi = -radius;
    int yi = 0;
    int err = 2 - 2 * radius;
    int e2;

    do {
        Paint_DrawPixel(paint, x - xi, y + yi, colored);
        Paint_DrawPixel(paint, x + xi, y + yi, colored);
        Paint_DrawPixel(paint, x + xi, y - yi, colored);
        Paint_DrawPixel(paint, x - xi, y - yi, colored);
        e2 = err;
        if (e2 <= yi) {
            err += ++yi * 2 + 1;
            if (-xi == yi && e2 <= xi) {
                e2 = 0;
            }
        }
        if (e2 > xi) {
            err += ++xi * 2 + 1;
        }
    } while (xi <= 0);
}

void Paint_DrawFilledCircle(Paint* paint, int x, int y, int radius, int colored) {
    int xi = -radius;
    int yi = 0;
    int err = 2 - 2 * radius;
    int e2;

    do {
        Paint_DrawPixel(paint, x - xi, y + yi, colored);
        Paint_DrawPixel(paint, x + xi, y + yi, colored);
        Paint_DrawPixel(paint, x + xi, y - yi, colored);
        Paint_DrawPixel(paint, x - xi, y - yi, colored);
        Paint_DrawLine(paint, x - xi, y + yi, x + xi, y + yi, colored);
        Paint_DrawLine(paint, x - xi, y - yi, x + xi, y - yi, colored);
        e2 = err;
        if (e2 <= yi) {
            err += ++yi * 2 + 1;
            if (-xi == yi && e2 <= xi) {
                e2 = 0;
            }
        }
        if (e2 > xi) {
            err += ++xi * 2 + 1;
        }
    } while (xi <= 0);
}

void Paint_SetWidth(Paint* paint, int width) {
    paint->width = (width % 8) ? (width + 8 - (width % 8)) : width;
}

void Paint_SetHeight(Paint* paint, int height) {
    paint->height = height;
}

int Paint_GetWidth(Paint* paint) {
    return paint->width;
}

int Paint_GetHeight(Paint* paint) {
    return paint->height;
}

void Paint_SetRotate(Paint* paint, int rotate) {
    paint->rotate = rotate;
}

int Paint_GetRotate(Paint* paint) {
    return paint->rotate;
}

unsigned char* Paint_GetImage(Paint* paint) {
    return paint->image;
}


void DrawBattery(Paint* paint, int x, int y, int width, int height, int level, int colored) {
    // Główny prostokąt baterii
    Paint_DrawRectangle(paint, x, y, x + width, y + height, colored);

    // Mały prostokąt jako złącze baterii
    int connectorWidth = 5;
    int connectorHeight = height / 2;
    Paint_DrawRectangle(paint, x + width, y + (height - connectorHeight) / 2, x + width + connectorWidth, y + (height + connectorHeight) / 2, colored);

    // Wysokość jednego poziomu naładowania
    int levelHeight = (height - 4) / 3; // -4 to marginesy

    for (int i = 0; i < level; i++) {
        // Rysuj poziomy naładowania
        int levelY = y + height - 2 - (i + 1) * levelHeight; // -2 to dolny margines
        Paint_DrawFilledRectangle(paint, x + 2, levelY, x + width - 2, levelY + levelHeight - 1, colored); // -1 to górny margines
    }
}





void LoadBattery(Paint* paint, int x, int y, int width, int height, int level, int colored) {
    // Wysokość jednego poziomu naładowania
    int levelHeight = (height - 4) / 3; // -4 to marginesy

    for (int i = 0; i < level; i++) {
        // Rysuj poziomy naładowania
        int levelY = y + height - 2 - (i + 1) * levelHeight; // -2 to dolny margines
        Paint_DrawFilledRectangle(paint, x + 2, levelY, x + width - 2, levelY + levelHeight - 1, colored); // -1 to górny margines
    }
}




void Paint_DrawBitmap(Paint* paint, const unsigned char* bitmap, int x, int y, int width, int height, int colored) {
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            if (bitmap[(i / 8) + j * (width / 8)] & (1 << (7 - (i % 8)))) {
                Paint_DrawPixel(paint, x + i, y + j, !colored);
            } else {
                Paint_DrawPixel(paint, x + i, y + j, colored);
            }
        }
    }
}













