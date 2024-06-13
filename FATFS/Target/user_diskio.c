#include "main.h"
#include "fatfs.h"
#include <stdio.h>
#include <string.h>

#define SD_CS_GPIO_Port CS2_GPIO_Port
#define SD_CS_Pin CS2_Pin

extern SPI_HandleTypeDef hspi1;

static volatile DSTATUS Stat = STA_NOINIT;

DSTATUS USER_initialize(BYTE pdrv);
DSTATUS USER_status(BYTE pdrv);
DRESULT USER_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
DRESULT USER_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif
#if _USE_IOCTL == 1
DRESULT USER_ioctl(BYTE pdrv, BYTE cmd, void *buff);
#endif

Diskio_drvTypeDef USER_Driver = {
  USER_initialize,
  USER_status,
  USER_read,
#if _USE_WRITE == 1
  USER_write,
#endif
#if _USE_IOCTL == 1
  USER_ioctl,
#endif
};

void SD_Select(void) {
    printf("SD_Select\n");
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
}

void SD_Deselect(void) {
    printf("SD_Deselect\n");
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
}

uint8_t SPI_Send(uint8_t data) {
    uint8_t receive;
    HAL_SPI_TransmitReceive(&hspi1, &data, &receive, 1, HAL_MAX_DELAY);
    return receive;
}

void SD_SendCommand(uint8_t cmd, uint32_t arg, uint8_t crc) {
    SD_Select();
    SPI_Send(cmd | 0x40);
    SPI_Send(arg >> 24);
    SPI_Send(arg >> 16);
    SPI_Send(arg >> 8);
    SPI_Send(arg);
    SPI_Send(crc);
    SPI_Send(0xFF); // Send one more 0xFF to ensure proper CS hold time
}

uint8_t SD_GetResponse(void) {
    uint8_t response;
    for (int i = 0; i < 8; i++) {
        response = SPI_Send(0xFF);
        if (response != 0xFF) {
            break;
        }
    }
    return response;
}

DSTATUS USER_initialize(BYTE pdrv) {
    if (pdrv != 0) return STA_NOINIT;

    printf("Inicjalizacja karty SD\n");

    SD_Deselect();
    HAL_Delay(50);
    for (uint8_t i = 0; i < 10; i++) {
        SPI_Send(0xFF); // Wysyłanie 80 taktów do inicjalizacji karty SD
    }

    SD_Select();

    // Wysłanie CMD0, aby przełączyć kartę SD do trybu SPI
    SD_SendCommand(0, 0, 0x95);
    uint8_t response = SD_GetResponse();
    printf("Odpowiedź CMD0: %02X\n", response);
    if (response != 0x01) {
        SD_Deselect();
        printf("Nie udało się zresetować karty SD\n");
        return STA_NOINIT;
    }

    // Wysłanie CMD8, aby sprawdzić wersję karty SD
    SD_SendCommand(8, 0x1AA, 0x87);
    uint8_t r7_resp[5];
    for (int i = 0; i < 5; i++) {
        r7_resp[i] = SPI_Send(0xFF);
    }
    printf("Odpowiedź CMD8: %02X %02X %02X %02X %02X\n", r7_resp[0], r7_resp[1], r7_resp[2], r7_resp[3], r7_resp[4]);
    if (r7_resp[0] != 0x01 || r7_resp[3] != 0x01 || r7_resp[4] != 0xAA) {
        printf("Karta SD nie obsługuje CMD8, kontynuacja z CMD1\n");
        // Starsze karty SD (v1.x) lub MMC
        for (int retry = 0; retry < 10; retry++) {
            SD_SendCommand(55, 0, 0x01);  // CMD55
            SD_SendCommand(1, 0, 0x01);  // CMD1
            response = SD_GetResponse();
            if (response == 0x00) break;
            printf("CMD1 próba %d, odpowiedź: %02X\n", retry, response);
        }
    } else {
        // Karta SD obsługująca CMD8 (v2.x)
        for (int retry = 0; retry < 10; retry++) {
            SD_SendCommand(55, 0, 0x01);  // CMD55
            SD_SendCommand(41, 0x40000000, 0x01);  // ACMD41
            response = SD_GetResponse();
            if (response == 0x00) break;
            printf("ACMD41 próba %d, odpowiedź: %02X\n", retry, response);
        }
    }

    // Wysłanie CMD58, aby sprawdzić, czy karta jest w trybie wysokiego napięcia
    SD_SendCommand(58, 0, 0);
    for (int i = 0; i < 4; i++) {
        r7_resp[i] = SPI_Send(0xFF);
    }
    printf("Odpowiedź CMD58: %02X %02X %02X %02X\n", r7_resp[0], r7_resp[1], r7_resp[2], r7_resp[3]);

    SD_Deselect();
    Stat = 0;
    printf("Karta SD zainicjalizowana poprawnie\n");
    return Stat;
}

DSTATUS USER_status(BYTE pdrv) {
    if (pdrv != 0) return STA_NOINIT;
    return Stat;
}

DRESULT USER_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count) {
    if (pdrv != 0 || !count) return RES_PARERR;

    for (UINT i = 0; i < count; i++) {
        SD_SendCommand(17, (sector + i) * 512, 0); // CMD17 to read single block
        if (SD_GetResponse() != 0x00) {
            SD_Deselect();
            return RES_ERROR;
        }
        for (int j = 0; j < 512; j++) {
            buff[j] = SPI_Send(0xFF);
        }
        buff += 512;
        SD_Deselect();
    }

    return RES_OK;
}

#if _USE_WRITE == 1
DRESULT USER_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count) {
    if (pdrv != 0 || !count) return RES_PARERR;

    for (UINT i = 0; i < count; i++) {
        SD_SendCommand(24, (sector + i) * 512, 0); // CMD24 to write single block
        if (SD_GetResponse() != 0x00) {
            SD_Deselect();
            return RES_ERROR;
        }
        for (int j = 0; j < 512; j++) {
            SPI_Send(buff[j]);
        }
        buff += 512;
        SD_Deselect();
    }

    return RES_OK;
}
#endif

#if _USE_IOCTL == 1
DRESULT USER_ioctl(BYTE pdrv, BYTE cmd, void *buff) {
    if (pdrv != 0) return RES_PARERR;
    DRESULT res = RES_ERROR;

    switch (cmd) {
        case CTRL_SYNC:
            SD_Select();
            while (SPI_Send(0xFF) != 0xFF);
            SD_Deselect();
            res = RES_OK;
            break;
        case GET_SECTOR_COUNT:
            *(DWORD*)buff = (16 * 1024 * 1024) / 512;  // Assuming 16GB card, sector size 512 bytes
            res = RES_OK;
            break;
        case GET_SECTOR_SIZE:
            *(WORD*)buff = 512;
            res = RES_OK;
            break;
        case GET_BLOCK_SIZE:
            *(DWORD*)buff = 1;
            res = RES_OK;
            break;
        default:
            res = RES_PARERR;
    }

    return res;
}
#endif