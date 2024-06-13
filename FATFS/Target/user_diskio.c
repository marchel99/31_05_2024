/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.c
  * @brief   This file includes a diskio driver skeleton to be completed by the user.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
 /* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "user_diskio.h"
#include "stm32l4xx_hal.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SD_CS_GPIO_Port GPIOA
#define SD_CS_Pin GPIO_PIN_9
#define SPI_TIMEOUT 1000

/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL
  USER_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1; 
void SD_Select(void) {
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
}

void SD_Deselect(void) {
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
}

uint8_t SPI_Send(uint8_t data) {
    uint8_t receive;
    HAL_SPI_TransmitReceive(&hspi1, &data, &receive, 1, SPI_TIMEOUT);
    return receive;
}

void SD_SendCommand(uint8_t cmd, uint32_t arg, uint8_t crc) {
    SD_Select();
    SPI_Send(cmd | 0x40);        // Send command
    SPI_Send(arg >> 24);         // Send argument
    SPI_Send(arg >> 16);
    SPI_Send(arg >> 8);
    SPI_Send(arg);
    SPI_Send(crc);               // Send CRC
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

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */

DSTATUS USER_initialize (BYTE pdrv) {
    if (pdrv != 0) return STA_NOINIT; // Obsługujemy tylko jeden napęd (0)

    printf("Inicjalizacja karty SD\n");

    SD_Deselect();
    HAL_Delay(10);
    for (uint8_t i = 0; i < 10; i++) {
        SPI_Send(0xFF); // Wysyłanie 80 taktów do inicjalizacji karty SD
    }

    // Wysyłamy CMD0, aby zresetować kartę
    SD_SendCommand(0, 0, 0x95);
    if (SD_GetResponse() != 0x01) {
        SD_Deselect();
        printf("Nie udało się zresetować karty SD\n");
        return STA_NOINIT;
    }

    // Dodatkowe procedury inicjalizacyjne mogą być wymagane w zależności od typu karty

    SD_Deselect();
    Stat = 0;
    printf("Karta SD zainicjalizowana poprawnie\n");
    return Stat;
}



/**
  * @brief  Gets Disk Status
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (BYTE pdrv) {
    if (pdrv != 0) return STA_NOINIT;
    return Stat;
}

/**
  * @brief  Reads Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address in LBA
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count) {
    if (pdrv != 0 || count == 0) return RES_PARERR;

    for (UINT i = 0; i < count; i++) {
        SD_SendCommand(17, (sector + i) * 512, 0); // CMD17 to odczyt pojedynczego bloku
        if (SD_GetResponse() != 0x00) {
            SD_Deselect();
            return RES_ERROR;
        }

        // Czekamy na token startu danych (0xFE)
        while (SPI_Send(0xFF) != 0xFE);

        // Odczytujemy 512 bajtów danych
        for (UINT j = 0; j < 512; j++) {
            buff[j + i * 512] = SPI_Send(0xFF);
        }

        // Pomijamy dwa bajty CRC
        SPI_Send(0xFF);
        SPI_Send(0xFF);
    }

    SD_Deselect();
    return RES_OK;
}

/**
  * @brief  Writes Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address in LBA
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count) {
    if (pdrv != 0 || count == 0) return RES_PARERR;

    for (UINT i = 0; i < count; i++) {
        SD_SendCommand(24, (sector + i) * 512, 0); // CMD24 to zapis pojedynczego bloku
        if (SD_GetResponse() != 0x00) {
            SD_Deselect();
            return RES_ERROR;
        }

        SPI_Send(0xFE); // Wysyłamy token startu danych

        // Wysyłamy 512 bajtów danych
        for (UINT j = 0; j < 512; j++) {
            SPI_Send(buff[j + i * 512]);
        }

        // Wysyłamy dwa bajty CRC
        SPI_Send(0xFF);
        SPI_Send(0xFF);

        // Sprawdzamy odpowiedź od karty
        if ((SPI_Send(0xFF) & 0x1F) != 0x05) {
            SD_Deselect();
            return RES_ERROR;
        }

        // Czekamy, aż karta zakończy zapis
        while (SPI_Send(0xFF) == 0);
    }

    SD_Deselect();
    return RES_OK;
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff) {
    if (pdrv != 0) return RES_PARERR;
    DRESULT res = RES_ERROR;

    switch (cmd) {
        case CTRL_SYNC:
            SD_Select();
            while (SPI_Send(0xFF) != 0xFF);
            SD_Deselect();
            res = RES_OK;
            break;

        // Dodatkowe komendy IOCTL można dodać tutaj

        default:
            res = RES_PARERR;
    }

    return res;
}
#endif /* _USE_IOCTL == 1 */