/*!
ENS160.c

 * @license  The MIT License (MIT)
 * marchel99
 */




#ifndef DFRobot_ENS160_H
#define DFRobot_ENS160_H

#include <stdint.h>
#include <stddef.h>

#define ENS160_PART_ID_REG      0xD0
#define ENS160_PART_ID          0x38

#define ENS160_OPMODE_REG       0x74
#define ENS160_CONFIG_REG       0x76
#define ENS160_TEMP_IN_REG      0xE2
#define ENS160_DATA_STATUS_REG  0x04
#define ENS160_DATA_AQI_REG     0x0C
#define ENS160_DATA_TVOC_REG    0x10
#define ENS160_DATA_ECO2_REG    0x12
#define ENS160_DATA_MISR_REG    0x1E

#define eINTDataDrdyEN          0x04
#define eIntGprDrdyDIS          0x01

#define POLY 0x31

#define ERR_DATA_BUS      -1
#define ERR_IC_VERSION    -2
#define NO_ERR            0

typedef struct {
    uint8_t misr;
    uint8_t ENS160Status;
} DFRobot_ENS160;

typedef struct {
    DFRobot_ENS160 ens160;
    void (*writeReg)(uint8_t reg, const void* pBuf, size_t size);
    size_t (*readReg)(uint8_t reg, void* pBuf, size_t size);
} DFRobot_ENS160_I2C;

typedef struct {
    DFRobot_ENS160 ens160;
    void (*writeReg)(uint8_t reg, const void* pBuf, size_t size);
    size_t (*readReg)(uint8_t reg, void* pBuf, size_t size);
} DFRobot_ENS160_SPI;

int DFRobot_ENS160_begin(DFRobot_ENS160* ens160);
void DFRobot_ENS160_setPWRMode(DFRobot_ENS160* ens160, uint8_t mode);
void DFRobot_ENS160_setINTMode(DFRobot_ENS160* ens160, uint8_t mode);
void DFRobot_ENS160_setTempAndHum(DFRobot_ENS160* ens160, float ambientTemp, float relativeHumidity);
uint8_t DFRobot_ENS160_getENS160Status(DFRobot_ENS160* ens160);
uint8_t DFRobot_ENS160_getAQI(DFRobot_ENS160* ens160);
uint16_t DFRobot_ENS160_getTVOC(DFRobot_ENS160* ens160);
uint16_t DFRobot_ENS160_getECO2(DFRobot_ENS160* ens160);
uint8_t DFRobot_ENS160_getMISR(DFRobot_ENS160* ens160);
uint8_t DFRobot_ENS160_calcMISR(DFRobot_ENS160* ens160, uint8_t data);

//int DFRobot_ENS160_I2C_begin(DFRobot_ENS160_I2C* ens160, TwoWire *pWire, uint8_t i2cAddr);
//int DFRobot_ENS160_SPI_begin(DFRobot_ENS160_SPI* ens160, SPIClass *pSpi, uint8_t csPin);

#endif
