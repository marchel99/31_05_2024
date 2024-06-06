/*!
 * @file  DFRobot_ENS160.h
 * @brief  Define infrastructure of DFRobot_ENS160 class
 * @details  This is a Digital Metal-Oxide Multi-Gas Sensor. It can be controlled by I2C and SPI port.
 * @n        Detection of a variety of gases, such as volatile organic compounds (VOCs), as 
 * @n        well as hydrogen and nitrogen dioxide, has superior selectivity and accuracy.
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  [qsjhyy](yihuan.huang@dfrobot.com)
 * @version  V1.0
 * @date  2021-10-25
 * @url  https://github.com/DFRobot/DFRobot_ENS160
 */
#ifndef __DFRobot_ENS160_H__
#define __DFRobot_ENS160_H__

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stddef.h>

#define POLY   (0x1D)   ///< 0b00011101 = x^8+x^4+x^3+x^2+x^0 (x^8 is implicit)
#define ENS160_PART_ID   (0x160)   ///< ENS160 chip version

/* Error codes */
#define NO_ERR             0    // No error
#define ERR_DATA_BUS     (-1)   // Data bus error
#define ERR_IC_VERSION   (-2)   // Chip version error

/* ENS160 register address */
#define ENS160_PART_ID_REG     (0x00)   ///< This 2-byte register contains the part number in little endian of the ENS160.
#define ENS160_OPMODE_REG      (0x10)   ///< This 1-byte register sets the Operating Mode of the ENS160.
#define ENS160_CONFIG_REG      (0x11)   ///< This 1-byte register configures the action of the INTn pin.
#define ENS160_COMMAND_REG     (0x12)   ///< This 1-byte register allows some additional commands to be executed on the ENS160.
#define ENS160_TEMP_IN_REG     (0x13)   ///< This 2-byte register allows the host system to write ambient temperature data to ENS160 for compensation.
#define ENS160_RH_IN_REG       (0x15)   ///< This 2-byte register allows the host system to write relative humidity data to ENS160 for compensation.
#define ENS160_DATA_STATUS_REG (0x20)   ///< This 1-byte register indicates the current STATUS of the ENS160.
#define ENS160_DATA_AQI_REG    (0x21)   ///< This 1-byte register reports the calculated Air Quality Index according to the UBA.
#define ENS160_DATA_TVOC_REG   (0x22)   ///< This 2-byte register reports the calculated TVOC concentration in ppb.
#define ENS160_DATA_ECO2_REG   (0x24)   ///< This 2-byte register reports the calculated equivalent CO2-concentration in ppm, based on the detected VOCs and hydrogen.
#define ENS160_DATA_T_REG      (0x30)   ///< This 2-byte register reports the temperature used in its calculations (taken from TEMP_IN, if supplied).
#define ENS160_DATA_RH_REG     (0x32)   ///< This 2-byte register reports the relative humidity used in its calculations (taken from RH_IN if supplied).
#define ENS160_DATA_MISR_REG   (0x38)   ///< This 1-byte register reports the calculated checksum of the previous DATA_ read transaction (of n-bytes).
#define ENS160_GPR_WRITE_REG   (0x40)   ///< This 8-byte register is used by several functions for the Host System to pass data to the ENS160.
#define ENS160_GPR_READ_REG    (0x48)   ///< This 8-byte register is used by several functions for the ENS160 to pass data to the Host System.

/* CMD(0x12) register command */
#define ENS160_COMMAND_NOP          (0x00)   ///< reserved. No command.
#define ENS160_COMMAND_GET_APPVER   (0x0E)   ///< Get FW Version Command.
#define ENS160_COMMAND_CLRGPR       (0xCC)   ///< Clears GPR Read Registers Command.

/* OPMODE(Address 0x10) register mode */
#define ENS160_SLEEP_MODE      (0x00)   ///< DEEP SLEEP mode (low power standby).
#define ENS160_IDLE_MODE       (0x01)   ///< IDLE mode (low-power).
#define ENS160_STANDARD_MODE   (0x02)   ///< STANDARD Gas Sensing Modes.

/* Convenience Macro */
#define ENS160_CONCAT_BYTES(msb, lsb)   (((uint16_t)msb << 8) | (uint16_t)lsb)   ///< Macro combines two 8-bit data into one 16-bit data

typedef struct {
    uint8_t GPRDrdy: 1; /**< General purpose register data ready */
    uint8_t dataDrdy: 1; /**< Measured data ready */
    uint8_t validityFlag: 2; /**< 0: Normal operation, 1: Warm-Up phase, 2: Initial Start-Up phase, 3: Invalid output */
    uint8_t reserved: 2; /**< Reserved bit */
    uint8_t stater: 1; /**< High indicates that an error is detected. E.g. Invalid Operating Mode has been selected. */
    uint8_t status: 1; /**< High indicates that an OPMODE is running */
} sSensorStatus_t;

typedef struct {
    I2C_HandleTypeDef* hi2c;
    uint8_t i2cAddr;
    uint8_t ENS160Status;
    uint8_t misr;
} DFRobot_ENS160_I2C;

void DFRobot_ENS160_I2C_Init(DFRobot_ENS160_I2C* instance, I2C_HandleTypeDef* hi2c, uint8_t i2cAddr);
int DFRobot_ENS160_I2C_Begin(DFRobot_ENS160_I2C* instance);
void DFRobot_ENS160_I2C_WriteReg(DFRobot_ENS160_I2C* instance, uint8_t reg, const void* pBuf, size_t size);
size_t DFRobot_ENS160_I2C_ReadReg(DFRobot_ENS160_I2C* instance, uint8_t reg, void* pBuf, size_t size);
void DFRobot_ENS160_SetPWRMode(DFRobot_ENS160_I2C* instance, uint8_t mode);
void DFRobot_ENS160_SetINTMode(DFRobot_ENS160_I2C* instance, uint8_t mode);
void DFRobot_ENS160_SetTempAndHum(DFRobot_ENS160_I2C* instance, float ambientTemp, float relativeHumidity);
uint8_t DFRobot_ENS160_GetStatus(DFRobot_ENS160_I2C* instance);
uint8_t DFRobot_ENS160_GetAQI(DFRobot_ENS160_I2C* instance);
uint16_t DFRobot_ENS160_GetTVOC(DFRobot_ENS160_I2C* instance);
uint16_t DFRobot_ENS160_GetECO2(DFRobot_ENS160_I2C* instance);
uint8_t DFRobot_ENS160_GetMISR(DFRobot_ENS160_I2C* instance);
uint8_t DFRobot_ENS160_CalcMISR(DFRobot_ENS160_I2C* instance, uint8_t data);

#endif // __DFRobot_ENS160_H__
