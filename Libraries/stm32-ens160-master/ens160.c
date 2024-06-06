#include "ENS160.h"
#include <string.h>

void DFRobot_ENS160_I2C_Init(DFRobot_ENS160_I2C *instance, I2C_HandleTypeDef *hi2c, uint8_t i2cAddr)
{
    instance->hi2c = hi2c;
    instance->i2cAddr = (i2cAddr != 0) ? i2cAddr : 0x53; // Ustaw domyślny adres, jeśli nie podano
    instance->misr = 0;                                  // Mirror of DATA_MISR (0 is hardware default)
}

int DFRobot_ENS160_I2C_Begin(DFRobot_ENS160_I2C *instance)
{
    uint8_t idBuf[2];
    if (HAL_I2C_Mem_Read(instance->hi2c, instance->i2cAddr, ENS160_PART_ID_REG, I2C_MEMADD_SIZE_8BIT, idBuf, sizeof(idBuf), HAL_MAX_DELAY) != HAL_OK)
    {
        return ERR_DATA_BUS;
    }
    if (ENS160_PART_ID != ENS160_CONCAT_BYTES(idBuf[1], idBuf[0]))
    {
        return ERR_IC_VERSION;
    }
    DFRobot_ENS160_SetPWRMode(instance, ENS160_STANDARD_MODE);
    DFRobot_ENS160_SetINTMode(instance, 0x00);
    return NO_ERR;
}

void DFRobot_ENS160_I2C_WriteReg(DFRobot_ENS160_I2C *instance, uint8_t reg, const void *pBuf, size_t size)
{
    HAL_I2C_Mem_Write(instance->hi2c, instance->i2cAddr, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)pBuf, size, HAL_MAX_DELAY);
}

size_t DFRobot_ENS160_I2C_ReadReg(DFRobot_ENS160_I2C *instance, uint8_t reg, void *pBuf, size_t size)
{
    if (HAL_I2C_Mem_Read(instance->hi2c, instance->i2cAddr, reg, I2C_MEMADD_SIZE_8BIT, (uint8_t *)pBuf, size, HAL_MAX_DELAY) == HAL_OK)
    {
        return size;
    }
    return 0;
}

void DFRobot_ENS160_SetPWRMode(DFRobot_ENS160_I2C *instance, uint8_t mode)
{
    DFRobot_ENS160_I2C_WriteReg(instance, ENS160_OPMODE_REG, &mode, sizeof(mode));
    HAL_Delay(20);
}

void DFRobot_ENS160_SetINTMode(DFRobot_ENS160_I2C *instance, uint8_t mode)
{
    mode |= (0x04 | 0x01); // eINTDataDrdyEN | eIntGprDrdyDIS
    DFRobot_ENS160_I2C_WriteReg(instance, ENS160_CONFIG_REG, &mode, sizeof(mode));
    HAL_Delay(20);
}

void DFRobot_ENS160_SetTempAndHum(DFRobot_ENS160_I2C *instance, float ambientTemp, float relativeHumidity)
{
    uint16_t temp = (ambientTemp + 273.15) * 64;
    uint16_t rh = relativeHumidity * 512;
    uint8_t buf[4];

    buf[0] = temp & 0xFF;
    buf[1] = (temp & 0xFF00) >> 8;
    buf[2] = rh & 0xFF;
    buf[3] = (rh & 0xFF00) >> 8;
    DFRobot_ENS160_I2C_WriteReg(instance, ENS160_TEMP_IN_REG, buf, sizeof(buf));
}

uint8_t DFRobot_ENS160_GetStatus(DFRobot_ENS160_I2C *instance)
{
    DFRobot_ENS160_I2C_ReadReg(instance, ENS160_DATA_STATUS_REG, &instance->ENS160Status, sizeof(instance->ENS160Status));
    return instance->ENS160Status;
}

uint8_t DFRobot_ENS160_GetAQI(DFRobot_ENS160_I2C *instance)
{
    uint8_t data = 0;
    DFRobot_ENS160_I2C_ReadReg(instance, ENS160_DATA_AQI_REG, &data, sizeof(data));
    return data;
}

uint16_t DFRobot_ENS160_GetTVOC(DFRobot_ENS160_I2C *instance)
{
    uint8_t buf[2];
    DFRobot_ENS160_I2C_ReadReg(instance, ENS160_DATA_TVOC_REG, buf, sizeof(buf));
    return ENS160_CONCAT_BYTES(buf[1], buf[0]);
}

uint16_t DFRobot_ENS160_GetECO2(DFRobot_ENS160_I2C *instance)
{
    uint8_t buf[2];
    DFRobot_ENS160_I2C_ReadReg(instance, ENS160_DATA_ECO2_REG, buf, sizeof(buf));
    return ENS160_CONCAT_BYTES(buf[1], buf[0]);
}

uint8_t DFRobot_ENS160_GetMISR(DFRobot_ENS160_I2C *instance)
{
    uint8_t crc = 0;
    DFRobot_ENS160_I2C_ReadReg(instance, ENS160_DATA_MISR_REG, &crc, sizeof(crc));
    return crc;
}

uint8_t DFRobot_ENS160_CalcMISR(DFRobot_ENS160_I2C *instance, uint8_t data)
{
    uint8_t misr_xor = ((instance->misr << 1) ^ data) & 0xFF;
    if ((instance->misr & 0x80) == 0)
    {
        instance->misr = misr_xor;
    }
    else
    {
        instance->misr = misr_xor ^ POLY;
    }
    return instance->misr;
}
