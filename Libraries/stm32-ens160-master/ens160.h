/*!
ENS160.h

 * @license  The MIT License (MIT)
 * marchel99
 */
#ifndef __DFRobot_ENS160_H__
#define __DFRobot_ENS160_H__

//#include <Arduino.h>
//#include <Wire.h>
//#include <SPI.h>

// #define ENABLE_DBG   //!< Open this macro and you can see the details of the program
#ifdef ENABLE_DBG
  #define DBG(...) {Serial.print("[");Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.println(__VA_ARGS__);}
#else
  #define DBG(...)
#endif

#define POLY   ((uint8_t)0x1D)   ///< 0b00011101 = x^8+x^4+x^3+x^2+x^0 (x^8 is implicit)
#define ENS160_PART_ID   ((uint16_t)0x160)   ///< ENS160 chip version

/* ENS160 register address */
#define ENS160_PART_ID_REG     ((uint8_t)0x00)   ///< This 2-byte register contains the part number in little endian of the ENS160.

#define ENS160_OPMODE_REG      ((uint8_t)0x10)   ///< This 1-byte register sets the Operating Mode of the ENS160.
#define ENS160_CONFIG_REG      ((uint8_t)0x11)   ///< This 1-byte register configures the action of the INTn pin.
#define ENS160_COMMAND_REG     ((uint8_t)0x12)   ///< This 1-byte register allows some additional commands to be executed on the ENS160.

#define ENS160_TEMP_IN_REG     ((uint8_t)0x13)   ///< This 2-byte register allows the host system to write ambient temperature data to ENS160 for compensation.
#define ENS160_RH_IN_REG       ((uint8_t)0x15)   ///< This 2-byte register allows the host system to write relative humidity data to ENS160 for compensation.

#define ENS160_DATA_STATUS_REG ((uint8_t)0x20)   ///< This 1-byte register indicates the current STATUS of the ENS160.

#define ENS160_DATA_AQI_REG    ((uint8_t)0x21)   ///< This 1-byte register reports the calculated Air Quality Index according to the UBA.
#define ENS160_DATA_TVOC_REG   ((uint8_t)0x22)   ///< This 2-byte register reports the calculated TVOC concentration in ppb.
#define ENS160_DATA_ECO2_REG   ((uint8_t)0x24)   ///< This 2-byte register reports the calculated equivalent CO2-concentration in ppm, based on the detected VOCs and hydrogen.
#define ENS160_DATA_ETOH_REG   ((uint8_t)0x22)   ///< This 2-byte register reports the calculated ethanol concentration in ppb.

#define ENS160_DATA_T_REG      ((uint8_t)0x30)   ///< This 2-byte register reports the temperature used in its calculations (taken from TEMP_IN, if supplied).
#define ENS160_DATA_RH_REG     ((uint8_t)0x32)   ///< This 2-byte register reports the relative humidity used in its calculations (taken from RH_IN if supplied).

#define ENS160_DATA_MISR_REG   ((uint8_t)0x38)   ///< This 1-byte register reports the calculated checksum of the previous DATA_ read transaction (of n-bytes).

#define ENS160_GPR_WRITE_REG   ((uint8_t)0x40)   ///< This 8-byte register is used by several functions for the Host System to pass data to the ENS160.
#define ENS160_GPR_READ_REG    ((uint8_t)0x48)   ///< This 8-byte register is used by several functions for the ENS160 to pass data to the Host System.

/* CMD(0x12) register command */
#define ENS160_COMMAND_NOP          ((uint8_t)0x00)   ///< reserved. No command.
#define ENS160_COMMAND_GET_APPVER   ((uint8_t)0x0E)   ///< Get FW Version Command.
#define ENS160_COMMAND_CLRGPR       ((uint8_t)0xCC)   ///< Clears GPR Read Registers Command.

/* OPMODE(Address 0x10) register mode */
#define ENS160_SLEEP_MODE      ((uint8_t)0x00)   ///< DEEP SLEEP mode (low power standby).
#define ENS160_IDLE_MODE       ((uint8_t)0x01)   ///< IDLE mode (low-power).
#define ENS160_STANDARD_MODE   ((uint8_t)0x02)   ///< STANDARD Gas Sensing Modes.

/* Convenience Macro */
#define ENS160_CONCAT_BYTES(msb, lsb)   (((uint16_t)msb << 8) | (uint16_t)lsb)   ///< Macro combines two 8-bit data into one 16-bit data

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int size_t;

typedef struct {
    uint8_t   GPRDrdy: 1; /**< General purpose register data ready */
    uint8_t   dataDrdy: 1; /**< Measured data ready */
    uint8_t   validityFlag: 2; /**< 0: Normal operation, 1: Warm-Up phase, 2: Initial Start-Up phase, 3: Invalid output */
    uint8_t   reserved: 2; /**< Reserved bit */
    uint8_t   stater: 1; /**< High indicates that an error is detected. E.g. Invalid Operating Mode has been selected. */
    uint8_t   status: 1; /**< High indicates that an OPMODE is running */
} sSensorStatus_t;

typedef struct {
    int (*begin)(void);
    void (*setPWRMode)(uint8_t mode);
    void (*setINTMode)(uint8_t mode);
    void (*setTempAndHum)(float ambientTemp, float relativeHumidity);
    uint8_t (*getENS160Status)(void);
    uint8_t (*getAQI)(void);
    uint16_t (*getTVOC)(void);
    uint16_t (*getECO2)(void);
} DFRobot_ENS160;

typedef struct {
    uint8_t (*getMISR)(void);
    uint8_t (*calcMISR)(uint8_t data);
    void (*writeReg)(uint8_t reg, const void* pBuf, size_t size);
    size_t (*readReg)(uint8_t reg, void* pBuf, size_t size);
} DFRobot_ENS160_Methods;

#ifndef DISABLE_ENS160_I2C
typedef struct {
    int (*begin)(void);
} DFRobot_ENS160_I2C;

typedef struct {
    void (*writeReg)(uint8_t reg, const void* pBuf, size_t size);
    size_t (*readReg)(uint8_t reg, void* pBuf, size_t size);
} DFRobot_ENS160_I2C_Methods;
#endif

#ifndef DISABLE_ENS160_SPI
typedef struct {
    int (*begin)(void);
} DFRobot_ENS160_SPI;

typedef struct {
    void (*writeReg)(uint8_t reg, const void* pBuf, size_t size);
    size_t (*readReg)(uint8_t reg, void* pBuf, size_t size);
} DFRobot_ENS160_SPI_Methods;
#endif

#endif
