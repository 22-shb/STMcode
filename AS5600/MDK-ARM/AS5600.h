#ifndef _AS5600_H_
#define _AS5600_H_

#include "main.h"
#include "i2c.h"

/* AS5600 7-bit I2C address (0x36). HAL expects 7-bit address shifted internally,
   so use the 7-bit value here and HAL will handle R/W bit. */
#define AS5600_I2C_ADDR         0x36U

/* AS5600 register addresses (RAW ANGLE high/low) */
#define AS5600_RAW_HI_REG       0x0CU
#define AS5600_RAW_LO_REG       0x0DU

/* Timeout for HAL I2C operations (ms) */
#define AS5600_I2C_TIMEOUT_MS   100U

/* Public API */
void AS5600_WriteData(uint8_t Wordaddress, uint8_t Data);
uint8_t AS5600_ReadData(uint8_t Wordaddress);
uint16_t AS5600_ReadRawAngle(void);
float AS5600_RawToDegree(uint16_t angle);

#endif