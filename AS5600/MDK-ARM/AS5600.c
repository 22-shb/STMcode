#include "AS5600.h"

/*
  ******************************************************************************
  * @file    as5600.c
  * @brief   AS5600 driver implementation using HAL I2C
  ******************************************************************************
*/

void AS5600_WriteData(uint8_t Wordaddress, uint8_t Data)
{
	/* HAL expects 7-bit address (no R/W bit) in most STM32 HAL versions. */
	uint8_t buf = Data;
	HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(AS5600_I2C_ADDR << 1), Wordaddress, I2C_MEMADD_SIZE_8BIT, &buf, 1, AS5600_I2C_TIMEOUT_MS);
}

uint8_t AS5600_ReadData(uint8_t Wordaddress)
{
	uint8_t buf = 0;
	HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(AS5600_I2C_ADDR << 1), Wordaddress, I2C_MEMADD_SIZE_8BIT, &buf, 1, AS5600_I2C_TIMEOUT_MS);
	return buf;
}

uint16_t AS5600_ReadRawAngle(void)
{
	uint8_t hi = AS5600_ReadData(AS5600_RAW_HI_REG);
	uint8_t lo = AS5600_ReadData(AS5600_RAW_LO_REG);
	uint16_t val = ((uint16_t)hi << 8) | (uint16_t)lo;
	val &= 0x0FFF;
	return val;
}

float AS5600_RawToDegree(uint16_t angle)
{
	return ((float)angle) * 360.0f / 4096.0f;
}
