#include "AS5600.h"
#include <math.h>

/**
  * @brief  ???AS5600
  * @param  has5600: AS5600????
  * @param  hi2c: I2C????
  * @retval ????
  */
AS5600_StatusTypeDef AS5600_Init(AS5600_HandleTypeDef *has5600, I2C_HandleTypeDef *hi2c)
{
    if (has5600 == NULL || hi2c == NULL)
    {
        return AS5600_ERROR;
    }
    
    has5600->hi2c = hi2c;
    has5600->i2c_addr = AS5600_I2C_ADDR << 1;  // ???7???
    has5600->zero_position = 0;
    has5600->max_angle = AS5600_ANGLE_MAX;
    
    // ????????
    uint8_t status;
    if (AS5600_GetStatus(has5600, &status) != AS5600_OK)
    {
        return AS5600_I2C_ERROR;
    }
    
    return AS5600_OK;
}

/**
  * @brief  ?????????
  * @param  has5600: AS5600????
  * @param  angle: ?????
  * @retval ????
  */
AS5600_StatusTypeDef AS5600_ReadAngle(AS5600_HandleTypeDef *has5600, uint16_t *angle)
{
    uint8_t data[2];
    
    if (HAL_I2C_Mem_Read(has5600->hi2c, has5600->i2c_addr, 
                        AS5600_REG_ANGLE, I2C_MEMADD_SIZE_8BIT, 
                        data, 2, 100) != HAL_OK)
    {
        return AS5600_I2C_ERROR;
    }
    
    *angle = (data[0] << 8) | data[1];
    return AS5600_OK;
}

/**
  * @brief  ???????
  * @param  has5600: AS5600????
  * @param  raw_angle: ???????
  * @retval ????
  */
AS5600_StatusTypeDef AS5600_ReadRawAngle(AS5600_HandleTypeDef *has5600, uint16_t *raw_angle)
{
    uint8_t data[2];
    
    if (HAL_I2C_Mem_Read(has5600->hi2c, has5600->i2c_addr, 
                        AS5600_REG_RAW_ANGLE, I2C_MEMADD_SIZE_8BIT, 
                        data, 2, 100) != HAL_OK)
    {
        return AS5600_I2C_ERROR;
    }
    
    *raw_angle = (data[0] << 8) | data[1];
    return AS5600_OK;
}

/**
  * @brief  ???????
  * @param  has5600: AS5600????
  * @param  status: ?????
  * @retval ????
  */
AS5600_StatusTypeDef AS5600_GetStatus(AS5600_HandleTypeDef *has5600, uint8_t *status)
{
    if (HAL_I2C_Mem_Read(has5600->hi2c, has5600->i2c_addr, 
                        AS5600_REG_STATUS, I2C_MEMADD_SIZE_8BIT, 
                        status, 1, 100) != HAL_OK)
    {
        return AS5600_I2C_ERROR;
    }
    
    return AS5600_OK;
}

/**
  * @brief  ????????
  * @param  has5600: AS5600????
  * @retval ????
  */
AS5600_StatusTypeDef AS5600_GetMagnetStrength(AS5600_HandleTypeDef *has5600)
{
    uint8_t status;
    
    if (AS5600_GetStatus(has5600, &status) != AS5600_OK)
    {
        return AS5600_I2C_ERROR;
    }
    
    if ((status & AS5600_STATUS_MAGNET_DETECT) == 0)
    {
        return AS5600_NO_MAGNET;
    }
    
    if (status & 4)
    {
        return AS5600_MAGNET_TOO_WEAK;
    }
    
    if (status & 5)
    {
        return AS5600_MAGNET_TOO_STRONG;
    }
    
    return AS5600_OK;
}

/**
  * @brief  ??????
  * @param  has5600: AS5600????
  * @param  zero_pos: ????
  * @retval ????
  */
AS5600_StatusTypeDef AS5600_SetZeroPosition(AS5600_HandleTypeDef *has5600, uint16_t zero_pos)
{
    uint8_t data[2];
    
    data[0] = (zero_pos >> 8) & 0x0F;  // ?4?
    data[1] = zero_pos & 0xFF;         // ?8?
    
    if (HAL_I2C_Mem_Write(has5600->hi2c, has5600->i2c_addr, 
                         AS5600_REG_ZPOS, I2C_MEMADD_SIZE_8BIT, 
                         data, 2, 100) != HAL_OK)
    {
        return AS5600_I2C_ERROR;
    }
    
    has5600->zero_position = zero_pos;
    return AS5600_OK;
}

/**
  * @brief  ??????
  * @param  has5600: AS5600????
  * @param  zero_pos: ??????
  * @retval ????
  */
AS5600_StatusTypeDef AS5600_ReadZeroPosition(AS5600_HandleTypeDef *has5600, uint16_t *zero_pos)
{
    uint8_t data[2];
    
    if (HAL_I2C_Mem_Read(has5600->hi2c, has5600->i2c_addr, 
                        AS5600_REG_ZPOS, I2C_MEMADD_SIZE_8BIT, 
                        data, 2, 100) != HAL_OK)
    {
        return AS5600_I2C_ERROR;
    }
    
    *zero_pos = ((data[0] & 0x0F) << 8) | data[1];
    has5600->zero_position = *zero_pos;
    return AS5600_OK;
}

/**
  * @brief  ??????
  * @param  has5600: AS5600????
  * @param  max_angle: ?????
  * @retval ????
  */
AS5600_StatusTypeDef AS5600_SetMaxAngle(AS5600_HandleTypeDef *has5600, uint16_t max_angle)
{
    uint8_t data[2];
    
    data[0] = (max_angle >> 8) & 0x0F;  // ?4?
    data[1] = max_angle & 0xFF;         // ?8?
    
    if (HAL_I2C_Mem_Write(has5600->hi2c, has5600->i2c_addr, 
                         AS5600_REG_MANG, I2C_MEMADD_SIZE_8BIT, 
                         data, 2, 100) != HAL_OK)
    {
        return AS5600_I2C_ERROR;
    }
    
    has5600->max_angle = max_angle;
    return AS5600_OK;
}

/**
  * @brief  ???????????
  * @param  angle: ?????
  * @retval ????
  */
float AS5600_AngleToDegrees(uint16_t angle)
{
    return (angle * 360.0f) / 4096.0f;
}
