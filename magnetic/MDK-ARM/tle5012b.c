#include "stm32f1xx_hal.h"
#include "tle5012b.h"
// SPI ??
extern SPI_HandleTypeDef hspi1;

// TLE5012B??????
#define TLE5012B_REG_STATUS 0x00
#define TLE5012B_REG_ANGLE  0x02
#define TLE5012B_REG_CONTROL 0x10

// TLE5012B??????
#define TLE5012B_CMD_READ    0x80
#define TLE5012B_CMD_WRITE   0x00

// TLE5012B???SPI????
uint8_t TLE5012B_SPI_WriteRead(uint8_t reg, uint8_t *txData, uint8_t *rxData, uint8_t len)
{
    uint8_t txBuffer[2];
    txBuffer[0] = reg; // ?????
    txBuffer[1] = *txData;

    HAL_SPI_Transmit(&hspi1, txBuffer, 2, HAL_MAX_DELAY); // ????
    HAL_SPI_Receive(&hspi1, rxData, len, HAL_MAX_DELAY);  // ????

    return 0;
}

// TLE5012B???
void TLE5012B_Init(void)
{
    uint8_t cmd = 0x00;
    uint8_t rxData[2];
    
    // ??TLE5012B?????(??,???????)
    TLE5012B_SPI_WriteRead(TLE5012B_REG_CONTROL, &cmd, rxData, 2);
}
// ?????
uint16_t TLE5012B_Read_Angle(void)
{
    uint8_t cmd = TLE5012B_REG_ANGLE | TLE5012B_CMD_READ;  // ??????????
    uint8_t rxData[2];
    
    TLE5012B_SPI_WriteRead(cmd, &cmd, rxData, 2);
    
    // ??????????????16?????
    uint16_t angle = ((uint16_t)rxData[0] << 8) | rxData[1];
    
    return angle;
}

