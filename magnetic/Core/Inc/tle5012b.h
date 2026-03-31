#ifndef TLE5012B_H
#define TLE5012B_H

#include "tle5012b.h"
 
 
uint8_t TLE5012B_SPI_WriteRead(uint8_t reg, uint8_t *txData, uint8_t *rxData, uint8_t len);
void TLE5012B_Init(void);
uint16_t TLE5012B_Read_Angle(void);

#endif 
