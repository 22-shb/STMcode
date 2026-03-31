#ifndef TLE5012B_H
#define TLE5012B_H


uint16_t SPI_TransmitReceive(uint16_t data);
uint8_t CalculateCRC(uint16_t data);
uint16_t ReadSensorRegister(uint16_t reg_address);
void WriteSensorRegister(uint16_t reg_address, uint16_t data);
void SensorInit(void);
uint16_t ReadRawAngle(void);
float ReadAngleDegrees(void);
float ReadAngularSpeed(void);
void SensorDiagnostic(void);

#endif 