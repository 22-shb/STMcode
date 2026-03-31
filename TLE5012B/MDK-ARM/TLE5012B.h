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
void HardwareDiagnostic(void);
void SimpleCommunicationTest(void);


void TLE5012_Init_SSC(void);
// 读取角速度
float ReadAngularSpeed_SSC(void);
// 完整的角度读取函数（基于博客原理）
float ReadAngle_SSC(void);
// 写入寄存器（SSC协议）
void SSC_WriteRegister(uint16_t reg_addr, uint16_t data);
// 读取寄存器（SSC协议）
uint16_t SSC_ReadRegister(uint16_t reg_addr);
// 单次SSC传输
uint16_t SSC_Transfer(uint16_t data);
// 验证SSC帧的CRC
uint8_t VerifySSCFrame(uint16_t ssc_frame);
// 提取SSC帧中的数据（去掉CRC）
uint16_t ExtractDataFromSSCFrame(uint16_t ssc_frame);
// 构建SSC帧（根据博客：CRC在bit12-15）
uint16_t BuildSSCFrame(uint16_t data);
// 根据博客的CRC计算函数
uint8_t CalculateCRC2(uint16_t data);
void TestCRC_Calculation(void);
// SSC通信测试
void TestSSC_Communication(void);
// SPI模式测试
void TestSPI_Modes(void);

	

#endif 