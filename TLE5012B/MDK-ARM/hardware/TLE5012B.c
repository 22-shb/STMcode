/* USER CODE BEGIN 0 */
#include <stdio.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "spi.h"
#include "usart.h"
uint16_t SPI_TransmitReceive(uint16_t data)
{
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];
    uint16_t result;
    
    // 拆分16位数据为两个8位字节
    tx_buf[0] = (data >> 8) & 0xFF;
    tx_buf[1] = data & 0xFF;
    
    // 片选拉低
    HAL_GPIO_WritePin(GPIOB, 0x0001, 0);
    
    // 单次SPI传输
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 2, 1000);
    
    // 片选拉高
    HAL_GPIO_WritePin(GPIOB, 0x0001, 1);
    
    // 组合接收到的数据
    result = (rx_buf[0] << 8) | rx_buf[1];
    
    return result;
}

// CRC计算函数 - 官方例程中的CRC算法
uint8_t CalculateCRC(uint16_t data)
{
    // CRC查表（官方提供）
    static const uint8_t crc_table[] = {
        0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
        0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D
    };
    
    uint8_t crc = 0;
    uint8_t data_byte;
    
    // 处理高字节
    data_byte = (data >> 8) & 0xFF;
    crc = crc_table[(data_byte >> 4) ^ crc];
    crc = crc_table[(data_byte & 0x0F) ^ crc];
    
    // 处理低字节
    data_byte = data & 0xFF;
    crc = crc_table[(data_byte >> 4) ^ crc];
    crc = crc_table[(data_byte & 0x0F) ^ crc];
    
    return crc;
}

// 读取寄存器函数 - 官方例程风格
uint16_t ReadSensorRegister(uint16_t reg_address)
{
    uint16_t command;
    uint16_t response;
    
    // 构建读取命令（bit15=1表示读取）
    command = 0x8000 | reg_address;
    
    // 添加CRC（官方安全SPI协议）
    command |= (CalculateCRC(command) << 12);
    
    // 发送命令
    SPI_TransmitReceive(command);
    
    // 短暂延时
    HAL_Delay(1);
    
    // 发送哑元数据并接收响应
    response = SPI_TransmitReceive(0x0000);
    
    return response;
}

// 写入寄存器函数 - 官方例程风格
void WriteSensorRegister(uint16_t reg_address, uint16_t data)
{
    uint16_t command;
    
    // 构建写入命令（bit15=0表示写入）
    command = reg_address;
    
    // 添加CRC
    command |= (CalculateCRC(command) << 12);
    
    // 发送命令
    SPI_TransmitReceive(command);
    HAL_Delay(1);
    
    // 添加CRC到数据并发送
    data |= (CalculateCRC(data) << 12);
    SPI_TransmitReceive(data);
}

// 设备初始化 - 官方例程的初始化序列
void SensorInit(void)
{
    uint16_t device_id;
    char msg[100];
    
    // 读取设备ID验证连接（MOD1寄存器）
    device_id = ReadSensorRegister(0x0081);
    
    sprintf(msg, "Device ID: 0x%04X\r\n", device_id);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    // 检查设备ID是否正确（低5位应为00001b）
    if ((device_id & 0x001F) == 0x0001) {
        sprintf(msg, "TLE5012B detected successfully\r\n");
    } else {
        sprintf(msg, "Warning: Unknown device ID\r\n");
    }
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    // 配置MOD2寄存器 - 滤波器设置（官方推荐值）
    // 启用预测器 + 200Hz滤波器带宽
    WriteSensorRegister(0x0082, 0x0801);
    HAL_Delay(10);
    
    // 配置MOD3寄存器 - 接口设置
    // 自动更新模式 + SPI接口
    WriteSensorRegister(0x0083, 0x0004);
    HAL_Delay(10);
    
    sprintf(msg, "Sensor initialization complete\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
}

// 读取角度值 - 官方例程方法
uint16_t ReadRawAngle(void)
{
    uint16_t raw_data;
    
    // 读取角度值寄存器
    raw_data = ReadSensorRegister(0x0021);
    
    // 检查数据有效性（bit15）
    if ((raw_data & 0x8000) == 0) {
        return 0xFFFF; // 数据无效
    }
    
    // 返回15位角度数据
    return raw_data & 0x7FFF;
}

// 读取角度并转换为度
float ReadAngleDegrees(void)
{
    uint16_t raw_angle;
    
    raw_angle = ReadRawAngle();
    
    if (raw_angle == 0xFFFF) {
        return -999.0f; // 错误值
    }
    
    // 转换为角度（度）
    return (raw_angle * 360.0f) / 32768.0f;
}

// 读取角速度
float ReadAngularSpeed(void)
{
    uint16_t raw_speed;
    int16_t signed_speed;
    
    // 读取角速度寄存器
    raw_speed = ReadSensorRegister(0x0022);
    
    // 转换为有符号数
    signed_speed = (int16_t)(raw_speed << 1) >> 1;
    
    // 转换为度/秒
    return signed_speed * 0.015625f;
}

// 诊断函数 - 读取所有关键寄存器
void SensorDiagnostic(void)
{
    uint16_t reg_value;
    char msg[100];
    
    sprintf(msg, "\r\n=== Sensor Diagnostic ===\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    // 读取MOD1寄存器
    reg_value = ReadSensorRegister(0x0081);
    sprintf(msg, "MOD1: 0x%04X\r\n", reg_value);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    // 读取MOD2寄存器
    reg_value = ReadSensorRegister(0x0082);
    sprintf(msg, "MOD2: 0x%04X\r\n", reg_value);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    // 读取MOD3寄存器
    reg_value = ReadSensorRegister(0x0083);
    sprintf(msg, "MOD3: 0x%04X\r\n", reg_value);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    // 读取角度寄存器
    reg_value = ReadSensorRegister(0x0021);
    sprintf(msg, "ANGLE: 0x%04X\r\n", reg_value);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    sprintf(msg, "==========================\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
}
/* USER CODE END 0 */
