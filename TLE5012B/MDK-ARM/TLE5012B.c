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


// 硬件诊断函数
void HardwareDiagnostic(void)
{
    char msg[100];
    
    sprintf(msg, "\r\n=== 硬件诊断开始 ===\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    // 1. 测试SPI基本通信
    uint8_t tx_test[2] = {0x80, 0x81}; // 读取MOD1寄存器
    uint8_t rx_test[2] = {0};
    
    HAL_GPIO_WritePin(GPIOB, 0x0001, 0);
    HAL_SPI_TransmitReceive(&hspi1, tx_test, rx_test, 2, 1000);
    HAL_GPIO_WritePin(GPIOB, 0x0001, 1);
    
    sprintf(msg, "SPI测试 - 发送: 0x8081, 接收: 0x%02X%02X\r\n", rx_test[0], rx_test[1]);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    // 2. 测试CS引脚
    HAL_GPIO_WritePin(GPIOB, 0x0001, 0);
    sprintf(msg, "CS引脚状态: 低电平\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOB, 0x0001, 1);
    sprintf(msg, "CS引脚状态: 高电平\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    // 3. 检查电源
    sprintf(msg, "请检查: 3.3V电源, GND, 磁铁位置\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    sprintf(msg, "=== 硬件诊断结束 ===\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
}

// 简单的通信测试（不使用CRC）
void SimpleCommunicationTest(void)
{
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];
    char msg[100];
    uint16_t result;
    
    sprintf(msg, "\r\n=== 简单通信测试 ===\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    // 测试1: 发送0x0000
    tx_buf[0] = 0x00; tx_buf[1] = 0x00;
    HAL_GPIO_WritePin(GPIOB, 0x0001, 0);
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 2, 1000);
    HAL_GPIO_WritePin(GPIOB, 0x0001, 1);
    result = (rx_buf[0] << 8) | rx_buf[1];
    sprintf(msg, "发送0x0000 -> 接收: 0x%04X\r\n", result);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    // 测试2: 发送0xFFFF
    tx_buf[0] = 0xFF; tx_buf[1] = 0xFF;
    HAL_GPIO_WritePin(GPIOB, 0x0001, 0);
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 2, 1000);
    HAL_GPIO_WritePin(GPIOB, 0x0001, 1);
    result = (rx_buf[0] << 8) | rx_buf[1];
    sprintf(msg, "发送0xFFFF -> 接收: 0x%04X\r\n", result);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    // 测试3: 尝试读取角度
    tx_buf[0] = 0x80; tx_buf[1] = 0x21;
    HAL_GPIO_WritePin(GPIOB, 0x0001, 0);
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 2, 1000);
    HAL_GPIO_WritePin(GPIOB, 0x0001, 1);
    
    HAL_Delay(10);
    
    tx_buf[0] = 0xFF; tx_buf[1] = 0xFF;
    HAL_GPIO_WritePin(GPIOB, 0x0001, 0);
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 2, 1000);
    HAL_GPIO_WritePin(GPIOB, 0x0001, 1);
    result = (rx_buf[0] << 8) | rx_buf[1];
    sprintf(msg, "角度读取测试 -> 接收: 0x%04X\r\n", result);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    if (result == 0x0000 || result == 0xFFFF) {
        sprintf(msg, "警告: 可能没有接收到有效数据\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    } else {
        sprintf(msg, "数据有效位: 0x%04X\r\n", result & 0x7FFF);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    }
}
/* USER CODE END 0 */






/* USER CODE BEGIN 0 */
#include <stdio.h>
#include <string.h>

// 根据博客的CRC计算函数
uint8_t CalculateCRC2(uint16_t data)
{
    uint8_t crc = 0;
    uint8_t input_byte;
    const uint8_t crc_table[16] = {
        0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
        0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D
    };
    
    // 处理高字节
    input_byte = (data >> 8) & 0xFF;
    crc = crc_table[((input_byte >> 4) & 0x0F) ^ crc];
    crc = crc_table[(input_byte & 0x0F) ^ crc];
    
    // 处理低字节
    input_byte = data & 0xFF;
    crc = crc_table[((input_byte >> 4) & 0x0F) ^ crc];
    crc = crc_table[(input_byte & 0x0F) ^ crc];
    
    return crc;
}

// 构建SSC帧（根据博客：CRC在bit12-15）
uint16_t BuildSSCFrame(uint16_t data)
{
    uint8_t crc = CalculateCRC(data);
    return (crc << 12) | (data & 0x0FFF);
}

// 提取SSC帧中的数据（去掉CRC）
uint16_t ExtractDataFromSSCFrame(uint16_t ssc_frame)
{
    return ssc_frame & 0x0FFF;
}

// 验证SSC帧的CRC
uint8_t VerifySSCFrame(uint16_t ssc_frame)
{
    uint16_t data = ssc_frame & 0x0FFF;
    uint8_t received_crc = (ssc_frame >> 12) & 0x0F;
    uint8_t calculated_crc = CalculateCRC(data);
    
    return (received_crc == calculated_crc);
}

// 单次SSC传输
uint16_t SSC_Transfer(uint16_t data)
{
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];
    uint16_t ssc_frame;
    uint16_t response;
    
    // 构建SSC帧
    ssc_frame = BuildSSCFrame(data);
    
    tx_buf[0] = (ssc_frame >> 8) & 0xFF;
    tx_buf[1] = ssc_frame & 0xFF;
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 2, 1000);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    
    response = (rx_buf[0] << 8) | rx_buf[1];
    return response;
}

// 读取寄存器（SSC协议）
uint16_t SSC_ReadRegister(uint16_t reg_addr)
{
    uint16_t response;
    
    // 第一次传输：发送读取命令
    SSC_Transfer(0x8000 | reg_addr);
    HAL_Delay(1);
    
    // 第二次传输：读取数据
    response = SSC_Transfer(0x0000);
    
    // 验证CRC
    if (!VerifySSCFrame(response)) {
        return 0xFFFF; // CRC错误
    }
    
    return ExtractDataFromSSCFrame(response);
}

// 写入寄存器（SSC协议）
void SSC_WriteRegister(uint16_t reg_addr, uint16_t data)
{
    // 第一次传输：发送写入命令
    SSC_Transfer(reg_addr); // 注意：写入命令没有0x8000
    HAL_Delay(1);
    
    // 第二次传输：发送数据
    SSC_Transfer(data);
}
// 完整的角度读取函数（基于博客原理）
float ReadAngle_SSC(void)
{
    uint16_t raw_data;
    float angle;
    
    // 读取角度寄存器（0x0021）
    raw_data = SSC_ReadRegister(0x0021);
    
    if (raw_data == 0xFFFF) {
        return -999.0f; // 读取失败
    }
    
    // 检查数据有效性（bit15）
    if ((raw_data & 0x8000) == 0) {
        return -998.0f; // 数据无效
    }
    
    // 提取15位角度数据
    raw_data = raw_data & 0x7FFF;
    
    // 转换为角度（度）
    angle = (raw_data * 360.0f) / 32768.0f;
    
    return angle;
}

// 读取角速度
float ReadAngularSpeed_SSC(void)
{
    uint16_t raw_speed;
    int16_t signed_speed;
    
    // 读取角速度寄存器（0x0022）
    raw_speed = SSC_ReadRegister(0x0022);
    
    if (raw_speed == 0xFFFF) {
        return -999.0f;
    }
    
    // 转换为有符号数
    signed_speed = (int16_t)(raw_speed << 1) >> 1;
    
    // 转换为度/秒
    return signed_speed * 0.015625f;
}
// 设备初始化
void TLE5012_Init_SSC(void)
{
    uint16_t device_id;
    char msg[100];
    
    sprintf(msg, "TLE5012B SSC协议初始化...\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    // 读取MOD1寄存器验证设备
    device_id = SSC_ReadRegister(0x0081);
    
    sprintf(msg, "设备ID: 0x%04X\r\n", device_id);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    // 验证设备ID（低5位应为00001b）
    if ((device_id & 0x001F) == 0x0001) {
        sprintf(msg, "TLE5012B检测成功\r\n");
    } else {
        sprintf(msg, "设备ID验证失败\r\n");
    }
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    // 配置MOD2寄存器（滤波器设置）
    SSC_WriteRegister(0x0082, 0x0801); // 启用预测器 + 200Hz带宽
    HAL_Delay(10);
    
    // 配置MOD3寄存器（接口设置）
    SSC_WriteRegister(0x0083, 0x0004); // 自动更新模式
    HAL_Delay(10);
    
    sprintf(msg, "初始化完成\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
}
// CRC测试函数
void TestCRC_Calculation(void)
{
    char msg[100];
    uint16_t test_data[] = {0x8021, 0x0000, 0xFFFF, 0x1234};
    uint8_t i;
    
    sprintf(msg, "\r\n=== CRC计算测试 ===\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    for (i = 0; i < 4; i++) {
        uint16_t data = test_data[i];
        uint8_t crc = CalculateCRC(data);
        uint16_t ssc_frame = BuildSSCFrame(data);
        
        sprintf(msg, "数据: 0x%04X, CRC: 0x%X, SSC帧: 0x%04X\r\n", 
                data, crc, ssc_frame);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    }
    
    sprintf(msg, "=== CRC测试结束 ===\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
}

// SSC通信测试
void TestSSC_Communication(void)
{
    char msg[100];
    uint16_t response;
    
    sprintf(msg, "\r\n=== SSC通信测试 ===\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    // 测试读取MOD1寄存器
    response = SSC_ReadRegister(0x0081);
    
    if (response == 0xFFFF) {
        sprintf(msg, "SSC通信失败 - CRC错误或无响应\r\n");
    } else {
        sprintf(msg, "SSC通信成功 - MOD1: 0x%04X\r\n", response);
    }
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    // 测试读取角度
    float angle = ReadAngle_SSC();
    if (angle > -900.0f) {
        sprintf(msg, "角度读取成功: %.2f度\r\n", angle);
    } else {
        sprintf(msg, "角度读取失败: %.1f\r\n", angle);
    }
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    sprintf(msg, "=== SSC通信测试结束 ===\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
}

// SPI模式测试
void TestSPI_Modes(void)
{
    char msg[100];
    
    sprintf(msg, "\r\n=== SPI模式测试 ===\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    
    // 根据博客，TLE5012B通常使用SPI模式1或3
    
    // 测试模式1 (CPOL=0, CPHA=1)
    hspi1.Init.CLKPolarity = 0; // SPI_POLARITY_LOW
    hspi1.Init.CLKPhase = 1;    // SPI_PHASE_2EDGE
    HAL_SPI_Init(&hspi1);
    
    sprintf(msg, "测试SPI模式1 (CPOL=0, CPHA=1)...\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    TestSSC_Communication();
    
    HAL_Delay(2000);
    
    // 测试模式3 (CPOL=1, CPHA=1)
    hspi1.Init.CLKPolarity = 1; // SPI_POLARITY_HIGH
    hspi1.Init.CLKPhase = 1;    // SPI_PHASE_2EDGE
    HAL_SPI_Init(&hspi1);
    
    sprintf(msg, "测试SPI模式3 (CPOL=1, CPHA=1)...\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
    TestSSC_Communication();
}


