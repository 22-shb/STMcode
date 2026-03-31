#include "6050data.h"                  // Device header
#include "usart.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

MPU6050_Data_t mpu_data;
static float roll = 0,yaw = 0,pitch = 0;
static float comp_alpha = 0.60f;  // 互补滤波系数
const float RAD_TO_DEG = 57.2957795f; // 180/π
float roll_acc,pitch_acc;
float last_yaw,yaw_wucha;
float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;// 陀螺仪零偏校准

typedef struct getdata
{
  float ax;
  float ay;
  float az;
  float tem;
  float gx;
  float gy;
  float gz;

} data_t;
extern data_t data;

void MPU6050_Init(void)
{
    uint8_t SendAddress;
    uint8_t SendData;
    uint16_t devAddr = 0x68 << 1;  // 修正：使用左移后的地址
    
    SendAddress = 0x6B;
    SendData = 0x00; // 解除休眠状态
    HAL_I2C_Mem_Write(&hi2c1, devAddr, SendAddress, I2C_MEMADD_SIZE_8BIT, &SendData, 1, HAL_MAX_DELAY);
    HAL_Delay(100);
    
    SendAddress = 0x19; // 采样率分频器
    SendData = 0x07;    // 125Hz
    HAL_I2C_Mem_Write(&hi2c1, devAddr, SendAddress, I2C_MEMADD_SIZE_8BIT, &SendData, 1, HAL_MAX_DELAY);
    
    SendAddress = 0x1A; // 低通滤波器
    SendData = 0x06;    // 5Hz带宽
    HAL_I2C_Mem_Write(&hi2c1, devAddr, SendAddress, I2C_MEMADD_SIZE_8BIT, &SendData, 1, HAL_MAX_DELAY);
    
    SendAddress = 0x1B; // 陀螺仪
    SendData = 0x08;    // ±500 °/s
    HAL_I2C_Mem_Write(&hi2c1, devAddr, SendAddress, I2C_MEMADD_SIZE_8BIT, &SendData, 1, HAL_MAX_DELAY);
    
    SendAddress = 0x1C; // 加速度计
    SendData = 0x00;    // ±2g
    HAL_I2C_Mem_Write(&hi2c1, devAddr, SendAddress, I2C_MEMADD_SIZE_8BIT, &SendData, 1, HAL_MAX_DELAY);
}


void MPU6050_update(void)
{
    uint8_t buffer[14]; //定义一个数组来存储值
    uint16_t devAddr = 0x68 << 1;
    
    //通过mem函数连续读取14个值，因为他们寄存器是连续的
    if (HAL_I2C_Mem_Read(&hi2c1, devAddr, 0x3B, I2C_MEMADD_SIZE_8BIT, buffer, 14, HAL_MAX_DELAY) == HAL_OK)
    {
        //连续读取14个值
        mpu_data.accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
        mpu_data.accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
        mpu_data.accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);
        mpu_data.temp    = (int16_t)((buffer[6] << 8) | buffer[7]);
        mpu_data.gyro_x  = (int16_t)((buffer[8] << 8) | buffer[9]);
        mpu_data.gyro_y  = (int16_t)((buffer[10] << 8) | buffer[11]);
        mpu_data.gyro_z  = (int16_t)((buffer[12] << 8) | buffer[13]);
		}
}

void MPU6050_chuli(void)//把原始数据转换为角度
{
    
    mpu_data.accel_x_g = mpu_data.accel_x / ACCEL_SENSITIVITY_2G;//算加速度
    mpu_data.accel_y_g = mpu_data.accel_y / ACCEL_SENSITIVITY_2G;//ACCEL_SENSITIVITY_2G在main.h里面
    mpu_data.accel_z_g = mpu_data.accel_z / ACCEL_SENSITIVITY_2G;
    
    
    mpu_data.temp_c = mpu_data.temp / 340.0f + 36.53f;//转换为摄氏度
    
    
    mpu_data.gyro_x_dps = mpu_data.gyro_x / GYRO_SENSITIVITY_500DPS;//算角速度
    mpu_data.gyro_y_dps = mpu_data.gyro_y / GYRO_SENSITIVITY_500DPS;
    mpu_data.gyro_z_dps = mpu_data.gyro_z / GYRO_SENSITIVITY_500DPS;
}

void Calibrate_Gyro(void)//清除零飘
{
    float sum_x = 0, sum_y = 0, sum_z = 0;
    const int samples = 100;
    
    for (int i = 0; i < samples; i++) {
        // 读取陀螺仪数据
        MPU6050_update();
        MPU6050_chuli();
        sum_x += mpu_data.gyro_x_dps;
        sum_y += mpu_data.gyro_y_dps;
        sum_z += mpu_data.gyro_z_dps;
        //HAL_Delay(10);
    }
    
    gyro_bias_x = sum_x / samples;
    gyro_bias_y = sum_y / samples;
    gyro_bias_z = sum_z / samples;
    
}

// 使用校准后的陀螺仪数据

void angle_and_acceleration(void)
{
    /* 使用静态标志量在第一次调用时应用陀螺仪零偏校准 */
    static uint8_t first_run = 1;
    if (first_run == 1) {
        mpu_data.gyro_x_dps -= gyro_bias_x;
        mpu_data.gyro_y_dps -= gyro_bias_y;
        mpu_data.gyro_z_dps -= gyro_bias_z;
        first_run = 0;
    }
    float dt = 0.005;
    //处理陀螺仪角度
    //公式：角度 += 角速度*积分
    roll = roll + mpu_data.gyro_x_dps * dt;
    pitch = pitch + mpu_data.gyro_y_dps * dt;
    yaw = yaw + mpu_data.gyro_z_dps * dt;
   //处理加速度并算出欧拉角
    float ax = mpu_data.accel_x_g;
    float ay = mpu_data.accel_y_g;
    float az = mpu_data.accel_z_g;
 
    // 计算横滚角 Roll (绕X轴旋转)
    // roll = atan2(ay, az)
    roll_acc = atan2(ay, az) * RAD_TO_DEG;
    
    // 计算俯仰角 Pitch (绕Y轴旋转)
    // pitch = atan2(-ax, sqrt(ay*ay + az*az))
    float denom = sqrt(ay * ay + az * az);
    pitch_acc = atan2(-ax, denom) * RAD_TO_DEG;
    
    // 加速度计无法计算偏航角Yaw

    roll_acc  = comp_alpha * roll  + (1 - comp_alpha) * roll_acc;
    pitch_acc = comp_alpha * pitch + (1 - comp_alpha) * pitch_acc;
    //yaw无法更正
    //更新陀螺仪积分为融合后的角度（避免累积误差）
    roll = roll_acc;
    pitch = pitch_acc;
		
}

void Send_Sensor_Data(void)
{
    char buffer[128];  // 创建一个字符数组来存储要发送的文本
    
    sprintf(buffer, 
           "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n", //要用vofa显示波形图不能有文本描述，要去掉文本
           mpu_data.accel_x_g, mpu_data.accel_y_g, mpu_data.accel_z_g,    // 加速度数据
           roll, pitch, yaw,    // 陀螺仪数据  
           mpu_data.temp_c,     //温度数据
           roll_acc, pitch_acc,yaw);//欧拉角 
    
    // (uint8_t*)buffer 把字符数组转换成串口能发送的格式
    // strlen(buffer) 计算字符串的长度
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    // 注释：HAL_UART_Transmit 是阻塞函数，会等待发送完成
}


void MPU6050_get_euler_angle(void)
{
    MPU6050_update();//更新数值
    MPU6050_chuli();//转换为角度
    Calibrate_Gyro();//消除零偏差
    angle_and_acceleration();//计算欧拉角度
    Send_Sensor_Data();//发送数据
}
