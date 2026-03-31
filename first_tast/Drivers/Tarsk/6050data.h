#ifndef __6050data_H
#define __6050data_H

#include "main.h"
#include "i2c.h"

#define ACCEL_SENSITIVITY_2G    16384.0f    // LSB/g
#define ACCEL_SENSITIVITY_4G    8192.0f     // LSB/g  
#define ACCEL_SENSITIVITY_8G    4096.0f     // LSB/g
#define ACCEL_SENSITIVITY_16G   2048.0f     // LSB/g

#define GYRO_SENSITIVITY_250DPS 131.0f      // LSB/��/s
#define GYRO_SENSITIVITY_500DPS 65.5f       // LSB/��/s
#define GYRO_SENSITIVITY_1000DPS 32.8f      // LSB/��/s
#define GYRO_SENSITIVITY_2000DPS 16.4f      // LSB/��/s

#define MPU6050_ADDR (0x68 << 1)

typedef struct {
    int16_t accel_x, accel_y, accel_z;
    int16_t temp;
    int16_t gyro_x, gyro_y, gyro_z;
    
    // 计算后的角度
    float accel_x_g, accel_y_g, accel_z_g;
    float temp_c;
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;
} MPU6050_Data_t;


extern MPU6050_Data_t mpu_data;

void MPU6050_Init(void);

void MPU6050_update(void);
void MPU6050_chuli(void);
void Calibrate_Gyro(void);
void angle_and_acceleration(void);
void Send_Sensor_Data(void);

void MPU6050_get_euler_angle(void);
#endif
