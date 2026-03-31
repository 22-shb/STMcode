/*
 * @File: foc_driver.h
 * @Author: SimpleFOC C Port
 * @Description: FOC Core based on SimpleFOC Architecture
 */

#ifndef __FOC_DRIVER_H
#define __FOC_DRIVER_H

#include "main.h"
#include <math.h>
#include <string.h>

// 常量定义
#define PI 3.14159265358979f
#define _3PI_2 4.71238898038469f
#define _SQRT3 1.73205080757f

// 限制函数
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

// 控制模式
typedef enum {
    FOC_MODE_OPEN_LOOP,
    FOC_MODE_TORQUE,
    FOC_MODE_VELOCITY,
    FOC_MODE_ANGLE
} FOC_ControlMode_e;

// --- SimpleFOC 核心组件 1: 低通滤波器 ---
typedef struct {
    float Tf; // 时间常数 (Time constant)
    float prev_output;
} LowPassFilter_t;

// --- SimpleFOC 核心组件 2: PID控制器 ---
typedef struct {
    float P;
    float I;
    float D;
    float output_ramp; // 输出变化率限制 (Volts/sec)
    float limit;       // 输出限幅
    
    // 内部状态
    float error_prev;
    float output_prev;
    float integral_prev;
} PID_Controller_t;

// 编码器
typedef struct {
    float angle_raw;
    float angle_prev;
    float angle_consequent; // 累计角度
    float velocity;
    float full_rotations;
} Encoder_t;

// PWM 句柄
typedef struct {
    TIM_HandleTypeDef *tim1;
    TIM_HandleTypeDef *tim2;
    TIM_HandleTypeDef *tim3;
    uint32_t PIN1; 
    uint32_t PIN2;
    uint32_t PIN3;
} FOC_PWM_Handle_t;

// FOC 主对象
typedef struct {
    // 硬件
    TIM_HandleTypeDef *tim;
    FOC_PWM_Handle_t PWMPin;
    
    // 参数
    int dir;
    int pp;
    float voltage_ps;
    float voltage_limit;
    float zero_electric_angle;

    // 状态
    float electrical_angle;
    float shaft_angle;      // 机械角度 (rad)
    float shaft_velocity;   // 机械速度 (rad/s)
    
    // 变换变量
    float Ualpha, Ubeta;
    float Ua, Ub, Uc;

    // 控制模式
    FOC_ControlMode_e mode;
    float target;
    
    // 控制器
    PID_Controller_t pid_vel;
    PID_Controller_t pid_pos;
    LowPassFilter_t  lpf_vel; // 速度滤波

    // 传感器
    Encoder_t encoder;

} FOC_t;

// API
void FOC_Init(FOC_t *hfoc, float v_supply, float v_limit, int dir, int pp);
void FOC_PWM_Config(FOC_t *hfoc, TIM_HandleTypeDef *t1, TIM_HandleTypeDef *t2, TIM_HandleTypeDef *t3, uint32_t ch1, uint32_t ch2, uint32_t ch3);
void FOC_Alignment(FOC_t *hfoc);
void FOC_Loop(FOC_t *hfoc, float dt);

// PID & LPF 工具函数
void PID_Init(PID_Controller_t *pid, float P, float I, float D, float ramp, float limit);
float PID_Operator(PID_Controller_t *pid, float error, float dt);
void LPF_Init(LowPassFilter_t *lpf, float time_constant);
float LPF_Operator(LowPassFilter_t *lpf, float x, float dt);

// 外部依赖
extern uint16_t AS5600_ReadRawAngle(void);

#endif