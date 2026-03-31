#ifndef __STEPPER_MOTOR_H
#define __STEPPER_MOTOR_H

#include "stm32f1xx_hal.h"

// 电机引脚定义
#define MOTOR_PORT GPIOA
#define MOTOR_PIN1 GPIO_PIN_0
#define MOTOR_PIN2 GPIO_PIN_1
#define MOTOR_PIN3 GPIO_PIN_2
#define MOTOR_PIN4 GPIO_PIN_3

// 电机旋转方向
#define DIRECTION_CW  1   // 顺时针
#define DIRECTION_CCW 0   // 逆时针

// 电机结构体
typedef struct {
  GPIO_TypeDef* port;
  uint16_t pin1;
  uint16_t pin2;
  uint16_t pin3;
  uint16_t pin4;
  uint16_t speed;     // 转速控制参数
  uint8_t direction;  // 旋转方向
  uint8_t step_count; // 当前步数
} Stepper_Motor;

// 函数声明
void Stepper_Init(Stepper_Motor* motor);
void Stepper_SetSpeed(Stepper_Motor* motor, uint16_t speed);
void Stepper_Step(Stepper_Motor* motor, uint32_t steps);
void Stepper_Stop(Stepper_Motor* motor);

#endif
