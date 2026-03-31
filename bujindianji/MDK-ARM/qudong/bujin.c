#include "bujin.h"

// 四相八拍步进序列（更精细的控制）
static const uint8_t step_sequence[8][4] = {
  {1, 0, 0, 0},  // A
  {1, 1, 0, 0},  // AB
  {0, 1, 0, 0},  // B
  {0, 1, 1, 0},  // BC
  {0, 0, 1, 0},  // C
  {0, 0, 1, 1},  // CD
  {0, 0, 0, 1},  // D
  {1, 0, 0, 1}   // DA
};

void Stepper_Init(Stepper_Motor* motor)
{
  // 初始化电机参数
  motor->port = MOTOR_PORT;
  motor->pin1 = MOTOR_PIN1;
  motor->pin2 = MOTOR_PIN2;
  motor->pin3 = MOTOR_PIN3;
  motor->pin4 = MOTOR_PIN4;
  motor->speed = 10;
  motor->direction = DIRECTION_CW;
  motor->step_count = 0;
  
  // 停止电机
  Stepper_Stop(motor);
}

void Stepper_SetSpeed(Stepper_Motor* motor, uint16_t speed)
{
  // 速度值越小，电机转动越快（最小建议不小于3）
  if (speed < 3) speed = 3;
  motor->speed = speed;
}

void Stepper_Step(Stepper_Motor* motor, uint32_t steps)
{
  uint32_t i;
  uint8_t step_index;
  
  for (i = 0; i < steps; i++) {
    // 根据方向确定步进索引
    if (motor->direction == DIRECTION_CW) {
      step_index = motor->step_count;
    } else {
      step_index = 7 - motor->step_count;
    }
    
    // 设置引脚输出状态
    HAL_GPIO_WritePin(motor->port, motor->pin1, 
                      step_sequence[step_index][0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->port, motor->pin2, 
                      step_sequence[step_index][1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->port, motor->pin3, 
                      step_sequence[step_index][2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->port, motor->pin4, 
                      step_sequence[step_index][3] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    
    // 更新步数计数器
    motor->step_count++;
    if (motor->step_count >= 8) {
      motor->step_count = 0;
    }
    
    // 延时控制速度
    HAL_Delay(motor->speed);
  }
}

void Stepper_Stop(Stepper_Motor* motor)
{
  // 关闭所有引脚，停止电机
  HAL_GPIO_WritePin(motor->port, motor->pin1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(motor->port, motor->pin2, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(motor->port, motor->pin3, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(motor->port, motor->pin4, GPIO_PIN_RESET);
}
