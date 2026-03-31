#ifndef BUJIN_H
#define BUJIN_H

#include "stm32f1xx_hal.h"
#include "main.h"

// 使用说明：
// 1. 在STM32CubeMX初始化4个GPIO输出引脚，命名为 IN1, IN2, IN3, IN4
// 2. 添加本驱动源码文件到工程，即可调用控制函数
// 3. 函数中提到的正反应该按照你当前工程需求自行进行调整
// 4. 此套代码仅为演示28byj48步进电机的驱动原理，若想要进一步扩展，直接复制代码喂给AI，开始提要求就行
// 5. 理论来说，该款步进电机可以传入一个600us~1ms的步进驱动延时，但是为了精简演示，只使用ms级别，所以步进电机可能转速很慢，这是正常现象

/*
硬件接线：
28byj48         MCU
PC0     ->      IN1
PC1     ->      IN2
PC2     ->      IN3
PC3     ->      IN4

5V      ->      5V
GND     ->      GND
//也可以自定义引脚适配个人工程
*/


// 步进电机方向定义
typedef enum {
    STEP_MOTOR_DIR_FORWARD = 0,
    STEP_MOTOR_DIR_BACKWARD = 1
} StepMotor_DirectionTypeDef;

// 步进电机速度等级（对应 delay_ms）
typedef enum {
    SPEED_ULTRAFAST = 1,   // 单位：毫秒
    SPEED_FAST      = 3,
    SPEED_NORMAL    = 8,
    SPEED_SLOW      = 15
} StepMotor_SpeedTypeDef;

// 公共接口
void StepMotor_Init(void);
void StepMotor_SetSpeed(StepMotor_SpeedTypeDef speed, StepMotor_DirectionTypeDef direction, uint32_t step_count);
void StepMotor_RotateAngle(float angle_deg, StepMotor_SpeedTypeDef speed, StepMotor_DirectionTypeDef direction);
void StepMotor_RotateCircle(uint32_t circles, StepMotor_SpeedTypeDef speed, StepMotor_DirectionTypeDef direction);

#endif /* STEP_MOTOR_H */
