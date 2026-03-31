#include "bujin.h"
#include <stdint.h>

#define STEPS_PER_CIRCLE 4096
#define DEG_PER_STEP     (360.0f / STEPS_PER_CIRCLE)
#define IN1_GPIO_Port GPIOA
#define IN1_Pin GPIO_PIN_1
#define IN2_Pin GPIO_PIN_2
#define IN3_Pin GPIO_PIN_3
#define IN4_Pin GPIO_PIN_4

static const uint8_t step_sequence[8][4] = {
    {1, 0, 0, 0},
    {1, 1, 0, 0},
    {0, 1, 0, 0},
    {0, 1, 1, 0},
    {0, 0, 1, 0},
    {0, 0, 1, 1},
    {0, 0, 0, 1},
    {1, 0, 0, 1}
};

void StepMotor_Init(void) {
    // 引脚初始化已由 CubeMX 完成，此处保留接口便于扩展
}

static inline void StepMotor_WritePins(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4) {
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, in1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN2_Pin, in2 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN3_Pin, in3 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN4_Pin, in4 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void StepMotor_DriveOneStep(uint8_t step_index) {
    const uint8_t *seq = step_sequence[step_index % 8];
    StepMotor_WritePins(seq[0], seq[1], seq[2], seq[3]);
}


void StepMotor_SetSpeed(StepMotor_SpeedTypeDef speed, StepMotor_DirectionTypeDef direction, uint32_t step_count) {
    static uint8_t step = 0;

    for (uint32_t i = 0; i < step_count; i++) {
        StepMotor_DriveOneStep(step);

        HAL_Delay((uint32_t)speed); // 使用速度映射的 delay_ms

        if (direction == STEP_MOTOR_DIR_FORWARD) {
            step = (step + 1) % 8;
        } else {
            step = (step + 7) % 8;
        }
    }
}

void StepMotor_RotateAngle(float angle_deg, StepMotor_SpeedTypeDef speed, StepMotor_DirectionTypeDef direction) {
    uint32_t steps = (uint32_t)((angle_deg / DEG_PER_STEP) + 0.5f);
    StepMotor_SetSpeed(speed, direction, steps);
}

void StepMotor_RotateCircle(uint32_t circles, StepMotor_SpeedTypeDef speed, StepMotor_DirectionTypeDef direction) {
    StepMotor_SetSpeed(speed, direction, circles * STEPS_PER_CIRCLE);
}
