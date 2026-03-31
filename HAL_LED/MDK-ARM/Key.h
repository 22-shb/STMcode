#ifndef __STM32_KEY_SIMPLE_H
#define __STM32_KEY_SIMPLE_H

#include "main.h"
#include "gpio.h"

// 按键引脚定义 - 根据实际硬件修改
#define KEY1_PIN        GPIO_PIN_0
#define KEY1_PORT       GPIOA
#define KEY2_PIN        GPIO_PIN_1  
#define KEY2_PORT       GPIOA

// 按键状态
typedef enum {
    KEY_UP = 0,     // 按键抬起
    KEY_DOWN = 1    // 按键按下
} Key_Status;

// 函数声明
void KEY_Init(void);
uint8_t KEY_Scan(uint16_t GPIO_Pin, GPIO_TypeDef* GPIOx);

#endif