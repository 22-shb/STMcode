#ifndef THERMISTOR_H
#define THERMISTOR_H

#include "usart.h"
#include "string.h"
#include "stdio.h"
#include "math.h"
// 按键消抖时间(ms)
#define KEY_DEBOUNCE_TIME   20
#define R25         10000.0f
#define B_VALUE     3950.0f
#define R_FIXED     10000.0f
#define ADC_RES     4096.0f

extern uint16_t data[4];
extern char b[30];

float thermistor_calculate(void);

#endif
