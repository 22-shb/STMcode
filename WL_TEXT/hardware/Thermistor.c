#include "Thermistor.h"

 uint16_t data[4];
char b[30];

float thermistor_calculate(void)
{
    // 1. 直接获取最新的一笔 ADC 原始数据，去掉滑动平均和延时
    float current_adc = (float)data[0];

    // 2. 数据合法性拦截 (跳过短路或断路的数据)
    if (current_adc < 10.0f || current_adc > 4085.0f) return 0;

    // 3. 【已移除一阶低通滤波】直接拿原始 current_adc 算温度
    // 接法A (NTC接地)
    float Rt = R_FIXED * current_adc / (ADC_RES - current_adc);

    float kelvin = 1.0f / (1.0f / 298.15f + (1.0f / B_VALUE) * logf(Rt / R25));
    float final_celsius = kelvin - 273.15f;

    // 4. 打印逻辑
    int temp_int = (int)(final_celsius * 100);
    int whole = abs(temp_int / 100);
    int fraction = abs(temp_int % 100); 
    
    sprintf(b, "Temp: %s%d.%02d C\r\n", (final_celsius < 0 ? "-" : ""), whole, fraction);
    HAL_UART_Transmit(&huart1, (uint8_t *)b, strlen(b), 100);

    return final_celsius;
}


