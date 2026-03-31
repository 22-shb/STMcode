#include "stm32f10x.h"                  // Device header


// 按键消抖时间(ms)
#define KEY_DEBOUNCE_TIME   20

// 按键状态变量
static uint8_t key1_state = KEY_UP;
static uint8_t key2_state = KEY_UP;

/**
 * @brief 按键初始化
 */
void KEY_Init(void)
{
    // 引脚已经在CubeMX中配置为上拉输入模式
    // 这里可以添加额外的初始化代码
}

/**
 * @brief 简单按键扫描函数
 * @param GPIO_Pin: 按键引脚
 * @param GPIOx: 按键端口
 * @return 1:按键按下, 0:按键释放
 */
uint8_t KEY_Scan(uint16_t GPIO_Pin, GPIO_TypeDef* GPIOx)
{
    // 读取按键电平(假设低电平有效)
    if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET)
    {
        // 简单延时消抖
        HAL_Delay(KEY_DEBOUNCE_TIME);
        
        // 再次检测确认按键按下
        if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET)
        {
            // 等待按键释放
            while(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET);
            
            // 返回按键按下标志
            return 1;
        }
    }
    
    return 0;
}

/**
 * @brief 非阻塞按键扫描函数(推荐在主循环中使用)
 * @param GPIO_Pin: 按键引脚  
 * @param GPIOx: 按键端口
 * @return 1:检测到按键按下, 0:无按键按下
 */
uint8_t KEY_Scan_NonBlocking(uint16_t GPIO_Pin, GPIO_TypeDef* GPIOx)
{
    static uint32_t key_last_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 检查按键引脚电平(低电平有效)
    if(HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) == GPIO_PIN_RESET)
    {
        // 消抖时间检查
        if((current_time - key_last_time) > KEY_DEBOUNCE_TIME)
        {
            key_last_time = current_time;
            return 1;
        }
    }
    
    return 0;
}

/**
 * @brief 专用按键1扫描函数
 * @return 1:按键1按下, 0:无按下
 */
uint8_t KEY1_Scan(void)
{
    return KEY_Scan_NonBlocking(KEY1_PIN, KEY1_PORT);
}

/**
 * @brief 专用按键2扫描函数  
 * @return 1:按键2按下, 0:无按下
 */
uint8_t KEY2_Scan(void)
{
    return KEY_Scan_NonBlocking(KEY2_PIN, KEY2_PORT);
}