/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float Temperature;
    float Humidity;
} DHT22_Data_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DHT22_PORT GPIOA
#define DHT22_PIN  GPIO_PIN_1
// PA1 的配置位在 CRL 寄存器的第 4-7 位 (每个引脚占 4 位)
// 0x3 是推挽输出 (50MHz), 0x8 是带上拉/下拉输入 (F103 特性)
#define DHT22_IO_OUT() {GPIOA->CRL &= 0xFFFFFF0F; GPIOA->CRL |= 0x00000030;}
#define DHT22_IO_IN()  {GPIOA->CRL &= 0xFFFFFF0F; GPIOA->CRL |= 0x00000080; GPIOA->ODR |= GPIO_PIN_1;}

// 快捷读写宏
#define DHT22_DQ_OUT(x) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, x ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define DHT22_DQ_IN     HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void DWT_Init(void);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
 float Temperature, Humidity;
uint8_t data[5];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 切换引脚为输出模式
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

// 切换引脚为输入模式
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us) {
    uint32_t startTick = DWT->CYCCNT;
    // 使用 HAL_RCC_GetHCLKFreq() 动态获取真实主频，绝对不会出错
    uint32_t delayTicks = us * (HAL_RCC_GetHCLKFreq() / 1000000);
    while (DWT->CYCCNT - startTick < delayTicks);
}
// 读取 DHT22 数据
// 读取 DHT22 数据
uint8_t DHT22_Read_Data(DHT22_Data_t *DataStruct) {
    uint8_t i, j;
    uint16_t timeout = 0;

    // 【修复 1】：每次读取前，必须清空历史数据！
    for (i = 0; i < 5; i++) {
        data[i] = 0;
    }

    // 【修复 2】：利用开漏特性，输出 0 拉低总线
    DHT22_DQ_OUT(0);
    delay_us(1000);      // 拉低 1ms (手册要求 >1ms)
    
    // 输出 1 释放总线，因为是开漏+上拉，此时相当于变成了输入模式
    DHT22_DQ_OUT(1);
    delay_us(30);        // 拉高 30us
    
    // ------------------ 等待传感器响应 ------------------
    timeout = 0;
    // 等待 DHT22 拉低 (响应信号)
    while (DHT22_DQ_IN && timeout < 100) { 
        delay_us(1); 
        timeout++; 
    }
    // 【修复 3】：修正超时判断阈值
    if (timeout >= 100) return 0; // 超时失败

    // 等待 DHT22 释放总线 (变回高电平)
    timeout = 0;
    while (!DHT22_DQ_IN && timeout < 100) { 
        delay_us(1); 
        timeout++; 
    }
    if (timeout >= 100) return 0;

 // ------------------ 3. 开始接收 40 位数据 ------------------
    __disable_irq();  // 🔴 临时关闭全局中断！防止 SysTick 打断微秒级时序

    for (i = 0; i < 5; i++) {
        for (j = 0; j < 8; j++) {
            // 等待总线变高
            timeout = 0;
            while (!DHT22_DQ_IN && timeout < 100) { delay_us(1); timeout++; }
            if (timeout >= 100) { __enable_irq(); return 0; } // ⚠️ 异常退出前，千万别忘了开中断
            
            // 延时 40us
            delay_us(40); 
            
            // 如果 40us 后还是高电平，说明是数据 '1'
            if (DHT22_DQ_IN) {
                data[i] |= (1 << (7 - j));
                
                // 等待高电平结束
                timeout = 0;
                while (DHT22_DQ_IN && timeout < 100) { delay_us(1); timeout++; }
                if (timeout >= 100) { __enable_irq(); return 0; } // ⚠️ 异常退出前，千万别忘了开中断
            }
        }
    }

    __enable_irq();  // 🟢 40位数据安全读完，立刻恢复全局中断
    // -----------------------------------------------------------
    // 4. 校验与计算
    if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
        // 湿度 = (高8位 << 8 | 低8位) / 10
        DataStruct->Humidity = (float)((data[0] << 8) | data[1]) / 10.0f;
        
        // 温度处理 (最高位为 1 代表负数)
        int16_t rawTemp = ((data[2] & 0x7F) << 8) | data[3];
        if (data[2] & 0x80) rawTemp = -rawTemp;
        DataStruct->Temperature = (float)rawTemp / 10.0f;
        
        return 1; // 读取成功
    }
    
    return 0; // 校验失败
}
uint16_t tem_and_hum(void)
{
	
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init();
  uint8_t txBuffer[50];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  DHT22_Data_t myDHT22;
	HAL_Delay(3000);
  while (1)
  {

    if (DHT22_Read_Data(&myDHT22)) {
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);
            // 读取成功，可以在这里通过串口打印
      // HAL_UART_Transmit(&huart1,data,5,HAL_MAX_DELAY);
      sprintf((char *)txBuffer, "Temp: %.1f C, Humi: %.1f %%\r\n", myDHT22.Temperature, myDHT22.Humidity);
      HAL_UART_Transmit(&huart1, txBuffer, strlen((char *)txBuffer), 100);
      // printf("Temp: %.1f C, Humi: %.1f %%\r\n", myDHT22.Temperature, myDHT22.Humidity);
        } else {
          HAL_UART_Transmit(&huart1,(uint8_t *)"DHT22 Read Failed\r\n", 20, HAL_MAX_DELAY);
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);
        }
		HAL_Delay(2000); // 必须延时 2 秒，否则传感器不理你
//		if (DHT22_Start()) {
//    data[0] = DHT22_Read_Byte(); // 湿度高8位
//    data[1] = DHT22_Read_Byte(); // 湿度低8位
//    data[2] = DHT22_Read_Byte(); // 温度高8位
//    data[3] = DHT22_Read_Byte(); // 温度低8位
//    data[4] = DHT22_Read_Byte(); // 校验位

//    if (data[4] == (uint8_t)(data[0] + data[1] + data[2] + data[3])) {
//        Humidity = (float)((data[0] << 8) | data[1]) / 10.0;
//        Temperature = (float)((data[2] << 8) | data[3]) / 10.0;
//    }
//}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
