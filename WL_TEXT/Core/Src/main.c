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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "font.h"
#include "string.h"
#include "stdio.h"
#include "math.h"
#include "Thermistor.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char str[20];
volatile uint8_t state = 0; //0是关闭，1是开启
//extern uint16_t data[4];
float temperature = 0.0f;

uint32_t led_start_time = 0;
uint8_t led_running = 0; // 标记 LED 是否正在计时
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



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
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(20);
	OLED_Init();
  HAL_TIM_Base_Start_IT(&htim4); // 启动定时器中断
  HAL_TIM_Base_Start(&htim1); // 启动定时器1

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // 启动定时器1的PWM输出

  HAL_ADC_Start_DMA(&hadc1,(uint32_t *)data,sizeof(data)/sizeof(uint16_t));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(state == 0)
    {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // 熄灭LED
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); // 设置占空比为0%
      led_running = 0; // 重置 LED 计时标志
      OLED_NewFrame();
      OLED_PrintString(0, 0, "烘干:停止", &font16x16, OLED_COLOR_NORMAL);
      OLED_ShowFrame();
      // HAL_Delay(100);
    }
    else if(state == 1)
    {
// --- 1. LED 高电平逻辑（非阻塞） ---
        if (led_running == 0) 
        {
            // 刚进入 state 1，点亮 LED 并记录当前时间
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
            led_start_time = HAL_GetTick(); 
            led_running = 1;
        }
        else 
        {
            // 如果已经运行了 1000ms，就熄灭（或者维持原样，根据你的业务逻辑调）
            if (HAL_GetTick() - led_start_time >= 1000) 
            {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
                // 如果你想让它循环亮灭，可以在这里把 led_running 设为 0 重新触发
            }
        }
      
      temperature = thermistor_calculate();
      

      if(temperature >= 32.0f)
      {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 5000); // 设置占空比为100%
      }
      else if(temperature <=30.0f)
      {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
      }

      OLED_NewFrame();
      sprintf(str, "温度:%dC", (int)(temperature + 0.5f)); // 四舍五入取整数
      OLED_PrintString(0, 16, str, &font16x16, OLED_COLOR_NORMAL);
      OLED_PrintString(0, 0, "烘干:开启 ", &font16x16, OLED_COLOR_NORMAL);//如果在“：”后面汉字报错可以在字符串最后加上一个空格
      OLED_ShowFrame();
      // HAL_Delay(500);
    }
    
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM4)
    {
        // 1. 定义静态变量（只初始化一次，保留上次的值）
        static uint8_t last_key_val = 1; // 假设上拉输入，默认高电平
        uint8_t current_key_val;

        // 2. 读取当前按键引脚电平
        current_key_val = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);

        // 3. 核心逻辑：检测下降沿（从1变到0，表示按下）
        // 如果你的按键是按下接高电平，就改成 (last_key_val == 0 && current_key_val == 1)
        if (last_key_val == 1 && current_key_val == 0) 
        {
            // 因为定时器本身就是 20ms 进一次，所以天然避开了抖动
            // 只要检测到跳变，就一定是稳定的按下
            state = !state; // 翻转你的全局状态变量码
            
            // 这里可以放你想执行的操作，比如：
            // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); 
        }

        // 4. 更新旧状态，为下一次比较做准备
        last_key_val = current_key_val;
    }
}
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
#ifdef USE_FULL_ASSERT
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
