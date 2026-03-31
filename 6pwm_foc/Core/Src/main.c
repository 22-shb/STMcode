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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI          3.14159265358979323846f

//初始变量及函数定义
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
//宏定义实现的一个约束函数,用于限制一个值的范围。
//具体来说，该宏定义的名称为 _constrain，接受三个参数 amt、low 和 high，分别表示要限制的值、最小值和最大值。该宏定义的实现使用了三元运算符，根据 amt 是否小于 low 或大于 high，返回其中的最大或最小值，或者返回原值。
//换句话说，如果 amt 小于 low，则返回 low；如果 amt 大于 high，则返回 high；否则返回 amt。这样，_constrain(amt, low, high) 就会将 amt 约束在 [low, high] 的范围内。

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float voltage_power_supply=1;
float shaft_angle=0,open_loop_timestamp=0;
float zero_electric_angle=0,Ualpha,Ubeta=0,Ua=0,Ub=0,Uc=0,dc_a=0,dc_b=0,dc_c=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// 电角度求解
float _electricalAngle(float shaft_angle, int pole_pairs) {
  return (shaft_angle * pole_pairs);
}

// 归一化角度到 [0,2PI]
float _normalizeAngle(float angle){
  float a = fmod(angle, 2*PI);   //取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + 2*PI);  
  //三目运算符。格式：condition ? expr1 : expr2 
  //其中，condition 是要求值的条件表达式，如果条件成立，则返回 expr1 的值，否则返回 expr2 的值。可以将三目运算符视为 if-else 语句的简化形式。
  //fmod 函数的余数的符号与除数相同。因此，当 angle 的值为负数时，余数的符号将与 _2PI 的符号相反。也就是说，如果 angle 的值小于 0 且 _2PI 的值为正数，则 fmod(angle, _2PI) 的余数将为负数。
  //例如，当 angle 的值为 -PI/2，_2PI 的值为 2PI 时，fmod(angle, _2PI) 将返回一个负数。在这种情况下，可以通过将负数的余数加上 _2PI 来将角度归一化到 [0, 2PI] 的范围内，以确保角度的值始终为正数。
}

//void setPwm(float Ua, float Ub, float Uc) 
//{
//	  // 计算占空比
//  // 限制占空比从0到1
//  dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
//  dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
//  dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );
//	                                                                                                                                                                                                                          
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,dc_a*htim1.Init.Period);
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,dc_b*htim1.Init.Period);
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,dc_c*htim1.Init.Period);
//}



// 修正的PWM设置函数 - 针对中央对齐模式
void setPwm(float Ua, float Ub, float Uc) 
{
    // 限制相电压在0到电源电压之间
    dc_a = _constrain(Ua, 0.0f, voltage_power_supply);
    dc_b = _constrain(Ub, 0.0f, voltage_power_supply);
    dc_c = _constrain(Uc, 0.0f, voltage_power_supply);
    
    // 对于PWM模式2（中央对齐模式1）
    // 实际占空比 = CCR / ARR
    // 但需要根据实际硬件连接验证极性
    
    // 方法1：直接计算占空比（假设高电平有效）
    uint32_t ccr_a = (uint32_t)(dc_a * 7200 / voltage_power_supply);
    uint32_t ccr_b = (uint32_t)(dc_b * 7200 / voltage_power_supply);
    uint32_t ccr_c = (uint32_t)(dc_c * 7200 / voltage_power_supply);
    
    // 安全限制
    ccr_a = ccr_a > 7200 ? 7200 : ccr_a;
    ccr_b = ccr_b > 7200 ? 7200 : ccr_b;
    ccr_c = ccr_c > 7200 ? 7200 : ccr_c;
    
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_a);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr_b);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr_c);
}

void setPhaseVoltage(float Uq, float Ud, float angle_el) {
    angle_el = _normalizeAngle(angle_el + zero_electric_angle);
    
    // 帕克逆变换
    Ualpha = Ud * cosf(angle_el) - Uq * sinf(angle_el);
    Ubeta = Ud * sinf(angle_el) + Uq * cosf(angle_el);
    
    // 空间矢量PWM - 使用更稳定的计算方法
    // 直接计算三相电压
    Ua = Ualpha;
    Ub = -0.5f * Ualpha + 0.8660254f * Ubeta;
    Uc = -0.5f * Ualpha - 0.8660254f * Ubeta;
    
    // 计算电压偏移到0-voltage_power_supply范围
    float v_offset = (fmaxf(fmaxf(Ua, Ub), Uc) + fminf(fminf(Ua, Ub), Uc)) / 2.0f;
    
    Ua = Ua - v_offset + voltage_power_supply / 2.0f;
    Ub = Ub - v_offset + voltage_power_supply / 2.0f;
    Uc = Uc - v_offset + voltage_power_supply / 2.0f;
    
    setPwm(Ua, Ub, Uc);
}

float velocityOpenloop(float target_velocity) {
    static uint32_t last_time = 0;
    uint32_t now_ms = HAL_GetTick();
    
    // 计算时间间隔（秒）
    float Ts = (float)(now_ms - last_time) * 1e-3f;
    
    // 如果时间间隔异常，使用固定时间
    if (Ts <= 0 || Ts > 0.1f) {
        Ts = 0.001f;  // 1ms
    }
    
    // 累计角度
    shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts);
    
    // 降低电压梯度，避免突变
    static float current_Uq = 0;
    float target_Uq = voltage_power_supply / 3.0f;
    
    // 添加低通滤波，减少突变
    float alpha = 0.1f;  // 滤波系数
    current_Uq = current_Uq * (1 - alpha) + target_Uq * alpha;
    
    setPhaseVoltage(current_Uq, 0, _electricalAngle(shaft_angle, 7));
    
    last_time = now_ms;
    return current_Uq;
}
// 在PWM设置函数中添加死区补偿
void setPwmWithDeadtime(float Ua, float Ub, float Uc) {
    // 假设死区时间为100ns，PWM周期为1/(72MHz/7200)≈100us
    float deadtime_compensation = 0.01f;  // 1%补偿
    
    // 应用死区补偿
    Ua = Ua * (1.0f - deadtime_compensation);
    Ub = Ub * (1.0f - deadtime_compensation);
    Uc = Uc * (1.0f - deadtime_compensation);
    
    setPwm(Ua, Ub, Uc);
}

//void setPhaseVoltage(float Uq,float Ud, float angle_el) {
//  angle_el = _normalizeAngle(angle_el + zero_electric_angle);
//  // 帕克逆变换
//Ualpha = Ud*cos(angle_el) - Uq*sin(angle_el);
//Ubeta = Ud*sin(angle_el) + Uq*cos(angle_el);

//  // 克拉克逆变换
//  Ua = Ualpha + voltage_power_supply/2;
//Ub = -0.5f * Ualpha + 0.86602540378f * Ubeta + voltage_power_supply/2;
//Uc = -0.5f * Ualpha - 0.86602540378f * Ubeta + voltage_power_supply/2;
//  setPwm(Ua,Ub,Uc);
//}

//float velocityOpenloop(float target_velocity){
//  unsigned long now_us = HAL_GetTick();  //获取从开启芯片以来的微秒数，它的精度是 4 微秒。 micros() 返回的是一个无符号长整型（unsigned long）的值
//  
//  //计算当前每个Loop的运行时间间隔
//  float Ts = (now_us - open_loop_timestamp) * 1e-6f;

//  //由于 micros() 函数返回的时间戳会在大约 70 分钟之后重新开始计数，在由70分钟跳变到0时，TS会出现异常，因此需要进行修正。如果时间间隔小于等于零或大于 0.5 秒，则将其设置为一个较小的默认值，即 1e-3f
//  if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
//  

//  // 通过乘以时间间隔和目标速度来计算需要转动的机械角度，存储在 shaft_angle 变量中。在此之前，还需要对轴角度进行归一化，以确保其值在 0 到 2π 之间。
//  shaft_angle = _normalizeAngle(shaft_angle + target_velocity*Ts);
//  //以目标速度为 10 rad/s 为例，如果时间间隔是 1 秒，则在每个循环中需要增加 10 * 1 = 10 弧度的角度变化量，才能使电机转动到目标速度。
//  //如果时间间隔是 0.1 秒，那么在每个循环中需要增加的角度变化量就是 10 * 0.1 = 1 弧度，才能实现相同的目标速度。因此，电机轴的转动角度取决于目标速度和时间间隔的乘积。

//  // 使用早前设置的voltage_power_supply的1/3作为Uq值，这个值会直接影响输出力矩
//  // 最大只能设置为Uq = voltage_power_supply/2，否则ua,ub,uc会超出供电电压限幅
//  float Uq = voltage_power_supply/3;
//  
//  setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle, 7));
//  
//  open_loop_timestamp = now_us;  //用于计算下一个时间间隔

//  return Uq;
//}


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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_TIM_Base_Start(&htim1);
	
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	

  while (1)
  {
		  // 固定控制频率为1kHz
    static uint32_t last_control_time = 0;
    uint32_t now = HAL_GetTick();
    
    if (now - last_control_time >= 1) {  // 1ms控制周期
        velocityOpenloop(5);  // 5rad/s
        
        // 可以添加调试输出
        // printf("Angle: %f, Uq: %f\r\n", shaft_angle, current_Uq);
        
        last_control_time = now;
    }
    
    // 添加少量延时避免CPU占用率过高
    HAL_Delay(1);

//		setPwm(5,0,0);
//		HAL_Delay(10);
//		setPwm(5,5,0);
//		HAL_Delay(10);
//		setPwm(0,5,0);
//		HAL_Delay(10);
//		setPwm(0,5,5);
//		HAL_Delay(10);
//		setPwm(0,0,5);
//		HAL_Delay(10);
//		setPwm(5,0,5);
//		HAL_Delay(10);
	
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
