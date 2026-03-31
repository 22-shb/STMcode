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
#include "math.h"
#include "stdio.h"
#include "string.h"
#include "foc_driver.h" // 引入 FOC 驱动库
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
FOC_t motor1;
uint32_t last_tick = 0;
char debug_buf[128]; // 调试缓冲区
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// 函数声明
void FOC_Init(FOC_t *hfoc, float v_supply, float v_limit, int dir, int pp);
void FOC_PWM_Config(FOC_t *hfoc, TIM_HandleTypeDef *t1, TIM_HandleTypeDef *t2, TIM_HandleTypeDef *t3, uint32_t ch1, uint32_t ch2, uint32_t ch3);
void FOC_Alignment(FOC_t *hfoc); // 零位校准
void FOC_Loop(FOC_t *hfoc, float dt); // 主循环，传入时间间隔 dt (秒)

// PID 相关工具
void PID_Init(PID_State_t *pid, PID_Param_t *param, float kp, float ki, float kd, float out_max);
float PID_Calc(PID_State_t *pid, PID_Param_t *param, float error, float dt); // 增加 dt 参数
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
//===================AS5600角度读取=====================
void AS5600_WriteData(uint8_t Wordaddress, uint8_t Data)//这个一般用不到默认就是读取角度的
{
	/* HAL expects 7-bit address (no R/W bit) in most STM32 HAL versions. */
	uint8_t buf = Data;
	HAL_I2C_Mem_Write(&hi2c2, (uint16_t)(AS5600_I2C_ADDR << 1), Wordaddress, I2C_MEMADD_SIZE_8BIT, &buf, 1, AS5600_I2C_TIMEOUT_MS);
}

uint8_t AS5600_ReadData(uint8_t Wordaddress)
{ 
	uint8_t buf = 0;
	HAL_I2C_Mem_Read(&hi2c2, (uint16_t)(AS5600_I2C_ADDR << 1), Wordaddress, I2C_MEMADD_SIZE_8BIT, &buf, 1, AS5600_I2C_TIMEOUT_MS);
	return buf;
}

uint16_t AS5600_ReadRawAngle(void)//机械角度
{
	uint8_t hi = AS5600_ReadData(AS5600_RAW_HI_REG);
	uint8_t lo = AS5600_ReadData(AS5600_RAW_LO_REG);
	uint16_t val = ((uint16_t)hi << 8) | (uint16_t)lo;
	val &= 0x0FFF;
	return val;
}

static float _normalizeAngle(float angle) {
    float a = fmod(angle, 2 * PI);
    return a >= 0 ? a : (a + 2 * PI);
}

/**
 * @brief 具体的编码器读取实现 (整合 AS5600)
 * @param dt: 时间间隔，用于计算速度
 */
void IIC_Encoder_Read(Encoder_t *enc, float dt) {
    // 1. 读取原始角度 (0 ~ 4095)
    uint16_t raw_val = AS5600_ReadRawAngle();
    
    // 2. 转换为弧度 (0 ~ 2PI)
    float val_rad = (float)raw_val * 2.0f * PI / 4096.0f;
    
    // 3. 计算角度变化量 (用于累计角度和速度)
    float d_angle = val_rad - enc->angle_raw;
    
    // 处理 0 <-> 2PI 的跳变
    if (d_angle > PI) d_angle -= 2 * PI;
    else if (d_angle < -PI) d_angle += 2 * PI;
    
    // 4. 更新结构体
    enc->angle_raw = val_rad;
    enc->angle_consequent += d_angle; // 累计多圈角度
    
    // 5. 计算速度 (带简单低通滤波)
    if (dt > 0.0001f) {
        float new_vel = d_angle / dt;
        // LPF: 80% 旧速度 + 20% 新速度，减少噪声
        enc->velocity = 0.8f * enc->velocity + 0.2f * new_vel;
    }
}
// =======
// 使用宏定义 LIMIT 替代原来的 _limit 函数
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/**
 * @brief 底层 PWM 设置函数 (包含 SVPWM 零序分量注入)
 * 这里输入的是通过克拉克逆变换得到的 Ua, Ub, Uc (物理电压值)
 */
static void _setPwm(FOC_t *hfoc, float Ua, float Ub, float Uc) {
    // -----------------------------------------------------------
    // SVPWM 核心算法：零序分量注入 (Zero Sequence Injection)
    // -----------------------------------------------------------
    // 这一步将原本的正弦波 (SPWM) 变成了 马鞍波 (SVPWM)
    // 作用：防止波峰削顶，使直流母线电压利用率提高约 15.47%
    
    // 1. 找出三相电压中的最大值和最小值
    // 使用标准库函数 fminf/fmaxf 替代繁琐的 if 判断，逻辑更清晰
    float v_min = fminf(Ua, fminf(Ub, Uc));
    float v_max = fmaxf(Ua, fmaxf(Ub, Uc));

    // 2. 计算中心偏置电压 (零序分量)
    float v_center = -0.5f * (v_min + v_max);

    // 3. 将偏置注入原电压 (生成马鞍波)
    float Ua_out = Ua + v_center;
    float Ub_out = Ub + v_center;
    float Uc_out = Uc + v_center;

    // -----------------------------------------------------------
    // PWM 占空比计算
    // -----------------------------------------------------------

    // 4. 限制电压范围 (防止超出电源电压)
    // 注意：SVPWM 后，相电压峰值实际上可以达到 voltage_ps / sqrt(3)
    // 这里简单限制在 voltage_limit 内
    Ua_out = LIMIT(Ua_out, -hfoc->voltage_limit, hfoc->voltage_limit);
    Ub_out = LIMIT(Ub_out, -hfoc->voltage_limit, hfoc->voltage_limit);
    Uc_out = LIMIT(Uc_out, -hfoc->voltage_limit, hfoc->voltage_limit);

    // 5. 归一化并计算占空比 (0.0 ~ 1.0)
    // 电压范围是 [-V_supply/2, +V_supply/2] -> 映射到 [0, 1]
    float dc_a = LIMIT((Ua_out / hfoc->voltage_ps) + 0.5f, 0.0f, 1.0f);
    float dc_b = LIMIT((Ub_out / hfoc->voltage_ps) + 0.5f, 0.0f, 1.0f);
    float dc_c = LIMIT((Uc_out / hfoc->voltage_ps) + 0.5f, 0.0f, 1.0f);

    // 6. 写入硬件寄存器
    __HAL_TIM_SetCompare(hfoc->PWMPin.tim1, hfoc->PWMPin.PIN1, (uint32_t)(dc_a * hfoc->PWMPin.tim1->Init.Period));
    __HAL_TIM_SetCompare(hfoc->PWMPin.tim2, hfoc->PWMPin.PIN2, (uint32_t)(dc_b * hfoc->PWMPin.tim2->Init.Period));
    __HAL_TIM_SetCompare(hfoc->PWMPin.tim3, hfoc->PWMPin.PIN3, (uint32_t)(dc_c * hfoc->PWMPin.tim3->Init.Period));
}

// --- PID 模块 ---

void PID_Init(PID_State_t *pid, PID_Param_t *param, float kp, float ki, float kd, float out_max) {
    memset(pid, 0, sizeof(PID_State_t));
    param->kp = kp;
    param->ki = ki;
    param->kd = kd;
    param->output_max = out_max;
    param->sum_max = out_max; 
}

float PID_Calc(PID_State_t *pid, PID_Param_t *param, float error, float dt) {
    float d_error;
    float output;
    
    if (dt <= 0.000001f) dt = 0.001f;

    pid->err = error;
    
    pid->sum += param->ki * pid->err * dt;
    pid->sum = LIMIT(pid->sum, -param->sum_max, param->sum_max);

    d_error = (pid->err - pid->err_last) / dt;
    pid->err_last = pid->err;

    output = (param->kp * pid->err) + pid->sum + (param->kd * d_error);
    
    output = LIMIT(output, -param->output_max, param->output_max);
    
    pid->output = output;
    return output;
}

// --- FOC 核心模块 ---

void FOC_Init(FOC_t *hfoc, float v_supply, float v_limit, int dir, int pp) {
    memset(hfoc, 0, sizeof(FOC_t));
    hfoc->voltage_ps = v_supply;
    hfoc->voltage_limit = (v_limit > v_supply) ? v_supply : v_limit;
    hfoc->dir = dir;
    hfoc->pp = pp;
    hfoc->mode = FOC_MODE_TORQUE;
}

void FOC_PWM_Config(FOC_t *hfoc, TIM_HandleTypeDef *t1, TIM_HandleTypeDef *t2, TIM_HandleTypeDef *t3, uint32_t ch1, uint32_t ch2, uint32_t ch3) {
    hfoc->PWMPin.tim1 = t1;
    hfoc->PWMPin.tim2 = t2;
    hfoc->PWMPin.tim3 = t3;
    hfoc->PWMPin.PIN1 = ch1;
    hfoc->PWMPin.PIN2 = ch2;
    hfoc->PWMPin.PIN3 = ch3;

    HAL_TIM_PWM_Start(t1, ch1);
    HAL_TIM_PWM_Start(t2, ch2);
    HAL_TIM_PWM_Start(t3, ch3);
}

float FOC_Get_ElectricalAngle(FOC_t *hfoc) {
    return _normalizeAngle((float)(hfoc->dir * hfoc->pp) * hfoc->encoder.angle_raw - hfoc->zero_electric_angle);
}

/**
 * @brief 设置力矩 (执行 Park逆变换 和 Clarke逆变换)
 * @param Uq Q轴电压
 * @param Ud D轴电压
 * @param angle_el 电角度
 */
void FOC_SetTorque(FOC_t *hfoc, float Uq, float Ud, float angle_el) {
    angle_el = _normalizeAngle(angle_el);
    float s = sinf(angle_el);
    float c = cosf(angle_el);

    // 1. Park 逆变换 (Inverse Park Transform)
    // 作用：将旋转坐标系 (Ud, Uq) 转换到 两相静止坐标系 (Alpha, Beta)
    hfoc->u_alpha = Ud * c - Uq * s;
    hfoc->u_beta  = Ud * s + Uq * c;

    // 2. Clarke 逆变换 (Inverse Clarke Transform)
    // 作用：将两相静止坐标系 (Alpha, Beta) 转换到 三相静止坐标系 (a, b, c)
    // 公式：
    // Ua = Ualpha
    // Ub = -0.5 * Ualpha + (sqrt(3)/2) * Ubeta
    // Uc = -0.5 * Ualpha - (sqrt(3)/2) * Ubeta
    hfoc->u_a = hfoc->u_alpha;
    hfoc->u_b = (-0.5f * hfoc->u_alpha) + (_SQRT3 / 2.0f * hfoc->u_beta);
    hfoc->u_c = (-0.5f * hfoc->u_alpha) - (_SQRT3 / 2.0f * hfoc->u_beta);

    // 3. 发送给 PWM 生成函数 (内部包含 SVPWM 处理)
    _setPwm(hfoc, hfoc->u_a, hfoc->u_b, hfoc->u_c);
}
void FOC_Alignment(FOC_t *hfoc) {
    float align_voltage = 3.0f; 
    
    // 使用 Ud (励磁) 校准到 0度 Alpha轴
    FOC_SetTorque(hfoc, 0, align_voltage, 0); 
    
    HAL_Delay(2000); 
    
    // 多读几次确保稳定 (丢弃前几次可能不稳定的值)
    for(int i=0; i<10; i++) {
        IIC_Encoder_Read(&hfoc->encoder, 0);
        HAL_Delay(10);
    }
    
    // 记录零点
    hfoc->zero_electric_angle = _normalizeAngle((float)(hfoc->dir * hfoc->pp) * hfoc->encoder.angle_raw);
    
    FOC_SetTorque(hfoc, 0, 0, 0);
}


static float open_loop_angle = 0.0f;


void FOC_Loop(FOC_t *hfoc, float dt) {
    // 1. 始终读取编码器，用于调试观察 (即使在开环模式下)
    IIC_Encoder_Read(&hfoc->encoder, dt); 

    float target_voltage_q = 2.5f;
    float target_voltage_d = 0.0f;

    switch (hfoc->mode) {
        case FOC_MODE_OPEN_LOOP:
            // === 开环测试模式 (神器) ===
            // 目标值 = 目标速度 (rad/s)
            // 完全无视编码器，强制让磁场转起来
            open_loop_angle += hfoc->target_value * dt;
            open_loop_angle = _normalizeAngle(open_loop_angle);
            hfoc->electrical_angle = open_loop_angle;
            
            // 给一个固定的电压，比如 2.5V (太低转不动，太高发烫)
            // 你可以根据发热情况调整这里的 2.5f
            target_voltage_q = 2.5f; 
            target_voltage_d = 0.0f;
            break;

        case FOC_MODE_TORQUE:
            hfoc->electrical_angle = FOC_Get_ElectricalAngle(hfoc);
            target_voltage_q = hfoc->target_value; 
            break;

        case FOC_MODE_VELOCITY:
            hfoc->electrical_angle = FOC_Get_ElectricalAngle(hfoc);
            // 速度环
            target_voltage_q = PID_Calc(&hfoc->pid_vel, &hfoc->param_vel, hfoc->target_value - hfoc->encoder.velocity, dt);
            break;

        case FOC_MODE_ANGLE:
            hfoc->electrical_angle = FOC_Get_ElectricalAngle(hfoc);
            // 位置环 -> 速度环
            {
                float pos_error = hfoc->target_value - hfoc->encoder.angle_consequent;
                float target_vel = PID_Calc(&hfoc->pid_pos, &hfoc->param_pos, pos_error, dt);
                target_voltage_q = PID_Calc(&hfoc->pid_vel, &hfoc->param_vel, target_vel - hfoc->encoder.velocity, dt);
            }
            break;
    }

    // 输出
    FOC_SetTorque(hfoc, target_voltage_q, target_voltage_d, hfoc->electrical_angle);
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // ==================== 3. FOC 初始化设置 ====================

  // (A) 初始化 FOC
  // 【严重发热修复 1】：电压限制从 5.0V 降级到 3.0V！
  // 【严重发热修复 2】：方向从 1 改为 -1！这是最常见的发热原因。
  // 如果改成 -1 后还是发热且不转，再改回 1，并检查极对数是不是填错了。
  FOC_Init(&motor1, 12.0f, 5.0f, -1, 7); 

  // (B) 配置 PWM
  FOC_PWM_Config(&motor1, &htim1, &htim1, &htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3);
  // __HAL_TIM_MOE_ENABLE(&htim1);

  // (C) 配置 PID 参数
  // 速度环: 限幅也同步降到 3.0V
  PID_Init(&motor1.pid_vel, &motor1.param_vel, 0.1f, 1.0f, 0.0f, 3.0f);
  
  // 位置环: 暂时不用
  PID_Init(&motor1.pid_pos, &motor1.param_pos, 5.0f, 0.0f, 0.0f, 20.0f);

  // (D) 传感器校准
  FOC_Alignment(&motor1);

  // (E) 设定为速度模式
  motor1.mode = FOC_MODE_VELOCITY; 
  motor1.target_value = 5.0f; // 目标 5 rad/s

  float last_tick = HAL_GetTick();


  while (1)
  {
   // ==================== 4. 主循环调用 ====================
    uint32_t now = HAL_GetTick();
    float dt = (now - last_tick) / 1000.0f;
    
    // 保持 1kHz 频率 (每1ms一次)
    if (dt >= 0.001f) {
        last_tick = now;
        FOC_Loop(&motor1, dt);
        
        // ==================== 5. 调试数据打印 (每100ms打印一次) ====================
        static int print_count = 0;
        print_count++;
        if (print_count >= 100) {
            print_count = 0;
            
            // 打印格式：Target, Actual, Voltage
            sprintf(debug_buf, "%.2f,%.2f,%.2f\r\n", 
                    motor1.target_value, 
                    motor1.encoder.velocity,
                    motor1.pid_vel.output);
            
            HAL_UART_Transmit(&huart1, (uint8_t*)debug_buf, strlen(debug_buf), 10);
        }
      }
/****************************测试编码器**************************/
	// 	float b =AS5600_ReadRawAngle();
	// 	float a = AS5600_GetAngle_Radians(b);
	// 	sprintf(temp_buffer, "%.2f,%.2f\n", a,b);
  //  HAL_UART_Transmit(&huart1, (uint8_t*)temp_buffer, strlen(temp_buffer), 100);
	// 	HAL_Delay(100);
/****************************************************************/
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
