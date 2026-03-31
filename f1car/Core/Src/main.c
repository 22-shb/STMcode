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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint16_t current_count;
    uint16_t last_count;
    int32_t pulse_diff;      //()这个是 可能为负数，所以用int32_t
    float speed_m_s_new;
    float speed_m_s_Filtered;
} EncoderData;

EncoderData Encode_speed_left;
EncoderData Encode_speed_right;

typedef struct {
		uint8_t r_f;
		uint8_t l_r;
		uint16_t target_speed;
}MOTOR;

MOTOR motor;

typedef struct {
    // --- 参数 (需要初始化) ---
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    
    float limit_max;    // 输出最大限幅 (如: 100或PWM最大值)
    float limit_min;    // 输出最小限幅 (如: -100或0)
    
    float dead_zone;    // 死区阈值 (误差在这个范围内视为0)
    float integral_max; // 积分限幅 (防积分饱和 Anti-windup)

    // --- 状态 (运行时变化) ---
    float prev_error;   // 上一次的误差
    float integral;     // 累积积分值
    
} PID_Handle;

PID_Handle motor_pid_left;
PID_Handle motor_pid_right;
PID_Handle motor_sensor_pid;

// 定义传感器引脚结构体，方便修改
typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} SensorPin;

// 这里填入你实际的引脚连接，从左(Sensor 0)到右(Sensor 7)
// 例如：Sensor 0 接 PA0, Sensor 1 接 PB5 ...
SensorPin sensors[8] = {
    {GPIOB, GPIO_PIN_12}, // Sensor 1 (最右)
    {GPIOB, GPIO_PIN_13}, // Sensor 2
    {GPIOB, GPIO_PIN_14}, // Sensor 3
    {GPIOB, GPIO_PIN_15}, // Sensor 4
    {GPIOA, GPIO_PIN_10}, // Sensor 5
    {GPIOA, GPIO_PIN_11}, // Sensor 6
    {GPIOA, GPIO_PIN_12}, // Sensor 7
    {GPIOB, GPIO_PIN_0}  // Sensor 8 (最左)
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 定义参数 (可以根据实际情况微调)
#define MAX_SPEED       0.25f   // 直道最快速度 (可以稍微快点)
#define MIN_SPEED       0.20f   // 弯道最慢速度 (一定要够慢，保证能转过来)
#define BRAKE_SENSITIVITY 0.8f  // 刹车灵敏度：数值越大，转弯时减速越狠 (建议 0.8 ~ 1.5)

#define PI 3.14159265358979323846

/*****************/
// 电机参数
#define ENCODER_LINES    13      // 编码器线数
#define GEAR_RATIO       20      // 减速比是20
#define FILTER_ALPHA     0.2f // 滤波系数
#define ENCODER_MODE    4.0f    // 你现在用的是2倍频

/*****************/     
// 轮子半径 (mm)
#define WHEEL_RADIUS    24.0f   
// 一圈车轮的总脉冲数 (理论值)
// 公式: 13 * 20 * 4 = 1040
#define COUNTS_PER_REV  (ENCODER_LINES * GEAR_RATIO * ENCODER_MODE)
// 采样时间 (秒) - 对应你的 10ms 中断
#define DT              0.01f
/*****************/
//电机最快速度为0~0.8米每秒
//但是我的电机转一圈要40个数，编码器模式下滤波13。要40个数才能转一圈，记录一下以防忘记
//还有循迹模块读取黑线是低电平
/*****************/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// 串口发送缓冲区
char tx_buffer[64];
float a,b,c,d;

uint16_t cnt_2 = 0;
uint32_t cnt_3 = 0;

volatile float speed_right;
volatile float speed_left;

//float base_speed_target = 0.3f; // 目标速度 (m/s)
//float motor_target_speed = 0.4f; // 电机目标速度 (m/s)

float track_error = 0;// 循迹偏差值
float turn_output = 0;// 转向 PID 输出

float last_valid_error = 0.0f; // 上一次有效的循迹偏差值
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void set_motor(uint8_t l_r,uint8_t r_f,uint16_t motor_speed);
void ALL_Init(void);
float PID_Compute(PID_Handle *pid, float target, float measured);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Send_Encoder_Data(void)
{
    int len = sprintf(tx_buffer, "%f,%f\n",a,b);//sprintf的返回值是int类型，返回字节数
    
    // ✅ 使用非阻塞发送，或设置合理的超时
    HAL_UART_Transmit(&huart1, (uint8_t*)tx_buffer, len, 100);
}
// void motor_left(uint16_t target,uint32_t encode_speed_1,uint32_t encode_speed_2)//encode是编码器计算后的速度
// {
// 	set_motor(0,0,PID_Compute(&motor_pid,target,encode_speed_1));
// 	set_motor(1,1,PID_Compute(&motor_pid,target,encode_speed_2));
// }

/**
 * @brief 高效读取8路传感器状态
 * @return uint8_t: 8位数据，1代表黑线，0代表白地
 * Bit7对应Sensor0(左), Bit0对应Sensor7(右)
 */
uint8_t Read_8_Sensors_Fast(void) {
    uint8_t sensor_byte = 0;

    // 循环展开或直接硬编码是最高效的，为了可读性写成循环，编译器开O2优化后是一样的
    for(int i = 0; i < 8; i++) {
        // 直接读取寄存器 IDR (Input Data Register)
        // (sensors[i].port->IDR & sensors[i].pin) 用于判断引脚电平
        // 只有当 IDR & pin 结果为 0 (低电平/黑线) 时，才把这一位置 1
        // 使用 "!" 取反，或者用 "== 0"
        if ((sensors[i].port->IDR & sensors[i].pin) == 0) {
            // 如果检测到低电平(低电平是黑线)，把对应位置 1
            // 我们希望 Sensor0 对应最高位 (0x80)，Sensor7 对应最低位 (0x01)
            sensor_byte |= (1 << (7 - i));
        }
    }
    return sensor_byte;
}

/**
 * @brief 计算循迹偏差值
 * @param sensor_data: 8位数据，每一位代表一个传感器状态 (0或1)
 * @return float: 偏差值 Error，范围在 -4.0 到 4.0 之间。0 表示正中心。
 */
float Get_Track_Error(uint8_t sensor_data) {
    // 传感器权重数组：从左到右
    float weights[8] = {-4.0f, -3.0f, -2.0f, -1.0f, 1.0f, 2.0f, 3.0f, 4.0f};
    
    float sum_weighted_values = 0;
    int active_count = 0;

    for (int i = 0; i < 8; i++) {
        // 检查第 i 位是否为 1 (假设 sensor_data 从高位到低位对应从左到右)
        if ((sensor_data >> (7 - i)) & 0x01) {
            sum_weighted_values += weights[i];
            active_count++;
        }
    }
		
		// 特殊处理：如果只有最外侧传感器触发 (假设 0x80 是左边缘，0x01 是右边缘)
    if (sensor_data == 0x80) return -6.0f; // 强制给一个超大偏差
    if (sensor_data == 0x01) return 6.0f;  // 强制给一个超大偏差

// 如果没有任何传感器检测到黑线
    if (active_count == 0) {
        // 既然是直角弯容易冲出去，说明之前的偏差已经很大了
        // 我们不仅要维持方向，还要加大力度
        if (last_valid_error > 0) return 8.0f;  // 极大值，迫使内侧轮反转
        else return -8.0f; 
    }
    float current_error = sum_weighted_values / active_count;
    
    // 【改进】更新最后一次有效的偏差
    last_valid_error = current_error;

    return current_error;
}

// 这里的 * 表示接收的是“真身”的地址
float encode_to_speed(EncoderData *motor_encoder_x, TIM_TypeDef* TIMx) 
{
    // 1. 读取当前的 CNT
    motor_encoder_x->current_count = TIMx->CNT;

    // 2. 这里的 last_count 是真身里的值，是 10ms 前保存的
    int32_t pulse_diff = (int16_t)(motor_encoder_x->current_count - motor_encoder_x->last_count);

    // 3. 计算速度 (保留你的公式逻辑)
    // 注意：把 calculation 拆开写更清晰，防止漏括号
    float distance = (float)pulse_diff / 1040.0f * (2.0f * PI * 24.0f); // mm
    motor_encoder_x->speed_m_s_new = distance / 0.01f / 1000.0f; // m/s (注意变量名虽然叫rpm但你算的是m/s)

    // 4. 【核心】更新真身的 last_count，这样下一次进来才能算对！
    motor_encoder_x->last_count = motor_encoder_x->current_count;

    // 5. 滤波
    motor_encoder_x->speed_m_s_Filtered = (motor_encoder_x->speed_m_s_new * FILTER_ALPHA) + 
                                  (motor_encoder_x->speed_m_s_Filtered * (1.0f - FILTER_ALPHA));

    return motor_encoder_x->speed_m_s_Filtered;
}

/**
 * @brief 设置电机速度
 * @param l_r是选择轮子左右（1是右0是左边）
 * @param r_f是选择轮子正反转（1是正转0是反转）
 * @param motor_speed: 电机速度值 (0~1000)
 */
void set_motor(uint8_t l_r,uint8_t r_f,uint16_t motor_speed)
{
	switch(r_f)
	{
		case 1:
		{
			if(l_r == 1)//右边轮子正转
			{
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
			}else
			{//左边轮子正转
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
			}
		}break;
		case 0:
		{
			if(l_r == 1)//右边轮子反转
			{
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
			}else
			{//左边轮子反转
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
			}
		}break;
	}
	if(l_r == 1)
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor_speed);
	else
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, motor_speed);
}

void PID_Init(void)
	
{

/*****************************************************************************
 *                             电机 PID 参数初始化
 * *****************************************************************************/
    motor_pid_left.Kp = 1000.0f;
    motor_pid_left.Ki = 200.0f;
    motor_pid_left.Kd = 0.0f;
    
    motor_pid_left.limit_max = 1000.0f;   // 输出最大 +1000
    motor_pid_left.limit_min = -1000.0f;  // 输出最小 -1000
	motor_pid_left.integral_max = 5.0f; 

    motor_pid_left.prev_error = 0.0f;    // 状态清零
    motor_pid_left.integral = 0.0f;
    motor_pid_right = motor_pid_left; // 右轮参数相结构体整体赋值
/*****************************************************************************
 *                              循迹 PID 参数初始化                       
*****************************************************************************/
    // 注意：循迹 PID 的参数通常很小，因为 error 范围是 -4 到 4，而速度微调量通常在 0.1~0.5 之间
    motor_sensor_pid.Kp = 0.1f;  // 经验值，需调试：代表偏离1个单位，一侧轮子加速0.05m/s
    motor_sensor_pid.Ki = 0.0f;  // 循迹通常不需要 I，或者给极小
    motor_sensor_pid.Kd = 0.2f;  // D大一点，为了微分先行，抑制过冲
    
    motor_sensor_pid.limit_max = 100.0f;  // 限制最大转向幅度，别让轮子倒转太猛  好像0.8是最大限制了是吧
    motor_sensor_pid.limit_min = -100.0f;
    
    motor_sensor_pid.prev_error = 0.0f;
    motor_sensor_pid.integral = 0.0f;
}

/**
 * @brief 计算 PID 输出
 * @param pid: 指向 PID_Handle 结构体的指针
 * @param target: 目标值 (Setpoint)
 * @param measured: 测量值 (Process Variable)
 * @return float: PID 输出值
 */
float PID_Compute(PID_Handle *pid, float target, float measured) {
    float output;
    
    // 1. 计算误差
    float error = target - measured;

    // 3. 积分项计算 (Integral) & 积分限幅 (Anti-windup)
    pid->integral += error;

    // 限制积分项的累积，防止长时间偏差导致积分过大（积分饱和）
    if (pid->integral > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (pid->integral < -pid->integral_max) {
        pid->integral = -pid->integral_max; 
    }

    // 4. 微分项计算 (Derivative)
   float derivative = error - pid->prev_error;
		// //尝试一些微分先行    这个不行
    // error = measured;
		// float derivative = error - pid->prev_error;

    // 5. PID 公式总和
    // Output = P*error + I*integral + D*(error - prev_error)
    output = (pid->Kp * error) + 
             (pid->Ki * pid->integral) + 
             (pid->Kd * derivative);

    // 6. 更新上一次误差
   pid->prev_error = error;
		// pid->prev_error = measured;
		//    // 7. 输出限幅 (Output Limiting)        这个也用不到，pwm输出最大也就1000
//    // 确保输出不超过执行器允许的范围
//    if (output > pid->limit_max) {
//        output = pid->limit_max;
//    } else if (output < pid->limit_min) {
//        output = pid->limit_min; 
//    }

    return output;
}

void ALL_Init(void)
{
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);

  HAL_TIM_Base_Start(&htim1);  // PWM定时器
  
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);//MOTOR A
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);//MOTOR B
	
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);//使能电机 

  HAL_TIM_Base_Start_IT(&htim4); // 启动定时器中断，用于速度计算
}

void test_encode_speed(void)
{
    //这个抢pid的控制了
//     set_motor(0,1,1000);
//		 set_motor(1,1,1000);
    //测试的时候一个一个的测试，如果读数反了的话就把编码器的线调换一下
    // a = encode_to_speed(&Encode_speed_left,TIM3);
   a = speed_right;
		Send_Encoder_Data();
		HAL_Delay(100);
}


// 通用电机执行函数：自动处理 PID 输出的正负号
// motor_id: 0=左, 1=右
// pid_out: PID 计算出的结果 (有正负)
void Motor_Apply_PID(uint8_t motor_id, float pid_out)
{
    uint8_t direction;
    uint16_t pwm_val;
    // 1. 判断方向
    if (pid_out >= 0) {
        direction = 1; // 正转 (根据你的硬件实际情况修改 1 或 0)
        pwm_val = (uint16_t)pid_out;
    } else {
        direction = 0; // 反转
        // 取绝对值 (因为 set_motor 只认正数 PWM)
        pwm_val = (uint16_t)(-pid_out); 
    }

		
    // 2. 驱动电机
    set_motor(motor_id, direction, pwm_val);
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	//启动编码器
	ALL_Init();
  PID_Init();
  while (1)
  {		
//    a = speed_right;
//    b = speed_left;
//    test_encode_speed();
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM4) // 10ms中断
  {
    // --- 1. 读取数据 ---
    speed_right = encode_to_speed(&Encode_speed_right, TIM2);
    speed_left  = encode_to_speed(&Encode_speed_left, TIM3);

    uint8_t sensor_data = Read_8_Sensors_Fast();
    track_error = Get_Track_Error(sensor_data);
    
    // --- 2. 计算转向 PID ---
    turn_output = PID_Compute(&motor_sensor_pid, 0.0f, track_error);

    // --- 3. 【核心大招】动态计算基础速度 (Dynamic Speed) ---
    // 逻辑：直道快跑，弯道慢跑
    float dynamic_base_speed = MAX_SPEED;
//    
    // 取转向输出的绝对值
    float abs_turn = (turn_output > 0) ? turn_output : -turn_output;

    // 核心公式：基础速度 = 最大速度 - (转向幅度 * 系数)
    dynamic_base_speed = MAX_SPEED - (abs_turn * BRAKE_SENSITIVITY);

//    // 守底线：速度不能低于最小值，防止停在弯道
//    if (dynamic_base_speed < MIN_SPEED) {
//        dynamic_base_speed = MIN_SPEED;
//    }

    // --- 4. 融合速度 (允许内侧轮反转) ---
    // 左轮 = 基础 + 转向
    // 右轮 = 基础 - 转向
    float target_speed_left  = dynamic_base_speed + turn_output;
    float target_speed_right = dynamic_base_speed - turn_output;

    // --- 5. 执行输出 ---
    // 这里的 PID 计算和 Apply 函数保持不变
    float out_left  = PID_Compute(&motor_pid_left,  target_speed_left,  speed_left);
    float out_right = PID_Compute(&motor_pid_right, target_speed_right, speed_right);

    Motor_Apply_PID(0, out_left);  // 左轮
    Motor_Apply_PID(1, out_right); // 右轮
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
