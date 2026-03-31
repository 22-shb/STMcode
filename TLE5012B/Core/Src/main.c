/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "TLE5012B.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HardwareDiagnostic(void);
void SimpleCommunicationTest(void);
uint16_t ReadAngle_Basic(void);
uint16_t ReadAngle_Alternative(void);
uint16_t ReadAngle_SingleTransaction(void);


void TestCRC_Calculation(void);
// SSC通信测试
void TestSSC_Communication(void);
// SPI模式测试
void TestSPI_Modes(void);
void TLE5012_Init_SSC(void);
// 读取角速度
float ReadAngularSpeed_SSC(void);
// 完整的角度读取函数（基于博客原理）
float ReadAngle_SSC(void);





HAL_StatusTypeDef TLE5012B_ReadAngle(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_GPIO, uint16_t CS_Pin, float *angle);
void TLE5012B_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 方法1: 最基本的读取（推荐先试这个）
uint16_t ReadAngle_Basic(void)
{
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];
    uint16_t result;
    
    // 发送读取命令
    tx_buf[0] = 0x80; // 0x8021 高字节
    tx_buf[1] = 0x21; // 0x8021 低字节
    
    HAL_GPIO_WritePin(GPIOB, 0x0001, 0);
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 2, 1000);
    HAL_GPIO_WritePin(GPIOB, 0x0001, 1);
    
    // 重要：给传感器处理时间
    HAL_Delay(5); // 增加到5ms
    
    // 读取数据
    tx_buf[0] = 0xFF; // 哑元数据
    tx_buf[1] = 0xFF; // 哑元数据
    
    HAL_GPIO_WritePin(GPIOB, 0x0001, 0);
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 2, 1000);
    HAL_GPIO_WritePin(GPIOB, 0x0001, 1);
    
    result = (rx_buf[0] << 8) | rx_buf[1];
    return result;
}

// 方法2: 使用不同的寄存器地址尝试
uint16_t ReadAngle_Alternative(void)
{
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];
    uint16_t result;
    
    // 尝试使用0x8020而不是0x8021
    tx_buf[0] = 0x80; // 0x8020 高字节
    tx_buf[1] = 0x20; // 0x8020 低字节
    
    HAL_GPIO_WritePin(GPIOB, 0x0001, 0);
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 2, 1000);
    HAL_GPIO_WritePin(GPIOB, 0x0001, 1);
    
    HAL_Delay(5);
    
    tx_buf[0] = 0xFF;
    tx_buf[1] = 0xFF;
    
    HAL_GPIO_WritePin(GPIOB, 0x0001, 0);
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 2, 1000);
    HAL_GPIO_WritePin(GPIOB, 0x0001, 1);
    
    result = (rx_buf[0] << 8) | rx_buf[1];
    return result;
}

// 方法3: 单次长传输
uint16_t ReadAngle_SingleTransaction(void)
{
    uint8_t tx_buf[4];
    uint8_t rx_buf[4];
    uint16_t result;
    
    // 一次性发送命令和哑元数据
    tx_buf[0] = 0x80; // 命令高字节
    tx_buf[1] = 0x21; // 命令低字节
    tx_buf[2] = 0xFF; // 哑元高字节
    tx_buf[3] = 0xFF; // 哑元低字节
    
    HAL_GPIO_WritePin(GPIOB, 0x0001, 0);
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 4, 1000);
    HAL_GPIO_WritePin(GPIOB, 0x0001, 1);
    
    // 从第3、4字节获取角度数据
    result = (rx_buf[2] << 8) | rx_buf[3];
    return result;
}







// 读取角度值（核心函数）
HAL_StatusTypeDef TLE5012B_ReadAngle(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_GPIO, uint16_t CS_Pin, float *angle) {
    uint16_t tx_data[3] = {0x8020, 0x0000, 0x0000}; // 0x8020为角度寄存器地址
    uint16_t rx_data[3] = {0};
    
    // 启动传输
    HAL_GPIO_WritePin(CS_GPIO, CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi, (uint8_t*)tx_data, (uint8_t*)rx_data, 3, 100);
    HAL_GPIO_WritePin(CS_GPIO, CS_Pin, GPIO_PIN_SET);

    // 校验数据有效性 (bit15)
    if(rx_data[1] & 0x8000) {
        // 提取15位角度值 (0-32767)
        uint16_t raw_angle = rx_data[1] & 0x7FFF;
        // 转换为角度 (0-360°)
        *angle = (raw_angle * 360.0f) / 32768.0f;
        return HAL_OK;
    }
    return HAL_ERROR;
}

// 初始化函数示例
void TLE5012B_Init(void) {
    // 发送配置命令 (示例：启用预测值模式)
    uint16_t config_cmd = 0x5080; // 配置寄存器地址+数据
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&config_cmd, 1, 100);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	char msg[100];
//    
//    sprintf(msg, "TLE5012B SSC协议测试启动...\r\n");
//    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
//    
//    HAL_Delay(1000);
//    
//    // 运行测试
//    TestCRC_Calculation();
//    HAL_Delay(1000);
//    
//    TestSPI_Modes();
//    HAL_Delay(1000);
//    
//    // 初始化设备
//    TLE5012_Init_SSC();
//    HAL_Delay(1000);
//    
//    sprintf(msg, "开始连续角度读取...\r\n");
//    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
//	
//	
//	
	
	
	
	
//	    char buf[120];
//    uint16_t raw_angle;
//    float angle_deg;
//    uint32_t test_counter = 0;
//    
//    // 运行诊断
//    HardwareDiagnostic();
//    SimpleCommunicationTest();
//    
//    HAL_Delay(2000);
//    
//    sprintf(buf, "\r\n开始角度读取测试...\r\n");
//    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
////	
//	
//	    // 初始化传感器
//    SensorInit();
//    
//    // 运行诊断
//    SensorDiagnostic();
//		HardwareDiagnostic();
//    
//    char buffer[100];
//    float angle, speed;
//    uint32_t last_print = 0;
		
		
    char buf[60];
    float angle;
//    uint8_t success;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	
	
  while (1)
  {

		
		
//        float angle = ReadAngle_SSC();
//        float speed = ReadAngularSpeed_SSC();
//        char buf[80];
//        
//        if (angle > -900.0f && speed > -900.0f) {
            sprintf(buf, "角度: %7.2f/s\r\n", angle);
//        } else {
//            sprintf(buf, "读取失败 - 角度: %.1f, 速度: %.1f\r\n", angle, speed);
//        }
//        
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);
//        HAL_Delay(200);
		

		
		
//		SimpleCommunicationTest();
//		
//		    // 读取角度和速度
//        angle = ReadAngleDegrees();
//        speed = ReadAngularSpeed();
//        
//        // 每100ms打印一次数据
//        if (HAL_GetTick() - last_print > 100) {
//            if (angle > -900.0f) { // 有效数据
//                sprintf(buffer, "Angle: %7.2f°, Speed: %6.2f°/s\r\n", angle, speed);
//            } else {
//                sprintf(buffer, "Angle read error\r\n");
//            }
//            
//            HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 100);
//            last_print = HAL_GetTick();
//        }
//        
//        HAL_Delay(10);
//		
//				

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
