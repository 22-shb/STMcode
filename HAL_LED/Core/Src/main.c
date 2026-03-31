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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"                  // Device header
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
uint16_t Keynum;
uint16_t Keynum1;
uint8_t c;


//quanjubianliang
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Pwm_breath(uint8_t CCRx);
uint16_t Key_getnum1(void);
uint16_t Key_getnum(void);
void pwm_bpm(uint8_t c,uint8_t s);
void Simple_Delay(uint32_t delay);

uint16_t Key_getnum1(void) {
    static uint8_t last_state = 0;
    static uint32_t last_time = 0;
    uint8_t current_state;
    uint32_t current_time = HAL_GetTick(); 
    
    current_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);
    
    // 检测按键按下（上升沿）
    if (last_state == 0 && current_state == 1) {
        if ((current_time - last_time) > 20) {  
            if(Keynum != 3) {
                Keynum1 = (Keynum1 % 3) + 1;  
            } else {
                Keynum1 = (Keynum1 % 4) + 1;  
            }
            last_time = current_time;
        }
    }
    
    last_state = current_state;
    return Keynum1;
}
uint16_t Key_getnum(void) {
    static uint8_t last_state = 0;
    static uint32_t last_time = 0;
    static uint16_t last_Keynum = 0; // 用于记录上一次的Keynum值//用全局变量就不行
    uint8_t current_state;
    uint32_t current_time = HAL_GetTick();  

    current_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
    
    if (last_state == 0 && current_state == 1) {
        if ((current_time - last_time) > 20) { 
            Keynum = (Keynum % 3) + 1;
            last_time = current_time;
        }
        if(Keynum != last_Keynum) { // 检测到Keynum变化
            Keynum1 = 1; // Keynum变化，重置Keynum1为1
            last_Keynum = Keynum;
        }
    }
    
    last_state = current_state;

    return Keynum;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim1){
    Key_getnum();
	  Key_getnum1();
	}

}

void Pwm_breath(uint8_t CCRx){
		float t = HAL_GetTick();
	
		float duty = 0.5*sin(6*3.14*t) + 0.5;
	
		uint16_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
	
		uint16_t ccr = duty * (arr + 1);
		if(CCRx == 1){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,ccr);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,0);
		}
		if(CCRx == 2){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,ccr);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,0);
		}
		if(CCRx == 3){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,0);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,ccr);
		}
}
void pwm_bpm(uint8_t c,uint8_t s){
	
	float t = HAL_GetTick();
	
	float duty = 0.5*sin(s*3.14*t) + 0.5;
	
	uint16_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
	
	uint16_t ccr = duty * (arr + 1);
	
	switch(c){
		case 1:{
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,ccr);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,0);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,0);
		}break;
		case 2:{
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,0);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,ccr);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,0);
		}break;
		case 3:{
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,0);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,0);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,ccr);
		}break;
	}
}

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
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_Init();
	SystemClock_Config();
	
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim3);
		
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	
  while (1)
  {
    switch (Keynum){
    case 1:
		  switch(Keynum1){
		  case 1:
        c = Keynum1;
			  Pwm_breath(1);
			  break;
		  case 2:
        c = Keynum1;
			  Pwm_breath(2);
			  break;
		  case 3:
        c = Keynum1;
			  Pwm_breath(3);
			  break;
	}
      break;
    case 2:
       switch(Keynum1){
        case 1:{
          pwm_bpm(c,6);
        }break;
        case 2:{
          pwm_bpm(c,4);
        }break;
        case 3:{
          pwm_bpm(c,2);
        }break;
  }
      break;
    case 3:
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,0);

      switch(Keynum1){
    case 1:{
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,1000);
      HAL_Delay(100);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,0);
      HAL_Delay(100);
    }break;
    case 2:{
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,1000);
      HAL_Delay(100);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,0);
      HAL_Delay(100);
    }break;
    case 3:{
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,1000);
      HAL_Delay(100);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,0);
      HAL_Delay(100);
    }break;
    case 4:{
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,1000);
      HAL_Delay(100);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,0);
      HAL_Delay(100);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,1000);
      HAL_Delay(100);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,0);
      HAL_Delay(100);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,1000);
      HAL_Delay(100);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,0);
    }
  }
      break;
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
