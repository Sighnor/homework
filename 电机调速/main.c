/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32_2.8_lcd.h"
#include "stdio.h"
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
uint32_t hall_a;
uint32_t hall_b;
uint32_t hall_c;
uint32_t tim2_t;
uint32_t tim3_t;
uint32_t timcnt;
int ADC_Value = 1500;
float u = 0;
uint32_t my_V;
double velocity = 0;
double my_velocity = 50.000;
u8 send1[30];
u8 send2[30];
u8 send3[30];
u8 send4[30];
float ek_2 = 0;
float ek_1 = 0;
float ek = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float PID_Control()
{
	float kp = 9;
	float ki = 0.37;
	float kd = 0.1;
	return kp * (ek - ek_1) + ki * (ek) + kd * (ek - 2 * ek_1 + ek_2); 
}

void PWM_Ctl(int hall_a,int hall_b,int hall_c)
{
	if(hall_a==1 & hall_b==0 & hall_c==1)
	{
	//TIM8_CH1_ON; TIM8_CH2_OFF; TIM8_CH3_OFF;
	//GPIO_OUT1_OFF; GPIO_OUT2_ON; GPIO_OUT3_OFF;
		HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_3);
		
		HAL_GPIO_WritePin(UL_GPIO_Port,UL_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(VL_GPIO_Port,VL_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(WL_GPIO_Port,WL_Pin,GPIO_PIN_RESET);
	}
	else if(hall_a==1 & hall_b==0 & hall_c==0)
	{
	//TIM8_CH1_ON; TIM8_CH2_OFF; TIM8_CH3_OFF;
	//GPIO_OUT1_OFF; GPIO_OUT2_OFF; GPIO_OUT3_ON;
		HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_3);
		
		HAL_GPIO_WritePin(UL_GPIO_Port,UL_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(VL_GPIO_Port,VL_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(WL_GPIO_Port,WL_Pin,GPIO_PIN_SET);
	}
	else if(hall_a==1 & hall_b==1 & hall_c==0)
	{
	//TIM8_CH1_OFF; TIM8_CH2_ON; TIM8_CH3_OFF;
	//GPIO_OUT1_OFF; GPIO_OUT2_OFF; GPIO_OUT3_ON;
		HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_3);
		
		HAL_GPIO_WritePin(UL_GPIO_Port,UL_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(VL_GPIO_Port,VL_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(WL_GPIO_Port,WL_Pin,GPIO_PIN_SET);
	}
	else if(hall_a==0 & hall_b==1 & hall_c==0)
	{
	//TIM8_CH1_OFF; TIM8_CH2_ON; TIM8_CH3_OFF;
	//GPIO_OUT1_ON; GPIO_OUT2_OFF; GPIO_OUT3_OFF;
		HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_3);
		
		HAL_GPIO_WritePin(UL_GPIO_Port,UL_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(VL_GPIO_Port,VL_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(WL_GPIO_Port,WL_Pin,GPIO_PIN_RESET);
	}
	else if(hall_a==0 & hall_b==1 & hall_c==1)
	{
	//TIM8_CH1_OFF; TIM8_CH2_OFF; TIM8_CH3_ON;
	//GPIO_OUT1_ON; GPIO_OUT2_OFF; GPIO_OUT3_OFF;
		HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
		HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_2);
		
		HAL_GPIO_WritePin(UL_GPIO_Port,UL_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(VL_GPIO_Port,VL_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(WL_GPIO_Port,WL_Pin,GPIO_PIN_RESET);
	}
	else if(hall_a==0 & hall_b==0 & hall_c==1)
	{
	//TIM8_CH1_OFF; TIM8_CH2_OFF; TIM8_CH3_ON;
	//GPIO_OUT1_OFF; GPIO_OUT2_ON; GPIO_OUT3_OFF;
		HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
		HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_2);
		
		HAL_GPIO_WritePin(UL_GPIO_Port,UL_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(VL_GPIO_Port,VL_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(WL_GPIO_Port,WL_Pin,GPIO_PIN_RESET);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
	HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);
	
	STM32_LCD_Init();
	LCD_Clear(Red);
	LCD_SetBackColor(Blue);
	LCD_SetTextColor(White);
	LCD_DisplayStringLine(Line0,"My test: ");
	
	HAL_TIM_IC_CaptureCallback(&htim2);
	HAL_Delay(4);
	HAL_TIM_IC_CaptureCallback(&htim2);
	
	HAL_TIM_Base_Start_IT(&htim3);
	
	__HAL_TIM_ENABLE_IT(&htim8, TIM_IT_BREAK);
	
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_3);
	
  while (1)
  {
    /* USER CODE END WHILE */
		
    /* USER CODE BEGIN 3 */
		if(HAL_GPIO_ReadPin(KEY_SEL_GPIO_Port,KEY_SEL_Pin)==GPIO_PIN_RESET)
		{
				HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_2);
				HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_3);
				HAL_GPIO_WritePin(UL_GPIO_Port,UL_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(VL_GPIO_Port,VL_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(WL_GPIO_Port,WL_Pin,GPIO_PIN_RESET);		
		}
		else
		{
			if(tim3_t>=10)//10ms
			{
				int temp;
				HAL_ADC_Start(&hadc3);
				HAL_ADC_Start_DMA(&hadc3,(uint32_t*)&temp,1);
				my_velocity = temp * 100 / 4096;
				velocity = (double)tim2_t/0.01/6/4;
				ek_2 = ek_1;
				ek_1 = ek;
				ek = my_velocity - velocity;
				printf("%lf,%lf\r\n", my_velocity, velocity);
				u = u + PID_Control();
				ADC_Value = ADC_Value + u;
				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,ADC_Value);
				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,ADC_Value);
				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,ADC_Value);
				tim2_t = 0;
				tim3_t = 0;
			}
			if(timcnt>=500)
			{
				HAL_ADC_Start(&hadc1);
				HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&my_V,1);
				sprintf((char*)send1,"V_line = %lf",my_V * 1.00 / 10);
				sprintf((char*)send2,"V_A    = %lf",ADC_Value * 3.300 / 4096);
				sprintf((char*)send3,"vec    = %lf",velocity);
				sprintf((char*)send4,"PWM    = %d",ADC_Value * 100 / 8401);
				LCD_DisplayStringLine(Line1,send1);
				LCD_DisplayStringLine(Line2,send2);
				LCD_DisplayStringLine(Line3,send3);
				LCD_DisplayStringLine(Line4,send4);
				timcnt = 0;
			}
		}
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
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM2)
	{
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10)==GPIO_PIN_SET)
		hall_c = 1;
		else
		hall_c = 0;	
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3)==GPIO_PIN_SET)
		hall_b = 1;
		else
		hall_b = 0;	
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)==GPIO_PIN_SET)
		hall_a = 1;
		else
		hall_a = 0;	
		
		PWM_Ctl(hall_a, hall_b, hall_c);
		
		tim2_t++;
	}
}

void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim)
{
		 if(htim->Instance==TIM8) 
		{
		//TIM8_CH1_OFF; TIM8_CH2_OFF; TIM8_CH3_OFF;
		//GPIO_OUT1_OFF; GPIO_OUT2_OFF; GPIO_OUT3_OFF;
				HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
				HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_2);
				HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_3);
				HAL_GPIO_WritePin(UL_GPIO_Port,UL_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(VL_GPIO_Port,VL_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(WL_GPIO_Port,WL_Pin,GPIO_PIN_RESET);
				__HAL_TIM_DISABLE_IT(&htim8, TIM_IT_BREAK); 
		}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		 if(htim->Instance==TIM3) 
		{
			tim3_t++;
			timcnt++;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
