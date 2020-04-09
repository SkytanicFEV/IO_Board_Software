/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"
#include "string.h"

uint8_t reps = 0;



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
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config(); //40MHz

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
//  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);//Write CS(SS) high

  // Turn on fault LEDs
	HAL_GPIO_WritePin(GPIOC, MOTOR_1_LED_Pin | MOTOR_2_LED_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, PGOOD_24V_LED_Pin, GPIO_PIN_SET);

//   LCD_Reboot();
//   LCD_Clear();
   HAL_Delay(1000);

   char test_rpm[] = "888";

   error_state = 0;

   startup_flag = 0;

	  LCD_Clear();
	  LCD_Start_Screen();
	  startup_flag = 1;

	  HAL_Delay(2000);
	  LCD_Clear();
	  LCD_RPM_Transmit(test_rpm, strlen(test_rpm));
	  startup_flag = 2;

	  HAL_Delay(1000);

	  // Check PGOOD signals
	if((!(GPIOA->IDR & PGOOD_1_Pin)) || (!(GPIOA->IDR & PGOOD_2_Pin)))
	{
		LCD_24V_Error();
	}


  while (1)
  {
		LCD_RPM_Transmit(rx_buffer,3);

	  HAL_Delay(250);
//		if(startup_flag == 2)
//		{
			LCD_Motor_Error(error_state);
//		}
		HAL_Delay(250);
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// Check to see if the output enable pin was interrupted
	if(GPIO_Pin == SWITCH_1_Pin)
	{
		if(GPIOC->IDR & SWITCH_1_Pin)
		{
			// Set the motor enables
			HAL_GPIO_WritePin(MOTOR_DRIVER_1_ENABLE_PORT, MOTOR_DRIVER_1_ENABLE_PIN, GPIO_PIN_SET);
			HAL_GPIO_WritePin(MOTOR_DRIVER_2_ENABLE_PORT, MOTOR_DRIVER_2_ENABLE_PIN, GPIO_PIN_SET);
		}
		else
		{
			// Reset the motor enables
			HAL_GPIO_WritePin(MOTOR_DRIVER_1_ENABLE_PORT, MOTOR_DRIVER_1_ENABLE_PIN, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(MOTOR_DRIVER_2_ENABLE_PORT, MOTOR_DRIVER_2_ENABLE_PIN, GPIO_PIN_RESET);
		}

	}
	if(GPIO_Pin == PGOOD_1_Pin)
	{
		if(!(GPIOA->IDR & PGOOD_1_Pin))
		{
			LCD_24V_Error();
		}
		// If pin comes back high then turn off the LED
		else if(GPIOA->IDR & PGOOD_2_Pin)
		{
			HAL_GPIO_WritePin(GPIOF, PGOOD_24V_LED_Pin, GPIO_PIN_RESET);

		}
	}
	if(GPIO_Pin == PGOOD_2_Pin)
	{
		if(!(GPIOA->IDR & PGOOD_2_Pin))
		{
			LCD_24V_Error();
		}
		// If pin comes back high then turn off the LED
		else if(GPIOA->IDR & PGOOD_1_Pin)
		{
			HAL_GPIO_WritePin(GPIOF, PGOOD_24V_LED_Pin, GPIO_PIN_RESET);

		}
	}
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	//START TIMER on CH4
//	htim1.Instance->CNT = TIM_PERIOD;
//	if(HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4) != HAL_OK)
//	{
//		Error_Handler();
//	}

}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//	switch(usart_state){
//		case (Motor1Recieve):
//				error_state &= ~0b01;
//				usart_state = Motor2TX;
//				CheckSystem();
////				LCD_Clear();
////				HAL_Delay(10);
////				LCD_RPM_Transmit(RX_DATA,3);
////				RX_DATA[0] = '0';
////				RX_DATA[1] = '0';
////				RX_DATA[2] = '0';
//
//				break;
//		case (Motor1Fail1):
//				error_state &= ~0b01;
//				usart_state = Motor2TX;
//				CheckSystem();
////				LCD_Clear();
////				HAL_Delay(10);
////				LCD_RPM_Transmit(RX_DATA,3);
////				RX_DATA[0] = '0';
////				RX_DATA[1] = '0';
////				RX_DATA[2] = '0';
//				break;
//		case (Motor2Recieve):
//				error_state &= ~0b01;
//				usart_state = Motor1TX;
//				CheckSystem();
////				LCD_Clear();
////				HAL_Delay(10);
////				LCD_RPM_Transmit(RX_DATA,3);
////				RX_DATA[0] = '0';
////				RX_DATA[1] = '0';
////				RX_DATA[2] = '0';
//				break;
//		case (Motor2Fail1):
//				error_state &= ~0b01;
//				usart_state = Motor1TX;
//				CheckSystem();
////				LCD_Clear();
////				HAL_Delay(10);
////				LCD_RPM_Transmit(RX_DATA,3);
////				RX_DATA[0] = '0';
////				RX_DATA[1] = '0';
////				RX_DATA[2] = '0';
//				break;
//	}
//}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
	//Timer reset
	if (reps >20){
		switch (usart_state){
		case (Motor1TX):
				CheckSystem();
				break;
		case (Motor1Recieve):
				usart_state = Motor1Fail1;
				CheckSystem();
				break;
		case (Motor1Fail1):
				error_state |= 0b01;
				usart_state = Motor2TX;
				CheckSystem();

				break;
		case (Motor2TX):
				CheckSystem();
				break;
		case(Motor2Recieve):
				usart_state = Motor2Fail1;
				CheckSystem();
				break;
		case(Motor2Fail1):
				error_state |= 0b10;
				usart_state = Motor1TX;
				break;
		default:
			break;
		}
	}
	else{
		reps++;
	}
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	while(69); //always true baby ;)
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
