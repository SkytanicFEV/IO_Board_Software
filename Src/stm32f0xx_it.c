/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/* USER CODE BEGIN 1 */

void EXTI0_1_IRQHandler(void)
{
	// Check for button 1 interrupt
	if(__HAL_GPIO_EXTI_GET_IT(BUTTON_1_Pin))
	{
		__HAL_GPIO_EXTI_CLEAR_IT(BUTTON_1_Pin);
		HAL_GPIO_EXTI_Callback(BUTTON_1_Pin);
	}
	// Check for button 2 interrupt
	if(__HAL_GPIO_EXTI_GET_IT(BUTTON_2_Pin))
	{
		__HAL_GPIO_EXTI_CLEAR_IT(BUTTON_2_Pin);
		HAL_GPIO_EXTI_Callback(BUTTON_2_Pin);
	}
}
void EXTI2_3_IRQHandler(void)
{

	// Check for button 3 interrupt
	if(__HAL_GPIO_EXTI_GET_IT(BUTTON_3_Pin))
	{
		__HAL_GPIO_EXTI_CLEAR_IT(BUTTON_3_Pin);
		HAL_GPIO_EXTI_Callback(BUTTON_3_Pin);
	}
	// Check for external switch interrupt
	if(__HAL_GPIO_EXTI_GET_IT(SWITCH_1_Pin))
	{
		__HAL_GPIO_EXTI_CLEAR_IT(SWITCH_1_Pin);
		HAL_GPIO_EXTI_Callback(SWITCH_1_Pin);
	}
}

void TIM1_CC_IRQHandler(void)
{
	//timer 1 down counted to 0 and handle this in main callback
	HAL_TIM_IRQHandler(&htim1);
}

//void EXTI4_15_IRQHandler(void)
//{
//	// Check for hall A interrupt
//	if(__HAL_GPIO_EXTI_GET_IT(BUTTON_1_Pin))
//	{
//		__HAL_GPIO_EXTI_CLEAR_IT(BUTTON_1_Pin);
//		HAL_GPIO_EXTI_Callback(BUTTON_1_Pin);
//	}
//	// Check for hall B interrupt
//	if(__HAL_GPIO_EXTI_GET_IT(BUTTON_2_Pin))
//	{
//		__HAL_GPIO_EXTI_CLEAR_IT(BUTTON_2_Pin);
//		HAL_GPIO_EXTI_Callback(BUTTON_2_Pin);
//	}
//	// Check for hall C interrupt
//	if(__HAL_GPIO_EXTI_GET_IT(BUTTON_3_Pin))
//	{
//		__HAL_GPIO_EXTI_CLEAR_IT(BUTTON_3_Pin);
//		HAL_GPIO_EXTI_Callback(BUTTON_3_Pin);
//	}
//	// Check for external interrupt
//	if(__HAL_GPIO_EXTI_GET_IT(SWITCH_1_Pin))
//	{
//		__HAL_GPIO_EXTI_CLEAR_IT(SWITCH_1_Pin);
//		HAL_GPIO_EXTI_Callback(SWITCH_1_Pin);
//	}
//}

void USART1_IRQHandler(void)
{
	static uint8_t last_char;
	if(huart1.Instance->ISR & USART_ISR_RXNE)
	{
		rx_buffer[last_char] = huart1.Instance->RDR;
		if (last_char >= 2){
			//have recieved the 3 characters necessary for updates
			last_char =0;
			switch(usart_state){
				case (Motor1Recieve):
						HAL_GPIO_WritePin(GPIOC, MOTOR_1_LED_Pin, GPIO_PIN_RESET);
						error_state &= ~0b01;
						usart_state = Motor2TX;
						break;
				case (Motor1Fail1):
						HAL_GPIO_WritePin(GPIOC, MOTOR_1_LED_Pin, GPIO_PIN_RESET);
						error_state &= ~0b01;
						usart_state = Motor2TX;
						break;
				case (Motor2Recieve):
						HAL_GPIO_WritePin(GPIOC, MOTOR_2_LED_Pin, GPIO_PIN_RESET);
						error_state &= ~0b10;
						usart_state = Motor1TX;
						break;
				case (Motor2Fail1):
						HAL_GPIO_WritePin(GPIOC, MOTOR_2_LED_Pin, GPIO_PIN_RESET);
						error_state &= ~0b10;
						usart_state = Motor1TX;
						break;
				default:
					break;
			}
		}
		else
		{
			last_char++;
		}
	}
	else
	{
		HAL_UART_IRQHandler(&huart1);
	}
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
