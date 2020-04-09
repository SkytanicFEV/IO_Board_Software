/**
  ******************************************************************************
  * File Name          : TIM.c
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
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

/* Includes ------------------------------------------------------------------*/
#include "tim.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

TIM_HandleTypeDef htim1;

/* TIM1 init function */
void MX_TIM1_Init(void)
{
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = TIM_CLOCKPRESCALER_DIV8;
	htim1.Init.CounterMode = TIM_COUNTERMODE_DOWN;
	htim1.Init.Period = TIM_PERIOD;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}


	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}

//	HAL_TIM_MspPostInit(&htim1);
	// Enable clock tree
	__HAL_RCC_TIM1_CLK_ENABLE();

	if(HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4) != HAL_OK)
		{
			Error_Handler();
	}
	//htim1.Instance->CR1 |= TIM_OPMODE_SINGLE;
	// Configure and enable TIM3 interrupt channel in NVIC
	HAL_NVIC_SetPriority(TIM1_CC_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if(timHandle->Instance==TIM1)
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**TIM1 GPIO Configuration
		PA7     ------> TIM1_CH1N
		PB0     ------> TIM1_CH2N
		PB1     ------> TIM1_CH3N
		PA8     ------> TIM1_CH1
		PA9     ------> TIM1_CH2
		PA10     ------> TIM1_CH3
		PA11     ------> TIM1_CH4
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_11;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM1)
  {
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();

    /**TIM1 GPIO Configuration
    PA7     ------> TIM1_CH1N
    PB0     ------> TIM1_CH2N
    PB1     ------> TIM1_CH3N
    PA8     ------> TIM1_CH1
    PA9     ------> TIM1_CH2
    PA10     ------> TIM1_CH3
    PA11     ------> TIM1_CH4
    PA12     ------> TIM1_ETR
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11);
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
