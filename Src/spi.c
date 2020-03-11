/**
  ******************************************************************************
  * File Name          : SPI.c
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
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
#include "spi.h"
#include <string.h>

uint8_t LCD_error_queue = 0;
uint8_t LCD_queue_prev = 0;

faultDisplay_t motor1_faultDisplay;
faultDisplay_t motor2_faultDisplay;
faultDisplay_t motor12_faultDisplay;
faultDisplay_t p24V_faultDisplay;


/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }


  // Initilize GPIO for SPI
  HAL_SPI_MspInit(&hspi1);
  //HAL_SPI_Init(&hspi1);


}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitTypeDef GPIO_InitStructC = {0};
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();
  
    __HAL_RCC_GPIOA_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**SPI1 GPIO Configuration    
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStructC.Pin  = GPIO_PIN_4;
    GPIO_InitStructC.Mode = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStructC);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();
  
    /**SPI1 GPIO Configuration    
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4);

  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
void LCD_Full_Send(char LCD_data[])
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET); //CS Low
	HAL_SPI_Transmit(&hspi1, LCD_data, strlen(LCD_data), 50000);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET); //CS High
	LCD_data = 0;
}

void LCD_Motor_Error(uint8_t Motor_Error_State)
{

	char Error_Str[10] = "!!ERROR!!\n";
	char Motor1_Error_Str[10] = "MD 1 Fault";
	char Motor2_Error_Str[10] = "MD 2 Fault";
	char Motor12_Error_Str[14] = "MD 1 & 2 Fault";

//	if(LCD_error_queue == 1)
//	{
//		LCD_queue_prev = 1;

//		LCD_error_queue = 0;

		switch(Motor_Error_State)
		{
		case (1):
				if(motor1_faultDisplay == Fault_NotDisplayed)
				{
					LCD_Command(0x11);  		//Set Cursor position
					LCD_Command(0x00);			//Column Number
					LCD_Command(0x03);			//Row Number

					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET); //CS Low
					HAL_SPI_Transmit(&hspi1, Motor1_Error_Str, strlen(Motor1_Error_Str), 50000);
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET); //CS High
				}
				break;
		case (2):
				if(motor2_faultDisplay == Fault_NotDisplayed)
				{
					LCD_Command(0x11);  		//Set Cursor position
					LCD_Command(0x00);			//Column Number
					LCD_Command(0x03);			//Row Number

					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET); //CS Low
					HAL_SPI_Transmit(&hspi1, Motor2_Error_Str, strlen(Motor2_Error_Str), 50000);
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET); //CS High
				}
				break;
		case(3):
				if(motor12_faultDisplay == Fault_NotDisplayed)
				{
					LCD_Command(0x11);  		//Set Cursor position
					LCD_Command(0x00);			//Column Number
					LCD_Command(0x03);			//Row Number

					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET); //CS Low
					HAL_SPI_Transmit(&hspi1, Motor12_Error_Str, strlen(Motor12_Error_Str), 50000);
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET); //CS High
				}
				break;
		default:
			break;
		}

//	}
//
//
//
//	if((LCD_error_queue == 0) & (LCD_queue_prev == 0))
//	{
//		LCD_queue_prev = 0;
//		LCD_Command(0x11);  		//Set Cursor position
//		LCD_Command(0x00);			//Column Number
//		LCD_Command(0x03);			//Row Number
//		LCD_error_queue = 1;
//
////		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET); //CS Low
////		HAL_SPI_Transmit(&hspi1, Error_Str, strlen(Error_Str), 50000);
////		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET); //CS High
//
//		char Motor_Error_Str[18] = "Motor Driver Fault";
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET); //CS Low
//		HAL_SPI_Transmit(&hspi1, Motor_Error_Str, strlen(Motor_Error_Str), 50000);
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET); //CS High
//
//		LCD_Command(0x11);  		//Set Cursor position
//		LCD_Command(0x00);			//Column Number
//		LCD_Command(0x02);			//Row Number for third line
//	}

	/* Write error stuff here */
	if(Motor_Error_State == 0b01)
	{
		HAL_GPIO_WritePin(GPIOC, MOTOR_1_LED_Pin, GPIO_PIN_SET);
	} else if(Motor_Error_State == 0b10)
	{
		HAL_GPIO_WritePin(GPIOC, MOTOR_2_LED_Pin, GPIO_PIN_SET);
	} else if(Motor_Error_State == 0b11)
	{
		HAL_GPIO_WritePin(GPIOC, MOTOR_1_LED_Pin|MOTOR_2_LED_Pin, GPIO_PIN_SET);
	}

}

void LCD_24V_Error(void)
{

	char Error_Str[10] = "!!ERROR!!\n";
//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET); //CS Low
//	HAL_SPI_Transmit(&hspi1, Error_Str, strlen(Error_Str), 50000);
//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET); //CS High

	if((LCD_error_queue == 0) & (LCD_queue_prev == 1))
	{
		LCD_queue_prev = 0;
	}

//	if(LCD_error_queue == 1)
//	{
	if(p24V_faultDisplay == Fault_NotDisplayed)
	{
//		LCD_queue_prev = 1;
		LCD_Command(0x11);  		//Set Cursor position
		LCD_Command(0x00);			//Column Number
		LCD_Command(0x02);			//Row Number
//		LCD_error_queue = 0;

		char Power_Error_Str[9] = "24V Fault";
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET); //CS Low
		HAL_SPI_Transmit(&hspi1, Power_Error_Str, strlen(Power_Error_Str), 50000);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET); //CS High
	}

//	}
//
//	if((LCD_error_queue == 0) & (LCD_queue_prev == 0))
//	{
//		LCD_queue_prev = 0;
//		LCD_Command(0x11);  		//Set Cursor position
//		LCD_Command(0x00);			//Column Number
//		LCD_Command(0x01);			//Row Number
//		LCD_error_queue = 1;
//
//		char Power_Error_Str[14] = "24V Rail Fault";
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET); //CS Low
//		HAL_SPI_Transmit(&hspi1, Power_Error_Str, strlen(Power_Error_Str), 50000);
//		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET); //CS High
//
//		LCD_Command(0x11);  		//Set Cursor position
//		LCD_Command(0x00);			//Column Number
//		LCD_Command(0x02);			//Row Number for third line
//	}



	HAL_GPIO_WritePin(GPIOF, PGOOD_24V_LED_Pin, GPIO_PIN_SET);
}

void LCD_RPM_Transmit(uint8_t * RPM_Val, int length)
{
	char RPM_Str[6] = "RPM: ";

	LCD_Command(0x11);  		//Set Cursor position
	LCD_Command(0x00);			//Column Number
	LCD_Command(0x00);			//Row Number

	/* String */
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET); //CS Low
	HAL_SPI_Transmit(&hspi1, RPM_Str, strlen(RPM_Str), 50000);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET); //CS High
	/* RPM Value */
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET); //CS Low
	HAL_SPI_Transmit(&hspi1, RPM_Val, length, 50000);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET); //CS High
}

void LCD_Battery_Transmit(uint8_t Batt_Level)
{
	char Batt_Str[9] = "Battery: ";
	char Batt_Percent[2] = "%";
	char Batt_Level_String[2];
	IntToString(Batt_Level, Batt_Level_String, 2);
	/* Transmit Battery String */
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET); //CS Low
	HAL_SPI_Transmit(&hspi1, Batt_Str, strlen(Batt_Str), 50000);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET); //CS High
	/* Transmit Battery Level */
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET); //CS Low
	HAL_SPI_Transmit(&hspi1, Batt_Level_String, strlen(Batt_Level_String), 50000);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET); //CS High
	/* Transmit Battery Percent */
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET); //CS Low
	HAL_SPI_Transmit(&hspi1, Batt_Percent, strlen(Batt_Percent), 50000);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET); //CS High
}

void LCD_Start_Screen(void)
{
	char Start_Str[16] = "SKYTANIC:FEV-60";
	/* String */
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET); //CS Low
	HAL_SPI_Transmit(&hspi1, Start_Str, strlen(Start_Str), 50000);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET); //CS High
}

void LCD_Command(uint8_t LCD_cmd)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET); //CS Low
	HAL_SPI_Transmit(&hspi1, &LCD_cmd, 1, 50000);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET); //CS High
}

void LCD_Reboot(void)
{
	uint8_t blnk = 26;
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET); //CS Low
	//SPI_Transfer(84); //Write Enable?
	HAL_SPI_Transmit(&hspi1, &blnk, 1, 50000);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET); //CS High
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET); //CS Low
	//SPI_Transfer(84); //Write Enable?
	HAL_SPI_Transmit(&hspi1, &blnk, 1, 50000);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET); //CS High
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET); //CS Low
	//SPI_Transfer(84); //Write Enable?
	HAL_SPI_Transmit(&hspi1, &blnk, 1, 50000);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET); //CS High
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET); //CS Low
	//SPI_Transfer(84); //Write Enable?
	HAL_SPI_Transmit(&hspi1, &blnk, 1, 50000);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET); //CS High
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET); //CS Low
	//SPI_Transfer(84); //Write Enable?
	HAL_SPI_Transmit(&hspi1, &blnk, 1, 50000);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET); //CS High
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET); //CS Low
	//SPI_Transfer(84); //Write Enable?
	HAL_SPI_Transmit(&hspi1, &blnk, 1, 50000);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET); //CS High
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET); //CS Low
	//SPI_Transfer(84); //Write Enable?
	HAL_SPI_Transmit(&hspi1, &blnk, 1, 50000);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET); //CS High
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET); //CS Low
	//SPI_Transfer(84); //Write Enable?
	HAL_SPI_Transmit(&hspi1, &blnk, 1, 50000);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET); //CS High
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET); //CS Low
	//SPI_Transfer(84); //Write Enable?
	HAL_SPI_Transmit(&hspi1, &blnk, 1, 50000);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET); //CS High
	uint8_t clr = 26;
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET); //CS Low
	//SPI_Transfer(84); //Write Enable?
	HAL_SPI_Transmit(&hspi1, &clr, 1, 50000);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET); //CS High	uint8_t clr = 26;
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET); //CS Low
	//SPI_Transfer(84); //Write Enable?
	HAL_SPI_Transmit(&hspi1, &clr, 1, 50000);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET); //CS High
	HAL_Delay(20);
  	//LCD_Clear();
  	//LCD_Clear();
}

void LCD_Clear(void)
{
	uint8_t clr = LCD_CLEAR_CHARACTER;
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET); //CS Low
	HAL_Delay(1);
	//SPI_Transfer(84); //Write Enable?
	HAL_SPI_Transmit(&hspi1, &clr, 1, 50000);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET); //CS High
}



/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
