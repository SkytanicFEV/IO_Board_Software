/**
  ******************************************************************************
  * File Name          : SPI.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __spi_H
#define __spi_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi1;

#define LCD_CLEAR_CHARACTER						(0x0C)

void MX_SPI1_Init(void);

/* USER CODE BEGIN Prototypes */
void LCD_Full_Send(char LCD_data[]);
void LCD_Battery_Transmit(uint8_t Batt_Level);
void LCD_Motor_Error(uint8_t Motor_Error_State);
void LCD_24V_Error(void);
void LCD_RPM_Transmit(uint8_t * RPM_Val, int length);
void LCD_Start_Screen(void);
void LCD_Command(uint8_t LCD_cmd);
void LCD_Clear(void);
void LCD_Reboot(void);


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ spi_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
