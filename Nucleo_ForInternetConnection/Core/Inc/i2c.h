/**
  ******************************************************************************
  * @file    i2c.h
  * @brief   This file contains all the function prototypes for
  *          the i2c.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#ifndef __I2C_H__
#define __I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN Private defines */
#define I2C_RTC_WRITE_ADDR 0xa0
#define I2C_RTC_READ_ADDR 0xa1

// Currently unused
#define I2C_EXPANDER_01_WRITE_ADDR 0x40
#define I2C_EXPANDER_01_READ_ADDR 0x41

// Only LED, on 3 lowest bits
#define I2C_EXPANDER_LED_WRITE_ADDR 0x70
#define I2C_EXPANDER_LED_READ_ADDR 0x71

// P0 -> RS
// P1 -> RW
// P2 -> E
// P3 -> <backlight>
// P4 -> D4
// P5 -> D5
// P6 -> D6
// P7 -> D7

#define	I2C_EXPANDER_MAX_TIMEOUT	0xffff

// HD44780
#define I2C_EXPANDER_LCD_DISPLAY_WRITE_ADDR 0x7e
#define I2C_EXPANDER_LCD_DISPLAY_READ_ADDR 0x7f


/* USER CODE END Private defines */

void MX_I2C1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __I2C_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
