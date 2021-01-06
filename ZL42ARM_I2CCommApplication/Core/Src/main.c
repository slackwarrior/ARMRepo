/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char expander_data;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// test func, for communication
void LED_SendDataToExpander(uint8_t color) {
	uint8_t data_to_send[4];
	switch (color) {
	case 0:
		data_to_send[0] = 0x07;
		break;
	case 1:
		data_to_send[0] = 0x06;
		break;
	case 2:
		data_to_send[0] = 0x05;
		break;
	case 3:
		data_to_send[0] = 0x03;
		break;
	default:
		data_to_send[0] = 0x00;
		break;
	}

	HAL_I2C_Master_Transmit(&hi2c1, I2C_EXPANDER_LED_WRITE_ADDR, data_to_send,
			1, I2C_EXPANDER_MAX_TIMEOUT);
}

uint8_t LED_ReadDataFromExpander(){
	uint8_t i2c_data_buff[4];

	i2c_data_buff[0] = 0x8;
	HAL_I2C_Master_Transmit(&hi2c1, I2C_EXPANDER_LED_WRITE_ADDR, &i2c_data_buff[0],
				1, I2C_EXPANDER_MAX_TIMEOUT);

	// data_to_send[2]
	HAL_I2C_Master_Receive(&hi2c1, I2C_EXPANDER_LED_READ_ADDR, &i2c_data_buff[2], 1, I2C_EXPANDER_MAX_TIMEOUT);
	return i2c_data_buff[2];
}

void LCD_SendDataToExpander(const uint8_t *data, uint8_t datacount) {
	HAL_I2C_Master_Transmit(&hi2c1, I2C_EXPANDER_LCD_DISPLAY_WRITE_ADDR, data,
			datacount, I2C_EXPANDER_MAX_TIMEOUT);
}

uint8_t LCD_ReadDataFromExpander() {
	uint8_t buffer_to_send;
	uint8_t read_buffer[4];
	buffer_to_send = 0xf0;		// read only HiNibble of the expander

	buffer_to_send = buffer_to_send & ~(1);			// clear RS
	buffer_to_send = buffer_to_send | (1 << 1);		// set R/W
	buffer_to_send = buffer_to_send & ~(1 << 2); 	// clear E
	buffer_to_send = buffer_to_send | (1 << 3);
	HAL_I2C_Master_Transmit(&hi2c1, I2C_EXPANDER_LCD_DISPLAY_WRITE_ADDR,
			buffer_to_send, 1, I2C_EXPANDER_MAX_TIMEOUT);

	buffer_to_send = buffer_to_send & ~(1);			// clear RS
	buffer_to_send = buffer_to_send | (1 << 1);		// set R/W
	buffer_to_send = buffer_to_send | (1 << 2); 	// set E
	buffer_to_send = buffer_to_send | (1 << 3);
	HAL_I2C_Master_Transmit(&hi2c1, I2C_EXPANDER_LCD_DISPLAY_WRITE_ADDR,
			buffer_to_send, 1, I2C_EXPANDER_MAX_TIMEOUT);

	// read data from I2C expander
	HAL_I2C_Master_Receive(&hi2c1, I2C_EXPANDER_LCD_DISPLAY_READ_ADDR,
			&read_buffer[0], 1, I2C_EXPANDER_MAX_TIMEOUT);

	buffer_to_send = buffer_to_send | (1 << 1);		// set R/W
	buffer_to_send = buffer_to_send & ~(1 << 2); 	// clear E
	buffer_to_send = buffer_to_send | (1 << 3);
	HAL_I2C_Master_Transmit(&hi2c1, I2C_EXPANDER_LCD_DISPLAY_WRITE_ADDR,
			buffer_to_send, 1, I2C_EXPANDER_MAX_TIMEOUT);

	buffer_to_send = buffer_to_send | (1 << 1);		// set R/W
	buffer_to_send = buffer_to_send | (1 << 2); 	// set E
	buffer_to_send = buffer_to_send | (1 << 3);
	HAL_I2C_Master_Transmit(&hi2c1, I2C_EXPANDER_LCD_DISPLAY_WRITE_ADDR,
			buffer_to_send, 1, I2C_EXPANDER_MAX_TIMEOUT);

	// read data from I2C expander
	HAL_I2C_Master_Receive(&hi2c1, I2C_EXPANDER_LCD_DISPLAY_READ_ADDR,
			&read_buffer[1], 1, I2C_EXPANDER_MAX_TIMEOUT);

	buffer_to_send = buffer_to_send & ~(1 << 1);		// set R/W
		buffer_to_send = buffer_to_send & ~(1 << 2); 	// clear E
		buffer_to_send = buffer_to_send | (1 << 3);
		HAL_I2C_Master_Transmit(&hi2c1, I2C_EXPANDER_LCD_DISPLAY_WRITE_ADDR,
				buffer_to_send, 1, I2C_EXPANDER_MAX_TIMEOUT);

	return read_buffer[0] | (read_buffer[1] >> 4);
}

void LCD_SetBacklight(uint8_t state) {
	uint8_t buff[2];
	if (0 != state)
		buff[0] |= (1 << 3);
	else
		buff[0] &= ~(1 << 3);

	LCD_SendDataToExpander(buff, 1);
}

void I2C_LCD_SendData(uint8_t data) {
	uint8_t databuff[2];
	// send HiNibble
	databuff[0] = (data & 0xf0) | 0x09;
	LCD_SendDataToExpander(databuff, 1);
	HAL_Delay(1);

	databuff[0] = (data & 0xf0) | 0x0d;
	LCD_SendDataToExpander(databuff, 1);
	HAL_Delay(1);

	// send LoNibble
	databuff[0] = ((data & 0x0f) << 4) | 0x09;
	LCD_SendDataToExpander(databuff, 1);
	HAL_Delay(1);

	databuff[0] = ((data & 0x0f) << 4) | 0x0d;
	LCD_SendDataToExpander(databuff, 1);
	HAL_Delay(1);
}

void I2C_LCD_SendCommandByte(uint8_t data) {
	uint8_t databuff[2];

	// send HiNibble
	databuff[0] = (data & 0xf0) | 0x08;
	LCD_SendDataToExpander(databuff, 1);
	HAL_Delay(1);

	databuff[0] = (data & 0xf0) | 0x0c;
	LCD_SendDataToExpander(databuff, 1);
	HAL_Delay(1);

	// send LoNibble
	databuff[0] = ((data & 0x0f) << 4) | 0x08;
	LCD_SendDataToExpander(databuff, 1);
	HAL_Delay(1);

	databuff[0] = ((data & 0x0f) << 4) | 0x0c;
	LCD_SendDataToExpander(databuff, 1);
	HAL_Delay(1);
}

uint8_t I2C_LCD_ReadData() {
	uint8_t databuff[2];
	//init_buff[0] = 0xf8;	// D[7..4] = 0x03 BL = 1, E = 0, RW = 0, RS = 0
	//LCD_SendDataToExpander(init_buff, 1);
	//init_buff[0] = 0xfc;	// D[7..4] = 0x03 BL = 1, E = 0, RW = 0, RS = 0
	//LCD_SendDataToExpander(init_buff, 1);
	return 0;
}

void I2C_LCDInit() {
	uint8_t init_buff[2];

	HAL_Delay(1500);
	init_buff[0] = 0x38;	// D[7..4] = 0x03 BL = 1, E = 0, RW = 0, RS = 0
	LCD_SendDataToExpander(init_buff, 1);

	init_buff[0] = 0x3c;	// D[7..4] = 0x03 BL = 1, E = 1, RW = 0, RS = 0
	LCD_SendDataToExpander(init_buff, 1);
	HAL_Delay(4);

	// (2)
	init_buff[0] = 0x38;	// D[7..4] = 0x03 BL = 1, E = 0, RW = 0, RS = 0
	LCD_SendDataToExpander(init_buff, 1);

	init_buff[0] = 0x3c;	// D[7..4] = 0x03 BL = 1, E = 1, RW = 0, RS = 0
	LCD_SendDataToExpander(init_buff, 1);
	HAL_Delay(1);

	// (3)
	init_buff[0] = 0x38;	// D[7..4] = 0x03 BL = 1, E = 0, RW = 0, RS = 0
	LCD_SendDataToExpander(init_buff, 1);

	init_buff[0] = 0x3c;	// D[7..4] = 0x03 BL = 1, E = 1, RW = 0, RS = 0
	LCD_SendDataToExpander(init_buff, 1);
	HAL_Delay(1);

	//
	init_buff[0] = 0x28;	// D[7..4] = 0x02 BL = 1, E = 0, RW = 0, RS = 0
	LCD_SendDataToExpander(init_buff, 1);

	init_buff[0] = 0x2c;	// D[7..4] = 0x02 BL = 1, E = 1, RW = 0, RS = 0
	LCD_SendDataToExpander(init_buff, 1);

	// Właściwa inicjalizacja zakończona, ustawianie parametrów
	// Ustawiamy parametry: N = 1 F = 0
	I2C_LCD_SendCommandByte(0x28);

	I2C_LCD_SendCommandByte(0x08);

	I2C_LCD_SendCommandByte(0x01);

	// I/D = 1 S = 0
	I2C_LCD_SendCommandByte(0x06);

}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	char i2c_data_count;
	char i2c_comm_buffer[256];

	uint8_t backlight;

	uint8_t data_to_send, command_to_send;

	uint8_t testvar;

	uint8_t counter;
	uint8_t line_01[16] = "Juz umiem       ";
	uint8_t *line1_ptr = &line_01[0];
	uint8_t line_02[16] = "  w wyswietlacz!";
	uint8_t *line2_ptr = &line_02[0];

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
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */

	// LCD init
	I2C_LCDInit();
	// (1)

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		I2C_LCD_SendCommandByte(0x08 + 0x04 + 0x02);

		// set pos to 0,0
		I2C_LCD_SendCommandByte(0x80);
		for (counter = 0; counter < 16; counter++) {
			I2C_LCD_SendData(*line1_ptr++);
		}
		line1_ptr = &line_01;

		i2c_comm_buffer[0]=LCD_ReadDataFromExpander();

		// set pos to 0,1
		I2C_LCD_SendCommandByte(0x80 + 0x40);
		for (counter = 0; counter < 16; counter++) {
			I2C_LCD_SendData(*line2_ptr++);
		}
		line2_ptr = &line2_ptr;

		testvar = LED_ReadDataFromExpander();
		if (testvar & 0x80){
			LED_SendDataToExpander(5);
		}
		else {
			LED_SendDataToExpander(0);
		}
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
	/** Enables the Clock Security System
	 */
	HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
