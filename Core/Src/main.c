/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "can.h"
#include "i2c.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define REFRESH_RATE 1000 // ms
#define CAN_TX_TIMEOUT 100 // ms
#define SET_RTC

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
RTC_TimeTypeDef timeNow;
RTC_DateTypeDef dateNow;
uint32_t softTimerOLED;
char lcd_line[32];
const char *day_of_week[] =
{ "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday" };

CAN_TxHeaderTypeDef txHeader;
uint8_t txData[8];
uint32_t txMailbox;
uint16_t txTimeoutCounter;

CAN_RxHeaderTypeDef rxHeader;
uint8_t rxData[8];
uint8_t canDatacheckTemperature = 0;
uint8_t canDatacheckPressure = 0;

float bmpPressure, bmpTemperature;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void GetCompilationTime(RTC_TimeTypeDef *timePtr);
void GetCompilationDate(RTC_DateTypeDef *datePtr);
void CalculateDayOfWeek(RTC_DateTypeDef *datePtr);
void SendCanMessage(void);
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
	MX_USART2_UART_Init();
	MX_CAN1_Init();
	MX_I2C1_Init();
	MX_RTC_Init();
	/* USER CODE BEGIN 2 */

	ssd1306_Init();
	ssd1306_Fill(Black);
	ssd1306_UpdateScreen();
	HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);

#ifdef SET_RTC
	GetCompilationDate(&dateNow);
	CalculateDayOfWeek(&dateNow);
	HAL_RTC_SetDate(&hrtc, &dateNow, RTC_FORMAT_BIN);
	GetCompilationTime(&timeNow);
	HAL_RTC_SetTime(&hrtc, &timeNow, RTC_FORMAT_BIN);
#endif

	HAL_CAN_Start(&hcan1);

	txHeader.IDE = CAN_ID_STD;
	txHeader.RTR = CAN_RTR_DATA; // https://www.kvaser.com/can-protocol-tutorial/
	txHeader.TransmitGlobalTime = DISABLE;

	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	softTimerOLED = HAL_GetTick();
	while (1)
	{

		if (1 == canDatacheckTemperature)
		{
			canDatacheckTemperature = 0;
			ssd1306_SetCursor(2, 45);
			sprintf(lcd_line, "RX FIFO0: %.0f deg. C", bmpTemperature);
			ssd1306_WriteString(lcd_line, Font_6x8, White);
			ssd1306_UpdateScreen();
		}

		if (1 == canDatacheckPressure)
		{
			canDatacheckPressure = 0;
			ssd1306_SetCursor(2, 55);
			sprintf(lcd_line, "RX FIFO0: %.0f hPa", bmpPressure);
			ssd1306_WriteString(lcd_line, Font_6x8, White);
			ssd1306_UpdateScreen();
		}

		if ((HAL_GetTick() - softTimerOLED) > REFRESH_RATE)
		{
			softTimerOLED = HAL_GetTick();
			HAL_RTC_GetTime(&hrtc, &timeNow, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &dateNow, RTC_FORMAT_BIN);

			ssd1306_SetCursor(2, 5);
			sprintf(lcd_line, "TX: %02d:%02d:%02d", timeNow.Hours,
					timeNow.Minutes, timeNow.Seconds);
			ssd1306_WriteString(lcd_line, Font_6x8, White);
			ssd1306_SetCursor(2, 15);
			sprintf(lcd_line, "TX: %02d.%02d.%04d", dateNow.Date, dateNow.Month,
					(2000 + dateNow.Year));
			ssd1306_WriteString(lcd_line, Font_6x8, White);
			ssd1306_SetCursor(2, 25);
			sprintf(lcd_line, "TX: %s", day_of_week[dateNow.WeekDay - 1]);
			ssd1306_WriteString(lcd_line, Font_6x8, White);
			ssd1306_UpdateScreen();

			// Time
			txHeader.DLC = 3;  // data length
			txHeader.StdId = 0x101;  // ID of the message/frame
			txData[0] = timeNow.Hours;
			txData[1] = timeNow.Minutes;
			txData[2] = timeNow.Seconds;
			SendCanMessage();

			// Date
			txHeader.DLC = 4;  // data length
			txHeader.StdId = 0x102;  // ID of the message/frame
			txData[0] = dateNow.Date;
			txData[1] = dateNow.Month;
			txData[2] = dateNow.Year;
			txData[3] = dateNow.WeekDay;
			SendCanMessage();

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
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI
			| RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 16;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Enable MSI Auto calibration
	 */
	HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData);
	if ((rxHeader.StdId == 0x103) && (rxHeader.DLC == 1))
	{
		canDatacheckTemperature = 1;
		bmpTemperature = rxData[0];
		HAL_GPIO_TogglePin(BLUE_LED_GPIO_Port, BLUE_LED_Pin);
	}
	else if ((rxHeader.StdId == 0x107) && (rxHeader.DLC == 2))
	{
		canDatacheckPressure = 1;
		bmpPressure = (((uint16_t) rxData[0]) << 8) | ((uint16_t) rxData[1]);
		HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
	}
}

void GetCompilationTime(RTC_TimeTypeDef *timePtr)
{
	char temp[] = __TIME__;

	timePtr->Seconds = atoi(temp + 6);
	*(temp + 5) = 0;
	timePtr->Minutes = atoi(temp + 3);
	*(temp + 2) = 0;
	timePtr->Hours = atoi(temp);
}

void GetCompilationDate(RTC_DateTypeDef *datePtr)
{
	const char *months[] =
	{ "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct",
			"Nov", "Dec" };
	char temp[] = __DATE__;
	uint8_t i;

	datePtr->Year = atoi(temp + 9);
	*(temp + 6) = 0;
	datePtr->Date = atoi(temp + 4);
	*(temp + 3) = 0;
	for (i = 0; i < 12; i++)
	{
		if (!strcmp(temp, months[i]))
		{
			datePtr->Month = i + 1;
			return;
		}
	}
}

// https://www.geeksforgeeks.org/find-day-of-the-week-for-a-given-date/
void CalculateDayOfWeek(RTC_DateTypeDef *datePtr)
{
	const int16_t t[] =
	{ 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };
	int16_t y, m, d;
	m = datePtr->Month;
	y = (datePtr->Year) + 2000 - (m < 3);
	d = datePtr->Date;
	datePtr->WeekDay = (y + y / 4 - y / 100 + y / 400 + t[m - 1] + d) % 7;
	if (datePtr->WeekDay == 0)
	{
		datePtr->WeekDay = 7;
	}
}

void SendCanMessage(void)
{
	if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox) != HAL_OK)
	{
		Error_Handler();
	}
	txTimeoutCounter = 0;
	while ((HAL_CAN_IsTxMessagePending(&hcan1, txMailbox))
			&& (txTimeoutCounter < CAN_TX_TIMEOUT))
	{
		//				__NOP();
		txTimeoutCounter++;
		HAL_Delay(1);
	}
	if (txTimeoutCounter == CAN_TX_TIMEOUT)
	{
		Error_Handler();
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
	HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
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
