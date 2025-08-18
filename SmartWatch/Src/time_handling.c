/*
 * time_handling.c
 *
 *  Created on: Aug 18, 2025
 *      Author: ilia1
 */
#include "time_handling.h"
#include "ssd1306_fonts.h"
#include "ssd1306.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>

uint8_t uart_rx_buf [1] = {0};
uint8_t hhmmss[9] = {0};
uint8_t toFill = 0;
uint8_t prevNum = 0;

void set_time (uint8_t hr, uint8_t min, uint8_t sec) {
	RTC_TimeTypeDef sTime = {0};
	sTime.Hours = hr;
	sTime.Minutes = min;
	sTime.Seconds = sec;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
		Error_Handler();
	}
}

void set_date (uint8_t year, uint8_t month, uint8_t date, uint8_t day) { // monday = 1
	RTC_DateTypeDef sDate = {0};
	sDate.WeekDay = day;
	sDate.Month = month;
	sDate.Date = date;
	sDate.Year = year;
	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
		Error_Handler();
	}

	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x2345);  // backup register
	// If in main loop RTC_BKP_DR1 register contains 0x2345 -> don't update time
}

void get_time_date(char *time, char *date)
{
  RTC_DateTypeDef gDate;
  RTC_TimeTypeDef gTime;

  /* Get the RTC current Time */
  HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
  /* Get the RTC current Date */
  HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);

  /* Display time Format: hh:mm:ss */
  sprintf((char*)time,"%02d:%02d:%02d",gTime.Hours, gTime.Minutes, gTime.Seconds);

  /* Display date Format: dd-mm-yyyy */
  sprintf((char*)date,"%02d-%02d-%2d",gDate.Date, gDate.Month, 2000 + gDate.Year);
}

void parseTime(uint8_t* hhmmss) {
	set_time(hhmmss[2], hhmmss[1], hhmmss[0]);
	set_date(hhmmss[6], hhmmss[3], hhmmss[5], hhmmss[4]+1);
	HAL_UART_Transmit(&huart2, &hhmmss[6], 1, 10);
	// no need to divide as each element of hhmmss is uint8_t == 1 byte
	memset(hhmmss, 0, sizeof(*hhmmss));
}

void render_time(void) {
	// HH:MM:SS\n
	char time[9];
	// yyyy:mm:dd\n
	char date[11];
	get_time_date(time, date);
	// Forward byte to UART2
	//HAL_UART_Transmit(&huart2, (uint8_t*)time, 8, 100);
	ssd1306_Fill(0);
	ssd1306_SetCursor (0,0);
	ssd1306_WriteString (time, Font_14x15, White);
	ssd1306_SetCursor (0,30);
	ssd1306_WriteString (date, Font_7x10, White);
	ssd1306_UpdateScreen();
}

