#ifndef TIME_HANDLING_H
#define TIME_HANDLING_H

#include <stdint.h>
#include "main.h"

extern uint8_t uart_rx_buf [1];
extern uint8_t hhmmss[9];
extern uint8_t toFill;
extern uint8_t prevNum;

extern RTC_HandleTypeDef hrtc;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

void set_time (uint8_t hr, uint8_t min, uint8_t sec);

void set_date (uint8_t year, uint8_t month, uint8_t date, uint8_t day);

void get_time_date(char *time, char *date);

void parseTime(uint8_t* hhmmss);

void render_time(void);
#endif
