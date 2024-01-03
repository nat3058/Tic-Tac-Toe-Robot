/*
 * lcd_driver.c
 *
 *  Created on: Dec 4, 2023
 *      Author: aumair
 */


#include "stm32_adafruit_lcd.h"
#include "stm32_adafruit_ts.h"
#include "audio_driver.h"
#include "lcd_driver.h"
#include "stm32l4xx_hal.h"
#include <stdlib.h>

const uint8_t realBoard[9] = {2, 5, 8, 1, 4, 7, 0, 3, 6};

void LCD_Init() {
	BSP_LCD_Init();
	BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
}

void LCD_CreateButton(unsigned char* text, int* audio) {
	uint16_t h_size = BSP_LCD_GetXSize() / 3;
	uint16_t v_size = BSP_LCD_GetYSize() / 3;
	uint16_t x_pos = (BSP_LCD_GetXSize() / 2) - h_size;
	uint16_t y_pos = (BSP_LCD_GetYSize() / 2) - (v_size / 2);
	LCD_Clear();
	BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	BSP_LCD_FillRect(x_pos, y_pos, h_size * 2, v_size);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() / 2, text, CENTER_MODE);
	TS_StateTypeDef ts;
	while (1) {
		BSP_TS_GetState(&ts);
		if (ts.TouchDetected) {
			if (ts.X > x_pos && ts.X < (x_pos + h_size * 2) &&
			ts.Y > y_pos && ts.Y < (y_pos + v_size)) {
				break;
			}
		}
		HAL_Delay(100);
	}
	*audio = GAME_START;
	for (int i = 0; i < 2; ++i) {
		BSP_LCD_DrawRect(x_pos, y_pos, h_size * 2, v_size);
		BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
		HAL_Delay(100);
		BSP_LCD_DrawRect(x_pos, y_pos, h_size * 2, v_size);
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		HAL_Delay(100);
	}
	LCD_Clear();
}

int LCD_CreateOptions(unsigned char* option1, unsigned char* option2, int* audio) {
	uint16_t h_size = BSP_LCD_GetXSize() / 6;
	uint16_t v_size = BSP_LCD_GetYSize() / 5;
	BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	BSP_LCD_FillRect(h_size/2, v_size, h_size * 5, v_size);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_DisplayStringAt(0, v_size, option1, CENTER_MODE);
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	BSP_LCD_FillRect(h_size/2, v_size*3.5, h_size * 5, v_size);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_DisplayStringAt(0, v_size*3.5, option2, CENTER_MODE);
	TS_StateTypeDef ts;
	while (1) {
		BSP_TS_GetState(&ts);
		if (ts.TouchDetected) {
			if (ts.X > h_size/2 && ts.X < h_size*5.5) {
				if (ts.Y > v_size && ts.Y < v_size*2) {
					*audio = GAME_START;
					LCD_Clear();
					return 1;
				} else if (ts.Y > v_size*3 && ts.Y < v_size*4) {
					*audio = GAME_END;
					LCD_Clear();
					return 2;
				}
			}
		}
		HAL_Delay(100);
	}
}

void LCD_Display_Text(unsigned char* text, float pos, int time) {
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize() * pos, text, CENTER_MODE);
	HAL_Delay(time);
}

void LCD_StartCountdown(uint8_t time) {
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_SetFont(&Font24);
	for (int i = time; i >= 0; i--) {
		if (i / 10 != (i+1) / 10)
			BSP_LCD_ClearStringLine(4);
		unsigned char* buf[3];
		itoa(i, buf, 10);
		BSP_LCD_DisplayStringAt(0, LINE(4), buf, CENTER_MODE);
		HAL_Delay(1000);
	}
	BSP_LCD_SetFont(&LCD_DEFAULT_FONT);
}

void LCD_DrawBoard() {
	uint16_t h_size = BSP_LCD_GetXSize() / 3;
	uint16_t v_size = BSP_LCD_GetYSize() / 3;
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_DrawHLine(0, v_size, BSP_LCD_GetXSize());
	BSP_LCD_DrawHLine(0, v_size * 2, BSP_LCD_GetXSize());
	BSP_LCD_DrawVLine(h_size, 0, BSP_LCD_GetYSize());
	BSP_LCD_DrawVLine(h_size * 2, 0, BSP_LCD_GetYSize());
}

void LCD_DrawBoardState(int* board_state) {
	LCD_Clear();
	LCD_DrawBoard();
	for (int i = 0; i < 9; i++) {
		if (board_state[i] == 1)
			LCD_DrawX(i);
		if (board_state[i] == 2)
			LCD_DrawO(i);
	}
}

void LCD_DrawO(int square) {
	uint16_t h_size = BSP_LCD_GetXSize() / 3;
	uint16_t v_size = BSP_LCD_GetYSize() / 3;
	uint16_t min = h_size < v_size ? h_size : v_size;
	square = realBoard[square];
	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	BSP_LCD_DrawCircle((square % 3)*h_size+h_size/2, (square / 3)*v_size+v_size/2, 0.45*min);
}

void LCD_DrawX(int square) {
	uint16_t h_size = BSP_LCD_GetXSize() / 3;
	uint16_t v_size = BSP_LCD_GetYSize() / 3;
	uint16_t min = h_size < v_size ? h_size : v_size;;
	square = realBoard[square];
	uint16_t center_x = (square % 3)*h_size+h_size/2;
	uint16_t center_y = (square / 3)*v_size+v_size/2;
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	BSP_LCD_DrawLine(center_x-0.45*min, center_y-0.45*min, center_x+0.45*min, center_y+0.45*min);
	BSP_LCD_DrawLine(center_x-0.45*min, center_y+0.45*min, center_x+0.45*min, center_y-0.45*min);
}

void LCD_Clear() {
	BSP_LCD_Clear(LCD_DEFAULT_BACKCOLOR);
}
