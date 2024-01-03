/*
 * lcd_driver.h
 *
 *  Created on: Dec 4, 2023
 *      Author: aumair
 */

#ifndef INC_LCD_DRIVER_H_
#define INC_LCD_DRIVER_H_

#ifdef __cplusplus
 extern "C" {
#endif

void LCD_Init();
void LCD_CreateButton(unsigned char*, int*);
int LCD_CreateOptions(unsigned char*, unsigned char*, int*);
void LCD_Display_Text(unsigned char*, float, int);
void LCD_StartCountdown(uint8_t);
void LCD_DrawBoard();
void LCD_DrawBoardState(int*);
void LCD_DrawO(int);
void LCD_DrawX(int);
void LCD_Clear();

#ifdef __cplusplus
}
#endif
#endif /* INC_LCD_DRIVER_H_ */
