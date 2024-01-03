/*
 * ttt_game.h
 *
 *  Created on: Dec 3, 2023
 *      Author: lukes
 */

#ifndef TTT_GAME_H_
#define TTT_GAME_H_


#ifdef __cplusplus
 extern "C" {
#endif

#include "motor_driver.h"
#include "stm32l4xx_hal.h"

struct Square{
    int idx;
    int value;
};

int determine_square_idx(int*, int*);
void print_prompt(void);
void print_board(int*);
char get_xo(int num);
int check_for_win(int*);
int easyAI(int*);
int impossibleAI(int*);
int minimax(int*, int);
void mainApp(struct Plotter*, UART_HandleTypeDef*);

#ifdef __cplusplus
}
#endif
#endif /* TTT_GAME_H_ */
