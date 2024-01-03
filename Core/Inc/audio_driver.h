/*
 * audio_driver.h
 *
 *  Created on: Dec 3, 2023
 *      Author: lukes
 */

#ifndef INC_AUDIO_DRIVER_H_
#define INC_AUDIO_DRIVER_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32l4xx_hal.h"
#define X_DIR_GPIO_Port GPIOF
#define X_DIR_GPIO_Pin GPIO_PIN_5

enum Level {
	NO_MUSIC = 0,
	GAME_START = 1,
	GAME_END = 2,
	PLAYER_WIN = 3,
	PLAYER_LOSE = 4,
	PLAYER_DRAW = 5,
	GAME_BACKGROUND_MUSIC = 6,
	PLAYER_TURN = 7
};

void Audio_StateMachine(DAC_HandleTypeDef*);

#ifdef __cplusplus
}
#endif

#endif /* INC_AUDIO_DRIVER_H_ */
