/*
 * audio_driver.c
 *
 *  Created on: Dec 3, 2023
 *      Author: lukes
 */


#include "audio_driver.h"
#include "bwomp.h"
#include "chime.h"
#include "espionage.h"
#include "mario.h"
#include "trumpet.h"
#include "whistle.h"
#include "youwin.h"

uint32_t current_audio_state = NO_MUSIC; // GLOBAL!

void Audio_StateMachine(DAC_HandleTypeDef* phdac) {
	static uint32_t prev_audio_state;
	static uint32_t index = 0;
	static uint32_t background_index = 0;

	// AUDIO STATE MACHINE
	// check every state
	// for every state, check if you are at the end of the audio file
	// if not, continue playing the audio for this state

	if(current_audio_state != prev_audio_state) {
		index = 0;
	}

	switch(current_audio_state) {
	case NO_MUSIC:
		background_index = 0;
		break;
	case GAME_START:
		if(index > TRUMPET_NUM_ELEMENTS) {
			index = 0;
			background_index = 0;
			current_audio_state = GAME_BACKGROUND_MUSIC;
			return;
		}
		HAL_DAC_SetValue(phdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, trumpet_data[index++]);
		break;
	case GAME_END:
		if(index > MARIO_NUM_ELEMENTS) {
			index = 0;
			current_audio_state = NO_MUSIC;
			return;
		}
		HAL_DAC_SetValue(phdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, mario_data[index++]);
		break;
	case PLAYER_WIN:
		if(index > YOU_WIN_NUM_ELEMENTS) {
			index = 0;
			current_audio_state = NO_MUSIC;
			return;
		}
		HAL_DAC_SetValue(phdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, youwin_data[index++]);
		break;
	case PLAYER_LOSE:
		if(index > BWOMP_NUM_ELEMENTS) {
			index = 0;
			current_audio_state = NO_MUSIC;
			return;
		}
		HAL_DAC_SetValue(phdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, bwomp_data[index++]);
		break;
	case PLAYER_DRAW:
		if(index > WHISTLE_NUM_ELEMENTS) {
			index = 0;
			current_audio_state = NO_MUSIC;
			return;
		}
		HAL_DAC_SetValue(phdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, whistle_data[index++]);
		break;
	case GAME_BACKGROUND_MUSIC:
		HAL_DAC_SetValue(phdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, espionage_data[background_index]);
		break;
	case PLAYER_TURN:
		if(index > CHIME_NUM_ELEMENTS) {
			index = 0;
			current_audio_state = GAME_BACKGROUND_MUSIC;
			return;
		}
		HAL_DAC_SetValue(phdac, DAC_CHANNEL_1, DAC_ALIGN_8B_R, chime_data[index++]);
		break;
	}

	if(++background_index > ESPIONAGE_NUM_ELEMENTS)
		background_index = 0;

	prev_audio_state = current_audio_state;
}
