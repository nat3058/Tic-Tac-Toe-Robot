/*
 * motor_driver.c
 *
 *  Created on: Nov 20, 2023
 *      Author: aumair
 */

#include "motor_driver.h"
#include "actuator_driver.h"
#include "stdio.h"

extern TIM_HandleTypeDef htim16;

// X direction is width; Y direction is length
const float CENTER_X[3] = {0.15, 0.45, 0.75}; // fraction of the total width corresponding to the center of each square in the x-direction 
const float CENTER_Y[3] = {0.40, 0.65, 0.90}; // fraction of the total length corresponding to the center of each square in the y-directio

const GPIO_TypeDef* DIR_GPIO_Port[2] = {X_DIR_GPIO_Port, Y_DIR_GPIO_Port};
const uint16_t DIR_GPIO_Pin[2] = {X_DIR_GPIO_Pin, Y_DIR_GPIO_Pin};

uint8_t xPlus = 0;
uint8_t xMinus = 0;
uint8_t yPlus = 0;
uint8_t yMinus = 0;

// init the motors and find the total length and width (in units of time (milliseconds)) 
void Motor_Init(struct Plotter* this) {
	// move to all four cornors
	Motor_Move_XY(this, 0, 0);
	Motor_Move_XY(this, 1, 1);
	Motor_Move_XY(this, 0, 0);
}

void Motor_Move_XY(struct Plotter* this, float xf, float yf) {
	Motor_Move(this, 'x', xf);
	Motor_Move(this, 'y', yf);
}

void Motor_Reset(struct Plotter* this) {
	Motor_Move_XY(this, 0, 0);
}

void Motor_Move_Square(struct Plotter* this, uint8_t s) {
	Motor_Move_XY(this, CENTER_X[s % 3], CENTER_Y[s / 3]);
}

void Motor_DrawX(struct Plotter* this, uint16_t width) {
	Actuator_Up();
	// width parameter referes to the width of the X drawing
	// to start the first line, move up 0.5 * width in X direction (vertical) and move right 0.5 * width in Y direction (horizontal)  
	Motor_Move_XY(this, ((float) this->posX - width/2)/this->width, ((float) this->posY - width/2)/this->length);

	// draw first line of the X
	Actuator_Down();
	HAL_GPIO_WritePin(DIR_GPIO_Port[0], DIR_GPIO_Pin[0], 1); // set motor direction to move away from origin (X-axis)
	HAL_GPIO_WritePin(DIR_GPIO_Port[1], DIR_GPIO_Pin[1], 1); // set motor direction to move away from origin (Y-axis)
	HAL_TIM_PWM_Start(this->ptimx, this->timChannelX); // drive PWMs for both directions to draw a slanted line!
	HAL_TIM_PWM_Start(this->ptimy, this->timChannelY);
	HAL_Delay(width); // draw line as per width of the X
	HAL_TIM_PWM_Stop(this->ptimx, this->timChannelX);
	HAL_TIM_PWM_Stop(this->ptimy, this->timChannelY);
	this->posX += width; // update to current location
	this->posY += width; // update to current location

	Actuator_Up();
	Motor_Move_XY(this, ((float) this->posX - width)/this->width, (float) this->posY/this->length); // go up only in X direction
	// draw second line of the X
	Actuator_Down();
	HAL_GPIO_WritePin(DIR_GPIO_Port[0], DIR_GPIO_Pin[0], 1); // set motor direction to move away from origin (X-axis)
	HAL_GPIO_WritePin(DIR_GPIO_Port[1], DIR_GPIO_Pin[1], 0); // set motor direction to move closer to origin (Y-axis)
	HAL_TIM_PWM_Start(this->ptimx, this->timChannelX);
	HAL_TIM_PWM_Start(this->ptimy, this->timChannelY);
	HAL_Delay(width);
	HAL_TIM_PWM_Stop(this->ptimx, this->timChannelX);
	HAL_TIM_PWM_Stop(this->ptimy, this->timChannelY);
	this->posX += width;
	this->posY -= width; // update to current location

	Actuator_Up();
	Motor_Move_XY(this, ((float) this->posX - width/2)/this->width, ((float) this->posY + width/2)/this->length); // go back to the center of the square
}

void Motor_DrawBoard(struct Plotter* this) {
	Actuator_Up();
	float line_1 = (CENTER_X[0] + CENTER_X[1])/2;
	float line_2 = (CENTER_X[1] + CENTER_X[2])/2;
	float line_3 = (CENTER_Y[0] + CENTER_Y[1])/2;
	float line_4 = (CENTER_Y[1] + CENTER_Y[2])/2;
	Motor_Move_XY(this, line_1, 2*line_3 - line_4); // draw first line
	Actuator_Down();
	Motor_Move_XY(this, line_1, 0.99); // drag the drawing tip attached to the acutator horizontally
	Actuator_Up();
	Motor_Move_XY(this, line_2, 0.99);
	Actuator_Down();
	Motor_Move_XY(this, line_2, 2*line_3 - line_4);
	Actuator_Up();
	Motor_Move_XY(this, 0, line_3);
	Actuator_Down();
	Motor_Move_XY(this, 0.99, line_3); // drag the drawing tip attached to the acutator vertically
	Actuator_Up();
	Motor_Move_XY(this, 0.99, line_4);
	Actuator_Down();
	Motor_Move_XY(this, 0, line_4);
	Actuator_Up();
	Motor_Reset(this);
}

void Motor_Move(struct Plotter* this, char side, float destf) {
	TIM_HandleTypeDef* ptim;
	uint32_t channel;
	uint32_t* side_ptr;
	uint32_t* pos_ptr;
	if (side == 'x') {
		ptim = this->ptimx;
		channel = this->timChannelX;
		side_ptr = &this->width;
		pos_ptr = &this->posX;
	} else {
		ptim = this->ptimy;
		channel = this->timChannelY;
		side_ptr = &this->length;
		pos_ptr = &this->posY;
	}
	uint32_t dest = (int)(destf*(*side_ptr));
	HAL_TIM_PWM_Start(ptim, channel);
	if (destf == 0) {
		HAL_GPIO_WritePin(DIR_GPIO_Port[side - 'x'], DIR_GPIO_Pin[side - 'x'], 0); // move closer to origin
		while (side == 'x' ? !xMinus : !yMinus) {
			HAL_Delay(1);
		}
		dest = 0;
	} else if (destf == 1) {
		int truePos = *pos_ptr;
		HAL_GPIO_WritePin(DIR_GPIO_Port[side - 'x'], DIR_GPIO_Pin[side - 'x'], 1); // move away from origin
		HAL_TIM_Base_Start(&htim16);
		__HAL_TIM_SET_COUNTER(&htim16, 0);
		while (side == 'x' ? !xPlus : !yPlus) {
			HAL_Delay(1);
		}
		truePos = __HAL_TIM_GET_COUNTER(&htim16);
		HAL_TIM_Base_Stop(&htim16);
		*side_ptr = truePos;
		printf("%s: %i\n\r", side == 'x' ? "width" : "length", *side_ptr);
		dest = *side_ptr;
	} else {
		if (dest > *pos_ptr) {
			HAL_GPIO_WritePin(DIR_GPIO_Port[side - 'x'], DIR_GPIO_Pin[side - 'x'], 1);
			HAL_Delay(dest - *pos_ptr);
		} else {
			HAL_GPIO_WritePin(DIR_GPIO_Port[side - 'x'], DIR_GPIO_Pin[side - 'x'], 0);
			HAL_Delay(*pos_ptr - dest);
		}
	}
	HAL_TIM_PWM_Stop(ptim, channel);
	*pos_ptr = dest;
}
