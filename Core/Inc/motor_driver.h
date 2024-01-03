/*
 * motor_driver.h
 *
 *  Created on: Nov 20, 2023
 *      Author: aumair
 */
#ifndef __MOTOR_DRIVER_H
#define __MOTOR_DRIVER_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32l4xx_hal.h"
#define X_DIR_GPIO_Port GPIOF
#define X_DIR_GPIO_Pin GPIO_PIN_5
#define Y_DIR_GPIO_Port GPIOD
#define Y_DIR_GPIO_Pin GPIO_PIN_2
#define X_PLUS_Limit_Port GPIOB
#define X_PLUS_Limit_Pin GPIO_PIN_10
#define X_MINUS_Limit_Port GPIOF
#define X_MINUS_Limit_Pin GPIO_PIN_3
#define Y_PLUS_Limit_Port GPIOG
#define Y_PLUS_Limit_Pin GPIO_PIN_1
#define Y_MINUS_Limit_Port GPIOG
#define Y_MINUS_Limit_Pin GPIO_PIN_0

struct Plotter {
	uint32_t width;
	uint32_t length;
	uint32_t posX;
	uint32_t posY;
	TIM_HandleTypeDef* ptimx;
	TIM_HandleTypeDef* ptimy;
	uint32_t timChannelX;
	uint32_t timChannelY;
};

void Motor_Init(struct Plotter*);
void Motor_Move_XY(struct Plotter*, float, float);
void Motor_Reset(struct Plotter*);
void Motor_Move_Square(struct Plotter*, uint8_t);
void Motor_DrawX(struct Plotter*, uint16_t);
void Motor_DrawBoard(struct Plotter*);
void Motor_Move(struct Plotter*, char, float);

#ifdef __cplusplus
}
#endif
#endif /* __MOTOR_DRIVER_H */
