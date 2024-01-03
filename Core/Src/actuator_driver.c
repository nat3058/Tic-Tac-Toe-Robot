/*
 * actuator_driver.c
 *
 *  Created on: Dec 1, 2023
 *      Author: Ummy
 */

#include "actuator_driver.h"

const GPIO_TypeDef* ACTUATOR_GPIO_Ports[2] = {ACTUATOR_GPIO_Port_0, ACTUATOR_GPIO_Port_1};
const uint16_t ACTUATOR_GPIO_Pins[2] = {ACTUATOR_GPIO_Pin_0, ACTUATOR_GPIO_Pin_1};

void Actuator_Toggle() {
	Actuator_Down();
	HAL_Delay(500);
	Actuator_Up();
}

void Actuator_Down() {
	Actuator_Write(0, 1);
	HAL_Delay(25);
	Actuator_Write(0, 0);
}

void Actuator_Up() {
	Actuator_Write(1, 0);
	HAL_Delay(250);
	Actuator_Write(0, 0);
}

void Actuator_Write(uint8_t val0, uint8_t val1) {
	  HAL_GPIO_WritePin(ACTUATOR_GPIO_Ports[0], ACTUATOR_GPIO_Pins[0], val0);
	  HAL_GPIO_WritePin(ACTUATOR_GPIO_Ports[1], ACTUATOR_GPIO_Pins[1], val1);
}
