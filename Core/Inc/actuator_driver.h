/*
 * actuator_driver.h
 *
 *  Created on: Dec 1, 2023
 *      Author: Ummy
 */

#ifndef INC_ACTUATOR_DRIVER_H_
#define INC_ACTUATOR_DRIVER_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32l4xx_hal.h"

#define ACTUATOR_GPIO_Port_0 GPIOB
#define ACTUATOR_GPIO_Pin_0 GPIO_PIN_9
#define ACTUATOR_GPIO_Port_1 GPIOB
#define ACTUATOR_GPIO_Pin_1 GPIO_PIN_8

void Actuator_Toggle();
void Actuator_Down();
void Actuator_Up();
void Actuator_Write(uint8_t, uint8_t);

#ifdef __cplusplus
}
#endif
#endif /* INC_ACTUATOR_DRIVER_H_ */
