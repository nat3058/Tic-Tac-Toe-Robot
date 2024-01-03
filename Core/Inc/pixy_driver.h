/*
 * pixy_driver.h
 *
 *  Created on: Dec 3, 2023
 *      Author: lukes
 */

#ifndef INC_PIXY_DRIVER_H_
#define INC_PIXY_DRIVER_H_

#ifdef __cplusplus
 extern "C" {
#endif


#include "stm32l4xx_hal.h"
#include <stdlib.h>

#define CCC_MAX_SIGNATURE                   7
#define CCC_RESPONSE_BLOCKS                 0x21
#define CCC_REQUEST_BLOCKS                  0x20

// Defines for sigmap:
// You can bitwise "or" these together to make a custom sigmap.
// For example if you're only interested in receiving blocks
// with signatures 1 and 5, you could use a sigmap of
// PIXY_SIG1 | PIXY_SIG5
#define CCC_SIG1                     1
#define CCC_SIG2                     2
#define CCC_SIG3                     4
#define CCC_SIG4                     8
#define CCC_SIG5                     16
#define CCC_SIG6                     32
#define CCC_SIG7                     64
#define CCC_COLOR_CODES              128

#define CCC_SIG_ALL                  0xff // all bits or'ed together

#define WIDTH		315
#define HEIGHT		207

void Pixy_UpdateBoard(UART_HandleTypeDef*, int*);
int Pixy_TranslateCoords(uint16_t, uint16_t);

#ifdef __cplusplus
}
#endif
#endif /* INC_PIXY_HEADER_H_ */
