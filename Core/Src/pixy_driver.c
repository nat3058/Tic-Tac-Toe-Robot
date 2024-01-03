/*
 * pixy_driver.c
 *
 *  Created on: Dec 3, 2023
 *      Author: lukes
 */

#include <pixy_driver.h>
#include <string.h>
#include <stdio.h>
// request bytes to obtain the data we need
uint8_t getBlocks[] = {
		0xae,	// 16-bit sync
		0xc1,	// 16-bit syn
		0x20,	// Type of packet
		0x02,	// Length of payload
		0x01,	// Sigmap - indicate which signatures to receive data from
		0xff	// Maximum blocks to return - 0 (none) - 255 (all blocks)
};


uint8_t getResolution[] = {
	0xae,
	0xc1,
	0x0c,	// 12
	0x02,	// Length of payload
	0xff	// unused
};

// be sure to enable FIFO in IOC UART settings!
void Pixy_UpdateBoard(UART_HandleTypeDef* phuart, int* board) {
	uint8_t recvBuf[200];

	HAL_UART_Transmit(phuart, getBlocks, 6, HAL_MAX_DELAY); //blocking code
	HAL_UART_Receive(phuart, recvBuf, 150, 1000);

	int numOfBlocks = recvBuf[3] / 14;

	for (int b = 0; b < numOfBlocks; ++b) {
		int x_idx = 8 + (14 * b); // find x idx of the block in our buffer 
		int y_idx = x_idx + 2;

		uint16_t x = (((uint16_t)(recvBuf[x_idx + 1]) << 8) + recvBuf[x_idx]); //calculate the x_idx
		uint16_t y = (((uint16_t)(recvBuf[y_idx + 1]) << 8) + recvBuf[y_idx]);
		printf("block %d coords: (%d, %d)\n\r", b+1, x, y);
		int idx = Pixy_TranslateCoords(x, y);
		if (idx >= 0 && idx < 9 && board[idx] == 0) {
			board[idx] = 1; //mark this human square selected square as taken
			return;
		}
	}
}

int Pixy_TranslateCoords(uint16_t x, uint16_t y) {
	int x_ttt = (x - 111)/50;
	int y_ttt = (y - 69)/40;
	return y_ttt*3 + x_ttt;
}
