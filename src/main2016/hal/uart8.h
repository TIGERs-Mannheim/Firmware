/*
 * uart8.h
 *
 *  Created on: 24.10.2015
 *      Author: AndreR
 */

#ifndef UART8_H_
#define UART8_H_

#include "util/usart.h"
#include "../constants.h"

typedef struct _UART8Data
{
	uint8_t txData[UART8_TX_SIZE];
	uint8_t rxData[UART8_RX_SIZE];

	uint8_t extInstalled;
	USART uart;
} UART8Data;

extern UART8Data uart8;

void UART8Init();

#endif /* UART8_H_ */
