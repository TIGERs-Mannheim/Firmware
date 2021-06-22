/*
 * usart2.h
 *
 *  Created on: 01.01.2019
 *      Author: AndreR
 */

#pragma once

#include "ch.h"
#include "../constants.h"

typedef struct _USART2Data
{
	input_queue_t inQueue;
	output_queue_t outQueue;

	uint8_t txData[USART2_TX_SIZE];
	uint8_t rxData[USART2_RX_SIZE];
} USART2Data;

extern USART2Data usart2;

void USART2Init();
