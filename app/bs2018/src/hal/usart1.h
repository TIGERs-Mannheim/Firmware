/*
 * usart3.h
 *
 *  Created on: 23.10.2017
 *      Author: AndreR
 */

#pragma once

#include "ch.h"
#include "../constants.h"

typedef struct _USART1Data
{
	input_queue_t inQueue;
	output_queue_t outQueue;

	uint8_t txData[USART1_TX_SIZE];
	uint8_t rxData[USART1_RX_SIZE];
} USART1Data;

extern USART1Data usart1;

void USART1Init();
