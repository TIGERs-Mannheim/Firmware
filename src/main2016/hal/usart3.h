/*
 * usart3.h
 *
 *  Created on: 24.10.2015
 *      Author: AndreR
 */

#ifndef __USART3_H__
#define __USART3_H__

#include "ch.h"
#include "../constants.h"

typedef struct _USART3Data
{
	input_queue_t inQueue;
	output_queue_t outQueue;

	uint8_t txData[USART3_TX_SIZE];
	uint8_t rxData[USART3_RX_SIZE];
} USART3Data;

extern USART3Data usart3;

void USART3Init();

#endif
