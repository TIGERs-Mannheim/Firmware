/*
 * usart1.h
 *
 *  Created on: 15.01.2019
 *      Author: AndreR
 */

#pragma once

#include <stdint.h>

void USART1Init();
void USART1Transmit(const uint8_t* pData, uint16_t length);
uint8_t* USART1Read(uint16_t* pLength);
void USART1SendFeedback();
void USART1ReceiveCommand();
