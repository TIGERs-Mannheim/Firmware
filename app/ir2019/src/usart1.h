/*
 * usart1.h
 *
 *  Created on: 15.01.2019
 *      Author: AndreR
 */

#pragma once

#include <stdint.h>

void USART1Init();
void USART1Transmit(const void* pData, uint16_t length);
