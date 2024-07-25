#pragma once

#include "hal/uart_fifo.h"

extern UARTFifo usart1;

void USART1Init(uint32_t irqLevel);
