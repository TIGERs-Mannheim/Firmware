#pragma once

#include "hal/uart_fifo.h"

extern UARTFifo usart3;

void USART3Init(uint32_t irqLevel);
