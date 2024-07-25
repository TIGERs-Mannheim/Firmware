#pragma once

#include "hal/uart_fifo.h"

extern UARTFifo uart8;

void UART8Init(uint32_t irqLevel);
