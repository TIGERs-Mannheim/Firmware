#pragma once

#include "hal/uart_fifo.h"

extern UARTFifo uart7;

void UART7Init(uint32_t irqLevel);
