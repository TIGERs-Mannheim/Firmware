#pragma once

#include "hal/uart_fifo.h"

extern UARTFifo uart5;

void UART5Init(uint32_t irqLevel);
