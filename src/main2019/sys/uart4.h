#pragma once

#include "hal/uart_fifo.h"

extern UARTFifo uart4;

void UART4Init(uint32_t irqLevel);
