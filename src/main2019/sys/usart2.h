#pragma once

#include "hal/uart_dma.h"

extern UartDma usart2;

void USART2Init(uint32_t irqLevel, tprio_t taskPrio);
