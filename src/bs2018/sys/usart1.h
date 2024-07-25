#pragma once

#include "hal/uart_dma.h"

extern UartDma usart1;

void USART1Init(uint32_t irqLevel, tprio_t taskPrio);
