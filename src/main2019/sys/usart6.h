#pragma once

#include "hal/uart_dma.h"

extern UartDma usart6;

void USART6Init(uint32_t irqLevel, tprio_t taskPrio);
