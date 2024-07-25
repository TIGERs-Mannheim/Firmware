#pragma once

#include "hal/spi_lld.h"

extern SPILLD spi1;

void SPI1Init(uint32_t irqLevel);
