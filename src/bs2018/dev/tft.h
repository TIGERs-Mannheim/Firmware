#pragma once

#include "ch.h"

#define TFT_CMD_REG ((volatile uint16_t*)0x60000000)
#define TFT_DATA_REG ((volatile uint16_t*)0x60020000)

void TFTInit();
void TFTReset(uint8_t reset);
void TFTSetBrightness(uint8_t level);
