#pragma once

#include <stdint.h>

#define TFT_CMD_REG ((volatile uint16_t*)0x60000000)
#define TFT_DATA_REG ((volatile uint16_t*)0x60000100)

void DevTFTInit();
