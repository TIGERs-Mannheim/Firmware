#pragma once

#include <stdint.h>

int16_t FlashErase(uint32_t addr, uint32_t endAddr);
int16_t FlashProgram(uint32_t addr, const uint32_t* pData, uint32_t length); // TODO: length in words...
