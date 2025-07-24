#pragma once

#include <stdint.h>

#define BITS_CLEAR(reg, bit) (reg &= ~(bit))
#define BITS_SET(reg, bit) (reg |= (bit))

void BitsPack(void* pDstV, uint32_t offset_bits, uint32_t width_bits, uint32_t value);
uint32_t BitsUnpack(const void* pSrcV, uint32_t offset_bits, uint32_t width_bits);
