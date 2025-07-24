#include "bits.h"

void BitsPack(void* pDstV, uint32_t offset_bits, uint32_t width_bits, uint32_t value)
{
	uint8_t* pDst = pDstV;
	uint32_t srcBitIndex = 0;

	for(uint32_t dstBitIndex = offset_bits; dstBitIndex < (offset_bits + width_bits); dstBitIndex++)
	{
		uint32_t dstByte = dstBitIndex / 8;
		uint32_t dstBit = dstBitIndex % 8;

		pDst[dstByte] &= ~(1 << dstBit);
		pDst[dstByte] |= ((value >> srcBitIndex) & 0x01) << dstBit;

		srcBitIndex++;
	}
}

uint32_t BitsUnpack(const void* pSrcV, uint32_t offset_bits, uint32_t width_bits)
{
	const uint8_t* pSrc = pSrcV;
	uint32_t result = 0;
	uint32_t dstBitIndex = 0;

	for(uint32_t srcBitIndex = offset_bits; srcBitIndex < (offset_bits + width_bits); srcBitIndex++)
	{
		uint32_t srcByte = srcBitIndex / 8;
		uint32_t srcBit = srcBitIndex % 8;

		result |= ((pSrc[srcByte] >> srcBit) & 0x01) << dstBitIndex;
		dstBitIndex++;
	}

	return result;
}
