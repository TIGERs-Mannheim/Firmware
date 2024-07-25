/*
 * cobs.c
 *
 *  Created on: 23.09.2013
 *      Author: AndreR
 *
 * Taken from:  PPP Consistent Overhead Byte Stuffing (COBS)
 * http://tools.ietf.org/html/draft-ietf-pppext-cobs-00.txt
 *
 * - Removed PPP flag extraction (not required and modifies data).
 * - Changed function prototypes slightly
 */

#include "cobs.h"
#include "errors.h"

#include <string.h>

typedef enum
{
	Unused = 0x00, 		/* Unused (framing character placeholder) */
	DiffZero = 0x01, 	/* Range 0x01 - 0xD1:                     */
	DiffZeroMax = 0xD1, /* n-1 explicit characters plus a zero    */
	Diff = 0xD2, 		/* 209 explicit characters, no added zero */
	RunZero = 0xD3, 	/* Range 0xD3 - 0xDF:                     */
	RunZeroMax = 0xDF, 	/* 3-15 zeroes                            */
	Diff2Zero = 0xE0, 	/* Range 0xE0 - 0xFF:                     */
	Diff2ZeroMax = 0xFF,/* 0-31 explicit characters plus 2 zeroes */
} StuffingCode;

/* These macros examine just the top 3/4 bits of the code byte */
#define isDiff2Zero(X) (X >= Diff2Zero)
#define isRunZero(X)   (X >= RunZero && X <= RunZeroMax)

/* Convert from single-zero code to corresponding double-zero code */
#define ConvertZP (Diff2Zero - DiffZero) // = 0xDF = 223

/* Highest single-zero code with a corresponding double-zero code */
#define MaxConvertible (Diff2ZeroMax - ConvertZP) // = 0x20 = 32

int16_t COBSStartEncode(COBSState* pState, uint32_t sizeIn, uint8_t* pOut, uint32_t sizeOut)
{
	pState->code = DiffZero;
	pState->pOut = pOut;
	pState->pOutOrig = pOut;
	pState->pCode = pState->pOut++;

	if(sizeOut < COBSMaxStuffedSize(sizeIn))
		return ERROR_NOT_ENOUGH_MEMORY;

	return 0;
}

void COBSFeedEncode(COBSState* pState, const void* _pData, uint32_t dataSize)
{
	const uint8_t* pData = (const uint8_t*)_pData;

	for(uint32_t i = 0; i < dataSize; i++)
	{
		uint8_t c = pData[i];

		if (c == 0) // If it's a zero, do one of these operations
		{
			if (isRunZero(pState->code) && pState->code < RunZeroMax) // If in ZRE mode and below max zero count
			{
				pState->code++;	// increment ZRE count
			}
			else if (pState->code == Diff2Zero)	// If in two zeros and no character state
			{
				pState->code = RunZero;	// switch to ZRE state
			}
			else if (pState->code <= MaxConvertible) // If in diffZero state and below max char count for ZPE:
			{
				pState->code += ConvertZP; // switch to ZPE mode
			}
			else // cannot convert to ZPE (>31 chars) or above max ZRE (>15 '0's)
			{
				*pState->pCode = pState->code;	// save code to this' block code position

				pState->pCode = pState->pOut++; // store code position for new block
				pState->code = DiffZero;	// start new block by single encoded zero
			}
		}
		else // else, non-zero; do one of these operations
		{
			if(isDiff2Zero(pState->code))
			{
				*pState->pCode = pState->code - ConvertZP;
				pState->pCode = pState->pOut++;
				pState->code = DiffZero;
			}
			else if(pState->code == RunZero)
			{
				*pState->pCode = Diff2Zero;
				pState->pCode = pState->pOut++;
				pState->code = DiffZero;
			}
			else if(isRunZero(pState->code))
			{
				*pState->pCode = pState->code-1;
				pState->pCode = pState->pOut++;
				pState->code = DiffZero;
			}

			*pState->pOut++ = c;

			if(++pState->code == Diff)
			{
				*pState->pCode = pState->code;
				pState->pCode = pState->pOut++;
				pState->code = DiffZero;
			}
		}
	}
}

void COBSFinalizeEncode(COBSState* pState, uint32_t* pBytesWritten)
{
	*pState->pCode = pState->code;
	pState->pCode = pState->pOut++;

	if(pBytesWritten)
		*pBytesWritten = pState->pOut - pState->pOutOrig - 1;
}

int16_t COBSEncode(const uint8_t *pIn, uint32_t sizeIn, uint8_t *pOut, uint32_t sizeOut,
		uint32_t* pBytesWritten)
{
	COBSState state;
	int16_t result = COBSStartEncode(&state, sizeIn, pOut, sizeOut);
	if(result)
		return result;

	COBSFeedEncode(&state, pIn, sizeIn);
	COBSFinalizeEncode(&state, pBytesWritten);

	return 0;
}

int16_t COBSDecode(const uint8_t *pIn, uint32_t sizeIn,
		uint8_t *pOut, uint32_t sizeOut, uint32_t* pBytesWritten)
{
	const uint8_t* pOutOrig = pOut;
	const uint8_t *pEnd = pIn + sizeIn;
	const uint8_t *pLimit = pOut + sizeOut;

	if(sizeIn == 0)
		return ERROR_COBS_ZERO_SIZE;

	while (pIn < pEnd)
	{
		int32_t z, c = *pIn++; // c = code, z = zeros

		if (c == Diff)
		{
			z = 0;
			c--;
		}
		else if (isRunZero(c))
		{
			z = c & 0xF;
			c = 0;
		}
		else if (isDiff2Zero(c))
		{
			z = 2;
			c &= 0x1F;
		}
		else
		{
			z = 1;
			c--;
		}

		while (--c >= 0)
		{
			if(pOut < pLimit)
				*pOut = *pIn;
			++pOut;
			++pIn;
		}

		while (--z >= 0)
		{
			if(pOut < pLimit)
				*pOut = 0;
			++pOut;
		}
	}

	if(pBytesWritten)
		*pBytesWritten = pOut-pOutOrig-1;

	if (pOut >= pLimit)
		return ERROR_NOT_ENOUGH_MEMORY;

	return 0;
}
