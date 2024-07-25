/*
 * cobs.h
 *
 *  Created on: 23.09.2013
 *      Author: AndreR
 *
 *      Consistent Overhead Byte Stuffing with Zero Pair and Zero Run Elimination.
 */

#pragma once

#include <stdint.h>

typedef struct _COBSState
{
	uint8_t code;
	uint8_t* pOut;
	const uint8_t* pOutOrig;
	uint8_t* pCode;
} COBSState;

/**
 * Calculate maximum size of stuffed data in worst-case.
 */
#define COBSMaxStuffedSize(size) (size+size/208+1)

/**
 * Stuff bytes and remove zeros.
 *
 * @param pIn Data to be stuffed.
 * @param sizeIn Input data size.
 * @param pOut Stuffed output data.
 * @param sizeOut Maximum output size.
 * @param pBytesWritten Actual size of stuffed data, pass 0 if not needed.
 */
int16_t COBSEncode(const uint8_t *pIn, uint32_t sizeIn, uint8_t *pOut, uint32_t sizeOut,
		uint32_t* pBytesWritten);

/**
 * Unstuff COBS data.
 *
 * @param pIn Stuffed data.
 * @param sizeIn Stuffed data size.
 * @param pOut Unstuffed data.
 * @param sizeOut Maximum output size.
 * @param pBytesWritten Actual unstuffed data size.
 */
int16_t COBSDecode(const uint8_t *pIn, uint32_t sizeIn,
		uint8_t *pOut, uint32_t sizeOut, uint32_t* pBytesWritten);

int16_t	COBSStartEncode(COBSState* pState, uint32_t sizeIn, uint8_t* pOut, uint32_t sizeOut);
void	COBSFeedEncode(COBSState* pState, const void* _pData, uint32_t dataSize);
void	COBSFinalizeEncode(COBSState* pState, uint32_t* pBytesWritten);
