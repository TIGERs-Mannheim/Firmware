/*
 * delay_element.c
 *
 *  Created on: 09.04.2018
 *      Author: AndreR
 */

#include "delay_element.h"

#include <string.h>

void DelayElementInit(DelayElement* pDelay, uint32_t numElementsInitial, uint32_t numElementsMax, uint32_t elementSize, void* pData)
{
	pDelay->numElements = numElementsInitial;
	pDelay->numElementsMax = numElementsMax;
	pDelay->elementSize = elementSize;
	pDelay->nowPtr = 0;
	pDelay->pData = (uint8_t*)pData;

	memset(pData, 0, numElementsMax*elementSize);
}

void DelayElementSetNumElements(DelayElement* pDelay, uint32_t numElements)
{
	if(numElements <= pDelay->numElementsMax)
		pDelay->numElements = numElements;
	else
		pDelay->numElements = pDelay->numElementsMax;

	pDelay->nowPtr %= pDelay->numElements;
}

void* DelayElementGet(DelayElement* pDelay, uint32_t offsetToFuture)
{
	uint32_t index = (pDelay->nowPtr + offsetToFuture + 1) % pDelay->numElements;
	return pDelay->pData + index*pDelay->elementSize;
}

void DelayElementSet(DelayElement* pDelay, uint32_t offsetToFuture, void* pData)
{
	uint32_t index = (pDelay->nowPtr + offsetToFuture + 1) % pDelay->numElements;
	memcpy(pDelay->pData + index*pDelay->elementSize, pData, pDelay->elementSize);
}

void DelayElementTick(DelayElement* pDelay)
{
	++pDelay->nowPtr;
	pDelay->nowPtr %= pDelay->numElements;
}

void* DelayElementGetNow(DelayElement* pDelay)
{
	return DelayElementGet(pDelay, 1);
}

void DelayElementSetLast(DelayElement* pDelay, void* pData)
{
	DelayElementSet(pDelay, 0, pData);
}

void* DelayElementGetPrevious(DelayElement* pDelay)
{
	return DelayElementGet(pDelay, pDelay->numElements-1);
}
