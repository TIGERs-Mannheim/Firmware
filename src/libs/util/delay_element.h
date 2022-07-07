/*
 * delay_element.h
 *
 *  Created on: 09.04.2018
 *      Author: AndreR
 */

#pragma once

#include <stdint.h>

typedef struct _DelayElement
{
	uint32_t numElementsMax;
	uint32_t numElements;
	uint32_t elementSize;
	uint32_t nowPtr;
	uint8_t* pData;
} DelayElement;

void	DelayElementInit(DelayElement* pDelay, uint32_t numElementsInitial, uint32_t numElementsMax, uint32_t elementSize, void* pData);
void	DelayElementSetNumElements(DelayElement* pDelay, uint32_t numElements);
void*	DelayElementGet(DelayElement* pDelay, uint32_t offsetToFuture);
void	DelayElementSet(DelayElement* pDelay, uint32_t offsetToFuture, void* pData);
void	DelayElementTick(DelayElement* pDelay);
void*	DelayElementGetNow(DelayElement* pDelay);
void	DelayElementSetLast(DelayElement* pDelay, void* pData);
void*	DelayElementGetPrevious(DelayElement* pDelay);
