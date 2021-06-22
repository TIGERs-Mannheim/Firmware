/*
 * data_storage.c
 *
 *  Created on: 26.02.2016
 *      Author: AndreR
 */

#include "data_storage.h"
#include <string.h>

void DataStorageInit(DataStorage* pStore, uint8_t* pData, uint32_t coverTime, uint32_t sampleTime, uint32_t elementSize)
{
	pStore->curWriteIndex = 0;
	pStore->sampleTime = sampleTime;
	pStore->pData = pData;
	pStore->numElements = DATA_STORAGE_NUM_ELEMENTS(coverTime, sampleTime);
	pStore->pValidFlags = pData + pStore->numElements*elementSize;
	pStore->elementSize = elementSize;

	memset(pStore->pValidFlags, 0, pStore->numElements);
}

void DataStorageFill(DataStorage* pStore, void* pElement)
{
	for(uint32_t i = 0; i < pStore->numElements; i++)
	{
		uint8_t* pInsert = pStore->pData + i*pStore->elementSize;
		memcpy(pInsert, pElement, pStore->elementSize);
		pStore->pValidFlags[i] = 1;
	}
}

void DataStorageSetData(DataStorage* pStore, void* pElement, uint32_t age)
{
	uint32_t ageInSteps = (age+pStore->sampleTime/2)/pStore->sampleTime;

	if(ageInSteps >= pStore->numElements)
		return; // data is too old

	uint32_t insertionIndex = (pStore->curWriteIndex + (pStore->numElements-ageInSteps))%pStore->numElements;

	uint8_t* pInsert = pStore->pData + insertionIndex*pStore->elementSize;

	memcpy(pInsert, pElement, pStore->elementSize);

	pStore->pValidFlags[insertionIndex] = 1;
}

void DataStorageTick(DataStorage* pStore)
{
	++pStore->curWriteIndex;
	pStore->curWriteIndex %= pStore->numElements;

	pStore->pValidFlags[pStore->curWriteIndex] = 0;
}

int16_t DataStorageGet(DataStorage* pStore, uint32_t age, void* pElement)
{
	uint32_t ageInSteps = (age+pStore->sampleTime/2)/pStore->sampleTime;;

	if(ageInSteps >= pStore->numElements)
		return 1; // data requested is too old

	uint32_t requestIndex = (pStore->curWriteIndex + (pStore->numElements-ageInSteps))%pStore->numElements;

	if(pStore->pValidFlags[requestIndex] == 0)
		return 2; // no valid data for this time

	uint8_t* pReq = pStore->pData + requestIndex*pStore->elementSize;

	memcpy(pElement, pReq, pStore->elementSize);

	return 0;
}
