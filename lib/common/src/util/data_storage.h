/*
 * data_storage.h
 *
 *  Created on: 26.02.2016
 *      Author: AndreR
 */

#ifndef DATA_STORAGE_H_
#define DATA_STORAGE_H_

#include <stdint.h>

#define DATA_STORAGE_NUM_ELEMENTS(ct, st) ((uint32_t)((ct+st/2)/st))
#define DATA_STORAGE_SIZE(et, ct, st) (DATA_STORAGE_NUM_ELEMENTS(ct, st)*(sizeof(et)+1))

typedef struct _DataStorage
{
	uint8_t* pData;
	uint8_t* pValidFlags;
	uint32_t elementSize;
	uint32_t numElements;
	uint32_t sampleTime;
	uint32_t curWriteIndex; // increased by tick
} DataStorage;

// All ages in microseconds
void DataStorageInit(DataStorage* pStore, uint8_t* pData, uint32_t coverTime, uint32_t sampleTime, uint32_t elementSize);
void DataStorageSetData(DataStorage* pStore, void* pElement, uint32_t age);
void DataStorageTick(DataStorage* pStore);
int16_t DataStorageGet(DataStorage* pStore, uint32_t age, void* pElement);
void DataStorageFill(DataStorage* pStore, void* pElement);

#endif /* DATA_STORAGE_H_ */
