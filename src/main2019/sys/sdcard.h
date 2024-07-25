/*
 * sdcard.h
 *
 *  Created on: 09.01.2019
 *      Author: AndreR
 */

#pragma once

#include "ch.h"
#include "ff.h"

typedef struct _SDCID
{
	uint8_t MID;
	char OID[3];
	char PNM[6];
	char PRV[4];
	uint32_t PSN;
	uint16_t MDTYear;
	uint8_t MDTMonth;
} SDCID;

typedef struct _SDCSD
{
	uint8_t highCap;
	uint8_t dsrImp;
	uint32_t capacity;	// in kB
} SDCSD;

typedef struct _SDCard
{
	uint8_t present;
	uint8_t writeProtected;
	uint8_t ready;

	SDCID cid;
	SDCSD csd;

	uint32_t RCA;

	uint8_t speedClass;
	uint8_t uhsClass;

	binary_semaphore_t irqSem;

	FATFS fatFs;

	mutex_t transferMtx;
} SDCard;

extern SDCard sdCard;

void SDCardInit();
void SDCardTask(void* params);
int16_t SDCardRead(uint32_t blockAddr, uint8_t* pBuffer, uint32_t numBlocks);
int16_t SDCardWrite(uint32_t blockAddr, const uint8_t* pBuffer, uint32_t numBlocks);
int16_t SDCardGetStatus(uint8_t* pReady, uint8_t* pState);
void SDCardWaitUntilReady();
