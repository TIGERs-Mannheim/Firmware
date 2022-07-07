/*
 * flash_fs.h
 *
 *  Created on: 25.06.2014
 *      Author: AndreR
 */

#pragma once

#include <stdint.h>
#include "ch.h"

#define FLASH_TEMP_SIZE 4096

#define FLASH_FS_OPEN 0
#define FLASH_FS_CREATED 1
#define FLASH_FS_NOMEM 2
#define FLASH_FS_SIZE_MISMATCH 3

#define FLASH_FS_MAX_FILENAME 58
#define FLASH_FS_MAX_DATASIZE 64

#ifdef STM32H7XX
#define FLASH_FS_GRANULARITY_IN_WORDS 8
#else
#define FLASH_FS_GRANULARITY_IN_WORDS 1
#endif

typedef struct _FlashFile
{
	uint16_t del;
	uint16_t dataSize;
	uint16_t version;
	char filename[FLASH_FS_MAX_FILENAME];
	uint8_t data[FLASH_FS_MAX_DATASIZE];
} __attribute__((aligned(4),packed)) FlashFile;

typedef struct _FlashFS
{
	uint32_t flashAddr;
	uint32_t flashSize;

	uint32_t filesUsed;
	uint32_t filesDel;

	FlashFile* pStart;
	FlashFile* pEnd;

	uint8_t temp[FLASH_TEMP_SIZE];
	FlashFile* pTempFiles[FLASH_TEMP_SIZE/sizeof(FlashFile)];

	mutex_t writeMutex;

	uint32_t version;
	FlashFile* pVersionFile;

	uint32_t nul[FLASH_FS_GRANULARITY_IN_WORDS];
} FlashFS;

extern FlashFS flashFS;

void	FlashFSInit(uint32_t flashAddr, uint32_t flashSize);
FlashFile* FlashFSOpen(const char* pFilename, uint16_t version);

/**
 * Open an existing file on the FlashFS or create a new one.
 *
 * If the file exists, its content is copied to pData.
 * If the file does not exist, it is filled with pData.
 */
int16_t	FlashFSOpenOrCreate(const char* pFilename, uint16_t version, void* pData, uint16_t size, FlashFile** ppFH);
void	FlashFSRead(FlashFile* pFH, void* pDst);

/**
 * Write pData to the FlashFS.
 *
 * @note No data is written if the file content equals pData.
 */
int16_t	FlashFSWrite(FlashFile** ppFH, void* pData, uint16_t size);
void	FlashFSDelete(const char* pFilename);
int16_t	FlashFSCompact();
void 	FlashFSListFiles();
