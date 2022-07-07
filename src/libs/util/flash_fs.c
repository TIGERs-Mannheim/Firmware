/*
 * flash_fs.c
 *
 *  Created on: 25.06.2014
 *      Author: AndreR
 */

#include "flash_fs.h"
#include "util/flash.h"
#include "util/log.h"
#include "util/console.h"
#include <string.h>

FlashFS flashFS;

#define FLASH_FS_VERSION 0x3000

void FlashFSInit(uint32_t flashAddr, uint32_t flashSize)
{
	chMtxObjectInit(&flashFS.writeMutex);

	memset(flashFS.nul, 0x00, sizeof(flashFS.nul));
	flashFS.filesUsed = 0;
	flashFS.filesDel = 0;
	flashFS.flashAddr = flashAddr;
	flashFS.flashSize = flashSize;
	flashFS.pStart = (FlashFile*)flashAddr;
	flashFS.pEnd = (FlashFile*)(flashAddr+flashSize);

	for(FlashFile* pRead = flashFS.pStart; pRead < flashFS.pEnd; pRead++)
	{
		if(pRead->del == 0xFFFF)
		{
			if(pRead->dataSize != 0xFFFF)
				flashFS.filesUsed++;
		}
		else
		{
			flashFS.filesDel++;
		}
	}

	// open version file
	flashFS.version = FLASH_FS_VERSION;
	int16_t result = FlashFSOpenOrCreate("flash_fs/version", 0, &flashFS.version, sizeof(flashFS.version), &flashFS.pVersionFile);
	if(result == FLASH_FS_CREATED || flashFS.version != FLASH_FS_VERSION)
	{
		// different flash version => wipe it
		chMtxLock(&flashFS.writeMutex);
		FlashErase(flashFS.flashAddr, flashFS.flashAddr+flashFS.flashSize);
		chMtxUnlock(&flashFS.writeMutex);

		flashFS.filesDel = 0;
		flashFS.filesUsed = 0;

		flashFS.version = FLASH_FS_VERSION;
		FlashFSOpenOrCreate("flash_fs/version", 0, &flashFS.version, sizeof(flashFS.version), &flashFS.pVersionFile);
	}
}

FlashFile* FlashFSOpen(const char* pFilename, uint16_t version)
{
	for(FlashFile* pRead = flashFS.pStart; pRead < flashFS.pEnd; pRead++)
	{
		if(pRead->del == 0xFFFF && pRead->version == version && strncmp(pFilename, pRead->filename, FLASH_FS_MAX_FILENAME) == 0)
			return pRead;
	}

	return 0;
}

static void flashDelete(FlashFile* pFile)
{
	FlashProgram((uint32_t)pFile, flashFS.nul, FLASH_FS_GRANULARITY_IN_WORDS);
	flashFS.filesDel++;
	flashFS.filesUsed--;
}

void FlashFSDelete(const char* pFilename)
{
	chMtxLock(&flashFS.writeMutex);

	for(FlashFile* pRead = flashFS.pStart; pRead < flashFS.pEnd; pRead++)
	{
		if(pRead->del == 0xFFFF && pRead->dataSize != 0xFFFF && strncmp(pFilename, pRead->filename, FLASH_FS_MAX_FILENAME) == 0)
		{
			flashDelete(pRead);
		}
	}

	chMtxUnlock(&flashFS.writeMutex);
}

static FlashFile* flashCreate(const char* pFilename, uint16_t version, void* pData, uint16_t size)
{
	chMtxLock(&flashFS.writeMutex);

	// locate a free file slot
	FlashFile* pWrite;
	for(pWrite = flashFS.pStart; pWrite < flashFS.pEnd; pWrite++)
	{
		if(pWrite->del == 0xFFFF && pWrite->dataSize == 0xFFFF)
			break;
	}

	if(pWrite == flashFS.pEnd)
	{
		LogError("no free file slot found");
		chMtxUnlock(&flashFS.writeMutex);
		return 0;
	}

	// assemble data
	FlashFile* pNew = (FlashFile*)flashFS.temp;
	memset(pNew, 0x00, sizeof(FlashFile));
	pNew->dataSize = size;
	pNew->del = 0xFFFF;
	pNew->version = version;
	memcpy(pNew->filename, pFilename, strlen(pFilename));
	memcpy(pNew->data, pData, size);

	// burn in new data
	FlashProgram((uint32_t)pWrite, (uint32_t*)flashFS.temp, sizeof(FlashFile)/4);
	flashFS.filesUsed++;

	chMtxUnlock(&flashFS.writeMutex);

	return pWrite;
}

int16_t FlashFSOpenOrCreate(const char* pFilename, uint16_t version, void* pData, uint16_t size, FlashFile** ppFH)
{
	size_t filenameLength = strlen(pFilename);
	if(filenameLength > FLASH_FS_MAX_FILENAME)
	{
		return FLASH_FS_SIZE_MISMATCH;
	}

	// check if file exists
	FlashFile* pFH = FlashFSOpen(pFilename, version);
	if(pFH)
	{
		*ppFH = pFH;
		FlashFSRead(pFH, pData);
		return FLASH_FS_OPEN;
	}

	// in case the file exists and just the version changed, we need to delete it
	FlashFSDelete(pFilename);

	if(flashFS.filesUsed*sizeof(FlashFile) > FLASH_TEMP_SIZE)
		return FLASH_FS_NOMEM;

	*ppFH = flashCreate(pFilename, version, pData, size);

	if(flashFS.filesDel+flashFS.filesUsed >= flashFS.flashSize/sizeof(FlashFile))
	{
		// flash is completely used, clean it up
		FlashFSCompact();
	}

	return FLASH_FS_CREATED;
}

void FlashFSRead(FlashFile* pFH, void* pDst)
{
	memcpy(pDst, pFH->data, pFH->dataSize);
}

int16_t FlashFSWrite(FlashFile** ppFH, void* pData, uint16_t size)
{
	FlashFile* pFH = *ppFH;

	// check size
	if(pFH->dataSize != size)
		return FLASH_FS_SIZE_MISMATCH;

	// compare content
	if(memcmp(pFH->data, pData, size) == 0)
		return 0;

	*ppFH = flashCreate(pFH->filename, pFH->version, pData, size);

	chMtxLock(&flashFS.writeMutex);

	flashDelete(pFH);

	chMtxUnlock(&flashFS.writeMutex);

	if(flashFS.filesDel+flashFS.filesUsed >= flashFS.flashSize/sizeof(FlashFile))
	{
		// flash is completely used, clean it up
		FlashFSCompact();
	}

	return 0;
}

int16_t FlashFSCompact()
{
	if(flashFS.filesUsed*sizeof(FlashFile) > FLASH_TEMP_SIZE)
	{
		LogErrorC("bytesUsed > FLASH_TEMP_SIZE", flashFS.filesUsed*sizeof(FlashFile));
		return FLASH_FS_NOMEM;
	}

	chMtxLock(&flashFS.writeMutex);

	// backup valid flash files
	FlashFile* pTempWrite = (FlashFile*)flashFS.temp;
	uint16_t fileCounter = 0;
	for(FlashFile* pRead = (FlashFile*)flashFS.flashAddr; pRead < flashFS.pEnd; pRead++)
	{
		if(pRead->del == 0xFFFF && pRead->dataSize != 0xFFFF)
		{
			memcpy(pTempWrite, pRead, sizeof(FlashFile));
			flashFS.pTempFiles[fileCounter++] = pRead;
			pTempWrite++;
		}
	}

	if(fileCounter != flashFS.filesUsed)
	{
		LogWarnC("Flash FS inconsistent", fileCounter << 16 | flashFS.filesUsed);
	}

	// wipe out flash
	FlashErase(flashFS.flashAddr, flashFS.flashAddr+flashFS.flashSize);

	// restore saved data at original location
	for(uint16_t file = 0; file < fileCounter; file++)
	{
		FlashFile* pData = ((FlashFile*)flashFS.temp)+file;
		FlashProgram((uint32_t)flashFS.pTempFiles[file], (uint32_t*)pData, sizeof(FlashFile)/4);
	}

	flashFS.filesDel = 0;

	chMtxUnlock(&flashFS.writeMutex);

	return 0;
}

void FlashFSListFiles()
{
	ConsolePrint("--- Flash FS ---\r\n");
	ConsolePrint("Version: 0x%08X\r\n", flashFS.version);
	ConsolePrint("Used: %8u of %8u Files\r\n", flashFS.filesUsed, FLASH_TEMP_SIZE/sizeof(FlashFile));
	ConsolePrint("Stat: %8u of %8u Files\r\n", flashFS.filesUsed+flashFS.filesDel, flashFS.flashSize/sizeof(FlashFile));

	ConsolePrint("File                              Version     Size   OnDisk\r\n");

	chThdSleep(MS2ST(5));

	for(FlashFile* pRead = (FlashFile*)flashFS.flashAddr; pRead < flashFS.pEnd; pRead++)
	{
		if(pRead->del == 0xFFFF && pRead->dataSize != 0xFFFF)
			ConsolePrint("%-32s %8hu %8hu %8hu\r\n", pRead->filename, pRead->version, pRead->dataSize, sizeof(FlashFile));

		chThdSleep(MS2ST(2));
	}
}
