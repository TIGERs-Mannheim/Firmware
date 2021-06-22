/*
 * fw_loader_fatfs.c
 *
 *  Created on: 09.04.2020
 *      Author: AndreR
 */

#include "fw_loader_fatfs.h"
#include "util/crc.h"
#include "util/console.h"
#include "util/flash.h"
#include "fatfs/ff.h"
#include "errors.h"
#include <string.h>

FwLoaderFatFsGlobal fwLoaderFatFs;

static int16_t doLoad(const FwUpdaterProgram* pProgram, FwUpdaterProgress* pProgress, const char* pFilename);
static FRESULT getCrc(FIL* pFile, uint32_t startAddr, uint32_t endAddr, uint32_t* pCrc);

int16_t FwLoaderFatFsUSB(const FwUpdaterProgram* pProgram, FwUpdaterProgress* pProgress)
{
	// determine filename
	const char* pFilename;
	switch(pProgram->procId)
	{
		case FW_ID_MAIN2016: pFilename = "0:/main2016.bin"; break;
		case FW_ID_MAIN2019: pFilename = "0:/main2019.bin"; break;
		case FW_ID_MOT2019: pFilename = "0:/motor2019.bin"; break;
		case FW_ID_IR2019: pFilename = "0:/ir2019.bin"; break;
		default: return ERROR_FW_LOADER_NO_MEDIA;
	}

	return doLoad(pProgram, pProgress, pFilename);
}

int16_t FwLoaderFatFsSDCard(const FwUpdaterProgram* pProgram, FwUpdaterProgress* pProgress)
{
	// determine filename
	const char* pFilename;
	switch(pProgram->procId)
	{
		case FW_ID_MAIN2016: pFilename = "1:/main2016.bin"; break;
		case FW_ID_MAIN2019: pFilename = "1:/main2019.bin"; break;
		case FW_ID_MOT2019: pFilename = "1:/motor2019.bin"; break;
		case FW_ID_IR2019: pFilename = "1:/ir2019.bin"; break;
		default: return ERROR_FW_LOADER_NO_MEDIA;
	}

	return doLoad(pProgram, pProgress, pFilename);
}

static int16_t doLoad(const FwUpdaterProgram* pProgram, FwUpdaterProgress* pProgress, const char* pFilename)
{
	int16_t result;

	pProgress->totalBytes = 0;
	pProgress->doneBytes = 0;
	pProgress->pStepDescription = "Checking for update";

	// try to stat file to get program size
	FRESULT fresult;
	FILINFO info;
	memset(&info, 0, sizeof(FILINFO));

	fresult = f_stat(pFilename, &info);
	if(fresult)
		return ERROR_FW_LOADER_NO_MEDIA;

	uint32_t newProgramSize = info.fsize;

	FIL programFile;

	f_open(&programFile, (TCHAR*)pFilename, FA_READ | FA_OPEN_EXISTING);

	// align program size to 32 bytes ("flash word" size)
	newProgramSize = (newProgramSize+31)/32*32;

	if(newProgramSize > pProgram->maxSize)
	{
		return ERROR_FW_LOADER_TOO_BIG;
	}

	ConsolePrint("New program size: %u\r\n", newProgramSize);

	// get CRC of new program
	uint32_t newProgramCrc;

	fresult = getCrc(&programFile, 0, newProgramSize, &newProgramCrc);
	if(fresult)
	{
		f_close(&programFile);
		return ERROR_FW_LOADER_NO_MEDIA;
	}

	// compute active program CRC
	uint32_t execProgramCrc = CRC32CalcChecksum((uint8_t*)pProgram->execAddr, newProgramSize);

	ConsolePrint("Exec CRC: 0x%08X, new CRC: 0x%08X\r\n", execProgramCrc, newProgramCrc);

	if(execProgramCrc == newProgramCrc)
	{
		f_close(&programFile);
		return ERROR_FW_LOADER_UP_TO_DATE;
	}

	// check if partial data is already present
	uint8_t eraseNeeded = 0;
	uint32_t offset = 0;
	for(uint32_t* pRead = (uint32_t*)(pProgram->loadAddr + newProgramSize - sizeof(uint32_t)); pRead > (uint32_t*)pProgram->loadAddr; pRead--)
	{
		if(*pRead != 0xFFFFFFFF)
		{
			offset = (uint32_t)pRead - pProgram->loadAddr + sizeof(uint32_t);
			break;
		}
	}

	// check if we can continue from partial data
	if(offset%32 != 0)
	{
		// offset is not aligned to 32B, need to erase, flash only supports 32B aligned writes
		eraseNeeded = 1;

		ConsolePrint("Unaligned partial data found\r\n");
	}
	else if(offset)
	{
		uint32_t partialCrc = CRC32CalcChecksum((uint8_t*)pProgram->loadAddr, offset);

		uint32_t partialNewCrc;
		fresult = getCrc(&programFile, 0, offset, &partialNewCrc);
		if(fresult)
		{
			f_close(&programFile);
			return ERROR_FW_LOADER_NO_MEDIA;
		}

		if(partialCrc != partialNewCrc)
		{
			ConsolePrint("Incorrect partial data found\r\n");
			eraseNeeded = 1;
		}
		else
		{
			ConsolePrint("Partial data found, continuing\r\n");
		}
	}

	while(1)
	{
		if(eraseNeeded)
		{
			// partial data was present but incorrect, need to restart
			pProgress->pStepDescription = "Formatting";

			result = FlashErase(pProgram->loadAddr, pProgram->loadAddr + newProgramSize);
			if(result)
				return result;

			offset = 0;
		}

		pProgress->pStepDescription = "Loading data";
		pProgress->totalBytes = newProgramSize;
		pProgress->doneBytes = 0;

		// Load actual data
		fresult = f_lseek(&programFile, offset);
		if(fresult)
		{
			f_close(&programFile);
			return ERROR_FW_LOADER_NO_MEDIA;
		}

		for(; offset < newProgramSize; offset += FW_LOADER_FATFS_BUFFER_SIZE)
		{
			pProgress->doneBytes = offset;

			memset(fwLoaderFatFs.readBuf, 0xFF, FW_LOADER_FATFS_BUFFER_SIZE);

			uint32_t bytesToRead = FW_LOADER_FATFS_BUFFER_SIZE;
			if(bytesToRead > newProgramSize - offset)
				bytesToRead = newProgramSize - offset;

			UINT bytesRead;
			fresult = f_read(&programFile, fwLoaderFatFs.readBuf, bytesToRead, &bytesRead);
			if(fresult)
			{
				f_close(&programFile);
				return ERROR_FW_LOADER_NO_MEDIA;
			}

			result = FlashProgram(pProgram->loadAddr + offset, (uint32_t*)fwLoaderFatFs.readBuf, bytesToRead/4);
			if(result)
				return result;
		}

		uint32_t loadProgramCrc = CRC32CalcChecksum((uint8_t*)pProgram->loadAddr, newProgramSize);
		if(loadProgramCrc != newProgramCrc)
		{
			ConsolePrint("Final CRC check failure for proc %hu\r\n", (uint16_t)pProgram->procId);
			eraseNeeded = 1;
			continue;
		}

		pProgress->doneBytes = newProgramSize;

		break;
	}

	f_close(&programFile);

	return 0;
}

static FRESULT getCrc(FIL* pFile, uint32_t startAddr, uint32_t endAddr, uint32_t* pCrc)
{
	UINT bytesRead;
	FRESULT fresult;

	fresult = f_lseek(pFile, startAddr);
	if(fresult)
		return fresult;

	uint32_t addr = 0;
	const uint32_t size = endAddr - startAddr;

	uint32_t crc = CRC32StartChecksum();

	while(addr < size)
	{
		uint32_t readSize = FW_LOADER_FATFS_BUFFER_SIZE;

		if(addr + FW_LOADER_FATFS_BUFFER_SIZE > size)
			readSize = size-addr;

		memset(fwLoaderFatFs.readBuf, 0xFF, FW_LOADER_FATFS_BUFFER_SIZE);

		fresult = f_read(pFile, fwLoaderFatFs.readBuf, readSize, &bytesRead);
		if(fresult)
			return fresult;

		crc = CRC32FeedChecksum(crc, fwLoaderFatFs.readBuf, readSize);

		addr += readSize;
	}

	*pCrc = CRC32StopChecksum(crc);

	return FR_OK;
}
