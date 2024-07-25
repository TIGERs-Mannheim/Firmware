#include "fw_loader_fatfs.h"
#include "util/crc.h"
#include "hal/flash.h"
#include "ff.h"
#include "errors.h"
#include <string.h>
#include <stdio.h>

static FRESULT getCrc(FwLoaderFatFs* pLoader, FIL* pFile, uint32_t startAddr, uint32_t endAddr, uint32_t* pCrc);

void FwLoaderFatFsInit(FwLoaderFatFs* pLoader, const char* pFilename)
{
	pLoader->pFilename = pFilename;

	pLoader->source.loadFunc = &FwLoaderFatFsLoad;
	pLoader->source.pUser = pLoader;
}

int16_t FwLoaderFatFsLoad(const FwUpdaterProgram* pProgram, FwUpdaterProgress* pProgress, void* pUser)
{
	FwLoaderFatFs* pLoader = (FwLoaderFatFs*)pUser;
	const char* pFilename = pLoader->pFilename;
	const uint32_t writeGranularity = pProgram->flashWriteGranularity;

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

	if(newProgramSize > pProgram->maxSize)
	{
		return ERROR_FW_LOADER_TOO_BIG;
	}

	printf("New program size: %u\r\n", newProgramSize);

	// get CRC of new program
	uint32_t newProgramCrc;

	fresult = getCrc(pLoader, &programFile, 0, newProgramSize, &newProgramCrc);
	if(fresult)
	{
		f_close(&programFile);
		return ERROR_FW_LOADER_NO_MEDIA;
	}

	// compute active program CRC
	uint32_t execProgramCrc = CRC32CalcChecksum((uint8_t*)pProgram->execAddr, newProgramSize);

	printf("Exec CRC: 0x%08X, new CRC: 0x%08X\r\n", execProgramCrc, newProgramCrc);

	if(execProgramCrc == newProgramCrc)
	{
		f_close(&programFile);
		return ERROR_FW_LOADER_UP_TO_DATE;
	}

	// check if partial data is already present
	uint8_t eraseNeeded = 0;
	uint32_t offset = 0;
	for(uint8_t* pRead = (uint8_t*)(pProgram->loadAddr + newProgramSize - sizeof(uint8_t)); pRead > (uint8_t*)pProgram->loadAddr; pRead--)
	{
		if(*pRead != 0xFF)
		{
			offset = (uint32_t)pRead - pProgram->loadAddr + sizeof(uint8_t);
			break;
		}
	}

	// check if we can continue from partial data
	if(offset%writeGranularity != 0)
	{
		// offset is not aligned to write granularity, need to erase, flash only supports aligned writes
		eraseNeeded = 1;

		printf("Unaligned partial data found (%uB)\r\n", offset);
	}
	else if(offset)
	{
		printf("%uB already found\r\n", offset);

		uint32_t partialCrc = CRC32CalcChecksum((uint8_t*)pProgram->loadAddr, offset);

		uint32_t partialNewCrc;
		fresult = getCrc(pLoader, &programFile, 0, offset, &partialNewCrc);
		if(fresult)
		{
			f_close(&programFile);
			return ERROR_FW_LOADER_NO_MEDIA;
		}

		if(partialCrc != partialNewCrc)
		{
			printf("Incorrect partial data found. Exists: 0x%08X <=> Required: 0x%08X\r\n", partialCrc, partialNewCrc);
			eraseNeeded = 1;
		}
		else
		{
			printf("Partial data found, continuing\r\n");
		}
	}

	while(1)
	{
		if(eraseNeeded)
		{
			// partial data was present but incorrect, need to restart
			printf("Erasing flash...");
			fflush(stdout);

			pProgress->pStepDescription = "Formatting";

			result = FlashErase(pProgram->loadAddr, pProgram->loadAddr + pProgram->maxSize);
			if(result)
				return result;

			offset = 0;

			printf("Done.\r\n");
		}

		printf("Loading data\r\n");
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
			printf("Writing at: %u (0x%08X)\r\n", offset, pProgram->loadAddr + offset);

			pProgress->doneBytes = offset;

			memset(pLoader->readBuf, 0xFF, FW_LOADER_FATFS_BUFFER_SIZE);

			uint32_t bytesToRead = FW_LOADER_FATFS_BUFFER_SIZE;
			if(bytesToRead > newProgramSize - offset)
				bytesToRead = newProgramSize - offset;

			UINT bytesRead;
			fresult = f_read(&programFile, pLoader->readBuf, bytesToRead, &bytesRead);
			if(fresult)
			{
				f_close(&programFile);
				return ERROR_FW_LOADER_NO_MEDIA;
			}

			uint32_t bytesToWrite = (bytesToRead + writeGranularity - 1)/writeGranularity*writeGranularity;

			result = FlashProgram(pProgram->loadAddr + offset, (uint32_t*)pLoader->readBuf, bytesToWrite/4);
			if(result)
				return result;
		}

		printf("Write complete.\r\n");

		uint32_t loadProgramCrc = CRC32CalcChecksum((uint8_t*)pProgram->loadAddr, newProgramSize);
		if(loadProgramCrc != newProgramCrc)
		{
			printf("Final CRC check failure\r\n");
			eraseNeeded = 1;
			continue;
		}

		// Program CRC
		printf("Writing CRC.\r\n");
		memset(pLoader->readBuf, 0xFF, FW_LOADER_FATFS_BUFFER_SIZE);

		uint32_t writeSize = ((6 + writeGranularity - 1) / writeGranularity) * writeGranularity;

		uint16_t *pCrcPresent = (uint16_t*) &pLoader->readBuf[writeSize - 6];
		uint32_t *pCrc = (uint32_t*) &pLoader->readBuf[writeSize - 4];

		*pCrc = loadProgramCrc;
		*pCrcPresent = 0;

		result = FlashProgram(pProgram->loadAddr + pProgram->maxSize - writeSize, (uint32_t*)pLoader->readBuf, writeSize/4);
		if(result)
			return result;

		pProgress->doneBytes = newProgramSize;

		break;
	}

	f_close(&programFile);

	return 0;
}

static FRESULT getCrc(FwLoaderFatFs* pLoader, FIL* pFile, uint32_t startAddr, uint32_t endAddr, uint32_t* pCrc)
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

		memset(pLoader->readBuf, 0xFF, FW_LOADER_FATFS_BUFFER_SIZE);

		fresult = f_read(pFile, pLoader->readBuf, readSize, &bytesRead);
		if(fresult)
			return fresult;

		crc = CRC32FeedChecksum(crc, pLoader->readBuf, readSize);

		addr += readSize;
	}

	*pCrc = CRC32StopChecksum(crc);

	return FR_OK;
}
