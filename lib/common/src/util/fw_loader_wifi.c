/*
 * fw_loader_wifi.c
 *
 *  Created on: 08.04.2020
 *      Author: AndreR
 */

#include "fw_loader_wifi.h"
#include "main/network.h"
#include "util/flash.h"
#include "util/console.h"
#include "util/crc.h"
#include "util/log.h"
#include "errors.h"

#define WIFI_TIMEOUT_MS 200
#define WIFI_RETRIES 5

#define REQ_STATE_EMPTY		0
#define REQ_STATE_REQUESTED	1
#define REQ_STATE_COMPLETE	2

FwLoaderWifiGlobal fwLoaderWifi = {
	.semResponseCrc = _BSEMAPHORE_DATA(fwLoaderWifi.semResponseCrc, 1),
	.semResponseSize = _BSEMAPHORE_DATA(fwLoaderWifi.semResponseSize, 1),
	.sliceMutex = _MUTEX_DATA(fwLoaderWifi.sliceMutex),
	.newDataCondition = _CONDVAR_DATA(fwLoaderWifi.newDataCondition),
};

static int16_t requestDataSlice(uint8_t procId, uint32_t offset);
static int16_t requestSize(uint8_t procId, uint32_t* pSize);
static int16_t requestCrc(uint8_t procId, uint32_t startAddr, uint32_t endAddr, uint32_t* pCrc);

int16_t FwLoaderWifi(const FwUpdaterProgram* pProgram, FwUpdaterProgress* pProgress)
{
	int16_t result;

	for(size_t i = 0; i < FW_LOADER_WIFI_NUM_CHUNKS; i++)
	{
		FwLoaderWifiRequest* pRequest = &fwLoaderWifi.requests[i];

		pRequest->state = REQ_STATE_EMPTY;
		pRequest->pData = &fwLoaderWifi.sliceData[i*FW_LOADER_WIFI_CHUNK_SIZE];
	}

	pProgress->totalBytes = 0;
	pProgress->doneBytes = 0;
	pProgress->pStepDescription = "Checking for update";

	// Try to contact source and get program size
	uint32_t newProgramSize;
	result = requestSize(pProgram->procId, &newProgramSize);
	if(result)
		return result;

	if(newProgramSize == 0xFFFFFFFF)
	{
		return ERROR_FW_LOADER_NO_MEDIA;
	}

	ConsolePrint("New program size: %u\r\n", newProgramSize);

	// get CRC of new program
	uint32_t newProgramCrc;
	result = requestCrc(pProgram->procId, 0, newProgramSize, &newProgramCrc);
	if(result)
		return result;

	// compute active program CRC
	uint32_t execProgramCrc = CRC32CalcChecksum((uint8_t*)pProgram->execAddr, newProgramSize);

	ConsolePrint("Exec CRC: 0x%08X, new CRC: 0x%08X\r\n", execProgramCrc, newProgramCrc);

	if(execProgramCrc == newProgramCrc)
	{
		// do a dummy request to inform Sumatra that new data is "read"
		PacketHeader header;
		BootloaderRequestData req;

		header.section = SECTION_BOOTLOADER;
		header.cmd = CMD_BOOTLOADER_REQUEST_DATA;

		req.procId = pProgram->procId;
		req.size = 4;
		req.offset = newProgramSize - 4;

		NetworkSendPacket(&header, &req, sizeof(BootloaderRequestData));

		return ERROR_FW_LOADER_UP_TO_DATE;
	}

	if(newProgramSize > pProgram->maxSize)
	{
		return ERROR_FW_LOADER_TOO_BIG;
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
		result = requestCrc(pProgram->procId, 0, offset, &partialNewCrc);
		if(result)
			return result;

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
		for(; offset < newProgramSize; offset += FW_LOADER_WIFI_SLICE_SIZE)
		{
			result = requestDataSlice(pProgram->procId, offset);
			if(result)
				return result;

			result = FlashProgram(pProgram->loadAddr + offset, (uint32_t*)fwLoaderWifi.sliceData, FW_LOADER_WIFI_SLICE_SIZE/4);
			if(result)
				return result;

			pProgress->doneBytes = offset;
		}

		// check result and restart loading data if CRC doesn't match
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

	return 0;
}

void FwLoaderWifiNetworkInput(PacketHeader* pHeader, uint16_t dataLength, uint8_t* pData)
{
	if(pHeader->section != SECTION_BOOTLOADER)
		return;

	switch(pHeader->cmd)
	{
		case CMD_BOOTLOADER_SIZE:
		{
			memcpy(&fwLoaderWifi.responseSize, pData, sizeof(BootloaderSize));

			chBSemSignal(&fwLoaderWifi.semResponseSize);
		}
		break;
		case CMD_BOOTLOADER_CRC32:
		{
			memcpy(&fwLoaderWifi.responseCrc, pData, sizeof(BootloaderCRC32));

			chBSemSignal(&fwLoaderWifi.semResponseCrc);
		}
		break;
		case CMD_BOOTLOADER_DATA:
		{
			BootloaderData* pBData = (BootloaderData*)pData;
			pData += sizeof(BootloaderData);
			dataLength -= sizeof(BootloaderData);

			chMtxLock(&fwLoaderWifi.sliceMutex);

			for(size_t i = 0; i < FW_LOADER_WIFI_NUM_CHUNKS; i++)
			{
				FwLoaderWifiRequest* pRequest = &fwLoaderWifi.requests[i];

				 if(pRequest->state != REQ_STATE_REQUESTED || pRequest->procId != pBData->procId || pRequest->offset != pBData->offset)
					 continue;

				if(dataLength > FW_LOADER_WIFI_CHUNK_SIZE)
					dataLength = FW_LOADER_WIFI_CHUNK_SIZE;

				memcpy(pRequest->pData, pData, dataLength);

				pRequest->state = REQ_STATE_COMPLETE;

				chCondSignal(&fwLoaderWifi.newDataCondition);

				break;
			}

			chMtxUnlock(&fwLoaderWifi.sliceMutex);
		}
		break;
	}
}

static int16_t requestDataSlice(uint8_t procId, uint32_t offset)
{
	PacketHeader header;
	BootloaderRequestData req;

	header.section = SECTION_BOOTLOADER;
	header.cmd = CMD_BOOTLOADER_REQUEST_DATA;

	req.procId = procId;
	req.size = FW_LOADER_WIFI_CHUNK_SIZE;

	chMtxLock(&fwLoaderWifi.sliceMutex);

	memset(fwLoaderWifi.sliceData, 0xFF, sizeof(fwLoaderWifi.sliceData));

	for(size_t i = 0; i < FW_LOADER_WIFI_NUM_CHUNKS; i++)
	{
		fwLoaderWifi.requests[i].state = REQ_STATE_EMPTY;
	}

	while(1)
	{
		size_t numCompleteChunks = 0;

		for(size_t i = 0; i < FW_LOADER_WIFI_NUM_CHUNKS; i++)
		{
			FwLoaderWifiRequest* pRequest = &fwLoaderWifi.requests[i];

			switch(pRequest->state)
			{
				case REQ_STATE_EMPTY:
				{
					pRequest->offset = offset + i*FW_LOADER_WIFI_CHUNK_SIZE;
					pRequest->procId = procId;
					pRequest->requestTime = chVTGetSystemTimeX();
					pRequest->retries = 0;
					pRequest->state = REQ_STATE_REQUESTED;

					req.offset = pRequest->offset;

					NetworkSendPacket(&header, &req, sizeof(BootloaderRequestData));
				}
				break;
				case REQ_STATE_REQUESTED:
				{
					if(chVTTimeElapsedSinceX(pRequest->requestTime) > MS2ST(WIFI_TIMEOUT_MS))
					{
						if(pRequest->retries >= WIFI_RETRIES)
						{
							chMtxUnlock(&fwLoaderWifi.sliceMutex);
							return ERROR_FW_LOADER_TIMEOUT;
						}

						// request timed out => re-send
						pRequest->requestTime = chVTGetSystemTimeX();
						pRequest->retries++;

						req.offset = pRequest->offset;
						NetworkSendPacket(&header, &req, sizeof(BootloaderRequestData));

						LogWarnC("Chunk timeout", pRequest->offset);
					}
				}
				break;
				case REQ_STATE_COMPLETE:
				{
					numCompleteChunks++;
				}
				break;
				default:
					break;
			}
		}

		if(numCompleteChunks >= FW_LOADER_WIFI_NUM_CHUNKS)
			break;

		if(chCondWaitTimeout(&fwLoaderWifi.newDataCondition, MS2ST(10)) == MSG_TIMEOUT)
			chMtxLock(&fwLoaderWifi.sliceMutex);
	}

	chMtxUnlock(&fwLoaderWifi.sliceMutex);

	return 0;
}

static int16_t requestSize(uint8_t procId, uint32_t* pSize)
{
	uint16_t attempts = WIFI_RETRIES;
	PacketHeader header;
	BootloaderRequestSize req;

	header.section = SECTION_BOOTLOADER;
	header.cmd = CMD_BOOTLOADER_REQUEST_SIZE;

	req.procId = procId;

	while(attempts--)
	{
		chBSemReset(&fwLoaderWifi.semResponseSize, 1);

		NetworkSendPacket(&header, &req, sizeof(BootloaderRequestSize));

		if(chBSemWaitTimeout(&fwLoaderWifi.semResponseSize, MS2ST(WIFI_TIMEOUT_MS)) != MSG_OK)
			continue;

		if(fwLoaderWifi.responseSize.procId != procId)
		{
			chThdSleepMilliseconds(WIFI_TIMEOUT_MS);
			continue;

		}

		*pSize = fwLoaderWifi.responseSize.size;

		return 0;
	}

	return ERROR_FW_LOADER_TIMEOUT;
}

static int16_t requestCrc(uint8_t procId, uint32_t startAddr, uint32_t endAddr, uint32_t* pCrc)
{
	uint16_t attempts = WIFI_RETRIES;
	PacketHeader header;
	BootloaderRequestCRC32 req;

	header.section = SECTION_BOOTLOADER;
	header.cmd = CMD_BOOTLOADER_REQUEST_CRC32;

	req.procId = procId;
	req.startAddr = startAddr;
	req.endAddr = endAddr;

	while(attempts--)
	{
		chBSemReset(&fwLoaderWifi.semResponseCrc, 1);

		NetworkSendPacket(&header, &req, sizeof(BootloaderRequestCRC32));

		if(chBSemWaitTimeout(&fwLoaderWifi.semResponseCrc, MS2ST(WIFI_TIMEOUT_MS)) != MSG_OK)
			continue;

		if(	fwLoaderWifi.responseCrc.procId != procId ||
			fwLoaderWifi.responseCrc.startAddr != startAddr ||
			fwLoaderWifi.responseCrc.endAddr != endAddr)
		{
			chThdSleepMilliseconds(WIFI_TIMEOUT_MS);
			continue;
		}

		*pCrc = fwLoaderWifi.responseCrc.crc;

		return 0;
	}

	return ERROR_FW_LOADER_TIMEOUT;
}
