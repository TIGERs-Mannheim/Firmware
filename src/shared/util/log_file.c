#include "log_file.h"

#include "ch.h"
#include "hal/sys_time.h"
#include "util/log.h"
#include "log_msgs.h"
#include "struct_ids.h"
#include "util/cobs.h"
#include "util/network_print.h"
#include <string.h>
#include <stdio.h>

#define EVENT_CLOSE			0x10000
#define EVENT_OPEN_NAMED	0x20000
#define EVENT_OPEN_AUTO		0x40000

LogFile logFile;

void LogFileInit(uint8_t* pWriteBuf, uint32_t bufSize)
{
	logFile.ready = 0;
	chMBObjectInit(&logFile.eventQueue, logFile.eventQueueData, LOG_FILE_QUEUE_SIZE);

	chMtxObjectInit(&logFile.writeMtx);

	logFile.pBufStart = pWriteBuf;
	logFile.pBufWrite = pWriteBuf;
	logFile.pBufEnd = pWriteBuf + bufSize;

	EMAFilterInit(&logFile.emaWriteSpeed, 0.99f);
}

void LogFileOpen(const char* pName)
{
	if(logFile.ready)
		LogFileClose();

	if(pName)
	{
		snprintf(logFile.logFilename, 64, "%s.dat", pName);
		if(chMBPostTimeout(&logFile.eventQueue, EVENT_OPEN_NAMED, TIME_MS2I(500)) != MSG_OK)
		{
			LogError("post failed");
		}
	}
	else
	{
		if(chMBPostTimeout(&logFile.eventQueue, EVENT_OPEN_AUTO, TIME_MS2I(500)) != MSG_OK)
		{
			LogError("post failed");
		}
	}
}

void LogFileClose()
{
	if(chMBPostTimeout(&logFile.eventQueue, EVENT_CLOSE, TIME_MS2I(500)) != MSG_OK)
	{
		LogError("post failed");
	}
}

static void writeData()
{
	UINT bytesWritten;
	const uint8_t* pBufWriteOrig = logFile.pBufWrite;

	uint32_t dataSize = pBufWriteOrig - logFile.pBufStart;

	if(logFile.file.fs == 0 || dataSize == 0)
		return;

	uint32_t start = SysTimeUSec();
	FRESULT fresult = f_write(&logFile.file, logFile.pBufStart, dataSize, &bytesWritten);
	FRESULT fresultSync = f_sync(&logFile.file);
	uint32_t writeTime = SysTimeUSec()-start;
	if(fresult == FR_OK)
	{
		uint32_t writeSpeed = ((1e9/writeTime)*bytesWritten)/1000;
		EMAFilterUpdate(&logFile.emaWriteSpeed, writeSpeed);
		logFile.event.fileSize += bytesWritten;
	}
	else
	{
		logFile.writeFail = 1;
		LogErrorC("f_write failed", fresult);
	}

	if(dataSize != bytesWritten)
	{
		LogErrorC("dataSize != bytesWritten", bytesWritten);
	}

	if(fresultSync != FR_OK)
	{
		LogErrorC("Sync failed", fresultSync | (dataSize << 16));
	}

	logFile.bytesWritten += dataSize;

	chMtxLock(&logFile.writeMtx);

	uint32_t bytesToMove = logFile.pBufWrite - pBufWriteOrig;
	memmove(logFile.pBufStart, pBufWriteOrig, bytesToMove);
	logFile.pBufWrite = logFile.pBufStart + bytesToMove;

	chMtxUnlock(&logFile.writeMtx);
}

static void encodeAndStore(const void* pLogMsg)
{
	uint32_t bytesWritten;
	LogChunkHeader* pHeader = (LogChunkHeader*)pLogMsg;

	chMtxLock(&logFile.writeMtx);

	logFile.bytesInput += pHeader->length;

	uint32_t bytesFree = logFile.pBufEnd - logFile.pBufWrite;
	if(bytesFree > 0)
		--bytesFree;

	int16_t cobsResult = COBSEncode(pLogMsg, pHeader->length, logFile.pBufWrite, bytesFree, &bytesWritten);
	if(cobsResult)
	{
		LogInfoC("Encoding log msg failed", cobsResult);
		logFile.bytesLost += pHeader->length;
		chMtxUnlock(&logFile.writeMtx);
		return;
	}

	logFile.pBufWrite[bytesWritten++] = 0;
	logFile.pBufWrite += bytesWritten;

	chMtxUnlock(&logFile.writeMtx);
}

void LogFileWrite(const void* pLogMsg)
{
	if(logFile.ready)
		encodeAndStore(pLogMsg);
}

void LogFileWriteMultiple(const void* pData, uint32_t dataLength)
{
	if(!logFile.ready)
		return;

	uint32_t bytesWritten = 0;

	while(bytesWritten + sizeof(LogChunkHeader) < dataLength)
	{
		if(((const LogChunkHeader*)pData)->length == 0)
		{
			LogErrorC("zero length logfile entry", ((const LogChunkHeader*)pData)->type);
			break;
		}

		encodeAndStore(pData);

		LogChunkHeader* pHeader = (LogChunkHeader*)pData;
		pData += pHeader->length;
		bytesWritten += pHeader->length;
	}
}

int16_t LogFilePackConfigFile(const ConfigFile* pFile, uint8_t* pOut, uint32_t outSize, uint32_t* pBytesWritten)
{
	uint32_t msgSize = pFile->size + sizeof(LogEntryHeader);

	*pBytesWritten = msgSize;

	if(outSize < msgSize)
	{
		LogErrorC("out buffer too small", msgSize);
		return 1;
	}

	LogEntryHeader* pHeader = (LogEntryHeader*)pOut;
	pHeader->type = pFile->pDesc->cfgId;
	pHeader->length = pFile->size + sizeof(LogEntryHeader);
	pHeader->timestamp = SysTimeUSec();

	pOut += sizeof(LogEntryHeader);

	memcpy(pOut, pFile->pData, pFile->size);

	return 0;
}

int16_t LogFilePackConfigDesc(const ConfigFileDesc* pCfg, uint8_t* pOut, uint32_t outSize, uint32_t* pBytesWritten)
{
	char* pReplace;
	uint32_t msgSize = 0;

	LogChunkHeader cHeader;
	cHeader.type = SID_MESSAGE_DESC;
	cHeader.length = sizeof(uint16_t)+sizeof(uint16_t)+strlen(pCfg->pName)+1;	// msg ID, numElements, name[]

	msgSize += cHeader.length + sizeof(LogChunkHeader);

	for(uint16_t el = 0; el < pCfg->numElements; el++)
	{
		const ElementDesc* pElDesc = &pCfg->elements[el];

		cHeader.type = SID_ELEMENT_DESC;
		cHeader.length = sizeof(uint8_t) + strlen(pElDesc->pUnit)		// type and unit
				+ strlen(pElDesc->pName) + 3;	// string lengths

		msgSize += cHeader.length + sizeof(LogChunkHeader);
	}

	*pBytesWritten = msgSize;

	if(outSize < msgSize)
	{
		LogErrorC("out buffer too small", msgSize);
		return 1;
	}

	cHeader.type = SID_MESSAGE_DESC;
	cHeader.length = sizeof(LogChunkHeader)+sizeof(uint16_t)+sizeof(uint16_t)+strlen(pCfg->pName)+1;	// msg ID, numElements, name[]

	memcpy(pOut, &cHeader, sizeof(LogChunkHeader));
	pOut += sizeof(LogChunkHeader);
	memcpy(pOut, &pCfg->cfgId, sizeof(uint16_t));
	pOut += sizeof(uint16_t);
	memcpy(pOut, &pCfg->numElements, sizeof(uint16_t));
	pOut += sizeof(uint16_t);
	memcpy(pOut, pCfg->pName, strlen(pCfg->pName)+1);
	while((pReplace = strchr((char*)pOut, '/')) != 0)
		*pReplace = '_';
	pOut += strlen(pCfg->pName)+1;

	for(uint16_t el = 0; el < pCfg->numElements; el++)
	{
		const ElementDesc* pElDesc = &pCfg->elements[el];

		cHeader.type = SID_ELEMENT_DESC;
		cHeader.length = sizeof(LogChunkHeader) + sizeof(uint8_t) + strlen(pElDesc->pUnit)	// type and unit
				+ strlen(pElDesc->pName) + 3;	// string lengths

		memcpy(pOut, &cHeader, sizeof(LogChunkHeader));
		pOut += sizeof(LogChunkHeader);
		memcpy(pOut, &pElDesc->type, sizeof(uint8_t));
		pOut += sizeof(uint8_t);
		memcpy(pOut, pElDesc->pName, strlen(pElDesc->pName)+1);
		while((pReplace = strchr((char*)pOut, '/')) != 0)
			*pReplace = '_';
		pOut += strlen(pElDesc->pName)+1;
		memcpy(pOut, pElDesc->pUnit, strlen(pElDesc->pUnit)+1);
		pOut += strlen(pElDesc->pUnit)+1;
		*pOut = 0;
		pOut += 1;
	}

	return 0;
}

static void writeHeader()
{
	LogChunkHeader* pHeader = (LogChunkHeader*)logFile.initBuf;

	// write message descs
	for(uint32_t msg = 0; msg < sizeof(logMessageDescriptions)/sizeof(logMessageDescriptions[0]); msg++)
	{
		const LogMessageDesc* pMsgDesc = &logMessageDescriptions[msg];

		// now write the message
		pHeader->type = SID_MESSAGE_DESC;
		pHeader->length = sizeof(LogChunkHeader)+sizeof(uint16_t)+sizeof(uint16_t)+strlen(pMsgDesc->pName)+1;	// msg ID, numElements, name[]

		uint8_t* pWrite = logFile.initBuf+sizeof(LogChunkHeader);

		memcpy(pWrite, &pMsgDesc->msgId, sizeof(uint16_t));
		pWrite += sizeof(uint16_t);
		memcpy(pWrite, &pMsgDesc->numElements, sizeof(uint16_t));
		pWrite += sizeof(uint16_t);
		memcpy(pWrite, pMsgDesc->pName, strlen(pMsgDesc->pName)+1);

		encodeAndStore(pHeader);
		writeData();

		for(uint16_t el = 0; el < pMsgDesc->numElements; el++)
		{
			const ElementDesc* pElDesc = &pMsgDesc->elements[el];

			pHeader->type = SID_ELEMENT_DESC;
			pHeader->length = sizeof(LogChunkHeader)+ sizeof(uint8_t) + strlen(pElDesc->pUnit)	// type and unit
					+ strlen(pElDesc->pName) + strlen(pElDesc->pDesc) + 3;	// string lengths

			pWrite = logFile.initBuf+sizeof(LogChunkHeader);

			memcpy(pWrite, &pElDesc->type, sizeof(uint8_t));
			pWrite += sizeof(uint8_t);
			memcpy(pWrite, pElDesc->pName, strlen(pElDesc->pName)+1);
			pWrite += strlen(pElDesc->pName)+1;
			memcpy(pWrite, pElDesc->pUnit, strlen(pElDesc->pUnit)+1);
			pWrite += strlen(pElDesc->pUnit)+1;
			memcpy(pWrite, pElDesc->pDesc, strlen(pElDesc->pDesc)+1);

			encodeAndStore(pHeader);
			writeData();
		}
	}

	logFile.event.pFilename = logFile.logFilename;
	logFile.event.fileSize = 0;
	logFile.event.bytesPerSecIn = 0;
	logFile.event.writeSpeed = 0;

	logFile.ready = 1;
	logFile.newLogFileStarted = 1;
}

void LogFileTask(void* params)
{
	(void)params;

	msg_t event;
	FRESULT fresult;

	chRegSetThreadName("LogFile");

	while(1)
	{
		if(chVTTimeElapsedSinceX(logFile.lastStatsTime) >= TIME_MS2I(1000))
		{
			logFile.lastStatsTime = chVTGetSystemTimeX();

			logFile.event.bytesPerSecIn = logFile.bytesInput;
			logFile.event.bytesPerSecWritten = logFile.bytesWritten;
			logFile.event.bytesPerSecLost = logFile.bytesLost;
			logFile.event.writeSpeed = (uint32_t)logFile.emaWriteSpeed.value;
			logFile.bytesInput = 0;
			logFile.bytesWritten = 0;
			logFile.bytesLost = 0;
		}

		uint32_t bufSize = logFile.pBufEnd - logFile.pBufStart;

		if((uint32_t)logFile.pBufWrite > (uint32_t)logFile.pBufStart+bufSize/4)
			writeData();

		if(logFile.writeFail)
		{
			LogFileClose();
		}

		if(chMBFetchTimeout(&logFile.eventQueue, &event, TIME_MS2I(1)) == MSG_OK)
		{
			switch(event)
			{
				case EVENT_OPEN_AUTO:
				{
					logFile.writeFail = 0;

					printf("Opening logfile with auto naming\r\n");

					unsigned int id = 0;

					do
					{
						snprintf(logFile.logFilename, 64, "log%u.dat", id++);
						fresult = f_open(&logFile.file, logFile.logFilename, FA_WRITE | FA_CREATE_NEW);
					} while(fresult == FR_EXIST);

					if(fresult != FR_OK)
					{
						LogErrorC("f_open failed", fresult);
						break;
					}

					writeHeader();

					printf("Logfile open\r\n");
				}
				break;
				case EVENT_OPEN_NAMED:
				{
					logFile.writeFail = 0;

					printf("Opening named logfile: %s\r\n", logFile.logFilename);

					fresult = f_open(&logFile.file, logFile.logFilename, FA_WRITE | FA_CREATE_ALWAYS);

					if(fresult != FR_OK)
					{
						LogErrorC("f_open failed", fresult);
						break;
					}

					writeHeader();

					printf("Logfile open\r\n");
					NetworkPrint("Logfile started: %s\n", logFile.logFilename);
				}
				break;
				case EVENT_CLOSE:
				{
					if(logFile.ready == 0)
						break;

					logFile.ready = 0;

					printf("Closing logfile...");

					writeData();

					f_close(&logFile.file);
					logFile.file.fs = 0;

					logFile.event.fileSize = 0;
					logFile.event.bytesPerSecIn = 0;
					logFile.event.writeSpeed = 0;
					logFile.event.bytesPerSecLost = 0;
					logFile.event.bytesPerSecWritten = 0;

					logFile.writeFail = 0;

					printf("done\r\n");
					NetworkPrint("Log file closed\n");
				}
				break;
				default:
				break;
			}
		}
	}
}

/**
General chunk header:
uint16_t chunkType	// MSB reserved for control chunks
uint16_t chunkLength

0x8000	message description
0x8001	element description
0x8002	unit description

File Order:
unit descriptions
message descriptions
- element desc
- ...
...
 */
