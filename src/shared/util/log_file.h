/*
 * log_file.h
 *
 *  Created on: 24.08.2014
 *      Author: AndreR
 */

#pragma once

#include <stdint.h>
#include "ch.h"
#include "ff.h"
#include "config.h"
#include "math/ema_filter.h"

typedef struct _LogFileEvent
{
	const char* pFilename;
	uint32_t fileSize;
	uint32_t bytesPerSecIn;
	uint32_t bytesPerSecWritten;
	uint32_t bytesPerSecLost;
	uint32_t writeSpeed;	// B/s
} LogFileEvent;

#define LOG_FILE_QUEUE_SIZE	10
#define LOG_FILE_WORKING_BUF_SIZE 512

typedef struct _LogFile
{
	FIL file;
	uint8_t ready;
	uint8_t writeFail;

	uint8_t* pBufStart;
	uint8_t* pBufEnd;
	uint8_t* pBufWrite;
	mutex_t writeMtx;

	msg_t eventQueueData[LOG_FILE_QUEUE_SIZE];
	mailbox_t eventQueue;

	char logFilename[64];
	uint8_t initBuf[LOG_FILE_WORKING_BUF_SIZE];

	LogFileEvent event;

	EMAFilter emaWriteSpeed;

	uint32_t bytesInput;	// short term
	uint32_t bytesWritten;
	uint32_t bytesLost;
	uint32_t lastStatsTime;

	uint8_t newLogFileStarted; // used by config system to dump all configs once
} LogFile;

extern LogFile logFile;

void	LogFileInit(uint8_t* pWriteBuf, uint32_t bufSize);
void	LogFileOpen(const char* pName);
void	LogFileClose();
void	LogFileTask(void* params);
void	LogFileWrite(const void* pLogMsg);
void	LogFileWriteMultiple(const void* pData, uint32_t dataLength);

int16_t LogFilePackConfigDesc(const ConfigFileDesc* pCfg, uint8_t* pOut, uint32_t outSize, uint32_t* pBytesWritten);
int16_t LogFilePackConfigFile(const ConfigFile* pFile, uint8_t* pOut, uint32_t outSize, uint32_t* pBytesWritten);
