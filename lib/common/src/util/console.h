/*
 * console.h
 *
 *  Created on: 24.05.2011
 *      Author: AndreR
 */

#ifndef __CONSOLE_H__
#define __CONSOLE_H__

#include <stdint.h>

#include "util/fifo_lin.h"

typedef void(*NotifyTxReady)(void*);

typedef struct _ConsoleData
{
	uint8_t* pDataTx;
	uint16_t txSize;

	uint8_t* pDataRx;
	uint16_t rxSize;

	uint16_t printSize;

	NotifyTxReady txCallback;
	void* pUserData;
} ConsoleData;

typedef struct _Console
{
	ConsoleData data;

	FifoLin txFifo;
	FifoLin rxFifo;

	uint8_t* pCurScan;
	uint32_t scanSize;
} Console;

extern Console console;

void		ConsoleInit(ConsoleData data);
uint16_t	ConsoleNumCmdsReady();
uint8_t		ConsoleIsCmdReady();
int16_t		ConsoleCmdProcessed();
int16_t		ConsoleCmpCmd(const char* cmd);
int32_t		ConsoleScanCmd(const char* fmt, ...);
int32_t		ConsolePrint(const char* fmt, ...);
int16_t		ConsoleLockPrintBuf(uint32_t maxSize, uint8_t** ppPrintBuf);
int16_t		ConsoleUnlockPrintBuf(uint32_t realSize);

#endif
