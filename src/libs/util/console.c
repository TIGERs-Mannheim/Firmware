/*
 * console.c
 *
 *  Created on: 25.05.2011
 *      Author: AndreR
 */

#include "console.h"

#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>

Console console;

uint8_t verifyCmdReady()
{
	if(console.pCurScan)
		return 1;

	if(console.rxFifo.numPackets > 0)
	{
		FifoLinGet(&console.rxFifo, &console.pCurScan, &console.scanSize);
		return 1;
	}

	return 0;
}

void ConsoleInit(ConsoleData data)
{
	console.data = data;

	FifoLinInit(&console.txFifo, console.data.pDataTx, console.data.txSize);
	FifoLinInit(&console.rxFifo, console.data.pDataRx, console.data.rxSize);

	console.pCurScan = 0;
	console.scanSize = 0;
}

uint16_t ConsoleNumCmdsReady()
{
	return console.rxFifo.numPackets;
}

uint8_t ConsoleIsCmdReady()
{
	if(console.pCurScan)
		return 1;

	return ConsoleNumCmdsReady() > 0 ? 1 : 0;
}

int16_t ConsoleCmdProcessed()
{
	console.pCurScan = 0;

	FifoLinDelete(&console.rxFifo);

	return 0;
}

int16_t ConsoleCmpCmd(const char* cmd)
{
	if(verifyCmdReady() == 0)
		return 0;

	int16_t result = 0;

	if(strcmp((char*)console.pCurScan, cmd) == 0)
		result = 1;

	return result;
}

int32_t ConsoleScanCmd(const char* fmt, ...)
{
	if(verifyCmdReady() == 0)
		return 0;

	va_list args;
	int32_t storedFields = 0;

	va_start(args, fmt);
	storedFields = vsscanf((const char*)console.pCurScan, fmt, args);
	va_end(args);

	return storedFields;
}


int32_t ConsolePrint(const char* fmt, ...)
{
	va_list args;
	int32_t bytesWritten;
	uint8_t* pPrintBuf;
	int16_t result;

	result = FifoLinReserve(&console.txFifo, console.data.printSize, &pPrintBuf);
	if(result)
		return result;

	va_start(args, fmt);
	bytesWritten = vsnprintf((char*)pPrintBuf, console.data.printSize, fmt, args);
	va_end(args);

	FifoLinCommit(&console.txFifo, bytesWritten);

	if(console.data.txCallback)
		(*console.data.txCallback)(console.data.pUserData);

	return bytesWritten;
}

int16_t ConsoleLockPrintBuf(uint32_t maxSize, uint8_t** ppPrintBuf)
{
	int16_t result;

	result = FifoLinReserve(&console.txFifo, maxSize, ppPrintBuf);
	if(result)
		return result;

	return result;
}

int16_t ConsoleUnlockPrintBuf(uint32_t realSize)
{
	int16_t result;

	result = FifoLinCommit(&console.txFifo, realSize);

	if(console.data.txCallback)
		(*console.data.txCallback)(console.data.pUserData);

	return result;
}
