/*
 * network_print.c
 *
 *  Created on: 23.10.2014
 *      Author: AndreR
 */

#include "network_print.h"
#include "commands.h"
#include "intercom_constants.h"

#include <stdarg.h>
#include <stdio.h>

NetworkPrintData networkPrint;

#define TOTAL_HEADER_SIZE (sizeof(PacketHeader)+sizeof(SystemConsolePrint))

void NetworkPrintInit(NetworkPrintOutputFunc pOutFunc)
{
	chMtxObjectInit(&networkPrint.printMutex);
	networkPrint.printSize = sizeof(networkPrint.netPrint) - TOTAL_HEADER_SIZE;
	networkPrint.pOutFunc = pOutFunc;

	uint8_t* pData = networkPrint.netPrint;
	PacketHeader* pHeader = (PacketHeader*)pData;
	pData += sizeof(PacketHeader);
	SystemConsolePrint* pConPrint = (SystemConsolePrint*)pData;
	pData += sizeof(SystemConsolePrint);
	networkPrint.pPrintPos = (char*)pData;

	pHeader->cmd = CMD_SYSTEM_CONSOLE_PRINT;
	pHeader->section = SECTION_SYSTEM;

	pConPrint->source = 1;
}

int32_t NetworkPrint(const char* fmt, ...)
{
	va_list args;
	int32_t bytesWritten;

	if(networkPrint.pOutFunc == 0)
		return 0;

	chMtxLock(&networkPrint.printMutex);
	{
		va_start(args, fmt);
		bytesWritten = vsnprintf(networkPrint.pPrintPos, networkPrint.printSize, fmt, args);
		va_end(args);

		if(bytesWritten > networkPrint.printSize)
			bytesWritten = networkPrint.printSize;

		(*networkPrint.pOutFunc)(networkPrint.netPrint, bytesWritten+TOTAL_HEADER_SIZE);
	}
	chMtxUnlock(&networkPrint.printMutex);

	return bytesWritten;
}
