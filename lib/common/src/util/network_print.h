/*
 * network_print.h
 *
 *  Created on: 23.10.2014
 *      Author: AndreR
 */

#pragma once

#include "ch.h"

#define NETWORK_PRINT_SIZE 100

typedef void(*NetworkPrintOutputFunc)(void* pBuf, uint16_t dataLength);

typedef struct _NetworkPrint
{
	uint8_t netPrint[NETWORK_PRINT_SIZE];
	char* pPrintPos;
	uint16_t printSize;
	mutex_t printMutex;
	NetworkPrintOutputFunc pOutFunc;
} NetworkPrintData;

extern NetworkPrintData networkPrint;

void NetworkPrintInit(NetworkPrintOutputFunc pOutFunc);
int32_t NetworkPrint(const char* fmt, ...);
