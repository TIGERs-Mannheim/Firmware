/*
 * terminal.h
 *
 *  Created on: 26.10.2014
 *      Author: AndreR
 */

#pragma once

#include "ch.h"

#define TERMINAL_INPUT_BUFFER_SIZE 80
#define TERMINAL_HISTORY_SIZE 20

typedef void(*TerminalCmdCb)(char*, uint16_t);

typedef struct _TerminalHistoryEntry
{
	char dataCur[TERMINAL_INPUT_BUFFER_SIZE];
	char dataOrig[TERMINAL_INPUT_BUFFER_SIZE];
	uint16_t lengthOrig;
	uint16_t lengthCur; // if command is extended in history
} TerminalHistoryEntry;

typedef struct _Terminal
{
	TerminalHistoryEntry history[TERMINAL_HISTORY_SIZE];
	TerminalHistoryEntry* pHistoryCur;
	TerminalHistoryEntry* pHistoryLast;

	uint16_t cursorPos;

	struct
	{
		char param[16];
		uint16_t paramPos;
		char inter[16];
		uint16_t interPos;
		char final;
	} esc;

	TerminalCmdCb cmdCb;

	input_queue_t* pInQueue;
	output_queue_t* pOutQueue;
} Terminal;

extern Terminal terminal;

void	TerminalInit(input_queue_t* pInQueue, output_queue_t* pOutQueue);
void	TerminalTask(void* params);
void	TerminalWrite(const uint8_t* pData, uint32_t dataLength);
