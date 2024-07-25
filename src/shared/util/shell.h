#pragma once

#include "ch.h"
#include "shell_cmd.h"

#define SHELL_INPUT_BUFFER_SIZE 80
#define SHELL_HISTORY_SIZE 20

typedef void(*ShellWriteFunc)(const char*, size_t);

typedef struct _ShellHistoryEntry
{
	char dataCur[SHELL_INPUT_BUFFER_SIZE];
	char dataOrig[SHELL_INPUT_BUFFER_SIZE];
	uint16_t lengthOrig;
	uint16_t lengthCur; // if command is extended in history
} ShellHistoryEntry;

typedef struct _Shell
{
	ShellHistoryEntry history[SHELL_HISTORY_SIZE];
	ShellHistoryEntry* pHistoryCur;
	ShellHistoryEntry* pHistoryLast;

	uint16_t cursorPos;

	uint8_t escapeMode;

	struct
	{
		char param[16];
		uint16_t paramPos;
		char inter[16];
		uint16_t interPos;
		char final;
	} esc;

	char activeCmdData[SHELL_INPUT_BUFFER_SIZE+1];
	ShellCmdHandler cmdHandler;

	ShellWriteFunc writeFunc;
} Shell;

void ShellInit(Shell* pShell, ShellWriteFunc writeFunc);
void ShellParse(Shell* pShell, char data);
