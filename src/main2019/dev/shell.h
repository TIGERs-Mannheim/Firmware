#pragma once

#include "util/shell.h"
#include "hal/uart_dma.h"
#include "ch.h"

typedef struct _DevShell
{
	Shell shell;

	UartDma* pUsart;

	THD_WORKING_AREA(waTask, 4096);
	thread_t* pTask;

	char injectBuf[SHELL_INPUT_BUFFER_SIZE];
	size_t injectSize;
	mutex_t injectMutex;
} DevShell;

extern DevShell devShell;

void DevShellInit(UartDma* pUsart, tprio_t taskPrio);
void DevShellInject(const uint8_t* pData, size_t dataLength);
