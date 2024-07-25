#include "shell.h"
#include <string.h>

#define EVENT_MASK_UART			EVENT_MASK(0)
#define EVENT_MASK_INJECT		EVENT_MASK(1)

DevShell devShell;

static void shellTask(void* params);
static void uartWriteFunc(const char* pData, size_t length);

void DevShellInit(UartDma* pUsart, tprio_t taskPrio)
{
	chMtxObjectInit(&devShell.injectMutex);

	devShell.pUsart = pUsart;

	ShellInit(&devShell.shell, &uartWriteFunc);

	devShell.pTask = chThdCreateStatic(devShell.waTask, sizeof(devShell.waTask), taskPrio, &shellTask, 0);
}

void DevShellInject(const uint8_t* pData, size_t dataLength)
{
	if(devShell.injectSize != 0)
		return;

	chMtxLock(&devShell.injectMutex);

	if(dataLength > sizeof(devShell.injectBuf))
		dataLength = sizeof(devShell.injectBuf);

	memcpy(devShell.injectBuf, pData, dataLength);
	devShell.injectSize = dataLength;

	chMtxUnlock(&devShell.injectMutex);

	chEvtSignal(devShell.pTask, EVENT_MASK_INJECT);
}

static void uartWriteFunc(const char* pData, size_t length)
{
	UartDmaWrite(devShell.pUsart, pData, length);
}

static void shellTask(void* params)
{
	(void)params;

	uint8_t data;

	chRegSetThreadName("Shell");

	event_listener_t uartListener;
	chEvtRegisterMask(&devShell.pUsart->eventSource, &uartListener, EVENT_MASK_UART);

	while(1)
	{
		chEvtWaitAnyTimeout(ALL_EVENTS, TIME_INFINITE);

		while(UartDmaRead(devShell.pUsart, &data, 1) > 0)
		{
			ShellParse(&devShell.shell, data);
		}

		if(devShell.injectSize)
		{
			chMtxLock(&devShell.injectMutex);

			for(size_t i = 0; i < devShell.injectSize; i++)
				ShellParse(&devShell.shell, devShell.injectBuf[i]);

			ShellParse(&devShell.shell, '\n');

			devShell.injectSize = 0;

			chMtxUnlock(&devShell.injectMutex);
		}
	}
}
