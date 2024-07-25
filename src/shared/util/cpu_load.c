#include "cpu_load.h"
#include "ch.h"
#include "hal/init_hal.h"
#include <stdio.h>

#pragma GCC optimize ("Os")

CPULoad cpuLoad;

static const char* tStateNames[] = { CH_STATE_NAMES };

void CPULoadPrintTasks()
{
	printf("Name            rPrio\tCPU Usage\tStack Free\tState\t\tMutices\r\n");

	for(thread_t* pThread = chRegFirstThread(); pThread != 0; pThread = chRegNextThread(pThread))
	{
		uint16_t cpuTime = pThread->usage;

		const char* pName;
		if(pThread->name)
			pName = pThread->name;
		else
			pName = "Unknown";

		printf("%-16s%u\t%3hu.%02hu\t\t%u\t\t%-10s\t", pName,
				pThread->realprio, cpuTime/100, cpuTime%100, chThdGetFreeStack(pThread),
				tStateNames[pThread->state]);

		for(mutex_t* pMutex = pThread->mtxlist; pMutex != 0; pMutex = pMutex->next)
		{
			printf("0x%08X ", pMutex);
		}

		printf("\r\n");

		chThdSleepMilliseconds(10);
	}
}

void CPULoadPrintUsage()
{
	uint16_t cpuTime = (uint16_t)(cpuLoad.usage*10000.0f);

	printf("CPU Usage: %3hu.%02hu\r\n", cpuTime/100, cpuTime%100);
}

void CPULoadUpdateUsage()
{
	if(chVTTimeElapsedSinceX(cpuLoad.lastUsageUpdateTime) < TIME_MS2I(1000))
		return;

	cpuLoad.lastUsageUpdateTime = chVTGetSystemTimeX();

	uint64_t total = 0;
	for(thread_t* pThread = chRegFirstThread(); pThread != 0; pThread = chRegNextThread(pThread))
		total += pThread->stats.cumulative;

	for(thread_t* pThread = chRegFirstThread(); pThread != 0; pThread = chRegNextThread(pThread))
	{
		pThread->usage = (uint16_t)((float)pThread->stats.cumulative/(float)total * 10000.0f);
		pThread->stats.cumulative = 0;
	}

	thread_t* pIdle = chSysGetIdleThreadX();

	cpuLoad.usage = 1.0f - pIdle->usage / 10000.0f;
}
