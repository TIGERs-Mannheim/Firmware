#include "fw_updater.h"
#include "util/log.h"
#include "errors.h"
#include <string.h>
#include <stdio.h>

static void fwLoadTask(void* params)
{
	FwUpdater* pUpdater = (FwUpdater*)params;

	chRegSetThreadName("FwLoad");

	// reset progress struct
	memset(&pUpdater->progress, 0, sizeof(FwUpdaterProgress));
	pUpdater->progress.pStepDescription = "waiting";

	FwUpdaterSource* pSource = pUpdater->pActiveSource;

	// load new programs
	uint8_t anyUpdate = 0;
	const FwUpdaterProgram* pProgram = &pUpdater->program;

	printf("Loading new program code at load addr: 0x%08X\r\n", pProgram->loadAddr);

	int16_t result = (*pSource->loadFunc)(pProgram, &pUpdater->progress, pSource->pUser);
	switch(result)
	{
		case 0:
		{
			anyUpdate = 1;
		}
		break;
		case ERROR_FW_LOADER_UP_TO_DATE:
		{
			printf("Program is up to date\r\n");
			pUpdater->progress.pStepDescription = "Up to date";
		}
		break;
		case ERROR_FW_LOADER_TIMEOUT:
		{
			printf("Timed out\r\n");
			pUpdater->progress.pStepDescription = "Timeout";
		}
		break;
		case ERROR_FW_LOADER_NO_MEDIA:
		{
			printf("Media missing\r\n");
			pUpdater->progress.pStepDescription = "File not found";
		}
		break;
		default:
		{
			printf("Load failed: 0x%04X\r\n", result);
			pUpdater->progress.pStepDescription = "failed";
		}
		break;
	}

	if(anyUpdate)
	{
		NVIC_SystemReset();
	}

	pUpdater->pActiveSource = 0;

	chThdExit(0);
}

void FwUpdaterInit(FwUpdater* pUpdater, const FwUpdaterProgram* pProgram)
{
	pUpdater->program = *pProgram;
}

void FwUpdaterLoad(FwUpdater* pUpdater, FwUpdaterSource* pSource)
{
	if(pUpdater->pTask != 0 && chThdTerminatedX(pUpdater->pTask) == 0)
	{
		printf("Program update already in progress\r\n");
		return;
	}

	pUpdater->pActiveSource = pSource;

	pUpdater->pTask = chThdCreateStatic(pUpdater->waTask, sizeof(pUpdater->waTask), LOWPRIO, &fwLoadTask, pUpdater);
	chThdRelease(pUpdater->pTask);
}
