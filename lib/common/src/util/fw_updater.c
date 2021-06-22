/*
 * fw_loader.c
 *
 *  Created on: 07.04.2020
 *      Author: AndreR
 */

#include "fw_updater.h"
#include "util/console.h"
#include "util/log.h"
#include "util/flash.h"
#include "util/boot.h"
#include "errors.h"
#include <string.h>

FwUpdater fwUpdater;

static THD_WORKING_AREA(waFwLoad, 8192);

static void fwLoadTask(void* params)
{
	(void)params;

	chRegSetThreadName("FwLoad");

	// reset all progress structs
	for(uint8_t i = 0; i < fwUpdater.usedPrograms; i++)
	{
		memset(&fwUpdater.progresses[i], 0, sizeof(FwUpdaterProgress));
		fwUpdater.progresses[i].procId = fwUpdater.programs[i].procId;
		fwUpdater.progresses[i].pStepDescription = "waiting";
	}

	// find source to load new firmware from
	FwUpdaterSource* pSource = 0;
	for(uint8_t i = 0; i < fwUpdater.usedSources; i++)
	{
		if(fwUpdater.sources[i].sourceId == fwUpdater.activeSource)
		{
			pSource = &fwUpdater.sources[i];
			break;
		}
	}

	if(!pSource)
	{
		ConsolePrint("Unknown source to load firmware");
		chThdExit(ERROR_FW_LOADER_UNKNOWN_SOURCE);
	}

	// load new programs
	uint8_t anyUpdate = 0;
	for(uint8_t i = 0; i < fwUpdater.usedPrograms; i++)
	{
		const FwUpdaterProgram* pProgram = &fwUpdater.programs[i];

		ConsolePrint("Loading new program code at load addr: 0x%08X, ID: %hu\r\n", pProgram->loadAddr, (uint16_t)pProgram->procId);

		fwUpdater.info.updateProgram[i] = 0;

		int16_t result = (*pSource->loadFunc)(pProgram, &fwUpdater.progresses[i]);
		switch(result)
		{
			case 0:
				break;
			case ERROR_FW_LOADER_UP_TO_DATE:
			{
				ConsolePrint("Program is up to date\r\n");
				fwUpdater.progresses[i].pStepDescription = "Up to date";
				continue;
			}
			case ERROR_FW_LOADER_TIMEOUT:
			{
				ConsolePrint("Timed out\r\n", result);
				fwUpdater.progresses[i].pStepDescription = "Timeout";
				continue;
			}
			case ERROR_FW_LOADER_NO_MEDIA:
			{
				ConsolePrint("Media missing\r\n", result);
				fwUpdater.progresses[i].pStepDescription = "File not found";
				continue;
			}
			default:
			{
				ConsolePrint("Load failed: 0x%04X\r\n", result);
				fwUpdater.progresses[i].pStepDescription = "failed";
				continue;
			}
		}

		fwUpdater.info.updateProgram[i] = 1;
		anyUpdate = 1;
	}

	FlashFSWrite(&fwUpdater.pInfoFile, &fwUpdater.info, sizeof(FwUpdaterInfo));

	if(anyUpdate)
	{
		BootSetBootloaderSelected(1);
		BootReset();
	}

	chThdExit(0);
}

static void fwUpdateTask(void* params)
{
	(void)params;

	chRegSetThreadName("FwUpdate");

	int16_t result;
	uint8_t anyError = 0;

	// reset all progress structs
	for(uint8_t i = 0; i < fwUpdater.usedPrograms; i++)
	{
		memset(&fwUpdater.progresses[i], 0, sizeof(FwUpdaterProgress));
		fwUpdater.progresses[i].procId = fwUpdater.programs[i].procId;
		fwUpdater.progresses[i].pStepDescription = "waiting";
	}

	for(uint8_t prog = 0; prog < fwUpdater.usedPrograms; prog++)
	{
		if(fwUpdater.info.updateProgram[prog] == 0)
			continue;

		const FwUpdaterProgram* pProgram = &fwUpdater.programs[prog];

		ConsolePrint("Loading program %hu from 0x%08X => 0x%08X\r\n", (uint16_t)pProgram->procId, pProgram->loadAddr, pProgram->execAddr);

		// determine size of new program (actually used bytes)
		uint32_t newProgramSize = 0;
		for(uint32_t* pData = (uint32_t*)(pProgram->loadAddr + pProgram->maxSize - sizeof(uint32_t)); (uint32_t)pData > pProgram->loadAddr; pData--)
		{
			if(*pData != 0xFFFFFFFF)
			{
				newProgramSize = (uint32_t)pData - pProgram->loadAddr + sizeof(uint32_t);
				break;
			}
		}

		if(newProgramSize == 0)
		{
			ConsolePrint("No program data found in load area for program %hu", (uint16_t)prog);
			fwUpdater.progresses[prog].pStepDescription = "Up to date";
			continue;
		}

		// align to 32 byte
		newProgramSize = (newProgramSize+31)/32*32;

		fwUpdater.progresses[prog].pStepDescription = "Formatting";
		fwUpdater.progresses[prog].totalBytes = newProgramSize;

		chThdSleepMilliseconds(100); // let ConsolePrint finish

		result = FlashErase(pProgram->execAddr, pProgram->execAddr + pProgram->maxSize);
		if(result)
		{
			ConsolePrint("FlashErase failed: 0x%04hX\r\n", result);
			fwUpdater.progresses[prog].pStepDescription = "Format failed";
			anyError = 1;
			continue;
		}

		fwUpdater.progresses[prog].pStepDescription = "Programming";

		for(uint32_t offset = 0; offset < newProgramSize; offset += 128)
		{
			fwUpdater.progresses[prog].doneBytes = offset;

			result = FlashProgram(pProgram->execAddr + offset, (uint32_t*)(pProgram->loadAddr + offset), 128);
			if(result)
			{
				ConsolePrint("FlashProgram failed: 0x%04hX\r\n", result);
				fwUpdater.progresses[prog].pStepDescription = "Programming failed";
				anyError = 1;
				break;
			}
		}

		fwUpdater.progresses[prog].pStepDescription = "Programmed";
		fwUpdater.progresses[prog].doneBytes = newProgramSize;
	}

	for(uint8_t prog = 0; prog < fwUpdater.usedPrograms; prog++)
	{
		if(fwUpdater.info.updateProgram[prog] == 0)
			continue;

		fwUpdater.progresses[prog].pStepDescription = "Clean up";

		chThdSleepMilliseconds(50); // make sure step description reaches GUI

		const FwUpdaterProgram* pProgram = &fwUpdater.programs[prog];

		FlashErase(pProgram->loadAddr, pProgram->loadAddr + pProgram->maxSize);

		fwUpdater.info.updateProgram[prog] = 0;

		fwUpdater.progresses[prog].pStepDescription = "Done";
	}

	FlashFSWrite(&fwUpdater.pInfoFile, &fwUpdater.info, sizeof(FwUpdaterInfo));

	chThdSleepMilliseconds(200);

	if(!anyError)
		BootReset();
}

void FwUpdaterInit()
{
	FlashFSOpenOrCreate("fw/update", 0, &fwUpdater.info, sizeof(FwUpdaterInfo), &fwUpdater.pInfoFile);
}

void FwUpdaterLoad(uint8_t source)
{
	if(fwUpdater.pFwLoadThread != 0 && chThdTerminatedX(fwUpdater.pFwLoadThread) == 0)
	{
		ConsolePrint("Program update already in progress\r\n");
		return;
	}

	fwUpdater.activeSource = source;
	fwUpdater.pFwLoadThread = chThdCreateStatic(waFwLoad, sizeof(waFwLoad), LOWPRIO, &fwLoadTask, 0);
}

void FwUpdaterUpdate()
{
	if(fwUpdater.pFwLoadThread != 0 && chThdTerminatedX(fwUpdater.pFwLoadThread) == 0)
	{
		ConsolePrint("Program update already in progress\r\n");
		return;
	}

	fwUpdater.pFwLoadThread = chThdCreateStatic(waFwLoad, sizeof(waFwLoad), LOWPRIO, &fwUpdateTask, 0);
}

void FwUpdaterAddProgram(FwUpdaterProgram program)
{
	if(fwUpdater.usedPrograms >= FW_UPDATER_MAX_PROGRAMS)
	{
		LogError("Max number of programs reached");
		return;
	}

	memcpy(&fwUpdater.programs[fwUpdater.usedPrograms], &program, sizeof(FwUpdaterProgram));
	++fwUpdater.usedPrograms;
}

void FwUpdaterAddSource(FwUpdaterSource source)
{
	if(fwUpdater.usedSources >= FW_UPDATER_MAX_SOURCES)
	{
		LogError("Max numner of sources reached");
		return;
	}

	memcpy(&fwUpdater.sources[fwUpdater.usedSources], &source, sizeof(FwUpdaterSource));
	++fwUpdater.usedSources;
}

const char* FwUpdaterGetName(uint8_t programId)
{
	switch(programId)
	{
		case FW_ID_MAIN2016: return "main2016";
		case FW_ID_MAIN2019: return "main2019";
		case FW_ID_IR2019: return "ir2019";
		case FW_ID_MOT2019: return "mot2019";
		default: return "unknown";
	}
}
