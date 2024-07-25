#pragma once

#include "ch.h"

typedef struct _FwUpdaterProgram
{
	uint32_t execAddr;
	uint32_t loadAddr;
	uint32_t maxSize;
	uint32_t flashWriteGranularity;
} FwUpdaterProgram;

typedef struct _FwUpdaterProgress
{
	const char* pStepDescription;
	uint32_t totalBytes;
	uint32_t doneBytes;
} FwUpdaterProgress;

typedef int16_t(*FwUpdaterSourceFunc)(const FwUpdaterProgram*, FwUpdaterProgress*, void*);

typedef struct _FwUpdaterSource
{
	FwUpdaterSourceFunc loadFunc;
	void* pUser;
} FwUpdaterSource;

typedef struct _FwUpdater
{
	FwUpdaterProgram program;

	THD_WORKING_AREA(waTask, 4096);
	thread_t* pTask;

	FwUpdaterSource* pActiveSource;

	FwUpdaterProgress progress;
} FwUpdater;

void FwUpdaterInit(FwUpdater* pUpdater, const FwUpdaterProgram* pProgram);
void FwUpdaterLoad(FwUpdater* pUpdater, FwUpdaterSource* pSource);
