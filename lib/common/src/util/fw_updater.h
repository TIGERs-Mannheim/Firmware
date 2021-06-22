/*
 * fw_loader.h
 *
 *  Created on: 07.04.2020
 *      Author: AndreR
 */

#pragma once

#include "ch.h"
#include "util/flash_fs.h"

#define FW_UPDATER_SOURCE_WIFI		0
#define FW_UPDATER_SOURCE_USB		1
#define FW_UPDATER_SOURCE_SDCARD	2

#define FW_ID_MAIN2016	5
#define FW_ID_MAIN2019	6
#define FW_ID_IR2019	7
#define FW_ID_MOT2019	8

#define FW_UPDATER_MAX_PROGRAMS	4
#define FW_UPDATER_MAX_SOURCES	4

typedef struct _FwUpdaterProgram
{
	uint8_t procId;
	uint32_t execAddr;
	uint32_t loadAddr;
	uint32_t maxSize;
} FwUpdaterProgram;

typedef struct _FwUpdaterProgress
{
	uint8_t procId;
	const char* pStepDescription;
	uint32_t totalBytes;
	uint32_t doneBytes;
} FwUpdaterProgress;

typedef int16_t(*FwUpdaterSourceFunc)(const FwUpdaterProgram*, FwUpdaterProgress*);

typedef struct _FwUpdaterSource
{
	uint8_t sourceId;
	FwUpdaterSourceFunc loadFunc;
} FwUpdaterSource;

typedef struct _FwLoaderUpdateInfo
{
	uint8_t updateProgram[FW_UPDATER_MAX_PROGRAMS];
} FwUpdaterInfo;

typedef struct _FwUpdater
{
	FwUpdaterProgram programs[FW_UPDATER_MAX_PROGRAMS];
	uint8_t usedPrograms;

	FwUpdaterSource sources[FW_UPDATER_MAX_SOURCES];
	uint8_t usedSources;

	FwUpdaterInfo info;
	FlashFile* pInfoFile;

	thread_t* pFwLoadThread;
	uint8_t activeSource;

	FwUpdaterProgress progresses[FW_UPDATER_MAX_PROGRAMS];
} FwUpdater;

extern FwUpdater fwUpdater;

void FwUpdaterInit();
void FwUpdaterLoad(uint8_t source);
void FwUpdaterUpdate();
void FwUpdaterAddProgram(FwUpdaterProgram program);
void FwUpdaterAddSource(FwUpdaterSource source);
const char* FwUpdaterGetName(uint8_t programId);
