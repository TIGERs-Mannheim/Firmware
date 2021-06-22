/*
 * fw_loader_fatfs.h
 *
 *  Created on: 09.04.2020
 *      Author: AndreR
 */

#pragma once

#include "fw_updater.h"

#define FW_LOADER_FATFS_BUFFER_SIZE 8192

typedef struct _FwLoaderFatFs
{
	uint8_t readBuf[FW_LOADER_FATFS_BUFFER_SIZE];
} FwLoaderFatFsGlobal;

extern FwLoaderFatFsGlobal fwLoaderFatFs;

int16_t FwLoaderFatFsUSB(const FwUpdaterProgram* pProgram, FwUpdaterProgress* pProgress);
int16_t FwLoaderFatFsSDCard(const FwUpdaterProgram* pProgram, FwUpdaterProgress* pProgress);
