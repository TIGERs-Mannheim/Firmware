#pragma once

#include "fw_updater.h"

#define FW_LOADER_FATFS_BUFFER_SIZE 8192

typedef struct _FwLoaderFatFs
{
	FwUpdaterSource source;

	uint8_t readBuf[FW_LOADER_FATFS_BUFFER_SIZE];
	const char* pFilename;
} FwLoaderFatFs;

void	FwLoaderFatFsInit(FwLoaderFatFs* pLoader, const char* pFilename);
int16_t	FwLoaderFatFsLoad(const FwUpdaterProgram* pProgram, FwUpdaterProgress* pProgress, void* pUser);
