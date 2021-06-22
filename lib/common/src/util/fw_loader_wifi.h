/*
 * fw_loader_wifi.h
 *
 *  Created on: 08.04.2020
 *      Author: AndreR
 */

#pragma once

#include "fw_updater.h"
#include "commands.h"

#define FW_LOADER_WIFI_CHUNK_SIZE 56 // must be multiple of 4
#define FW_LOADER_WIFI_NUM_CHUNKS 4
#define FW_LOADER_WIFI_SLICE_SIZE (FW_LOADER_WIFI_CHUNK_SIZE * FW_LOADER_WIFI_NUM_CHUNKS) // must be multiple of 32

typedef struct _FwLoaderWifiRequest
{
	uint8_t state;
	uint8_t procId;
	uint16_t retries;
	uint32_t offset;
	systime_t requestTime;
	uint8_t* pData;
} FwLoaderWifiRequest;

typedef struct _FwLoaderWifi
{
	BootloaderSize responseSize;
	BootloaderCRC32 responseCrc;

	binary_semaphore_t semResponseSize;
	binary_semaphore_t semResponseCrc;

	FwLoaderWifiRequest requests[FW_LOADER_WIFI_NUM_CHUNKS];
	uint8_t sliceData[FW_LOADER_WIFI_SLICE_SIZE];

	mutex_t sliceMutex;
	condition_variable_t newDataCondition;
} FwLoaderWifiGlobal;

extern FwLoaderWifiGlobal fwLoaderWifi;

int16_t	FwLoaderWifi(const FwUpdaterProgram* pProgram, FwUpdaterProgress* pProgress);
void	FwLoaderWifiNetworkInput(PacketHeader* pHeader, uint16_t dataLength, uint8_t* pData);
