#pragma once

#include "ch.h"
#include "util/bootloader_protocol.h"

typedef struct _BootloaderData
{
	uint32_t bootloaderProgramAddr;
	uint32_t bootloaderMaxProgramSize;

	uint32_t flashAddr;
	uint32_t flashSize;

	uint32_t activeProgramAddr;
	uint32_t newProgramAddr;
	uint32_t maxProgramSize;

	uint32_t flashWriteGranularity;
	uint32_t flashWriteTime_us;
	uint32_t flashEraseTime_us;

	uint32_t bootloaderTimeout_ms;

	const char* pDeviceName;

	void (*pSystemDeinit)();
} BootloaderData;

typedef struct _BootloaderInterface
{
	struct _BootloaderInterface* pNext;

	uint8_t (*pReadByte)(void *pUser, uint8_t *pData);
	void (*pWriteData)(void *pUser, const void* const pData, uint32_t dataLength);

	void *pUser;

	Bootloader_Parser_t parser;
} BootloaderInterface;

#define BOOTLOADER_OPERATION_STATE_NONE		0
#define BOOTLOADER_OPERATION_STATE_ACTIVE	1
#define BOOTLOADER_OPERATION_STATE_ACK		2
#define BOOTLOADER_OPERATION_STATE_NACK		3

typedef struct _BootloaderOperation
{
	uint32_t type;
	uint32_t state;
	uint32_t addr;
	uint32_t length;
} BootloaderOperation;

typedef struct _Bootloader
{
	BootloaderData data;

	BootloaderInterface* pInterfaceList;
	BootloaderInterface* pSelectedInterface;

	uint8_t tmpBuf[32];

	THD_WORKING_AREA(waTask, 256);
	thread_t* pTask;

	systime_t initiateExitTime;

	BootloaderOperation currentOperation;
	BootloaderOperation lastOperation;
} Bootloader;

void BootloaderInit(Bootloader* pBoot, BootloaderData* pInit, tprio_t prio);
void BootloaderAddInterface(Bootloader* pBoot, BootloaderInterface* pInterface);
