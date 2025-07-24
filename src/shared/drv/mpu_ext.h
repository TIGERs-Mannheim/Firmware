#pragma once

#include "hal/uart_dma.h"
#include "util/cobs.h"
#include "ext_commands.h"

#define MPU_EXT_MAX_PAYLOAD_SIZE (512*4)
#define MPU_EXT_HEADER_SIZE sizeof(ExtPacketHeader)
#define MPU_EXT_TRAILER_SIZE sizeof(uint32_t)
#define MPU_EXT_MAX_PACKET_SIZE (MPU_EXT_MAX_PAYLOAD_SIZE + MPU_EXT_HEADER_SIZE + MPU_EXT_TRAILER_SIZE)
#define MPU_EXT_MAX_COBS_SIZE COBSMaxStuffedSize(MPU_EXT_MAX_PACKET_SIZE)

#define MPU_EXT_EVENT_PACKET_RECEIVED	EVENT_MASK(0)

typedef struct _MpuExtStats
{
	uint32_t rxWire;
	uint32_t rxDecoded;
	uint32_t rxPayload;

	uint32_t txPayload;
	uint32_t txEncoded;
	uint32_t txWire;
} MpuExtStats;

typedef struct _MpuExtPacketHandler
{
	ExtPacketHeader cmd;
	void* pStorage;
	size_t storageSize;
	uint8_t updated;
	uint32_t updateTimestamp_us;

	struct _MpuExtPacketHandler* pNext;
	mutex_t storageMutex;
} MpuExtPacketHandler;

typedef struct _MpuExt
{
	// Interface
	UartDma* pUart;

	// Payload encoding/decoding (COBS)
	uint8_t rxProcessingBuf[MPU_EXT_MAX_COBS_SIZE+1];
	uint16_t rxProcBufUsed;

	uint8_t cobsProcBuf[MPU_EXT_MAX_COBS_SIZE+1];

	uint8_t txAssemblyBuf[MPU_EXT_MAX_COBS_SIZE+1];
	mutex_t txAssemblyMutex;

	MpuExtStats liveStats;
	MpuExtStats stats;

	// OS data
	THD_WORKING_AREA(waTask, 256);
	thread_t* pTask;

	event_source_t eventSource;

	// Application data
	uint8_t isInstalled;

	MpuExtPacketHandler* pPacketHandlerHead;
} MpuExt;

void MpuExtInit(MpuExt* pExt, UartDma* pUart, tprio_t prio);
int16_t MpuExtSendPacket(MpuExt* pExt, ExtPacketHeader* pHeader, const void* _pData, size_t dataLength);
void MpuExtAddPacketHandler(MpuExt* pExt, MpuExtPacketHandler* pHandler);
