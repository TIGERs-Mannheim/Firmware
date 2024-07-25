#include "mpu_ext.h"
#include "util/log.h"
#include "util/crc.h"
#include "hal/sys_time.h"
#include "errors.h"
#include <string.h>

#define EVENT_MASK_UART	EVENT_MASK(0)

static void mpuExtTask(void* params);
static void processReceivedData(MpuExt* pExt);

void MpuExtInit(MpuExt* pExt, UartDma* pUart, tprio_t prio)
{
	chMtxObjectInit(&pExt->txAssemblyMutex);
	chEvtObjectInit(&pExt->eventSource);

	pExt->pUart = pUart;
	pExt->pPacketHandlerHead = 0;

	pExt->pTask = chThdCreateStatic(pExt->waTask, sizeof(pExt->waTask), prio, &mpuExtTask, pExt);
}

int16_t MpuExtSendPacket(MpuExt* pExt, PacketHeader* pHeader, const void* _pData, size_t dataLength)
{
	if(dataLength > MPU_EXT_MAX_PAYLOAD_SIZE)
		return ERROR_NOT_ENOUGH_MEMORY;

	uint8_t* pData = (uint8_t*)_pData;

	chMtxLock(&pExt->txAssemblyMutex);

	pExt->liveStats.txPayload += MPU_EXT_HEADER_SIZE + dataLength;

	// Compute CRC of header and data
	uint32_t crc = CRC32StartChecksum();
	crc = CRC32FeedChecksum(crc, pHeader, MPU_EXT_HEADER_SIZE);
	crc = CRC32FeedChecksum(crc, pData, dataLength);
	crc = CRC32StopChecksum(crc);

	// Do step-by-step COBS encoding to transmit buffer and append zero for packet termination
	COBSState cobs;
	COBSStartEncode(&cobs, MPU_EXT_HEADER_SIZE+dataLength+MPU_EXT_TRAILER_SIZE, pExt->txAssemblyBuf, sizeof(pExt->txAssemblyBuf));
	COBSFeedEncode(&cobs, pHeader, MPU_EXT_HEADER_SIZE);
	COBSFeedEncode(&cobs, pData, dataLength);
	COBSFeedEncode(&cobs, &crc, MPU_EXT_TRAILER_SIZE);
	uint32_t bytesEncoded = 0;
	COBSFinalizeEncode(&cobs, &bytesEncoded);

	pExt->txAssemblyBuf[bytesEncoded++] = 0;

	pExt->liveStats.txEncoded += bytesEncoded;

	// Send data via UART, if there is not enough space the whole packet will be lost
	size_t bytesWritten = UartDmaWrite(pExt->pUart, pExt->txAssemblyBuf, bytesEncoded);

	pExt->liveStats.txWire += bytesWritten;

	chMtxUnlock(&pExt->txAssemblyMutex);

	return 0;
}

void MpuExtAddPacketHandler(MpuExt* pExt, MpuExtPacketHandler* pHandler)
{
	if(pExt->pPacketHandlerHead == 0)
	{
		pExt->pPacketHandlerHead = pHandler;
	}
	else
	{
		MpuExtPacketHandler* pLast = pExt->pPacketHandlerHead;
		while(pLast->pNext)
			pLast = pLast->pNext;

		pLast->pNext = pHandler;
	}

	pHandler->pNext = 0;
	chMtxObjectInit(&pHandler->storageMutex);
}

static void mpuExtTask(void* params)
{
	MpuExt* pExt = (MpuExt*)params;

	chRegSetThreadName("MPU_EXT");

	event_listener_t uartListener;
	chEvtRegisterMask(&pExt->pUart->eventSource, &uartListener, EVENT_MASK_UART);

	uint32_t last1sTime = SysTimeUSec();

	while(1)
	{
		// Execute on any UART event or every 10ms
		eventmask_t events = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(10));

		if(events & EVENT_MASK_UART)
		{
			processReceivedData(pExt);
		}

		// Update statistics
		uint32_t tNow_us = SysTimeUSec();
		uint32_t statsTDelta_us = tNow_us - last1sTime;

		if(statsTDelta_us > 1000000)
		{
			last1sTime = tNow_us;

			float dt_s = statsTDelta_us * 1e-6f;

			pExt->stats.rxWire = pExt->liveStats.rxWire / dt_s;
			pExt->stats.rxDecoded = pExt->liveStats.rxDecoded / dt_s;
			pExt->stats.rxPayload = pExt->liveStats.rxPayload / dt_s;
			pExt->stats.txPayload = pExt->liveStats.txPayload / dt_s;
			pExt->stats.txEncoded = pExt->liveStats.txEncoded / dt_s;
			pExt->stats.txWire = pExt->liveStats.txWire / dt_s;
			memset(&pExt->liveStats, 0, sizeof(MpuExtStats));
		}
	}
}

static void processReceivedData(MpuExt* pExt)
{
	// Check how many bytes can be read and processed from UART
	size_t rxBytesFree = sizeof(pExt->rxProcessingBuf) - pExt->rxProcBufUsed;
	uint8_t* pRx = &pExt->rxProcessingBuf[pExt->rxProcBufUsed];

	size_t bytesRead = UartDmaRead(pExt->pUart, pRx, rxBytesFree);
	pExt->rxProcBufUsed += bytesRead;

	pExt->liveStats.rxWire += bytesRead;

	// process received bytes
	for(size_t i = 0; i < pExt->rxProcBufUsed; i++)
	{
		uint8_t c = pExt->rxProcessingBuf[i];

		if(c == 0)
		{
			// packet complete
			uint32_t decodedSize;
			if(COBSDecode(pExt->rxProcessingBuf, i, pExt->cobsProcBuf, sizeof(pExt->cobsProcBuf), &decodedSize) == 0)
			{
				pExt->liveStats.rxDecoded += decodedSize;

				if(decodedSize < (MPU_EXT_HEADER_SIZE + MPU_EXT_TRAILER_SIZE))
				{
					LogWarnC("Short packet", decodedSize);
				}
				else if(CRC32CalcChecksum(pExt->cobsProcBuf, decodedSize) != CRC32_MAGIC_NUMBER)
				{
					LogWarnC("CRC error", decodedSize);
				}
				else
				{
					pExt->liveStats.rxPayload += decodedSize - MPU_EXT_TRAILER_SIZE;
					pExt->isInstalled = 1;

					PacketHeader* pHeader = (PacketHeader*)pExt->cobsProcBuf;
					uint8_t* pData = pExt->cobsProcBuf + MPU_EXT_HEADER_SIZE;
					size_t dataLength = decodedSize - MPU_EXT_TRAILER_SIZE - MPU_EXT_HEADER_SIZE;

					// Iterate over all handlers
					for(MpuExtPacketHandler* pHandler = pExt->pPacketHandlerHead; pHandler; pHandler = pHandler->pNext)
					{
						// If there is a command match => copy data
						if(pHandler->cmd.section == pHeader->section && pHandler->cmd.cmd == pHeader->cmd && pHandler->storageSize >= dataLength)
						{
							chMtxLock(&pHandler->storageMutex);
							pHandler->updateTimestamp_us = SysTimeUSec();
							memcpy(pHandler->pStorage, pData, dataLength);
							pHandler->updated = 1;
							chMtxUnlock(&pHandler->storageMutex);
						}
					}

					// Inform listeners about new packet
					chEvtBroadcastFlags(&pExt->eventSource, MPU_EXT_EVENT_PACKET_RECEIVED);
				}
			}
			else
			{
				LogWarn("COBS decode error");
			}

			memmove(pExt->rxProcessingBuf, pExt->rxProcessingBuf + i + 1, pExt->rxProcBufUsed - (i+1));
			pExt->rxProcBufUsed -= i+1;
			i = -1;
		}
	}

	if(pExt->rxProcBufUsed == sizeof(pExt->rxProcessingBuf))
	{
		// we filled the whole buffer without delimiter => start again
		pExt->rxProcBufUsed = 0;
		LogWarn("rx proc buf overrun");
	}
}
