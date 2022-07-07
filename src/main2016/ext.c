/*
 * ext.c
 *
 *  Created on: 31.05.2017
 *      Author: AndreR
 */

#include "ext.h"
#include "hal/uart8.h"
#include "util/log.h"
#include "robot/robot.h"
#include "errors.h"
#include "util/sys_time.h"
#include "util/console.h"

Ext ext;

static void usart8Callback(void* pUserData)
{
	(void)pUserData;

	uint8_t* pData;
	uint32_t dataSize;

	while(FifoLinGet(&uart8.uart.rxFifo, &pData, &dataSize) == 0)
	{
		PacketHeader* pHeader = (PacketHeader*)pData;
		pData += sizeof(PacketHeader);
		dataSize -= sizeof(PacketHeader);

		if(pHeader->section == SECTION_SYSTEM && pHeader->cmd == CMD_SYSTEM_MATCH_CTRL)
		{
			if(dataSize < sizeof(SystemMatchCtrl)-SYSTEM_MATCH_CTRL_USER_DATA_SIZE)
			{
				LogWarnC("Invalid match ctrl size from EXT", dataSize);
			}
			else
			{
				memcpy(&robot.ext.matchCtrl, pData, dataSize);
				robot.ext.lastMatchCtrlTime = SysTimeUSec();
			}
		}

		FifoLinDelete(&uart8.uart.rxFifo);
	}
}

void ExtInit()
{
	chMtxObjectInit(&ext.sendPacketMutex);

	uart8.uart.callback = &usart8Callback;
}

int16_t ExtSendPacket(PacketHeader* pHeader, const void* _pData, uint16_t dataLength)
{
	if(!uart8.extInstalled)
		return ERROR_INVALID_HANDLE;

	uint8_t* pData = (uint8_t*)_pData;

	if(dataLength + PACKET_HEADER_SIZE > UART8_MAX_PACKET_SIZE)
		return ERROR_NOT_ENOUGH_MEMORY;

	chMtxLock(&ext.sendPacketMutex);	// prevent re-entering

	memcpy(ext.txProcessingBuf, pHeader->_data, PACKET_HEADER_SIZE);
	memcpy(ext.txProcessingBuf + PACKET_HEADER_SIZE, pData, dataLength);

	// data to send is now in ext.txProcessingBuf
	int16_t result = USARTSend(&uart8.uart, dataLength+PACKET_HEADER_SIZE, ext.txProcessingBuf);
	if(result)
	{
		LogErrorC("USARTSend error", result);
	}

	chMtxUnlock(&ext.sendPacketMutex);

	return result;
}
