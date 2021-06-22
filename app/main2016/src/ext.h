/*
 * ext.h
 *
 *  Created on: 31.05.2017
 *      Author: AndreR
 */

#pragma once

#include "ch.h"
#include "constants.h"
#include "commands.h"

typedef struct _Ext
{
	uint8_t txProcessingBuf[UART8_MAX_PACKET_SIZE];
	mutex_t sendPacketMutex;
} Ext;

extern Ext ext;

void	ExtInit();
int16_t ExtSendPacket(PacketHeader* pHeader, const void* _pData, uint16_t dataLength);
