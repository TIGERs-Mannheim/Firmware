/*
 * network.h
 *
 *  Created on: 01.07.2010
 *      Author: AndreR
 */

#pragma once

#include "util/flash_fs.h"
#include "util/fifo_lin.h"
#include "util/config.h"
#include "util/ema_filter.h"
#include "util/sx1280.h"
#include "util/rf_queue.h"

#include "commands.h"
#include "intercom_constants.h"

#include "ch.h"

// Configuration
#define NETWORK_TX_PROCESSING_SIZE	1000
#define NETWORK_RX_PROCESSING_SIZE	1000
#define NETWORK_PRINT_SIZE 100
#define NETWORK_RELIABLE_BUF_SIZE 400

#define WIFI_PACKET_SIZE 35

#define WIFI_TX_SIZE 2048
#define WIFI_RX_SIZE 2048

#define NETWORK_REL_EVENT_SIZE 4

typedef struct _Network
{
	uint8_t txProcessingBuf[NETWORK_TX_PROCESSING_SIZE];
	uint8_t rxProcessingBuf[NETWORK_RX_PROCESSING_SIZE];

	uint8_t netPrint[NETWORK_PRINT_SIZE];

	uint8_t reliableBuf[NETWORK_RELIABLE_BUF_SIZE];
	FifoLin reliableCmds;
	uint16_t seq;
	msg_t relEventsData[NETWORK_REL_EVENT_SIZE];
	mailbox_t relEvents;

	mutex_t sendPacketMutex;
	mutex_t printMutex;

	RFQueue queue;
	EMAFilter avgRssi;

	uint32_t lastUSec;

	SystemMatchCtrl lastMatchCtrlCmd;
	uint32_t matchCtrlTime;

	ConfigNetwork config;
	ConfigFile* pNetworkConfig;

	EMAFilter avgRxPeriod;

	uint8_t linkDisabled;
} Network;

extern Network network;

void	NetworkInit();
void	NetworkTask(void* params);
void	NetworkReliableTask(void* params);
int16_t NetworkSendPacket(const PacketHeader* pHeader, const void* _pData, uint16_t dataLength);
void	NetworkSendPacketRaw(void* _pData, uint16_t dataLength);
int16_t NetworkSendPacketReliable(const PacketHeader* pHeader, const void* _pData, uint16_t dataLength);
void	NetworkSaveConfig();

void	NetworkImplInit();
int16_t NetworkImplReceiveAndTransmit(uint8_t rxBytes, void* pRxData, SX1280RxResult* pResult, uint8_t txBytes, const void* pTxData);
void	NetworkImplProcessPacket(PacketHeader* pHeader, uint8_t* pBuf, uint16_t dataLength);
int16_t NetworkImplSetChannel(uint8_t channel);
int16_t NetworkImplSetAddress(uint8_t address);
