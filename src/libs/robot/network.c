/*
 * network.c
 *
 *  Created on: 01.07.2010
 *      Author: AndreR
 */

#include "ctrl.h"
#include "ctrl_motor.h"
#include "ctrl_tigga.h"
#include "network.h"
#include "skills.h"
#include "util/bits.h"
#include "util/float16_t.h"
#include "util/boot.h"
#include "util/sys_time.h"
#include "util/flash_fs.h"
#include "util/init_hal.h"
#include "util/log.h"
#include "robot.h"
#include "util/crc8.h"
#include "util/fw_updater.h"
#include "util/fw_loader_wifi.h"

#include "errors.h"
#include "commands.h"
#include "intercom_constants.h"
#include "struct_ids.h"

#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>

#define EVENT_REL_ACK 0x10000
#define EVENT_REL_RUN 0x20000

#define PROCESSED 0
#define PASS_ON 1

Network network = {
	.config = { 0xFF, 100 },
};

static const ConfigFileDesc configFileDescSettings =
	{ SID_CFG_NET_SETTINGS, 3, "net/config", 2, (ElementDesc[]) {
		{ UINT8, "bot_id", "", "Bot ID" },
		{ UINT8, "channel", "", "Channel" },
	} };

static uint8_t processPacket(PacketHeader* pHeader, uint8_t* pBuf, uint16_t dataLength);

static void configUpdateCallback(uint16_t cfgId)
{
	switch(cfgId)
	{
		case SID_CFG_NET_SETTINGS:
		{
			NetworkImplSetAddress(network.config.botId);
			NetworkImplSetChannel(network.config.channel);

			CtrlUpdateBotId(network.config.botId);
		}
		break;
	}
}

void NetworkInit()
{
	// data setup
	chMtxObjectInit(&network.printMutex);
	chMtxObjectInit(&network.sendPacketMutex);

	EMAFilterInit(&network.avgRxPeriod, 0.96f);
	EMAFilterInit(&network.avgRssi, 0.9f);

	FifoLinInit(&network.reliableCmds, network.reliableBuf, NETWORK_RELIABLE_BUF_SIZE);
	chMBObjectInit(&network.relEvents, network.relEventsData, NETWORK_REL_EVENT_SIZE);

	network.linkDisabled = 1;

	network.pNetworkConfig = ConfigOpenOrCreate(&configFileDescSettings, &network.config, sizeof(ConfigNetwork), &configUpdateCallback, 1);

	network.lastUSec = 0;

	static uint8_t rxData[WIFI_RX_SIZE];
	static uint8_t txData[WIFI_TX_SIZE];
	RFQueueInit(&network.queue, txData, WIFI_TX_SIZE, rxData, WIFI_RX_SIZE, 32, 68);

	NetworkImplInit();
	NetworkImplSetAddress(network.config.botId);
	NetworkImplSetChannel(network.config.channel);

	ConfigNotifyUpdate(network.pNetworkConfig);

	network.matchCtrlTime = SysTimeUSec()-5000000;

	network.lastMatchCtrlCmd.curPosition[0] = SYSTEM_MATCH_CTRL_UNUSED_FIELD;
	network.lastMatchCtrlCmd.curPosition[1] = SYSTEM_MATCH_CTRL_UNUSED_FIELD;
	network.lastMatchCtrlCmd.curPosition[2] = SYSTEM_MATCH_CTRL_UNUSED_FIELD;
}

void NetworkSaveConfig()
{
	ConfigNotifyUpdate(network.pNetworkConfig);
}

void NetworkTask(void* params)
{
	(void)params;

	chRegSetThreadName("Network");

	int16_t result;
	network.avgRxPeriod.value = 0.1f;

	while(1)
	{
		if(network.linkDisabled)
		{
			chThdSleep(MS2ST(100));
			continue;
		}

		uint32_t tNow = SysTimeUSec();
		uint32_t tPeriod = tNow - network.lastUSec;
		network.lastUSec = tNow;
		EMAFilterUpdate(&network.avgRxPeriod, tPeriod*1e-6f);

		static uint8_t data[WIFI_PACKET_SIZE];
		SX1280RxResult rxResult;

		memset(data, 0, WIFI_PACKET_SIZE);
		uint32_t packetSize = RFQueueFetchTxPacket(&network.queue, data+2);
		data[0] = packetSize;
		data[1] = (uint8_t)(network.avgRssi.value*-2.0f);
		data[WIFI_PACKET_SIZE-1] = Crc8(data, WIFI_PACKET_SIZE-1);

		NetworkImplReceiveAndTransmit(WIFI_PACKET_SIZE, data, &rxResult, WIFI_PACKET_SIZE, data);

		if(rxResult.crcError == 0 && rxResult.abortError == 0 && rxResult.bytesReceived == WIFI_PACKET_SIZE
			&& Crc8(data, WIFI_PACKET_SIZE-1) == data[WIFI_PACKET_SIZE-1])
		{
			// successful reception
//			float bsRssi = data[1]*-0.5f;
			EMAFilterUpdate(&network.avgRssi, rxResult.rssiSync*-0.5f);
			RFQueueFeedRxPacket(&network.queue, data+2, data[0]);
		}

		uint32_t numDelimiters = network.queue.rxFifo.numPackets;
		for(uint32_t i = 0; i < numDelimiters; i++)
		{
			uint32_t len;
			result = RFQueueGetPacket(&network.queue, network.rxProcessingBuf, NETWORK_RX_PROCESSING_SIZE, &len);
			if(result)
			{
				LogErrorC("GetPacket", result);
				break;
			}

			if(len > PACKET_HEADER_SIZE)
			{
				PacketHeader* pHeader = (PacketHeader*)network.rxProcessingBuf;
				uint8_t* pBuf = network.rxProcessingBuf+PACKET_HEADER_SIZE;
				uint16_t dataLength = len-PACKET_HEADER_SIZE;

				// Handle extended packets
				if(pHeader->section & SECTION_EXTENDED_PACKET)
				{
					PacketHeaderEx* pHeaderEx = (PacketHeaderEx*)pHeader;

					PacketHeader ack;
					ack.cmd = CMD_SYSTEM_ACK;
					ack.section = SECTION_SYSTEM;

					NetworkSendPacket(&ack, &pHeaderEx->seq, sizeof(uint16_t));

					uint8_t* pBufOrig = pBuf;
					pHeader->section &= ~SECTION_EXTENDED_PACKET;
					pBuf += PACKET_HEADER_EX_SIZE-PACKET_HEADER_SIZE;
					dataLength -= PACKET_HEADER_EX_SIZE-PACKET_HEADER_SIZE;

					memcpy(pBufOrig, pHeader, sizeof(PacketHeader));
					pHeader = (PacketHeader*)pBufOrig;
				}

				if(processPacket(pHeader, pBuf, dataLength) == PASS_ON)
				{
					NetworkImplProcessPacket(pHeader, pBuf, dataLength);
				}
			}
		}
	}
}

void NetworkReliableTask(void* params)
{
	(void)params;
	msg_t event;
	msg_t chResult;

	uint8_t* pBuf = 0;
	PacketHeaderEx* pHeaderEx = 0;
	uint32_t cmdSize;
	uint32_t lastSendTime = chVTGetSystemTimeX()-MS2ST(200);

	chRegSetThreadName("NetRel");

	while(1)
	{
		chResult = chMBFetch(&network.relEvents, &event, MS2ST(10));

		if(FifoLinGet(&network.reliableCmds, &pBuf, &cmdSize) == 0)
			pHeaderEx = (PacketHeaderEx*)pBuf;
		else
			pHeaderEx = 0;

		if(chResult == MSG_OK && (event & EVENT_REL_ACK) && pHeaderEx)
		{
			if((event & 0xFFFF) == pHeaderEx->seq)
			{
				FifoLinDelete(&network.reliableCmds);

				if(FifoLinGet(&network.reliableCmds, &pBuf, &cmdSize) == 0)
					pHeaderEx = (PacketHeaderEx*)pBuf;
				else
					pHeaderEx = 0;

				lastSendTime = chVTGetSystemTimeX()-MS2ST(200);
			}
		}

		if(pHeaderEx && chVTTimeElapsedSinceX(lastSendTime) > MS2ST(100))
		{
			chMtxLock(&network.sendPacketMutex);	// prevent re-entering

			// data to send is now in network.txProcessingBuf
			int16_t result = RFQueueSend(&network.queue, pBuf, cmdSize);
			if(result)
			{
				FifoLinClear(&network.queue.txFifo);
			}

			chMtxUnlock(&network.sendPacketMutex);

			lastSendTime = chVTGetSystemTimeX();
		}
	}
}

int16_t NetworkSendPacket(const PacketHeader* pHeader, const void* _pData, uint16_t dataLength)
{
	const uint8_t* pData = (const uint8_t*)_pData;

	if(dataLength + PACKET_HEADER_SIZE > NETWORK_TX_PROCESSING_SIZE)
		return ERROR_NOT_ENOUGH_MEMORY;

	chMtxLock(&network.sendPacketMutex);	// prevent re-entering

	memcpy(network.txProcessingBuf, pHeader->_data, PACKET_HEADER_SIZE);
	memcpy(network.txProcessingBuf + PACKET_HEADER_SIZE, pData, dataLength);

	// data to send is now in network.txProcessingBuf
	int16_t result = RFQueueSend(&network.queue, network.txProcessingBuf, dataLength+PACKET_HEADER_SIZE);
	if(result)
	{
		LogErrorC("NRF24QueueSend error", result);
	}
	if(result)
	{
		FifoLinClear(&network.queue.txFifo);
		LogWarnC("txFifo full => cleared", pHeader->section << 8 | pHeader->cmd);
	}

	chMtxUnlock(&network.sendPacketMutex);

	return result;
}

void NetworkSendPacketRaw(void* _pData, uint16_t dataLength)
{
	uint8_t* pData = (uint8_t*)_pData;

	chMtxLock(&network.sendPacketMutex);	// prevent re-entering

	// data to send is now in network.txProcessingBuf
	int16_t result = RFQueueSend(&network.queue, pData, dataLength);
	if(result)
	{
		LogErrorC("NRF24QueueSend error", result);
	}
	if(result)
	{
		FifoLinClear(&network.queue.txFifo);
		LogWarn("txFifo full => cleared\r\n");
	}

	chMtxUnlock(&network.sendPacketMutex);
}

int16_t NetworkSendPacketReliable(const PacketHeader* pHeader, const void* _pData, uint16_t dataLength)
{
	int16_t result;
	uint8_t* pBuf;
	const uint8_t* pData = (const uint8_t*)_pData;

	result = FifoLinReserve(&network.reliableCmds, dataLength+PACKET_HEADER_EX_SIZE, &pBuf);
	if(result)
		return result;

	PacketHeaderEx* pHeaderEx = (PacketHeaderEx*)pBuf;
	pHeaderEx->cmd = pHeader->cmd;
	pHeaderEx->section = pHeader->section | SECTION_EXTENDED_PACKET;
	pHeaderEx->seq = network.seq++;

	memcpy(pBuf+PACKET_HEADER_EX_SIZE, pData, dataLength);

	result = FifoLinCommit(&network.reliableCmds, dataLength+PACKET_HEADER_EX_SIZE);
	if(result)
		return result;

	chMBPost(&network.relEvents, EVENT_REL_RUN, TIME_IMMEDIATE);

	return 0;
}

// network pre-processing function
// returns 0 if packet has been fully handled and should not be passed on to implementation
static uint8_t processPacket(PacketHeader* pHeader, uint8_t* pBuf, uint16_t dataLength)
{
	switch(pHeader->section)
	{
		case SECTION_SYSTEM:
		{
			switch(pHeader->cmd)
			{
				case CMD_SYSTEM_PING:
				{
					PacketHeader header;
					header.section = SECTION_SYSTEM;
					header.cmd = CMD_SYSTEM_PONG;

					NetworkSendPacket(&header, pBuf, dataLength);

					return PROCESSED;
				}
				break;
				case CMD_SYSTEM_ACK:
				{
					uint16_t* pSeq = (uint16_t*)pBuf;

					chMBPost(&network.relEvents, EVENT_REL_ACK + *pSeq, TIME_IMMEDIATE);

					return PROCESSED;
				}
				break;
				case CMD_SYSTEM_MATCH_CTRL:
				{
					SystemMatchCtrl* pCtrl = (SystemMatchCtrl*)pBuf;

					memcpy(&network.lastMatchCtrlCmd, pCtrl, sizeof(SystemMatchCtrl));
					network.matchCtrlTime = SysTimeUSec();

					return PROCESSED;
				}
				break;
				case CMD_SYSTEM_TIMESYNC:
				{
					if(dataLength < sizeof(uint32_t))
					{
						LogErrorC("timesync too short", dataLength);
						return PROCESSED;
					}

					uint32_t* pUnixTime = (uint32_t*)pBuf;

					sysTime.unixOffset = *pUnixTime;

					return PROCESSED;
				}
				break;
				case CMD_SYSTEM_VERSION:
				{
					RobotSendVersionInfo();

					return PROCESSED;
				}
				break;
				default:
					return PASS_ON;
			}
		}
		break;
		case SECTION_BOOTLOADER:
		{
			if(pHeader->cmd == CMD_BOOTLOADER_CHECK)
			{
				FwUpdaterLoad(FW_UPDATER_SOURCE_WIFI);
			}
			else
			{
				FwLoaderWifiNetworkInput(pHeader, dataLength, pBuf);
			}

			return PROCESSED;
		}
		break;
		case SECTION_CTRL:
		{
			switch(pHeader->cmd)
			{
				case CMD_CTRL_SET_CONTROLLER_TYPE:
				{
					CtrlSetControllerType* pCtrl = (CtrlSetControllerType*)pBuf;

					switch(pCtrl->controllerId)
					{
						case CTRL_TYPE_CALIBRATE:
						{
						}
						break;
						case CTRL_TYPE_MOTOR:
						{
							CtrlSetController(&ctrlMotorInstance);
						}
						break;
						case CTRL_TYPE_FUSION:
						{
						}
						break;
						case CTRL_TYPE_TIGGA:
						{
							CtrlSetController(&ctrlTiggaInstance);
						}
						break;
						default:
						{
							CtrlSetController(0);
						}
						break;
					}
				}
				break;
				default:
					return PASS_ON;
			}
		}
		break;
		case SECTION_CONFIG:
		{
			return PASS_ON;
		}
		break;
		case SECTION_DATA_ACQ:
		{
			if(pHeader->cmd == CMD_DATA_ACQ_SET_MODE)
			{
				DataAcqSetMode* pMode = (DataAcqSetMode*)pBuf;
				robot.dataAcquisitionMode = pMode->mode;

				return PROCESSED;
			}

			return PASS_ON;
		}
		default:
			return PASS_ON;
	}

	return 0;
}
