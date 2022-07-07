/*
 * network.h
 *
 *  Created on: 23.10.2017
 *      Author: AndreR
 */

#pragma once

#include "hal/eth.h"
#include "network/inet.h"
#include "commands.h"
#include "constants.h"
#include "util/flash_fs.h"
#include "ch.h"
#include "wifi.h"

#define NETWORK_MAX_FRAME_SIZE 1524
#define NETWORK_EVENT_QUEUE_SIZE 4

typedef int16_t(*UDPCallback)(uint8_t* pData, uint16_t dataLength);
typedef int16_t(*VisionCallback)(uint8_t* pData, uint16_t dataLength);

typedef struct _NetworkConfig
{
	struct _my
	{
		MAC mac;
		uint16_t port;
		IPv4Address ip;
	} my;

	struct _vision
	{
		IPv4Address ip;		// 224.5.23.2
		uint16_t port;		// 10002
	} vision;
} NetworkConfig;

typedef struct _Network
{
	uint8_t rxProcBuf[NETWORK_MAX_FRAME_SIZE];
	uint8_t txProcBuf[NETWORK_MAX_FRAME_SIZE];

	mutex_t sendPacketMutex;
	mailbox_t eventQueue;
	msg_t eventQueueData[NETWORK_EVENT_QUEUE_SIZE];

	uint32_t lastVisionTime;
	uint8_t visionAvailable;

	uint32_t lastSumatraTime;
	uint8_t sumatraAvailable;

	uint16_t ethernetIdent;

	BaseStationEthStats bsEth;
	BaseStationWifiStats bsWifi;

	NetworkConfig cfg;	// own values used as source

	struct	// values used as unicast destination
	{
		MAC mac;
		IPv4Address ip;
		uint16_t port;
	} ucast;

	MAC visionMAC; // 01-00-5E-05-17-02

	struct
	{
		uint32_t respTime;
		uint8_t respCounter;
		uint32_t lastSendTime;
		uint8_t robustness;
	} igmp;

	struct
	{
		uint16_t port;
		uint8_t requestState;
		systime_t lastQueryTime;
		systime_t lastResponseTime;
		uint8_t synced;
		uint8_t timeoutCnt;
	} sntp;

	FlashFile* pCfgFile;
} Network;

extern Network network;

void	NetworkInit();
void	NetworkTask(void* params);
void	NetworkSetMyIP(IPv4Address ip);
void	NetworkSetMyPort(uint16_t port);
void	NetworkSetMyMAC(uint8_t inc);
int16_t	NetworkSendPacket(const PacketHeader* pHeader, const uint8_t* pData, uint16_t dataLength);
void	NetworkSetUDPCallback(UDPCallback cb);
void	NetworkSetVisionCallback(VisionCallback cb);
int16_t NetworkSendIGMPv2MembershipReport(const IPv4Address* pMulticaseAddress);
int16_t	NetworkSendIGMPv3MembershipReport(uint16_t numberOfGroupRecords, uint8_t* recordData, uint16_t recordDataLength);
int16_t NetworkSendSNTPRequest(uint32_t unixTransmitTimestamp, IPv4Address dstIp, MAC dstMAC);
void	NetworkSaveConfig();
void	NetworkSetVisionPort(uint16_t port);
void	NetworkSetVisionIP(IPv4Address ip);
void	NetworkPrintConfig();
