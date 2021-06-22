/*
 * network.c
 *
 *  Created on: 05.01.2013
 *      Author: AndreR
 */

#include "network.h"

#include "wifi.h"
#include "constants.h"
#include "util/console.h"
#include "util/log.h"
#include "util/sys_time.h"
#include "hub.h"
#include "commands.h"

#include "network/inet.h"
#include "network/ethernet.h"
#include "network/arp.h"
#include "network/ipv4.h"
#include "network/udp.h"
#include "network/icmp.h"
#include "network/igmpv3.h"
#include "network/sntp.h"
#include "vision.h"

#include <string.h>

Network network = { .cfg = {
	.my = {
		.mac.u8 = { 2, 84, 73, 71, 65, 48 }, // ASCII: 02-T-I-G-A-0
		.ip.u8 = { 192, 168, 1, 210 },
		.port = 10201,
	},
	.vision = {
		.ip.u8 = { 224, 5, 23, 2 },
		.port = 10002,
	},
}, };

#define EVENT_LINK 0
#define EVENT_RX_DONE 1
#define EVENT_BOT_DATA 0x800

#define NTP_REQUEST_NONE	0
#define NTP_REQUEST_ACTIVE	1
#define NTP_REQUEST_ACK		2
#define NTP_REQUEST_TIMEOUT	3

int16_t NetworkSendARP(uint8_t operation, MAC targetMAC, IPv4Address targetIP);
int16_t NetworkSendICMPEchoReply(ICMPEcho* pEchoRequest, uint8_t* pData, uint16_t dataLength, MAC targetMAC, IPv4Address dstIp);

static void linkCallback()
{
	chSysLockFromISR();
	chMBPostI(&network.eventQueue, EVENT_LINK);
	chSysUnlockFromISR();
}

static void rxCallback()
{
	chSysLockFromISR();
	chMBPostI(&network.eventQueue, EVENT_RX_DONE);
	chSysUnlockFromISR();
}

static void botDataCallback(uint8_t id)
{
	chMBPost(&network.eventQueue, EVENT_BOT_DATA+id, TIME_IMMEDIATE);
}

void NetworkSaveConfig()
{
	FlashFSWrite(&network.pCfgFile, &network.cfg, sizeof(NetworkConfig));
}

void NetworkInit()
{
	chMtxObjectInit(&network.sendPacketMutex);
	chMBObjectInit(&network.eventQueue, network.eventQueueData, NETWORK_EVENT_QUEUE_SIZE);

	FlashFSOpenOrCreate("net/cfg", 3, &network.cfg, sizeof(NetworkConfig), &network.pCfgFile);

	network.ethernetIdent = 0;
	network.visionAvailable = 0;

	network.ucast.mac = MACSet(2, 0, 0, 0, 0, 0);
	network.ucast.ip = IPv4AddressSet(0, 0, 0, 0);
	network.ucast.port = 0;

	network.visionMAC = IPv4CalcMulticastMAC(network.cfg.vision.ip);

	network.igmp.respCounter = 0;
	network.igmp.respTime = 500;
	network.igmp.lastSendTime = 0;
	network.igmp.robustness = 2;

	network.sntp.port = 10204;

	MAC igmpMAC = MACSet(0x01, 0x00, 0x5E, 0x00, 0x00, 0x01);	// 224.0.0.1

	EthSetCallbacks(&linkCallback, &rxCallback);
	WifiSetBotDataCallback(&botDataCallback);

	EthAllowMAC(0, network.cfg.my.mac.u8, 1, 0);
	EthAllowMAC(1, igmpMAC.u8, 1, 0);
	EthAllowMAC(2, network.visionMAC.u8, 1, 0);
}

static int16_t unicastUdp(uint8_t* pData, uint16_t dataLength)
{
	(void)dataLength;

	// pData should point to a BaseStationACommand packet
	PacketHeader* pHeader = (PacketHeader*)pData;
	if(pHeader->section != SECTION_BASE_STATION)
	{
		ConsolePrint("Invalid section 0x%X\r\n", pHeader->section);
		return 0;
	}

	pData += PACKET_HEADER_SIZE;
	dataLength -= PACKET_HEADER_SIZE;

	switch(pHeader->cmd)
	{
		case CMD_BASE_STATION_ACOMMAND:
		{
			network.lastSumatraTime = chVTGetSystemTimeX();
			network.sumatraAvailable = 1;

			BaseStationACommand* pCmd = (BaseStationACommand*)pData;

			uint16_t botId = pCmd->id;
			pData++;
			dataLength--;

			HubNetworkInput(botId, pData, dataLength);
		}
		break;
		case CMD_BASE_STATION_PING:
		{
			int16_t result = NetworkSendPacket(pHeader, pData, dataLength);
			if(result)
			{
				ConsolePrint("NetworkSendPacket failed 0x%hX\r\n", result);
			}
		}
		break;
		case CMD_BASE_STATION_CONFIG_V2:
		{
			BaseStationConfigV2* pConfig = (BaseStationConfigV2*)pData;

			NetworkSetVisionIP(IPv4AddressSet(pConfig->visionIp[0], pConfig->visionIp[1], pConfig->visionIp[2], pConfig->visionIp[3]));
			NetworkSetVisionPort(pConfig->visionPort);

			WifiSetChannel(pConfig->module[0].channel);
			WifiSetMaxBot(pConfig->module[0].maxBots);
			WifiSetFixedRuntime(pConfig->module[0].fixedRuntime);
			WifiSetTimeout(pConfig->module[0].timeout);

			NetworkSaveConfig();
			WifiSaveConfig();
		}
		break;
		case CMD_BASE_STATION_CONFIG_V3:
		{
			BaseStationConfigV3* pConfig = (BaseStationConfigV3*)pData;

			NetworkSetVisionIP(IPv4AddressSet(pConfig->visionIp[0], pConfig->visionIp[1], pConfig->visionIp[2], pConfig->visionIp[3]));
			NetworkSetVisionPort(pConfig->visionPort);

			WifiSetChannel(pConfig->channel);
			WifiSetMaxBot(pConfig->maxBots);
			WifiSetFixedRuntime(pConfig->fixedRuntime);

			NetworkSaveConfig();
			WifiSaveConfig();
		}
		break;
		case CMD_BASE_STATION_CAM_VIEWPORT:
		{
			BaseStationCamViewport* pViewport = (BaseStationCamViewport*)pData;

			VisionUpdateViewport(pViewport);
		}
		break;
		default:
		{
			ConsolePrint("Invalid command 0x%X\r\n", pHeader->cmd);
			return 0;
		}
		break;
	}

	return 0;
}

static int16_t decodeFrame(uint8_t* pDecoded, uint16_t length)
{
	FrameHeader* pFrameHeader;

	int16_t result = EthernetDecode(pDecoded, length, &pDecoded, &length, &pFrameHeader);
	if(result)
	{
		return result;
	}

	if(pFrameHeader->dstMAC.cast.multicast == 0 && MACCompare(pFrameHeader->dstMAC, network.cfg.my.mac) == 0)
		return 0; // this should not happen, as we have a hardware MAC filter

//	ConsolePrint("A packet for us :)\r\n");

	if(pFrameHeader->type == ETHERNET_PROTOCOL_ARP)
	{
		ARPPacketIPv4* pARP;

		result = ARPDecode(pDecoded, length, &pARP);
		if(result)
			return result;

//		ConsolePrint("ARP\r\n");

		if(	pARP->operation == ARP_OPERATION_REQUEST && pARP->dstProtAddr.u32 == network.cfg.my.ip.u32)
		{
			// someone wants to know our MAC address
//			ConsolePrint("Replying to ARP!\r\n");

			return NetworkSendARP(ARP_OPERATION_REPLY, pARP->srcHwAddr, pARP->srcProtAddr);
		}
	}

	if(pFrameHeader->type == ETHERNET_PROTOCOL_IPV4)
	{
		IPv4Header* pIPHeader;

		result = IPv4Decode(pDecoded, length, &pDecoded, &length, &pIPHeader);
		if(result)
			return result;

		if(	pIPHeader->dstIP.u32 != network.cfg.vision.ip.u32 && pIPHeader->dstIP.u32 != network.cfg.my.ip.u32 &&
			pIPHeader->dstIP.u32 != IPv4AddressSet(224, 0, 0, 1).u32)
			return 0;	// we are not addressed

		if(pIPHeader->protocol == IPV4_PROTOCOL_UDP)
		{
			UDPHeader* pUDPHeader;

			result = UDPDecode(pDecoded, length, &pDecoded, &length, &pUDPHeader);
			if(result)
				return result;

			if(pIPHeader->dstIP.u32 == network.cfg.vision.ip.u32)
			{
				if(pUDPHeader->dstPort != network.cfg.vision.port)
				{
//					ConsolePrint("Detected non-official vision packet on port %hu\r\n", pUDPHeader->dstPort);
//					ConsolePrint("Source: %hu.%hu.%hu.%hu\r\n", (uint16_t)pIPHeader->srcIP.u8[0],
//							(uint16_t)pIPHeader->srcIP.u8[1], (uint16_t)pIPHeader->srcIP.u8[2], (uint16_t)pIPHeader->srcIP.u8[3]);
					return 0;
				}

//				ConsolePrint("Vision frame! Length: %hu\r\n", pUDPHeader->length);

				network.lastVisionTime = chVTGetSystemTimeX();
				network.visionAvailable = 1;

				VisionInput(pDecoded, length);

				return result;
			}

			if(pUDPHeader->dstPort == network.sntp.port)
			{
				NTPPacket* pNTP;

				result = SNTPDecode(pDecoded, length, &pNTP);
				if(result)
					return result;

				if(pIPHeader->srcIP.u32 == network.ucast.ip.u32)
				{
					network.sntp.requestState = NTP_REQUEST_ACK;
					network.sntp.timeoutCnt = 0;
					network.sntp.lastResponseTime = chVTGetSystemTimeX();
					network.sntp.synced = 1;

					int64_t receive = NTP_TO_UNIX(pNTP->receiveTimestampSec);
					int64_t orig = NTP_TO_UNIX(pNTP->originateTimestampSec);
					int32_t correction = receive-orig;

					sysTime.unixOffset += correction;
				}

				return 0;
			}

			if(pUDPHeader->dstPort != network.cfg.my.port)
				return 0;

			// check for auth packet
			PacketHeader* pHeader = (PacketHeader*)pDecoded;
			if(pHeader->section == SECTION_BASE_STATION && pHeader->cmd == CMD_BASE_STATION_AUTH)
			{
				pDecoded += PACKET_HEADER_SIZE;

				BaseStationAuth* pAuth = (BaseStationAuth*)pDecoded;

				if(pAuth->key == 0x42424242)
				{
					network.ucast.mac = pFrameHeader->srcMAC;
					network.ucast.ip = pIPHeader->srcIP;
					network.ucast.port = pUDPHeader->srcPort;

					ConsolePrint("Valid authentication from:\r\n");
					ConsolePrint("MAC: %02hX-%02hX-%02hX-%02hX-%02hX-%02hX\r\n", (uint16_t)network.ucast.mac.u8[0], (uint16_t)network.ucast.mac.u8[1],
							(uint16_t)network.ucast.mac.u8[2], (uint16_t)network.ucast.mac.u8[3], (uint16_t)network.ucast.mac.u8[4], (uint16_t)network.ucast.mac.u8[5]);
					ConsolePrint("IP: %hu.%hu.%hu.%hu\r\n", (uint16_t)network.ucast.ip.u8[0], (uint16_t)network.ucast.ip.u8[1],
							(uint16_t)network.ucast.ip.u8[2], (uint16_t)network.ucast.ip.u8[3]);
					ConsolePrint("Port: %hu\r\n", network.ucast.port);
				}
				else
				{
					ConsolePrint("Invalid base station authentication try\r\n");
				}

				return result;
			}

			unicastUdp(pDecoded, length);

			return result;
		}

		if(pIPHeader->protocol == IPV4_PROTOCOL_ICMP)
		{
			ICMPHeader* pICMPHeader;

			result = ICMPDecode(pDecoded, length, &pDecoded, &length, &pICMPHeader);
			if(result)
				return result;

			if(pICMPHeader->type == ICMP_TYPE_ECHO_REQUEST)
			{
				ICMPEcho* pICMPEcho;

				result = ICMPDecodeEcho(pDecoded, length, &pDecoded, &length, &pICMPEcho);
				if(result)
					return result;

				ConsolePrint("ICMP echo request, ident: %hu, seq: %hu, dataLen: %hu\r\n", pICMPEcho->identifier, pICMPEcho->sequenceNumber, length);

				result = NetworkSendICMPEchoReply(pICMPEcho, pDecoded, length, pFrameHeader->srcMAC, pIPHeader->srcIP);
				if(result)
					return result;
			}
		}

		if(pIPHeader->protocol == IPV4_PROTOCOL_IGMP)
		{
			uint8_t type = 0;
			IGMPv3GetType(pDecoded, &type);

			if(type == IGMP_MEMBERSHIP_QUERY)
			{
				IGMPv3MembershipQuery* pQuery;
				IPv4Address* pSourceList;

				if(length < IGMPV3_MEMBERSHIP_QUERY_SIZE)
				{
					ConsolePrint("IGMPv2 membership query\r\n");

					network.igmp.lastSendTime = chVTGetSystemTimeX();
					network.igmp.respCounter = 0;
					network.igmp.robustness = 1;
					uint32_t respTime = 50;
					network.igmp.respTime = chVTGetSystemTimeX() % respTime;

					return 0;
				}

				result = IGMPv3DecodeMembershipQuery(pDecoded, length, &pQuery, &pSourceList);
				if(result)
				{
					ConsolePrint("Decode error: 0x%04hX\r\n", result);
					return result;
				}

				if(pQuery->groupAddress.u32 == 0 || pQuery->groupAddress.u32 == network.cfg.vision.ip.u32)
				{
					// general query
					network.igmp.lastSendTime = chVTGetSystemTimeX();
					network.igmp.respCounter = 0;
					network.igmp.robustness = pQuery->QRV ? pQuery->QRV : 2;
					uint32_t respTime = IGMPv3DecodeMaxRespCode(pQuery->maxRespCode);
					network.igmp.respTime = chVTGetSystemTimeX() % respTime;
				}
			}

			if(type == IGMPV3_MEMBERSHIP_REPORT)
			{
			}
		}
	}

	return 0;
}

static void processWifiPacket(uint8_t id, uint32_t pktSize)
{
	int16_t result;

	if(pktSize == 0)
		return;

	// intercept and save robot feedback
	PacketHeader* pHeader = (PacketHeader*)&network.rxProcBuf[1];
	if(pHeader->section == SECTION_SYSTEM && pHeader->cmd == CMD_SYSTEM_MATCH_FEEDBACK)
	{
		SystemMatchFeedback* pFeedback = (SystemMatchFeedback*)&network.rxProcBuf[1+sizeof(PacketHeader)];
		HubSetMatchFeedback(id, pFeedback);
	}

	if(!EthGetLinkStatus()->up)
		return;

	// construct header
	PacketHeader header;
	header.section = SECTION_BASE_STATION;
	header.cmd = CMD_BASE_STATION_ACOMMAND;

	// construct BaseStationACommand
	BaseStationACommand* pACmd = (BaseStationACommand*)network.rxProcBuf;
	pACmd->id = id;
	pktSize++;

	result = NetworkSendPacket(&header, network.rxProcBuf, pktSize);
	if(result)
		ConsolePrint("Network send failed: 0x%hX\r\n", result);
}

void NetworkTask(void* params)
{
	(void)params;

	msg_t event;
	uint32_t pktSize;

	int32_t lastStatsTime = chVTGetSystemTimeX();

	chRegSetThreadName("Network");

	while (1)
	{
		if(chMBFetch(&network.eventQueue, &event, MS2ST(10)) == MSG_OK)
		{
			switch(event)
			{
				case EVENT_LINK:
				{
					if(EthLinkUpdate())
					{
						// link up
						network.igmp.lastSendTime = chVTGetSystemTimeX();
						network.igmp.respCounter = 0;
						network.igmp.robustness = 1;
						network.igmp.respTime = 1000;	// send IGMPv3 membership
					}
				}
				break;
				case EVENT_RX_DONE:
				{
					while(EthGetEthernetFrame(network.rxProcBuf, NETWORK_MAX_FRAME_SIZE, &pktSize) == 0)
						decodeFrame(network.rxProcBuf, pktSize);
				}
				break;
			}

			if(event & EVENT_BOT_DATA)
			{
				uint32_t id = event & 0xFF;

				while(RFQueueGetPacket(&wifi.bots[id].queue, network.rxProcBuf+1, NETWORK_MAX_FRAME_SIZE, &pktSize) == 0)
					processWifiPacket(wifi.bots[id].id, pktSize);
			}
		}

		// IGMPv2 and IGMPv3 work
		if(network.igmp.respTime != 0)
		{
			if(chVTTimeElapsedSinceX(network.igmp.lastSendTime) > MS2ST(network.igmp.respTime))
			{
				network.igmp.lastSendTime = chVTGetSystemTimeX();

				IGMPv3GroupRecord record;
				record.auxDataLen = 0;
				record.numberOfSources = 0;
				record.recordType = IGMPV3_MODE_IS_EXCLUDE;
				record.multicastAddress.u32 = network.cfg.vision.ip.u32;

				NetworkSendIGMPv3MembershipReport(1, record._data, IGMPV3_GROUP_RECORD_SIZE);

				NetworkSendIGMPv2MembershipReport(&network.cfg.vision.ip);

				network.igmp.respCounter++;
				if(network.igmp.respCounter >= network.igmp.robustness)
					network.igmp.respTime = 0;
			}
		}

		// SNTP work
		if(network.ucast.ip.u32 != 0)
		{
			if(chVTTimeElapsedSinceX(network.sntp.lastQueryTime) > S2ST(1) && network.sntp.requestState == NTP_REQUEST_ACTIVE)
			{
				// SNTP request timed out
				if(network.sntp.timeoutCnt < 200)
					++network.sntp.timeoutCnt;

				network.sntp.requestState = NTP_REQUEST_TIMEOUT;
			}

			if(network.sntp.timeoutCnt > 10)
			{
				network.sntp.synced = 0;
			}

			if(chVTTimeElapsedSinceX(network.sntp.lastQueryTime) > S2ST(5))
			{
				network.sntp.requestState = NTP_REQUEST_ACTIVE;
				NetworkSendSNTPRequest(SysTimeUnix(), network.ucast.ip, network.ucast.mac);
				network.sntp.lastQueryTime = chVTGetSystemTimeX();
			}
		}

		// Vision availability
		if(chVTTimeElapsedSinceX(network.lastVisionTime) > S2ST(1))
		{
			network.lastVisionTime = chVTGetSystemTimeX();
			network.visionAvailable = 0;
		}

		// Sumatra availability
		if(chVTTimeElapsedSinceX(network.lastSumatraTime) > S2ST(1))
		{
			network.lastSumatraTime = chVTGetSystemTimeX();
			network.sumatraAvailable = 0;
		}

		// Statistics
		if(chVTTimeElapsedSinceX(lastStatsTime) > MS2ST(100))
		{
			lastStatsTime = chVTGetSystemTimeX();

			// construct header
			PacketHeader header;
			header.section = SECTION_BASE_STATION;
			header.cmd = CMD_BASE_STATION_ETH_STATS;

			// Gather ETH stats
			EthStats ethStats = *EthGetStats();

			network.bsEth.rxFrames = ethStats.rxFramesProcessed;
			network.bsEth.rxBytes = ethStats.rxBytesProcessed;
			network.bsEth.txFrames = ethStats.txFramesProcessed;
			network.bsEth.txBytes = ethStats.txBytesProcessed;
			network.bsEth.rxFramesDmaOverrun = ethStats.rxMissedCtrl;
			network.bsEth.ntpSync = network.sntp.synced;

			NetworkSendPacket(&header, (uint8_t*)&network.bsEth, sizeof(BaseStationEthStats));

			// Gather WIFI stats
			network.bsWifi.updateRate = 1000000/wifi.runtime;

			uint8_t slotsUsed = 0;
			for(uint8_t i = 0; i < WIFI_MAX_BOTS; i++)
			{
				if(!wifi.bots[i].online)
					continue;

				network.bsWifi.bots[slotsUsed].botId = wifi.bots[i].id;
				// TODO: needs to be reworked, fields changed
				memcpy(network.bsWifi.bots[slotsUsed].queueStats, (uint8_t*)&wifi.bots[i].queue.stats, sizeof(RFQueueStats));

				slotsUsed++;
			}

			for(uint8_t slot = slotsUsed; slot < WIFI_MAX_BOTS; slot++)
				network.bsWifi.bots[slot].botId = 0xFF;

			header.cmd = CMD_BASE_STATION_WIFI_STATS;
			NetworkSendPacket(&header, (uint8_t*)&network.bsWifi, sizeof(BaseStationWifiStats));
		}
	}
}

int16_t NetworkSendPacket(const PacketHeader* pHeader, const uint8_t* pData, uint16_t dataLength)
{
	if(dataLength + PACKET_HEADER_SIZE > NETWORK_MAX_FRAME_SIZE)
		return ERROR_NOT_ENOUGH_MEMORY;

	chMtxLock(&network.sendPacketMutex);	// prevent re-entering

	memcpy(network.txProcBuf, pHeader->_data, PACKET_HEADER_SIZE);
	memcpy(network.txProcBuf + PACKET_HEADER_SIZE, pData, dataLength);

	uint16_t encLength = dataLength + PACKET_HEADER_SIZE;

	uint16_t dstPort = network.ucast.port;

	IPv4Address dstIp = network.ucast.ip;

	MAC dstMAC = network.ucast.mac;

	int16_t result = UDPEncode(network.txProcBuf, NETWORK_MAX_FRAME_SIZE, encLength, &encLength, network.cfg.my.port, dstPort);
	if(result)
	{
		chMtxUnlock(&network.sendPacketMutex);
		return result;
	}

	result = IPv4Encode(network.txProcBuf, NETWORK_MAX_FRAME_SIZE, encLength, &encLength,
						network.ethernetIdent, network.cfg.my.ip, dstIp, IPV4_PROTOCOL_UDP);
	if(result)
	{
		chMtxUnlock(&network.sendPacketMutex);
		return result;
	}

	result = EthernetEncode(network.txProcBuf, NETWORK_MAX_FRAME_SIZE, encLength, &encLength,
							network.cfg.my.mac, dstMAC, ETHERNET_PROTOCOL_IPV4);
	if(result)
	{
		chMtxUnlock(&network.sendPacketMutex);
		return result;
	}

	result = EthSendEthernetFrame(network.txProcBuf, encLength);
	if(result)
	{
		chMtxUnlock(&network.sendPacketMutex);
		return result;
	}

	network.ethernetIdent++;

	chMtxUnlock(&network.sendPacketMutex);

	return 0;
}

int16_t NetworkSendARP(uint8_t operation, MAC targetMAC, IPv4Address targetIP)
{
	if(ARP_PACKET_SIZE > NETWORK_MAX_FRAME_SIZE)
		return ERROR_NOT_ENOUGH_MEMORY;

	chMtxLock(&network.sendPacketMutex);	// prevent re-entering

	uint16_t encLength;

	int16_t result = ARPEncode(	network.txProcBuf, NETWORK_MAX_FRAME_SIZE, &encLength,
								operation, network.cfg.my.mac, network.cfg.my.ip, targetMAC, targetIP);
	if(result)
	{
		chMtxUnlock(&network.sendPacketMutex);
		return result;
	}

	result = EthernetEncode(network.txProcBuf, NETWORK_MAX_FRAME_SIZE, encLength, &encLength,
							network.cfg.my.mac, targetMAC, ETHERNET_PROTOCOL_ARP);
	if(result)
	{
		chMtxUnlock(&network.sendPacketMutex);
		return result;
	}

	result = EthSendEthernetFrame(network.txProcBuf, encLength);
	if(result)
	{
		chMtxUnlock(&network.sendPacketMutex);
		return result;
	}

	chMtxUnlock(&network.sendPacketMutex);

	return 0;
}

int16_t NetworkSendICMPEchoReply(ICMPEcho* pEchoRequest, uint8_t* pData, uint16_t dataLength, MAC targetMAC, IPv4Address dstIp)
{
	if(dataLength + ICMP_ECHO_SIZE + ICMP_HEADER_SIZE > NETWORK_MAX_FRAME_SIZE)
		return ERROR_NOT_ENOUGH_MEMORY;

	chMtxLock(&network.sendPacketMutex);	// prevent re-entering

	uint16_t encLength;

	memcpy(network.txProcBuf, pData, dataLength);

	int16_t result = ICMPEncodeEcho(network.txProcBuf, NETWORK_MAX_FRAME_SIZE, dataLength, &encLength,
								pEchoRequest->identifier, pEchoRequest->sequenceNumber);
	if(result)
	{
		chMtxUnlock(&network.sendPacketMutex);
		return result;
	}

	result = ICMPEncode(network.txProcBuf, NETWORK_MAX_FRAME_SIZE, encLength, &encLength, ICMP_TYPE_ECHO_REPLY, 0);
	if(result)
	{
		chMtxUnlock(&network.sendPacketMutex);
		return result;
	}

	result = IPv4Encode(network.txProcBuf, NETWORK_MAX_FRAME_SIZE, encLength, &encLength,
						network.ethernetIdent, network.cfg.my.ip, dstIp, IPV4_PROTOCOL_ICMP);
	if(result)
	{
		chMtxUnlock(&network.sendPacketMutex);
		return result;
	}

	result = EthernetEncode(network.txProcBuf, NETWORK_MAX_FRAME_SIZE, encLength, &encLength,
							network.cfg.my.mac, targetMAC, ETHERNET_PROTOCOL_IPV4);
	if(result)
	{
		chMtxUnlock(&network.sendPacketMutex);
		return result;
	}

	result = EthSendEthernetFrame(network.txProcBuf, encLength);
	if(result)
	{
		chMtxUnlock(&network.sendPacketMutex);
		return result;
	}

	network.ethernetIdent++;

	chMtxUnlock(&network.sendPacketMutex);

	return 0;
}

int16_t NetworkSendIGMPv3MembershipReport(uint16_t numberOfGroupRecords, uint8_t* recordData, uint16_t recordDataLength)
{
	if(recordDataLength + IGMPV3_MEMBERSHIP_REPORT_SIZE > NETWORK_MAX_FRAME_SIZE)
		return ERROR_NOT_ENOUGH_MEMORY;

	chMtxLock(&network.sendPacketMutex);	// prevent re-entering

	uint16_t encLength = 0;
	int16_t result = 0;

	result = IGMPv3EncodeMembershipReport(	network.txProcBuf, NETWORK_MAX_FRAME_SIZE,
											&encLength, numberOfGroupRecords, recordData, recordDataLength);
	if(result)
	{
		chMtxUnlock(&network.sendPacketMutex);
		return result;
	}

	IPv4Address dstIP = IPv4AddressSet(224, 0, 0, 22);
	MAC dstMAC = MACSet(0x01, 0x00, 0x5E, 0x00, 0x00, 0x16);	// 224.0.0.22

	result = IPv4EncodeWithRouterAlertOption(network.txProcBuf, NETWORK_MAX_FRAME_SIZE, encLength, &encLength,
						network.ethernetIdent, network.cfg.my.ip, dstIP, IPV4_PROTOCOL_IGMP);
	if(result)
	{
		chMtxUnlock(&network.sendPacketMutex);
		return result;
	}

	result = EthernetEncode(network.txProcBuf, NETWORK_MAX_FRAME_SIZE, encLength, &encLength,
							network.cfg.my.mac, dstMAC, ETHERNET_PROTOCOL_IPV4);
	if(result)
	{
		chMtxUnlock(&network.sendPacketMutex);
		return result;
	}

	result = EthSendEthernetFrame(network.txProcBuf, encLength);
	if(result)
	{
		chMtxUnlock(&network.sendPacketMutex);
		return result;
	}

	network.ethernetIdent++;

	chMtxUnlock(&network.sendPacketMutex);

	return 0;
}

int16_t NetworkSendIGMPv2MembershipReport(const IPv4Address* pMulticaseAddress)
{
	if(IGMPV2_MEMBERSHIP_REPORT_SIZE > NETWORK_MAX_FRAME_SIZE)
		return ERROR_NOT_ENOUGH_MEMORY;

	chMtxLock(&network.sendPacketMutex);	// prevent re-entering

	uint16_t encLength = 0;
	int16_t result = 0;

	result = IGMPv2EncodeMembershipReport(	network.txProcBuf, NETWORK_MAX_FRAME_SIZE,
											&encLength, pMulticaseAddress);
	if(result)
	{
		chMtxUnlock(&network.sendPacketMutex);
		return result;
	}

	IPv4Address dstIP = IPv4AddressSet(224, 0, 0, 2);
	MAC dstMAC = MACSet(0x01, 0x00, 0x5E, 0x00, 0x00, 0x02);	// 224.0.0.2

	result = IPv4EncodeWithRouterAlertOption(network.txProcBuf, NETWORK_MAX_FRAME_SIZE, encLength, &encLength,
						network.ethernetIdent, network.cfg.my.ip, dstIP, IPV4_PROTOCOL_IGMP);
	if(result)
	{
		chMtxUnlock(&network.sendPacketMutex);
		return result;
	}

	result = EthernetEncode(network.txProcBuf, NETWORK_MAX_FRAME_SIZE, encLength, &encLength,
							network.cfg.my.mac, dstMAC, ETHERNET_PROTOCOL_IPV4);
	if(result)
	{
		chMtxUnlock(&network.sendPacketMutex);
		return result;
	}

	result = EthSendEthernetFrame(network.txProcBuf, encLength);
	if(result)
	{
		chMtxUnlock(&network.sendPacketMutex);
		return result;
	}

	network.ethernetIdent++;

	chMtxUnlock(&network.sendPacketMutex);

	return 0;
}

int16_t NetworkSendSNTPRequest(uint32_t unixTransmitTimestamp, IPv4Address dstIp, MAC dstMAC)
{
	uint16_t encLength = 0;
	int16_t result = 0;

	if(NTP_PACKET_SIZE > NETWORK_MAX_FRAME_SIZE)
		return ERROR_NOT_ENOUGH_MEMORY;

	chMtxLock(&network.sendPacketMutex);	// prevent re-entering

	result = SNTPEncode(network.txProcBuf, NETWORK_MAX_FRAME_SIZE, &encLength, unixTransmitTimestamp);
	if(result)
	{
		chMtxUnlock(&network.sendPacketMutex);
		return result;
	}

	uint16_t dstPort = NTP_PORT;

	result = UDPEncode(network.txProcBuf, NETWORK_MAX_FRAME_SIZE, encLength, &encLength, network.sntp.port, dstPort);
	if(result)
	{
		chMtxUnlock(&network.sendPacketMutex);
		return result;
	}

	result = IPv4Encode(network.txProcBuf, NETWORK_MAX_FRAME_SIZE, encLength, &encLength,
						network.ethernetIdent, network.cfg.my.ip, dstIp, IPV4_PROTOCOL_UDP);
	if(result)
	{
		chMtxUnlock(&network.sendPacketMutex);
		return result;
	}

	result = EthernetEncode(network.txProcBuf, NETWORK_MAX_FRAME_SIZE, encLength, &encLength,
							network.cfg.my.mac, dstMAC, ETHERNET_PROTOCOL_IPV4);
	if(result)
	{
		chMtxUnlock(&network.sendPacketMutex);
		return result;
	}

	result = EthSendEthernetFrame(network.txProcBuf, encLength);
	if(result)
	{
		chMtxUnlock(&network.sendPacketMutex);
		return result;
	}

	network.ethernetIdent++;

	chMtxUnlock(&network.sendPacketMutex);

	return 0;
}

void NetworkSetMyPort(uint16_t port)
{
	network.cfg.my.port = port;
}

void NetworkSetMyIP(IPv4Address ip)
{
	network.cfg.my.ip = ip;
}

void NetworkSetMyMAC(uint8_t inc)
{
	network.cfg.my.mac.u8[5] = 48+inc;

	EthAllowMAC(0, network.cfg.my.mac.u8, 1, 0);
}

void NetworkSetVisionPort(uint16_t port)
{
	network.cfg.vision.port = port;
}

void NetworkSetVisionIP(IPv4Address ip)
{
	network.cfg.vision.ip = ip;
	network.visionMAC = IPv4CalcMulticastMAC(ip);

	EthAllowMAC(2, network.visionMAC.u8, 1, 0);
}

void NetworkPrintConfig()
{
	ConsolePrint("--- Network Configuration ---\r\n");
	ConsolePrint("MAC:       %02hX-%02hX-%02hX-%02hX-%02hX-%02hX\r\n", (uint16_t)network.cfg.my.mac.u8[0], (uint16_t)network.cfg.my.mac.u8[1],
			(uint16_t)network.cfg.my.mac.u8[2], (uint16_t)network.cfg.my.mac.u8[3],
			(uint16_t)network.cfg.my.mac.u8[4], (uint16_t)network.cfg.my.mac.u8[5]);
	ConsolePrint("My IP:     %hu.%hu.%hu.%hu:%hu\r\n", (uint16_t)network.cfg.my.ip.u8[0], (uint16_t)network.cfg.my.ip.u8[1],
			(uint16_t)network.cfg.my.ip.u8[2], (uint16_t)network.cfg.my.ip.u8[3], network.cfg.my.port);
	ConsolePrint("Vision IP: %hu.%hu.%hu.%hu:%hu\r\n", (uint16_t)network.cfg.vision.ip.u8[0], (uint16_t)network.cfg.vision.ip.u8[1],
			(uint16_t)network.cfg.vision.ip.u8[2], (uint16_t)network.cfg.vision.ip.u8[3], network.cfg.vision.port);
//	ConsolePrint("RST Mode: %hu\r\n", (uint16_t)rst.cfg.enabled);
//	ConsolePrint("RST Rate: %hu\r\n", (uint16_t)rst.cfg.rate);
//	ConsolePrint("RST Port: %hu\r\n", rst.cfg.port);
}
