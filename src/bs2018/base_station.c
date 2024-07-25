#include "base_station.h"
#include "dev/sky_fem.h"
#include "dev/sx1280.h"
#include "sys/eth0.h"
#include "module/radio/radio_settings.h"
#include "sx1280_def.h"
#include "constants.h"
#include "misc/inventory.h"
#include "router.h"

#define EVENT_MASK_BS_SOCKET_RX	EVENT_MASK(0)

static void baseStationTask(void* pParam);
static void handlePacket(NetPkt* pPkt);

BaseStation baseStation = {
	.config = {
		.base = {
			.ip.u8 = { 192, 168, 1, 210 },
			.port = 10201,
		},
		.vision = {
			.ip.u8 = { 224, 5, 23, 2 },
			.port = 10002,
		},
	},
};

CH_IRQ_HANDLER(Vector1C4) // SPDIFRX, used as low prio IRQ to get back to OS-level
{
	CH_IRQ_PROLOGUE();
	RadioBaseLowPrioIRQ(&baseStation.radioBase);
	CH_IRQ_EPILOGUE();
}

void BaseStationInit()
{
	FlashFSOpenOrCreate("base/cfg", 0, &baseStation.config, sizeof(BaseStationConfig), &baseStation.pCfgFile);

	// ### Network ###
	NetIfData netInit;
	memcpy(&netInit.mac, &InventoryGetEthernet()->mac, sizeof(MAC));
	netInit.ip = baseStation.config.base.ip;
	netInit.pEth = &eth0;

	NetIfInit(&baseStation.networkInterface, &netInit, TASK_PRIO_NET_IF);

	// ### SSL Vision ###
	SslVisionData visionInit;
	visionInit.pIf = &baseStation.networkInterface;
	visionInit.ip = baseStation.config.vision.ip;
	visionInit.port = baseStation.config.vision.port;
	visionInit.changeDetectionCameraDelay_ms = 50;
	visionInit.robotOutlierMaxVelXY_mDs = 5.0f;
	visionInit.robotOutlierMaxVelW_radDs = 50.0f;
	visionInit.ballOutlierMaxVelXY_mDs = 8.0f;
	visionInit.objectTimeout_ms = 100;
	visionInit.visionTimeout_ms = 1000;

	SslVisionInit(&baseStation.vision, &visionInit, TASK_PRIO_SSL_VISION);

	// ### Radio ###
	SKY66112CreateInterface(&devSkyFem, &baseStation.femInterface);

	RadioPhyData radioPhyInit;
	radioPhyInit.pSX = &devSX1280;
	radioPhyInit.pFem = &baseStation.femInterface;
	radioPhyInit.timeouts = radioSettings.phyTimeouts;

	RadioPhyInit(&baseStation.radioPhy, &radioPhyInit);

	RadioModuleData radioModuleInit;
	radioModuleInit.pPhy = &baseStation.radioPhy;
	radioModuleInit.pwrPin = (GPIOPin){ GPIOB, GPIO_PIN_5 };
	radioModuleInit.packetType = radioSettings.packetType;
	radioModuleInit.settingsCommon = radioSettings.settingsCommon;

	if(radioSettings.packetType == PACKET_TYPE_FLRC)
		radioModuleInit.settings.flrc = radioSettings.settingsFlrc;
	else
		radioModuleInit.settings.gfsk = radioSettings.settingsGfsk;

	RadioModuleInit(&baseStation.radioModule, &radioModuleInit);

	RadioBaseData radioBaseInit;
	radioBaseInit.pModule = &baseStation.radioModule;
	radioBaseInit.clientTimeout_us = radioSettings.connectionTimeout_us;
	radioBaseInit.lowPrioIRQn = SPDIF_RX_IRQn;

	RadioBaseInit(&baseStation.radioBase, &radioBaseInit);

	NVICEnableIRQ(SPDIF_RX_IRQn, IRQL_WIFI_LOW_PRIO);

	RadioBaseSetMode(&baseStation.radioBase, RADIO_MODE_ACTIVE);

	// ### Base Station Processing ###
	UDPSocketOpen(&baseStation.socket, &baseStation.networkInterface, baseStation.config.base.port, baseStation.networkInterface.data.ip);

	baseStation.pTask = chThdCreateStatic(baseStation.waTask, sizeof(baseStation.waTask), TASK_PRIO_BASE_STATION, &baseStationTask, 0);
}

void BaseStationSetIp(IPv4Address ip)
{
	baseStation.config.base.ip = ip;
	UDPSocketSetBindIp(&baseStation.socket, ip);
	NetIfSetIp(&baseStation.networkInterface, ip);

	FlashFSWrite(&baseStation.pCfgFile, &baseStation.config, sizeof(baseStation.config));
}

void BaseStationSetPort(uint16_t port)
{
	baseStation.config.base.port = port;
	baseStation.socket.port = port;

	FlashFSWrite(&baseStation.pCfgFile, &baseStation.config, sizeof(baseStation.config));
}

void BaseStationSetVisionIp(IPv4Address ip)
{
	baseStation.config.vision.ip = ip;
	UDPSocketSetBindIp(&baseStation.vision.socket, ip);

	FlashFSWrite(&baseStation.pCfgFile, &baseStation.config, sizeof(baseStation.config));
}

void BaseStationSetVisionPort(uint16_t port)
{
	baseStation.config.vision.port = port;
	baseStation.vision.socket.port = port;

	FlashFSWrite(&baseStation.pCfgFile, &baseStation.config, sizeof(baseStation.config));
}

static void baseStationTask(void*)
{
	chRegSetThreadName("BASE_STATION");

	event_listener_t udpListener;

	chEvtRegisterMask(&baseStation.socket.eventSource, &udpListener, EVENT_MASK_BS_SOCKET_RX);

	while(1)
	{
		eventmask_t events = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(10));

		// Handle all received packets
		if(events & EVENT_MASK_BS_SOCKET_RX)
		{
			while(1)
			{
				NetPkt* pPkt = UDPSocketReceive(&baseStation.socket);
				if(!pPkt)
					break;

				handlePacket(pPkt);

				UDPSocketFree(&baseStation.socket, pPkt);
			}
		}

		// Sumatra availability
		if(baseStation.sumatra.isOnline && chVTTimeElapsedSinceX(baseStation.sumatra.tLastCommandReception) > TIME_S2I(1))
			baseStation.sumatra.isOnline = 0;

		// Statistics
		if(chVTTimeElapsedSinceX(baseStation.sumatra.tLastStatsSend) > TIME_MS2I(100))
		{
			baseStation.sumatra.tLastStatsSend = chVTGetSystemTimeX();

			// construct header
			PacketHeader header;
			header.section = SECTION_BASE_STATION;
			header.cmd = CMD_BASE_STATION_ETH_STATS;

			// Gather ETH stats
			EthStats ethStats = *EthGetStats(&eth0);

			BaseStationEthStats bsEthStats;
			bsEthStats.rxFrames = ethStats.rxFramesProcessed;
			bsEthStats.rxBytes = ethStats.rxBytesProcessed;
			bsEthStats.txFrames = ethStats.txFramesProcessed;
			bsEthStats.txBytes = ethStats.txBytesProcessed;
			bsEthStats.rxFramesDmaOverrun = ethStats.rxMissedCtrl;
			bsEthStats.ntpSync = 0; // TODO: re-implement SNTP

			BaseStationSendSumatraPacket(&header, &bsEthStats, sizeof(BaseStationEthStats));

			// Gather WIFI stats
			BaseStationWifiStats wifiStats; // TODO: replace this struct, most fields are unused
			memset(&wifiStats, 0, sizeof(wifiStats));

			wifiStats.updateRate = 1000000/baseStation.radioBase.stats.cycleDuration_us;

			uint8_t slotsUsed = 0;
			for(uint8_t i = 0; i < RADIO_NUM_ROBOT_CLIENTS; i++)
			{
				RadioClientRx* pClient = &baseStation.radioBase.baseIn[i];
				if(!pClient->isOnline)
					continue;

				wifiStats.bots[slotsUsed].botId = i;
				wifiStats.bots[slotsUsed].bsRssi = pClient->avgRxRssi_mdBm/100;
				wifiStats.bots[slotsUsed].botRssi = wifiStats.bots[slotsUsed].bsRssi;

				slotsUsed++;
			}

			for(uint8_t slot = slotsUsed; slot < RADIO_NUM_ROBOT_CLIENTS; slot++)
				wifiStats.bots[slot].botId = 0xFF;

			header.cmd = CMD_BASE_STATION_WIFI_STATS;
			BaseStationSendSumatraPacket(&header, &wifiStats, sizeof(BaseStationWifiStats));
		}
	}
}

void BaseStationSendSumatraPacket(const PacketHeader* pHeader, const void* pData, size_t dataSize)
{
	if(baseStation.sumatra.ip.u32 == 0 || baseStation.sumatra.port == 0)
		return;

	NetPkt* pPkt = NetIfAllocPkt(&baseStation.networkInterface);
	if(!pPkt)
	{
		baseStation.stats.txFailed++;
		return;
	}

	if(!NetBufAddMem(&pPkt->buf, pHeader, sizeof(PacketHeader)))
	{
		baseStation.stats.txFailed++;
		return;
	}

	if(!NetBufAddMem(&pPkt->buf, pData, dataSize))
	{
		baseStation.stats.txFailed++;
		return;
	}

	pPkt->ipv4.dst = baseStation.sumatra.ip;
	pPkt->udp.dstPort = baseStation.sumatra.port;

	if(UDPSocketSend(&baseStation.socket, pPkt))
		baseStation.stats.txFailed++;
	else
		baseStation.stats.txGood++;
}

static void handlePacket(NetPkt* pPkt)
{
	PacketHeader* pHeader = NetBufPull(&pPkt->buf, sizeof(PacketHeader));
	if(!pHeader)
	{
		baseStation.stats.rxInvalid++;
		return;
	}

	if(pHeader->section != SECTION_BASE_STATION)
	{
		baseStation.stats.rxInvalid++;
		return;
	}

	// The following commands are always processed, independent of source
	switch(pHeader->cmd)
	{
		case CMD_BASE_STATION_AUTH:
		{
			BaseStationAuth* pAuth = NetBufPull(&pPkt->buf, sizeof(BaseStationAuth));
			if(!pAuth)
			{
				baseStation.stats.rxInvalid++;
				return;
			}

			if(pAuth->key == 0x42424242)
			{
				if(ArpCacheLookup(&baseStation.networkInterface.arp, pPkt->ipv4.src) == 0)
				{
					ArpSend(&baseStation.networkInterface.arp, ARP_OPERATION_REQUEST, MACSet(255, 255, 255, 255, 255, 255), pPkt->ipv4.src);
				}

				baseStation.sumatra.ip = pPkt->ipv4.src;
				baseStation.sumatra.port = pPkt->udp.srcPort;

				printf("Valid authentication from:\r\n");
				printf("MAC: %s\r\n", MACFormatAddress(pPkt->ethernet.src));
				printf("IP: %s\r\n", IPv4FormatAddress(baseStation.sumatra.ip));
				printf("Port: %hu\r\n", baseStation.sumatra.port);
			}
			else
			{
				baseStation.stats.rxAuthFailed++;
			}
		}
		break;
		case CMD_BASE_STATION_PING:
		{
			BaseStationSendSumatraPacket(pHeader, NetBufGetHead(&pPkt->buf), NetBufGetSizeUsed(&pPkt->buf));
		}
		break;
	}

	// Following commands are only accepted from the current Sumatra control instance
	if(pPkt->ipv4.src.u32 != baseStation.sumatra.ip.u32)
	{
		baseStation.stats.rxUnauthorized++;
		return;
	}

	baseStation.sumatra.tLastCommandReception = chVTGetSystemTimeX();
	baseStation.sumatra.isOnline = 1;

	switch(pHeader->cmd)
	{
		case CMD_BASE_STATION_CONFIG_V3:
		{
			BaseStationConfigV3* pConfig = NetBufPull(&pPkt->buf, sizeof(BaseStationConfigV3));
			if(!pConfig)
			{
				baseStation.stats.rxInvalid++;
				return;
			}

			BaseStationSetVisionIp(IPv4AddressSet(pConfig->visionIp[0], pConfig->visionIp[1], pConfig->visionIp[2], pConfig->visionIp[3]));
			BaseStationSetVisionPort(pConfig->visionPort);

			RadioBaseSetChannel(&baseStation.radioBase, pConfig->channel);
			RadioBaseSetMaxBots(&baseStation.radioBase, pConfig->maxBots);
			RadioBaseSetFixedRuntime(&baseStation.radioBase, pConfig->fixedRuntime);
		}
		break;
		case CMD_BASE_STATION_CAM_VIEWPORT:
		{
			BaseStationCamViewport* pViewport = NetBufPull(&pPkt->buf, sizeof(BaseStationCamViewport));
			if(!pViewport)
			{
				baseStation.stats.rxInvalid++;
				return;
			}

			SslVisionCameraViewport viewport;
			viewport.xRange_m[0] = pViewport->minX * 0.001f;
			viewport.xRange_m[1] = pViewport->maxX * 0.001f;
			viewport.yRange_m[0] = pViewport->minY * 0.001f;
			viewport.yRange_m[1] = pViewport->maxY * 0.001f;

			SslVisionUpdateViewport(&baseStation.vision, pViewport->camId, &viewport);
		}
		break;
		case CMD_BASE_STATION_ACOMMAND:
		{
			BaseStationACommand* pCmd = NetBufPull(&pPkt->buf, sizeof(BaseStationACommand));
			if(!pCmd)
			{
				baseStation.stats.rxInvalid++;
				return;
			}

			RouterSendBotData(pCmd->id, NetBufGetHead(&pPkt->buf), NetBufGetSizeUsed(&pPkt->buf));
		}
		break;
	}

	baseStation.stats.rxGood++;
}
