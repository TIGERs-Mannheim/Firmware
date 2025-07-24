#include "network.h"
#include "util/log.h"
#include "robot.h"
#include "hal/sys_time.h"
#include "module/radio/radio_bot.h"

#include "errors.h"
#include "commands.h"
#include "struct_ids.h"

#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>

#define EVENT_MASK_RX_BOT_DATA			EVENT_MASK(0)
#define EVENT_MASK_RELIABLE_CMD_ACKD	EVENT_MASK(1)
#define EVENT_MASK_RELIABLE_CMD_RUN		EVENT_MASK(2)
#define EVENT_MASK_LINK_ENABLE			EVENT_MASK(3)
#define EVENT_MASK_LINK_DISABLE			EVENT_MASK(4)
#define EVENT_MASK_START_SCAN			EVENT_MASK(5)

#define PROCESSED 0
#define PASS_ON 1

typedef enum NetworkEvent
{
	NETWORK_EVENT_NONE = 0,
	NETWORK_EVENT_DISABLE,
	NETWORK_EVENT_ENABLE,
	NETWORK_EVENT_SCAN_START,
	NETWORK_EVENT_SCAN_COMPLETE,
	NETWORK_EVENT_BROADCAST_RECEIVED,
} NetworkEvent;

Network network = {
	.config = { 0xFF },
};

static const ConfigFileDesc configFileDescSettings =
	{ SID_CFG_NET_SETTINGS, 4, "net/config", 1, (ElementDesc[]) {
		{ UINT8, "bot_id", "", "Bot ID" },
	} };

static void networkTask(void* params);
static void networkModeStatemachine(NetworkEvent event);
static uint8_t processPacket(PacketHeader* pHeader, uint8_t* pBuf, uint16_t dataLength);
static void registerShellCommands(ShellCmdHandler* pHandler);

static void configUpdateCallback(uint16_t cfgId)
{
	switch(cfgId)
	{
		case SID_CFG_NET_SETTINGS:
		{
			NetworkImplSetBotId(network.config.botId);
		}
		break;
		default:
		break;
	}
}

void NetworkInit(tprio_t taskPrio)
{
	// data setup
	chMtxObjectInit(&network.printMutex);
	chMtxObjectInit(&network.sendPacketMutex);
	chMtxObjectInit(&network.matchCtrlMutex);
	chMtxObjectInit(&network.baseStationBroadcastMutex);

	network.lastRxSeq = -1;

	FifoLinInit(&network.reliableCmds, network.reliableBuf, NETWORK_RELIABLE_BUF_SIZE);

	network.mode = NETWORK_MODE_DISABLED;

	network.pNetworkConfig = ConfigOpenOrCreate(&configFileDescSettings, &network.config, sizeof(ConfigNetwork), &configUpdateCallback, 1);

	NetworkImplSetBotId(network.config.botId);

	ConfigNotifyUpdate(network.pNetworkConfig);

	network.matchCtrlTime = SysTimeUSec()-5000000;

	network.lastMatchCtrlCmd.curPosition[0] = SYSTEM_MATCH_CTRL_UNUSED_FIELD;
	network.lastMatchCtrlCmd.curPosition[1] = SYSTEM_MATCH_CTRL_UNUSED_FIELD;
	network.lastMatchCtrlCmd.curPosition[2] = SYSTEM_MATCH_CTRL_UNUSED_FIELD;

	ShellCmdHandlerInit(&network.cmdHandler, 0);
	registerShellCommands(&network.cmdHandler);

	network.pNetworkTask = chThdCreateStatic(network.waNetworkTask, sizeof(network.waNetworkTask), taskPrio, networkTask, 0);
}

void NetworkSaveConfig()
{
	ConfigNotifyUpdate(network.pNetworkConfig);
}

static void networkTask(void* params)
{
	(void)params;
	uint32_t len;
	uint32_t lastReliableCmdSendTime = chVTGetSystemTimeX()-TIME_MS2I(200);

	chRegSetThreadName("Network");

	event_listener_t rxBotListener;
	chEvtRegisterMaskWithFlags(NetworkImplGetRadioEventSource(), &rxBotListener, EVENT_MASK_RX_BOT_DATA, RADIO_BOT_EVENT_PACKET_RECEIVED_FROM_BASE | RADIO_BOT_EVENT_SCAN_COMPLETE);

	while(1)
	{
		eventmask_t events = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(10));

		if(events & EVENT_MASK_RX_BOT_DATA)
		{
			eventflags_t flags = chEvtGetAndClearFlags(&rxBotListener);
			if(flags & RADIO_BOT_EVENT_SCAN_COMPLETE)
			{
				networkModeStatemachine(NETWORK_EVENT_SCAN_COMPLETE);
			}
		}

		if(events & EVENT_MASK_LINK_DISABLE)
			networkModeStatemachine(NETWORK_EVENT_DISABLE);

		if(events & EVENT_MASK_LINK_ENABLE)
			networkModeStatemachine(NETWORK_EVENT_ENABLE);

		if(events & EVENT_MASK_START_SCAN)
			networkModeStatemachine(NETWORK_EVENT_SCAN_START);

		networkModeStatemachine(NETWORK_EVENT_NONE);

		uint8_t isBaseLost = !NetworkImplGetState().isBaseOnline;
		uint8_t isNotAllocated = (network.lastBaseStationBroadcast.allocatedBotIds & (1 << network.config.botId)) == 0;

		if(network.mode == NETWORK_MODE_PAIRED && (isBaseLost || isNotAllocated))
			networkModeStatemachine(NETWORK_EVENT_SCAN_START);

		// Reliable command handling/retransmission
		if(events & EVENT_MASK_RELIABLE_CMD_ACKD)
		{
			lastReliableCmdSendTime = chVTGetSystemTimeX()-TIME_MS2I(200);
		}

		if(chVTTimeElapsedSinceX(lastReliableCmdSendTime) > TIME_MS2I(100) && network.mode == NETWORK_MODE_PAIRED)
		{
			uint8_t* pBuf = 0;
			uint32_t cmdSize;

			if(FifoLinGet(&network.reliableCmds, &pBuf, &cmdSize) == 0)
			{
				chMtxLock(&network.sendPacketMutex);
				NetworkImplSendPacket(pBuf, cmdSize);
				chMtxUnlock(&network.sendPacketMutex);

				lastReliableCmdSendTime = chVTGetSystemTimeX();
			}
		}

		// Process unicast packets for this bot
		while(NetworkImplGetPacketFromBase(network.config.botId, network.rxProcessingBuf, NETWORK_RX_PROCESSING_SIZE, &len) == 0)
		{
			if(len <= PACKET_HEADER_SIZE)
				continue;

			PacketHeader* pHeader = (PacketHeader*)network.rxProcessingBuf;
			uint8_t* pBuf = network.rxProcessingBuf+PACKET_HEADER_SIZE;
			uint16_t dataLength = len-PACKET_HEADER_SIZE;

			// Handle extended packets
			if(pHeader->cmd & CMD_EXTENDED_HEADER_MASK)
			{
				PacketHeaderEx* pHeaderEx = (PacketHeaderEx*)pHeader;

				PacketHeader ack;
				ack.cmd = CMD_SYSTEM_ACK;

				NetworkSendPacket(&ack, &pHeaderEx->seq, sizeof(uint16_t));

				if(network.lastRxSeq == pHeaderEx->seq)
					continue; // Duplicate packet received, probably lost ACK

				network.lastRxSeq = pHeaderEx->seq;

				uint8_t* pBufOrig = pBuf;
				pHeader->cmd &= ~CMD_EXTENDED_HEADER_MASK;
				pBuf += PACKET_HEADER_EX_SIZE-PACKET_HEADER_SIZE;
				dataLength -= PACKET_HEADER_EX_SIZE-PACKET_HEADER_SIZE;

				memcpy(pBufOrig, pHeader, sizeof(PacketHeader));
				pHeader = (PacketHeader*)pBufOrig;
			}

			if(network.mode == NETWORK_MODE_PAIRED)
			{
				if(processPacket(pHeader, pBuf, dataLength) == PASS_ON)
				{
					NetworkImplProcessPacket(pHeader, pBuf, dataLength);
				}
			}
		}

		// Process broadcast packets
		while(NetworkImplGetPacketFromBase(CMD_BOT_BROADCAST_ID, network.rxProcessingBuf, NETWORK_RX_PROCESSING_SIZE, &len) == 0)
		{
			if(len <= PACKET_HEADER_SIZE)
				continue;

			PacketHeader* pHeader = (PacketHeader*)network.rxProcessingBuf;
			uint8_t* pBuf = network.rxProcessingBuf+PACKET_HEADER_SIZE;
			uint16_t dataLength = len-PACKET_HEADER_SIZE;

			if(pHeader->cmd & CMD_EXTENDED_HEADER_MASK)
				continue; // Reliable broadcast packets don't make sense

			if(processPacket(pHeader, pBuf, dataLength) == PASS_ON)
			{
				NetworkImplProcessPacket(pHeader, pBuf, dataLength);
			}
		}
	}
}

int16_t NetworkSendPacket(const PacketHeader* pHeader, const void* _pData, uint16_t dataLength)
{
	const uint8_t* pData = (const uint8_t*)_pData;

	if(dataLength + PACKET_HEADER_SIZE > NETWORK_TX_PROCESSING_SIZE)
		return ERROR_NOT_ENOUGH_MEMORY;

	if(network.mode != NETWORK_MODE_PAIRED)
		return ERROR_RESSOURCE_UNAVAILABLE;

	chMtxLock(&network.sendPacketMutex);	// prevent re-entering

	memcpy(network.txProcessingBuf, &pHeader->cmd, PACKET_HEADER_SIZE);
	memcpy(network.txProcessingBuf + PACKET_HEADER_SIZE, pData, dataLength);

	// data to send is now in network.txProcessingBuf
	int16_t result = NetworkImplSendPacket(network.txProcessingBuf, dataLength+PACKET_HEADER_SIZE);

	chMtxUnlock(&network.sendPacketMutex);

	return result;
}

void NetworkSendPacketRaw(void* _pData, uint16_t dataLength)
{
	uint8_t* pData = (uint8_t*)_pData;

	if(network.mode != NETWORK_MODE_PAIRED)
		return;

	chMtxLock(&network.sendPacketMutex);	// prevent re-entering

	NetworkImplSendPacket(pData, dataLength);

	chMtxUnlock(&network.sendPacketMutex);
}

int16_t NetworkSendPacketReliable(const PacketHeader* pHeader, const void* _pData, uint16_t dataLength)
{
	uint8_t* pBuf;
	const uint8_t* pData = (const uint8_t*)_pData;

	int16_t result = FifoLinReserve(&network.reliableCmds, dataLength + PACKET_HEADER_EX_SIZE, &pBuf);
	if(result)
		return result;

	PacketHeaderEx* pHeaderEx = (PacketHeaderEx*)pBuf;
	pHeaderEx->cmd = pHeader->cmd | CMD_EXTENDED_HEADER_MASK;
	pHeaderEx->seq = network.sendSeq++;

	memcpy(pBuf+PACKET_HEADER_EX_SIZE, pData, dataLength);

	result = FifoLinCommit(&network.reliableCmds, dataLength+PACKET_HEADER_EX_SIZE);
	if(result)
		return result;

	chEvtSignal(network.pNetworkTask, EVENT_MASK_RELIABLE_CMD_RUN);

	return 0;
}

void NetworkEnableLink(uint8_t enable)
{
	if(enable)
		chEvtSignal(network.pNetworkTask, EVENT_MASK_LINK_ENABLE);
	else
		chEvtSignal(network.pNetworkTask, EVENT_MASK_LINK_DISABLE);
}

void NetworkStartScan()
{
	chEvtSignal(network.pNetworkTask, EVENT_MASK_START_SCAN);
}

static const char* pNetworkModeStrings[] = { "Disabled", "Idle", "Scanning", "Validate Scan", "Validate Channel", "Paired" };

const char* NetworkGetModeString(NetworkMode mode)
{
	if(mode >= sizeof(pNetworkModeStrings) / sizeof(pNetworkModeStrings[0]))
		return "Unknown";

	return pNetworkModeStrings[mode];
}

static NetworkScanEntry* findNetworkScanEntry(NetworkScanResult* pResult, uint8_t channel)
{
	const size_t numEntries = sizeof(pResult->baseStations) / sizeof(pResult->baseStations[0]);

	NetworkScanEntry* pFirstUnused = 0;

	for(size_t i = 0; i < numEntries; i++)
	{
		NetworkScanEntry* pEntry = &pResult->baseStations[i];
		if(pEntry->isUsed && pEntry->channel == channel)
			return pEntry;

		if(pEntry->isUsed == 0 && pFirstUnused == 0)
		{
			pFirstUnused = pEntry;
			pFirstUnused->rssi_dBm = -128.0f;
		}
	}

	return pFirstUnused;
}

static const NetworkScanEntry* findBestScanEntry(const NetworkScanResult* pResult)
{
	const size_t numEntries = sizeof(pResult->baseStations) / sizeof(pResult->baseStations[0]);

	const NetworkScanEntry* pBestEntry = 0;

	for(size_t i = 0; i < numEntries; i++)
	{
		const NetworkScanEntry* pEntry = &pResult->baseStations[i];
		if(pEntry->isUsed && (pEntry->allocatedBotIds & (1 << network.config.botId)))
		{
			if(pBestEntry)
			{
				if(pEntry->rssi_dBm > pBestEntry->rssi_dBm)
					pBestEntry = pEntry;
			}
			else
			{
				pBestEntry = pEntry;
			}
		}
	}

	return pBestEntry;
}

static void networkModeStatemachine(NetworkEvent event)
{
	NetworkMode newMode = network.mode;

	uint32_t timeInCurrentMode_ms = (SysTimeUSec() - network.modeEntryTime) / 1000;

	switch(network.mode)
	{
		case NETWORK_MODE_DISABLED:
		{
			if(event == NETWORK_EVENT_ENABLE)
			{
				NetworkImplSetMode(RADIO_MODE_SNIFFER);
				newMode = NETWORK_MODE_IDLE;
				chEvtAddEvents(EVENT_MASK_START_SCAN);
			}
		}
		break;
		case NETWORK_MODE_IDLE:
		{
			if(event == NETWORK_EVENT_SCAN_START)
			{
				network.scanLastChannel = NetworkImplGetState().channel;
				memset(&network.activeScanResult, 0, sizeof(network.activeScanResult));
				NetworkImplSetMode(RADIO_MODE_SNIFFER);
				NetworkImplStartScan(network.scanLastChannel);
				newMode = NETWORK_MODE_SCANNING;
			}
		}
		break;
		case NETWORK_MODE_SCANNING:
		{
			if(event == NETWORK_EVENT_DISABLE)
			{
				NetworkImplSetMode(RADIO_MODE_OFF);
				newMode = NETWORK_MODE_DISABLED;
			}
			else if(event == NETWORK_EVENT_SCAN_COMPLETE)
			{
				newMode = NETWORK_MODE_VALIDATE_SCAN;
			}
		}
		break;
		case NETWORK_MODE_VALIDATE_SCAN:
		{
			if(event == NETWORK_EVENT_DISABLE)
			{
				NetworkImplSetMode(RADIO_MODE_OFF);
				newMode = NETWORK_MODE_DISABLED;
			}
			else if(event == NETWORK_EVENT_BROADCAST_RECEIVED)
			{
				NetworkImplState state = NetworkImplGetState();
				NetworkScanEntry* pEntry = findNetworkScanEntry(&network.activeScanResult, state.channel);
				if(pEntry)
				{
					pEntry->isUsed = 1;
					pEntry->channel = state.channel;
					pEntry->baseStationId = network.lastBaseStationBroadcast.baseStationId;
					pEntry->allocatedBotIds = network.lastBaseStationBroadcast.allocatedBotIds;
					pEntry->rssi_dBm = (pEntry->rssi_dBm * 3.0f + state.lastBaseStationPacketRssi_dBm) / 4.0f;
				}
			}
			else if(timeInCurrentMode_ms > 50)
			{
				NetworkImplState state = NetworkImplGetState();
				if(state.channel == network.scanLastChannel)
				{
					// Scan complete
					const NetworkScanEntry* pEntry = findBestScanEntry(&network.activeScanResult);
					if(pEntry)
					{
						NetworkImplSetChannel(pEntry->channel);
						NetworkImplSetMode(RADIO_MODE_ACTIVE);
						newMode = NETWORK_MODE_VALIDATE_CHANNEL;
					}
					else
					{
						newMode = NETWORK_MODE_IDLE;
						chEvtAddEvents(EVENT_MASK_START_SCAN);
					}

					memcpy(&network.lastScanResult, &network.activeScanResult, sizeof(network.activeScanResult));
				}
				else
				{
					NetworkImplStartScan(network.scanLastChannel);
					newMode = NETWORK_MODE_SCANNING;
				}
			}
		}
		break;
		case NETWORK_MODE_VALIDATE_CHANNEL:
		{
			if(event == NETWORK_EVENT_BROADCAST_RECEIVED)
			{
				newMode = NETWORK_MODE_PAIRED;
			}
			else if(timeInCurrentMode_ms > 100)
			{
				newMode = NETWORK_MODE_IDLE;
				chEvtAddEvents(EVENT_MASK_START_SCAN);
			}
		}
		break;
		case NETWORK_MODE_PAIRED:
		{
			if(event == NETWORK_EVENT_DISABLE)
			{
				NetworkImplSetMode(RADIO_MODE_OFF);
				newMode = NETWORK_MODE_DISABLED;
			}
			else if(event == NETWORK_EVENT_SCAN_START)
			{
				newMode = NETWORK_MODE_IDLE;
				chEvtAddEvents(EVENT_MASK_START_SCAN);
			}
		}
		break;
	}

	if(newMode != network.mode)
	{
		network.modeEntryTime = SysTimeUSec();
		network.mode = newMode;
	}
}

// network pre-processing function
// returns 0 if packet has been fully handled and should not be passed on to implementation
static uint8_t processPacket(PacketHeader* pHeader, uint8_t* pBuf, uint16_t dataLength)
{
	switch(pHeader->cmd)
	{
		case CMD_SYSTEM_PING:
		{
			PacketHeader header;
			header.cmd = CMD_SYSTEM_PONG;

			NetworkSendPacket(&header, pBuf, dataLength);

			return PROCESSED;
		}
		case CMD_SYSTEM_ACK:
		{
			uint16_t* pAckSeq = (uint16_t*)pBuf;

			uint8_t* pBufRel = 0;
			uint32_t relCmdSize;

			if(FifoLinGet(&network.reliableCmds, &pBufRel, &relCmdSize) == 0)
			{
				PacketHeaderEx* pWaitingCmdHeader = (PacketHeaderEx*)pBufRel;

				if(*pAckSeq == pWaitingCmdHeader->seq)
				{
					FifoLinDelete(&network.reliableCmds);
					chEvtAddEvents(EVENT_MASK_RELIABLE_CMD_ACKD);
				}
			}

			return PROCESSED;
		}
		case CMD_SYSTEM_MATCH_CTRL:
		{
			SystemMatchCtrl* pCtrl = (SystemMatchCtrl*)pBuf;

			chMtxLock(&network.matchCtrlMutex);

			memcpy(&network.lastMatchCtrlCmd, pCtrl, sizeof(SystemMatchCtrl));
			network.matchCtrlTime = SysTimeUSec();
			network.matchCtrlUpdated = 1;

			chMtxUnlock(&network.matchCtrlMutex);

			return PROCESSED;
		}
		case CMD_SYSTEM_VERSION:
		{
			RobotSendVersionInfo();

			return PROCESSED;
		}
		case CMD_DATA_ACQ_SET_MODE:
		{
			DataAcqSetMode* pMode = (DataAcqSetMode*)pBuf;
			robot.dataAcquisitionMode = pMode->mode;

			return PROCESSED;
		}
		case CMD_BASE_STATION_BROADCAST:
		{
			if(dataLength < sizeof(BaseStationBroadcast))
			{
				LogErrorC("BaseStationBroadcast too short", dataLength);
				return PROCESSED;
			}

			BaseStationBroadcast* pBroadcast = (BaseStationBroadcast*)pBuf;

			chMtxLock(&network.baseStationBroadcastMutex);

			memcpy(&network.lastBaseStationBroadcast, pBroadcast, sizeof(BaseStationBroadcast));
			network.baseStationBroadcastTime = SysTimeUSec();

			if(network.mode == NETWORK_MODE_PAIRED)
			{
				network.baseStationBroadcastUpdated = 1;

				if(pBroadcast->unixTime)
					SysTimeSetUnixTime(pBroadcast->unixTime);
			}

			chMtxUnlock(&network.baseStationBroadcastMutex);

			networkModeStatemachine(NETWORK_EVENT_BROADCAST_RECEIVED);

			return PROCESSED;
		}
		default:
			return PASS_ON;
	}
}

SHELL_CMD(config, "Show current network configuration");

SHELL_CMD_IMPL(config)
{
	(void)pUser; (void)argc; (void)argv;

	printf("ID: %hu\r\n", network.config.botId);
}

SHELL_CMD(stats, "Show network statistics");

SHELL_CMD_IMPL(stats)
{
	(void)pUser; (void)argc; (void)argv;

	NetworkImplState state = NetworkImplGetState();

	printf("Mode: %s\r\n", NetworkGetModeString(network.mode));
	printf("Channel: %hu\r\n", state.channel);

	NetworkScanResult scan;
	memcpy(&scan, &network.lastScanResult, sizeof(NetworkScanResult));

	const size_t numEntries = sizeof(scan.baseStations) / sizeof(scan.baseStations[0]);

	printf("Last scan result:\r\n");
	printf(" CH ID     BotIDs    RSSI\r\n");
	for(size_t i = 0; i < numEntries; i++)
	{
		NetworkScanEntry* pEntry = &scan.baseStations[i];
		if(!pEntry->isUsed)
			continue;

		printf("%3hu %2hu 0x%08X %7.2f\r\n", pEntry->channel, pEntry->baseStationId, pEntry->allocatedBotIds, pEntry->rssi_dBm);
	}
}

SHELL_CMD(scan, "Start a network scan");

SHELL_CMD_IMPL(scan)
{
	(void)pUser; (void)argc; (void)argv;

	NetworkStartScan();

	printf("Scan initiated.\r\n");
}

SHELL_CMD(set_color, "Set team color, will trigger a reboot",
	SHELL_ARG(color, "y=yellow, b=blue")
);

SHELL_CMD_IMPL(set_color)
{
	(void)pUser; (void)argc;

	if(*argv[1] == 'b')
	{
		if(network.config.botId < CMD_BOT_COUNT_HALF)
		{
			network.config.botId += CMD_BOT_COUNT_HALF;
			NetworkSaveConfig();
			printf("Changed BotID to %hu", network.config.botId);
			chThdSleepMilliseconds(100);
			NVIC_SystemReset();
		}
	}
	else if(*argv[1] == 'y')
	{
		if(network.config.botId > CMD_BOT_HALF_MIN_1)
		{
			network.config.botId -= CMD_BOT_COUNT_HALF;
			NetworkSaveConfig();
			printf("Changed BotID to %hu", network.config.botId);
			chThdSleepMilliseconds(100);
			NVIC_SystemReset();
		}
	}
	else
	{
		fprintf(stderr, "Unknown color %c\r\n", *argv[1]);
	}
}

SHELL_CMD(set_id, "Set bot ID",
	SHELL_ARG(color, "Team color <y|b>"),
	SHELL_ARG(id, "New bot ID <0-15>")
);

SHELL_CMD_IMPL(set_id)
{
	(void)pUser; (void)argc;

	int id = atoi(argv[2]);

	if(id > 15)
	{
		fprintf(stderr, "Invalid bot ID\r\n");
		return;
	}

	if(*argv[1] == 'b')
	{
		id += CMD_BOT_COUNT_HALF;
	}
	else if(*argv[1] == 'y')
	{
	}
	else
	{
		fprintf(stderr, "Unknown color %c\r\n", *argv[1]);
		return;
	}

	network.config.botId = id;
	NetworkSaveConfig();

	printf("Changed BotID to %d\r\n", id);
}

static void registerShellCommands(ShellCmdHandler* pHandler)
{
	ShellCmdAdd(pHandler, stats_command);
	ShellCmdAdd(pHandler, scan_command);
	ShellCmdAdd(pHandler, config_command);
	ShellCmdAdd(pHandler, set_color_command);
	ShellCmdAdd(pHandler, set_id_command);
}
