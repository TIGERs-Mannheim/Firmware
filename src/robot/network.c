#include "network.h"
#include "util/log.h"
#include "robot.h"
#include "util/crc8.h"
#include "hal/sys_time.h"

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
static void registerShellCommands(ShellCmdHandler* pHandler);

static void configUpdateCallback(uint16_t cfgId)
{
	switch(cfgId)
	{
		case SID_CFG_NET_SETTINGS:
		{
			NetworkImplSetBotId(network.config.botId);
			NetworkImplSetChannel(network.config.channel);
		}
		break;
	}
}

void NetworkInit()
{
	// data setup
	chMtxObjectInit(&network.printMutex);
	chMtxObjectInit(&network.sendPacketMutex);
	chMtxObjectInit(&network.matchCtrlMutex);

	FifoLinInit(&network.reliableCmds, network.reliableBuf, NETWORK_RELIABLE_BUF_SIZE);
	chMBObjectInit(&network.relEvents, network.relEventsData, NETWORK_REL_EVENT_SIZE);

	network.linkDisabled = 1;

	network.pNetworkConfig = ConfigOpenOrCreate(&configFileDescSettings, &network.config, sizeof(ConfigNetwork), &configUpdateCallback, 1);

	NetworkImplSetBotId(network.config.botId);
	NetworkImplSetChannel(network.config.channel);

	ConfigNotifyUpdate(network.pNetworkConfig);

	network.matchCtrlTime = SysTimeUSec()-5000000;

	network.lastMatchCtrlCmd.curPosition[0] = SYSTEM_MATCH_CTRL_UNUSED_FIELD;
	network.lastMatchCtrlCmd.curPosition[1] = SYSTEM_MATCH_CTRL_UNUSED_FIELD;
	network.lastMatchCtrlCmd.curPosition[2] = SYSTEM_MATCH_CTRL_UNUSED_FIELD;

	ShellCmdHandlerInit(&network.cmdHandler, 0);
	registerShellCommands(&network.cmdHandler);
}

void NetworkSaveConfig()
{
	ConfigNotifyUpdate(network.pNetworkConfig);
}

void NetworkTask(void* params)
{
	(void)params;

	chRegSetThreadName("Network");

	while(1)
	{
		if(network.linkDisabled)
		{
			NetworkImplSetMode(RADIO_MODE_OFF);
			chThdSleep(TIME_MS2I(100));
			continue;
		}
		else
		{
			NetworkImplSetMode(RADIO_MODE_ACTIVE);
		}

		uint32_t len;
		while(NetworkImplGetPacket(network.rxProcessingBuf, NETWORK_RX_PROCESSING_SIZE, &len) == 0)
		{
			if(len <= PACKET_HEADER_SIZE)
				continue;

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

		chThdSleepMilliseconds(2);
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
	uint32_t lastSendTime = chVTGetSystemTimeX()-TIME_MS2I(200);

	chRegSetThreadName("NetRel");

	while(1)
	{
		chResult = chMBFetchTimeout(&network.relEvents, &event, TIME_MS2I(10));

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

				lastSendTime = chVTGetSystemTimeX()-TIME_MS2I(200);
			}
		}

		if(pHeaderEx && chVTTimeElapsedSinceX(lastSendTime) > TIME_MS2I(100))
		{
			chMtxLock(&network.sendPacketMutex);	// prevent re-entering

			NetworkImplSendPacket(pBuf, cmdSize);

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
	int16_t result = NetworkImplSendPacket(network.txProcessingBuf, dataLength+PACKET_HEADER_SIZE);

	chMtxUnlock(&network.sendPacketMutex);

	return result;
}

void NetworkSendPacketRaw(void* _pData, uint16_t dataLength)
{
	uint8_t* pData = (uint8_t*)_pData;

	chMtxLock(&network.sendPacketMutex);	// prevent re-entering

	NetworkImplSendPacket(pData, dataLength);

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

	chMBPostTimeout(&network.relEvents, EVENT_REL_RUN, TIME_IMMEDIATE);

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

					chMBPostTimeout(&network.relEvents, EVENT_REL_ACK + *pSeq, TIME_IMMEDIATE);

					return PROCESSED;
				}
				break;
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

SHELL_CMD(config, "Show current network configuration");

SHELL_CMD_IMPL(config)
{
	(void)pUser; (void)argc; (void)argv;

	printf("ID: %hu\r\n", network.config.botId);
	printf("Channel: %hu\r\n", network.config.channel);
}

SHELL_CMD(stats, "Show network statistics");

SHELL_CMD_IMPL(stats)
{
	(void)pUser; (void)argc; (void)argv;

	printf("Link: %s\r\n", network.linkDisabled ? "Disabled" : "Enabled");
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

SHELL_CMD(set_ch, "Set wireless channel",
	SHELL_ARG(ch, "New channel")
);

SHELL_CMD_IMPL(set_ch)
{
	(void)pUser; (void)argc;

	int ch = atoi(argv[1]);

	if(ch > 255 || ch < 0)
	{
		fprintf(stderr, "Invalid channel\r\n");
		return;
	}

	network.config.channel = ch;
	NetworkSaveConfig();

	printf("Changed channel to %d\r\n", ch);
}

static void registerShellCommands(ShellCmdHandler* pHandler)
{
	ShellCmdAdd(pHandler, stats_command);
	ShellCmdAdd(pHandler, config_command);
	ShellCmdAdd(pHandler, set_ch_command);
	ShellCmdAdd(pHandler, set_color_command);
	ShellCmdAdd(pHandler, set_id_command);
}
