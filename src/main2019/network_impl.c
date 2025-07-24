#include "robot/network.h"

#include "util/config.h"
#include "tiger_bot.h"
#include "dev/shell.h"
#include "util/log.h"

NetworkImplState NetworkImplGetState()
{
	NetworkImplState state;

	state.isSendBufferEmpty = RadioBotIsTxEmpty(&tigerBot.radioBot);
	state.isBaseOnline = tigerBot.radioBot.stats.isBaseOnline;
	state.radioMode = tigerBot.radioBot.mode;
	state.channel = tigerBot.radioBot.activeChannel;
	state.lastBaseStationPacketRssi_dBm = tigerBot.radioBot.stats.lastReceivedRssiFromBaseStation_mdBm * 1e-3f;

	return state;
}

void NetworkImplSetMode(RadioMode mode)
{
	RadioBotSetMode(&tigerBot.radioBot, mode);
}

int16_t NetworkImplSendPacket(const uint8_t* pData, uint32_t dataSize)
{
	return RadioBotSendPacket(&tigerBot.radioBot, pData, dataSize);
}

int16_t NetworkImplGetPacketFromBase(uint8_t dstId, uint8_t* pDst, uint32_t dstSize, uint32_t* pPktSize)
{
	return RadioBotGetPacketFromBase(&tigerBot.radioBot, dstId, pDst, dstSize, pPktSize);
}

int16_t NetworkImplSetBotId(uint8_t botId)
{
	return RadioBotSetBotId(&tigerBot.radioBot, botId);
}

void NetworkImplStartScan(uint8_t lastChannel)
{
	RadioBotStartScan(&tigerBot.radioBot, lastChannel);
}

void NetworkImplSetChannel(uint8_t channel)
{
	RadioBotSetChannel(&tigerBot.radioBot, channel);
}

event_source_t* NetworkImplGetRadioEventSource()
{
	return &tigerBot.radioBot.eventSource;
}

// this is the main network processing function
void NetworkImplProcessPacket(PacketHeader* pHeader, uint8_t* pBuf, uint16_t dataLength)
{
	switch(pHeader->cmd)
	{
		case CMD_SYSTEM_CONSOLE_COMMAND:
		{
			uint32_t length = dataLength - sizeof(SystemConsoleCommand);
			DevShellInject(pBuf + sizeof(SystemConsoleCommand), length);
		}
		break;
		case CMD_CONFIG_QUERY_FILE_LIST:
		case CMD_CONFIG_FILE_STRUCTURE:
		case CMD_CONFIG_ITEM_DESC:
		case CMD_CONFIG_READ:
		case CMD_CONFIG_WRITE:
			ConfigNetworkInput(pHeader, pBuf, dataLength);
			break;
		default:
			break;
	}
}
