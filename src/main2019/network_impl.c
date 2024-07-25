#include "constants.h"

#include "robot/network.h"

#include "util/config.h"
#include "tiger_bot.h"
#include "dev/shell.h"

#include <string.h>

void NetworkImplSetMode(RadioMode mode)
{
	RadioBotSetMode(&tigerBot.radioBot, mode);
}

uint8_t NetworkImplIsSendBufferEmpty()
{
	return RadioBotIsTxEmpty(&tigerBot.radioBot);
}

uint8_t NetworkImplIsBaseOnline()
{
	return tigerBot.radioBot.stats.isBaseOnline;
}

int16_t NetworkImplSendPacket(const uint8_t* pData, uint32_t dataSize)
{
	return RadioBotSendPacket(&tigerBot.radioBot, pData, dataSize);
}

int16_t NetworkImplGetPacket(uint8_t* pDst, uint32_t dstSize, uint32_t* pPktSize)
{
	return RadioBotGetMyPacketFromBase(&tigerBot.radioBot, pDst, dstSize, pPktSize);
}

void NetworkImplSetChannel(uint8_t channel)
{
	RadioBotSetChannel(&tigerBot.radioBot, channel);
}

int16_t NetworkImplSetBotId(uint8_t botId)
{
	return RadioBotSetBotId(&tigerBot.radioBot, botId);
}

// this is the main network processing function
void NetworkImplProcessPacket(PacketHeader* pHeader, uint8_t* pBuf, uint16_t dataLength)
{
	switch(pHeader->section)
	{
		case SECTION_SYSTEM:
		{
			switch(pHeader->cmd)
			{
				case CMD_SYSTEM_CONSOLE_COMMAND:
				{
					uint32_t length = dataLength-sizeof(SystemConsoleCommand);
					DevShellInject(pBuf+sizeof(SystemConsoleCommand), length);
				}
				break;
			}
		}
		break;

		case SECTION_CONFIG:
		{
			ConfigNetworkInput(pHeader, pBuf, dataLength);
		}
		break;

		default: break;
	}
}
