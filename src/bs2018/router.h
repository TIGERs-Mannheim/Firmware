#pragma once

#include "ch.h"
#include "commands.h"
#include "module/radio/radio_buf.h"

typedef struct _RouterClient
{
	mutex_t matchCtrlMtx;

	uint8_t matchCtrlData[sizeof(SystemMatchCtrl) + sizeof(PacketHeader)];
	SystemMatchCtrl* pMatchCtrl;
	uint16_t matchCtrlSize;
	systime_t tLastMatchCtrlSend;
	uint32_t tLastVisionCapture_us;

	SystemMatchFeedback lastMatchFeedback;
} RouterClient;

typedef struct _Router
{
	RouterClient bots[RADIO_NUM_ROBOT_CLIENTS];

	THD_WORKING_AREA(waRxTask, 2048);
	thread_t* pRxTask;

	THD_WORKING_AREA(waTxPrepareTask, 2048);
	thread_t* pTxPrepareTask;

	uint8_t rxProcBuf[RADIO_APP_MAX_PACKET_SIZE+2+sizeof(BaseStationACommand)];
} Router;

extern Router router;

void RouterInit();
int16_t RouterSendBotData(uint8_t botId, const uint8_t* pCmdData, uint16_t dataLength);
