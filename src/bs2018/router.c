#include "router.h"
#include "constants.h"
#include "base_station.h"
#include "hal/sys_time.h"
#include "util/log.h"
#include "errors.h"
#include "gui/robot_status.h"
#include "robot/skill_basics.h"

#define EVENT_MASK_TX_PREPARE_BOT		EVENT_MASK(0)
#define EVENT_MASK_TX_PREPARE_BROADCAST	EVENT_MASK(1)

#define EVENT_MASK_RX_BOT_DATA			EVENT_MASK(0)

static void routerTxPrepareTask(void*);
static void prepareBroadcastData();
static void routerRxTask(void*);
static void prepareBotTxData(uint8_t dstId);
static int16_t handleBotRxData(uint8_t srcId);

Router router;

void RouterInit()
{
	for(size_t i = 0; i < RADIO_NUM_ROBOT_CLIENTS; i++)
	{
		chMtxObjectInit(&router.bots[i].matchCtrlMtx);

		PacketHeader* pHeader = (PacketHeader*)router.bots[i].matchCtrlData;
		pHeader->section = SECTION_SYSTEM;
		pHeader->cmd = CMD_SYSTEM_MATCH_CTRL;

		router.bots[i].pMatchCtrl = (SystemMatchCtrl*)(router.bots[i].matchCtrlData + sizeof(PacketHeader));
	}

	router.pRxTask = chThdCreateStatic(router.waRxTask, sizeof(router.waRxTask), TASK_PRIO_ROUTER_RX, &routerRxTask, 0);
	router.pTxPrepareTask = chThdCreateStatic(router.waTxPrepareTask, sizeof(router.waTxPrepareTask), TASK_PRIO_ROUTER_TX, &routerTxPrepareTask, 0);
}

int16_t RouterSendBotData(uint8_t botId, const uint8_t* pCmdData, uint16_t dataLength)
{
	if(botId > RADIO_NUM_ROBOT_CLIENTS)
		return ERROR_INVALID_PARAMETER;

	RouterClient* pBot = &router.bots[botId];
	RadioClientRx* pRadioClient = &baseStation.radioBase.baseIn[botId];

	if(!pRadioClient->isOnline)
		return ERROR_RESSOURCE_UNAVAILABLE;

	if(dataLength < sizeof(PacketHeader))
		return ERROR_INVALID_PARAMETER;

	const PacketHeader* pHeader = (const PacketHeader*)pCmdData;

	// intercept match ctrl commands
	if(pHeader->section == SECTION_SYSTEM && pHeader->cmd == CMD_SYSTEM_MATCH_CTRL)
	{
		const uint8_t* pData = pCmdData + sizeof(PacketHeader);
		uint16_t cmdLength = dataLength - sizeof(PacketHeader);

		if(cmdLength >= sizeof(SystemMatchCtrl) - SYSTEM_MATCH_CTRL_USER_DATA_SIZE)
		{
			chMtxLock(&pBot->matchCtrlMtx);
			memcpy(pBot->pMatchCtrl, pData, cmdLength);
			pBot->matchCtrlSize = cmdLength;
			chMtxUnlock(&pBot->matchCtrlMtx);
		}
		else
		{
			LogErrorC("Invalid match ctrl size", cmdLength | ((uint32_t)botId << 16));
			return ERROR_INVALID_PARAMETER;
		}
	}
	else
	{
		return RadioBaseSendPacket(&baseStation.radioBase, botId, pCmdData, dataLength);
	}

	return 0;
}

static void routerTxPrepareTask(void*)
{
	chRegSetThreadName("ROUTER_TX");

	event_listener_t prepareTxBotListener;
	event_listener_t prepareTxBroadcastListener;

	chEvtRegisterMask(&baseStation.radioBase.eventSourcePrepareTx, &prepareTxBotListener, EVENT_MASK_TX_PREPARE_BOT);
	chEvtRegisterMaskWithFlags(&baseStation.radioBase.eventSource, &prepareTxBroadcastListener, EVENT_MASK_TX_PREPARE_BROADCAST, RADIO_BASE_EVENT_PREPARE_BROADCAST);

	while(1)
	{
		eventmask_t events = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(10));

		if(events & EVENT_MASK_TX_PREPARE_BOT)
		{
			eventflags_t flags = chEvtGetAndClearFlags(&prepareTxBotListener);

			for(size_t i = 0; i < 32; i++)
			{
				if(flags & (1 << i))
				{
					prepareBotTxData(i);
				}
			}
		}

		if(events & EVENT_MASK_TX_PREPARE_BROADCAST)
		{
			prepareBroadcastData();
		}
	}
}

static void routerRxTask(void*)
{
	chRegSetThreadName("ROUTER_RX");

	event_listener_t rxBotListener;

	chEvtRegisterMaskWithFlags(&baseStation.radioBase.eventSource, &rxBotListener, EVENT_MASK_RX_BOT_DATA, RADIO_BASE_EVENT_PACKET_RECEIVED);

	while(1)
	{
		eventmask_t events = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(10));

		if(events & EVENT_MASK_RX_BOT_DATA)
		{
			// go through all bots and move data to network in an ACommand wrapper
			for(size_t i = 0; i < RADIO_NUM_ROBOT_CLIENTS; i++)
			{
				while(handleBotRxData(i) == 0);
			}
		}
	}
}

static void velModeBasicEntrys(VelInput* pVel)
{
	pVel->xyw[0] = robotStatus.ultraState.x * 1000.0f;
	pVel->xyw[1] = robotStatus.ultraState.y * 1000.0f;
	pVel->xyw[2] = robotStatus.ultraState.w * 1000.0f;
	pVel->jerkMaxXY = 0; // deprecated
	pVel->jerkMaxW = 0;
	pVel->accMaxXY = (uint8_t)(robotStatus.limits.maxAccXY / LOCAL_VEL_MAX_ACC_XY * 255.0f);
	pVel->accMaxW = (uint8_t)(robotStatus.limits.maxAccW / LOCAL_VEL_MAX_ACC_W * 255.0f);
}

static void prepareBotTxData(uint8_t dstId)
{
	RouterClient* pBot = &router.bots[dstId];
	SystemMatchCtrl* pCtrl = pBot->pMatchCtrl;
	SslVisionObject* pVisionObj = &baseStation.vision.robots[dstId];

	uint8_t ctrlInputAvailable = baseStation.sumatra.isOnline;

	if(robotStatus.manualControlOn && robotStatus.selectedBotId == dstId)
	{
		ctrlInputAvailable = 1;

		if(RobotStatusIsChargeKicker())
			pCtrl->flags = SYSTEM_MATCH_CTRL_FLAGS_KICKER_AUTOCHG;
		else
			pCtrl->flags = 0;

		VelInput* pVel = (VelInput*)pCtrl->skillData;
		pBot->matchCtrlSize = sizeof(SystemMatchCtrl) - SYSTEM_MATCH_CTRL_USER_DATA_SIZE;

		switch(robotStatus.ultraState.mode)
		{
			case MOVE_MODE_OFF:
				pCtrl->skillId = 0;
				velModeBasicEntrys(pVel);
				break;
			case MOVE_MODE_VEL_XY:
			case MOVE_MODE_VEL_YW:
				pCtrl->skillId = 2;
				pBot->matchCtrlSize += sizeof(VelInput);
				velModeBasicEntrys(pVel);
				break;
		}

		if(robotStatus.dribblerOn)
			SkillBasicsSetKDInputDribbler(&pVel->kd, robotStatus.limits.dribbleSpeed, robotStatus.limits.dribbleForce);
		else
			SkillBasicsSetKDInputDribbler(&pVel->kd, 0.0f, 0.0f);

		if(robotStatus.kickState.mode == KICK_MODE_ARM_STRAIGHT)
			SkillBasicsSetKDInputKicker(&pVel->kd, KICKER_MODE_ARM, KICKER_DEVICE_STRAIGHT, robotStatus.kickState.speed);
		else if(robotStatus.kickState.mode == KICK_MODE_ARM_CHIP)
			SkillBasicsSetKDInputKicker(&pVel->kd, KICKER_MODE_ARM, KICKER_DEVICE_CHIP, robotStatus.kickState.speed);
		else
			SkillBasicsSetKDInputKicker(&pVel->kd, KICKER_MODE_DISARM, KICKER_DEVICE_STRAIGHT, 0.0f);
	}

	uint8_t txIdle = RadioBaseIsTxEmpty(&baseStation.radioBase, dstId);
	if((txIdle || chVTTimeElapsedSinceX(pBot->tLastMatchCtrlSend) > TIME_MS2I(100)) && ctrlInputAvailable)
	{
		pBot->tLastMatchCtrlSend = chVTGetSystemTimeX();

		chMtxLock(&pBot->matchCtrlMtx);
		chMtxLock(&pVisionObj->accessMtx);

		if(pBot->tLastVisionCapture_us != pVisionObj->tCapture_us)
		{
			pBot->tLastVisionCapture_us = pVisionObj->tCapture_us;

			pCtrl->curPosition[0] = pVisionObj->position_m[0] * 1000.0f + 0.5f;
			pCtrl->curPosition[1] = pVisionObj->position_m[1] * 1000.0f + 0.5f;
			pCtrl->curPosition[2] = pVisionObj->orientation_rad * 1000.0f + 0.5f;
			pCtrl->camId = pVisionObj->camIdOfLastDetection;

			uint32_t delay = (SysTimeUSec() - pVisionObj->tCapture_us)/250;
			if(delay > 255)
				delay = 255;

			pCtrl->posDelay = delay;
		}
		else
		{
			pCtrl->curPosition[0] = SYSTEM_MATCH_CTRL_UNUSED_FIELD;
			pCtrl->curPosition[1] = SYSTEM_MATCH_CTRL_UNUSED_FIELD;
			pCtrl->curPosition[2] = SYSTEM_MATCH_CTRL_UNUSED_FIELD;
			pCtrl->posDelay = 255;
			pCtrl->camId = 0;
		}

		chMtxUnlock(&pVisionObj->accessMtx);

		if(!baseStation.vision.isOnline)
			pCtrl->camId |= SYSTEM_MATCH_CTRL_CAM_ID_FLAG_NO_VISION;

		RadioBaseSendPacket(&baseStation.radioBase, dstId, pBot->matchCtrlData, pBot->matchCtrlSize + sizeof(PacketHeader));

		chMtxUnlock(&pBot->matchCtrlMtx);
	}
}

static void prepareBroadcastData()
{
	// TODO: Could send timesync here
	// TODO: Could send field geometry here
}

static int16_t handleBotRxData(uint8_t srcId)
{
	uint32_t pktSize = 0;
	int16_t result = RadioBaseGetPacket(&baseStation.radioBase, srcId, router.rxProcBuf+sizeof(BaseStationACommand), sizeof(router.rxProcBuf)-sizeof(BaseStationACommand), &pktSize);
	if(result)
		return result;

	// intercept and save robot feedback
	PacketHeader* pHeader = (PacketHeader*)(router.rxProcBuf+sizeof(BaseStationACommand));
	if(pHeader->section == SECTION_SYSTEM && pHeader->cmd == CMD_SYSTEM_MATCH_FEEDBACK && pktSize >= (sizeof(PacketHeader)+sizeof(SystemMatchFeedback)))
	{
		SystemMatchFeedback* pFeedback = (SystemMatchFeedback*)&router.rxProcBuf[sizeof(BaseStationACommand)+sizeof(PacketHeader)];
		memcpy(&router.bots[srcId].lastMatchFeedback, pFeedback, sizeof(SystemMatchFeedback));
	}

	if(!baseStation.sumatra.isOnline)
		return 0;

	// construct header
	PacketHeader header;
	header.section = SECTION_BASE_STATION;
	header.cmd = CMD_BASE_STATION_ACOMMAND;

	// construct BaseStationACommand
	BaseStationACommand* pACmd = (BaseStationACommand*)router.rxProcBuf;
	pACmd->id = srcId;
	pktSize++;

	BaseStationSendSumatraPacket(&header, router.rxProcBuf, pktSize);

	return 0;
}
