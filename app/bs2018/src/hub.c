/*
 * hub.c
 *
 *  Created on: 14.12.2017
 *      Author: AndreR
 */

#include "hub.h"

#include "util/console.h"
#include "util/log.h"
#include "util/angle_math.h"
#include "util/sys_time.h"

#include "network.h"
#include "gui/robot_status.h"
#include "main/skill_basics.h"

#include <math.h>
#include <string.h>

Hub hub;

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

static void wifiProcess(WifiRobot* pBot)
{
	HubStorage* pStore = &hub.robot[pBot->id];
	SystemMatchCtrl* pCtrl = pStore->pMatchCtrl;

	uint8_t ctrlInputAvailable = network.sumatraAvailable;

	if(robotStatus.manualControlOn && robotStatus.selectedBotId == pBot->id)
	{
		ctrlInputAvailable = 1;

		if(RobotStatusIsChargeKicker())
			pCtrl->flags = SYSTEM_MATCH_CTRL_FLAGS_KICKER_AUTOCHG;
		else
			pCtrl->flags = 0;

		VelInput* pVel = (VelInput*)pCtrl->skillData;
		pStore->matchCtrlSize = sizeof(SystemMatchCtrl) - SYSTEM_MATCH_CTRL_USER_DATA_SIZE;

		switch(robotStatus.ultraState.mode)
		{
			case MOVE_MODE_OFF:
				pCtrl->skillId = 0;
				velModeBasicEntrys(pVel);
				break;
			case MOVE_MODE_VEL_XY:
			case MOVE_MODE_VEL_YW:
				pCtrl->skillId = 2;
				pStore->matchCtrlSize += sizeof(VelInput);
				velModeBasicEntrys(pVel);
				break;
			case MOVE_MODE_POS:
				pCtrl->skillId = 4;
				pStore->matchCtrlSize += sizeof(GlobalPosInput);
				GlobalPosInput* pPos = (GlobalPosInput*)pCtrl->skillData;

				pPos->velMaxXY = (uint8_t)(robotStatus.limits.maxVelXY / GLOBAL_POS_MAX_VEL_XY * 255.0f);
				pPos->velMaxW = (uint8_t)(robotStatus.limits.maxVelW / GLOBAL_POS_MAX_VEL_W * 255.0f);
				pPos->accMaxXY = (uint8_t)(robotStatus.limits.maxAccXY / GLOBAL_POS_MAX_ACC_XY * 255.0f);
				pPos->accMaxW = (uint8_t)(robotStatus.limits.maxAccW / GLOBAL_POS_MAX_ACC_W * 255.0f);
				pPos->xyw[0] = robotStatus.ultraState.x;
				pPos->xyw[1] = robotStatus.ultraState.y;
				pPos->xyw[2] = robotStatus.ultraState.w;
				pPos->primaryDirection = 0;
				break;
		}

		pVel->kd.dribblerSpeed = robotStatus.dribbleState/100;
		pVel->kd.kickFlags = 0;

		if(robotStatus.kickState.mode == KICK_MODE_ARM_STRAIGHT)
			pVel->kd.kickFlags |= KICKER_DEVICE_STRAIGHT | (KICKER_MODE_ARM << 4);

		if(robotStatus.kickState.mode == KICK_MODE_ARM_CHIP)
			pVel->kd.kickFlags |= KICKER_DEVICE_CHIP | (KICKER_MODE_ARM << 4);

		pVel->kd.kickSpeed = (uint8_t)(robotStatus.kickState.speed*25.0f);
	}

	uint8_t txIdle = pBot->queue.txFifo.numPackets == 0 && pBot->queue.txBufUsed == 0;
	if((txIdle || chVTTimeElapsedSinceX(pStore->lastMatchCtrlSentTime) > MS2ST(100)) && ctrlInputAvailable)
	{
		pStore->lastMatchCtrlSentTime = chVTGetSystemTimeX();

		memcpy(pCtrl->curPosition, pStore->lastVisionPos, 3*sizeof(int16_t));
		uint32_t delay = (SysTimeUSec() - pStore->tVisionCapture)/250;
		if(delay > 255)
			delay = 255;
		pCtrl->posDelay = delay;
		pCtrl->camId = pStore->lastVisionCamId;
		if(network.visionAvailable == 0)
			pCtrl->camId |= SYSTEM_MATCH_CTRL_CAM_ID_FLAG_NO_VISION;

		pStore->lastVisionPos[0] = SYSTEM_MATCH_CTRL_UNUSED_FIELD;
		pStore->lastVisionPos[1] = SYSTEM_MATCH_CTRL_UNUSED_FIELD;
		pStore->lastVisionPos[2] = SYSTEM_MATCH_CTRL_UNUSED_FIELD;

		RFQueueSend(&pBot->queue, pStore->matchCtrlData, pStore->matchCtrlSize+sizeof(PacketHeader));
	}

	if(chVTTimeElapsedSinceX(pStore->lastTimeSync) > S2ST(5) && sysTime.unixOffset != 0)
	{
		pStore->lastTimeSync = chVTGetSystemTimeX();

		uint8_t data[sizeof(PacketHeader)+sizeof(uint32_t)];

		PacketHeader* pHeader = (PacketHeader*)data;
		pHeader->section = SECTION_SYSTEM;
		pHeader->cmd = CMD_SYSTEM_TIMESYNC;

		uint32_t* pTime = (uint32_t*)(data+sizeof(PacketHeader));
		*pTime = SysTimeUnix();

		RFQueueSend(&pBot->queue, data, sizeof(PacketHeader)+sizeof(uint32_t));
	}

	if(chVTTimeElapsedSinceX(pStore->visionCamRxTime) > MS2ST(100))
	{
		pStore->isOnVision = 0;
	}
}

void HubInit()
{
	for(uint8_t i = 0; i < WIFI_MAX_BOTS; i++)
	{
		PacketHeader* pHeader = (PacketHeader*)hub.robot[i].matchCtrlData;
		pHeader->section = SECTION_SYSTEM;
		pHeader->cmd = CMD_SYSTEM_MATCH_CTRL;

		hub.robot[i].pMatchCtrl = (SystemMatchCtrl*)(hub.robot[i].matchCtrlData+sizeof(PacketHeader));
		hub.robot[i].matchCtrlSize = 0;

		hub.robot[i].lastVisionPos[0] = SYSTEM_MATCH_CTRL_UNUSED_FIELD;
		hub.robot[i].lastVisionPos[1] = SYSTEM_MATCH_CTRL_UNUSED_FIELD;
		hub.robot[i].lastVisionPos[2] = SYSTEM_MATCH_CTRL_UNUSED_FIELD;

		hub.field.fieldLength = 0;
	}

	WifiSetHookBotProcess(&wifiProcess);
}

void HubSetMatchFeedback(uint8_t botId, const SystemMatchFeedback* pFeedback)
{
	memcpy(&hub.robot[botId].lastMatchFeedback, pFeedback, sizeof(SystemMatchFeedback));
}

void HubNetworkInput(uint8_t botId, const uint8_t* pCmdData, uint16_t dataLength)
{
	if(botId > WIFI_MAX_BOT_ID)
	{
		ConsolePrint("Invalid bot id %hu\r\n", botId);
		return;
	}

	WifiRobot* pBot = &wifi.bots[botId];
	if(!pBot->online)
	{
		ConsolePrint("Data for unconnected bot %hu\r\n", botId);
		return;
	}

	PacketHeader* pHeader = (PacketHeader*)pCmdData;

	// intercept match ctrl commands
	if(pHeader->section == SECTION_SYSTEM && pHeader->cmd == CMD_SYSTEM_MATCH_CTRL)
	{
		const uint8_t* pData = pCmdData+sizeof(PacketHeader);
		uint16_t cmdLength = dataLength - sizeof(PacketHeader);

		if(cmdLength >= sizeof(SystemMatchCtrl)-SYSTEM_MATCH_CTRL_USER_DATA_SIZE)
		{
			memcpy(hub.robot[botId].pMatchCtrl, pData, cmdLength);
			hub.robot[botId].matchCtrlSize = cmdLength;
		}
		else
		{
			LogErrorC("Invalid match ctrl size", cmdLength | ((uint32_t)botId << 16));
		}
	}
	else
	{
		int16_t result = RFQueueSend(&pBot->queue, pCmdData, dataLength);
		if(result)
		{
			LogErrorC("No memory in queue\r\n", botId);
			return;
		}
	}
}

void HubVisionInput(uint16_t botId, float x, float y, float orient, uint32_t tCapture, uint8_t camId)
{
	if(botId > WIFI_MAX_BOT_ID)
	{
		ConsolePrint("Invalid pose id %hu\r\n", botId);
		return;
	}

	HubStorage* pBot = &hub.robot[botId];

	if(pBot->lastVisionCamId != camId && chVTTimeElapsedSinceX(pBot->visionCamRxTime) < MS2ST(50))
	{
		return;
	}

	const float maxVelXY = 5.0f;
	const float maxVelW = 50.0f;
	float visionDt = (tCapture-pBot->tVisionCapture)*1e-6f;

	float searchRadiusXY = maxVelXY*visionDt;
	float searchRadiusW = maxVelW*visionDt;

	float diffX = (x - pBot->lastVisionPosFloat[0])*1e-3f;
	float diffY = (y - pBot->lastVisionPosFloat[1])*1e-3f;
	float diffXY = sqrtf(diffX*diffX+diffY*diffY);
	float diffW = AngleNormalize(orient - pBot->lastVisionPosFloat[2]);

	if(diffXY > searchRadiusXY || fabsf(diffW) > searchRadiusW)
	{
		// this is an invalid sample, vel is impossible
		return;
	}

	pBot->lastVisionPosFloat[0] = x;
	pBot->lastVisionPosFloat[1] = y;
	pBot->lastVisionPosFloat[2] = orient;

	pBot->tVisionCapture = tCapture;
	pBot->lastVisionPos[0] = (int16_t)(x+0.5f);
	pBot->lastVisionPos[1] = (int16_t)(y+0.5f);
	pBot->lastVisionPos[2] = (int16_t)(orient*1000.0f+0.5f);
	pBot->lastVisionCamId = camId;
	pBot->visionCamRxTime = chVTGetSystemTimeX();
	pBot->isOnVision = 1;
}
