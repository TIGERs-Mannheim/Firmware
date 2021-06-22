/*
 * hub.h
 *
 *  Created on: 14.12.2017
 *      Author: AndreR
 */

#pragma once

#include "wifi.h"

typedef struct _HubStorage
{
	SystemMatchFeedback lastMatchFeedback;

	float lastVisionPosFloat[3];
	int16_t lastVisionPos[3];
	uint32_t tVisionCapture;	// SysTime, [us]
	uint8_t lastVisionCamId;
	systime_t visionCamRxTime;
	uint8_t isOnVision;

	uint32_t lastTimeSync;

	uint8_t matchCtrlData[sizeof(SystemMatchCtrl)+sizeof(PacketHeader)];
	SystemMatchCtrl* pMatchCtrl;
	uint16_t matchCtrlSize;
	uint32_t lastMatchCtrlSentTime;
} HubStorage;

typedef struct _Field
{
	int32_t fieldLength;
	int32_t fieldWidth;
	int32_t goalWidth;
	int32_t goalDepth;
	int32_t boundaryWidth;
} Field;

typedef struct _Hub
{
	HubStorage robot[WIFI_MAX_BOTS];
	Field field;
} Hub;

extern Hub hub;

void HubInit();
void HubSetMatchFeedback(uint8_t botId, const SystemMatchFeedback* pFeedback);
void HubNetworkInput(uint8_t botId, const uint8_t* pCmdData, uint16_t dataLength);
void HubVisionInput(uint16_t botId, float x, float y, float orient, uint32_t tCapture, uint8_t camId);
