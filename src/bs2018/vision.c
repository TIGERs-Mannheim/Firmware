/*
 * vision.c
 *
 *  Created on: 22.05.2013
 *      Author: AndreR
 */

#include "vision.h"
#include "util/console.h"
#include "wifi.h"
#include "errors.h"
#include "util/sys_time.h"
#include <string.h>
#include "hub.h"

#include "protobuf-c/messages_robocup_ssl_wrapper.pb-c.h"

Vision vision;

static void* allocMem(void *allocator_data, size_t size)
{
	(void)allocator_data;

	if(vision.allocUsed + size > VISION_ALLOC_BUF_SIZE)
	{
		ConsolePrint("WARN: vision out of memory\r\n");
		return 0;
	}

	void* pMem = vision.pAllocBuf + vision.allocUsed;

	size_t pad = (4-(size%4))%4;

	vision.allocUsed += size+pad;

	return pMem;
}

static void freeMem(void *allocator_data, void *pointer)
{
	(void)allocator_data;
	(void)pointer;
}

static int16_t isInViewport(uint32_t camId, float xIn, float yIn)
{
	if(camId >= VISION_MAX_CAMS)
		return 1;

	VisionViewport* pView = &vision.cams[camId];

	int16_t x = (int16_t)xIn;
	int16_t y = (int16_t)yIn;

	if(x >= pView->minX && x <= pView->maxX && y >= pView->minY && y <= pView->maxY)
		return 1;

	return 0;
}

int16_t VisionInput(uint8_t* pData, uint16_t length)
{
	vision.allocUsed = 0;	// free vision buffer

	SSLWrapperPacket* pPacket = 0;
	pPacket = ssl__wrapper_packet__unpack(&vision.allocator, length, pData);
	if(pPacket == 0)
		return ERROR_VISION_PARSE;

	if(pPacket->geometry != 0)
	{
		SSLGeometryFieldSize* field = pPacket->geometry->field;
		hub.field.boundaryWidth = field->boundary_width;
		hub.field.fieldLength = field->field_length;
		hub.field.fieldWidth = field->field_width;
		hub.field.goalDepth = field->goal_depth;
		hub.field.goalWidth = field->goal_width;
	}

	if(pPacket->detection == 0)
		return 0;

	float procDelay = (float)(pPacket->detection->t_sent - pPacket->detection->t_capture);	// [s]
	procDelay += 0.001;	// 1ms transport delay
	if(procDelay < 0)
		procDelay = 0;

	uint32_t procDelayUs = (uint32_t)(procDelay*1e6f);
	uint32_t tCapture = SysTimeUSec()-procDelayUs;

	uint32_t camId = pPacket->detection->camera_id;

	// YELLOW run
	uint32_t numRobots = pPacket->detection->n_robots_yellow;
	SSLDetectionRobot** ppRobots = pPacket->detection->robots_yellow;
	for(uint32_t i = 0; i < numRobots; i++)
	{
		SSLDetectionRobot* pRobot = ppRobots[i];

		if(pRobot->has_robot_id == 0 || pRobot->has_orientation == 0)
			continue;

		if(isInViewport(camId, pRobot->x, pRobot->y))
			HubVisionInput(pRobot->robot_id, pRobot->x, pRobot->y, pRobot->orientation, tCapture, camId);
	}

	// BLUE run
	numRobots = pPacket->detection->n_robots_blue;
	ppRobots = pPacket->detection->robots_blue;
	for(uint32_t i = 0; i < numRobots; i++)
	{
		SSLDetectionRobot* pRobot = ppRobots[i];

		if(pRobot->has_robot_id == 0 || pRobot->has_orientation == 0)
			continue;

		if(isInViewport(camId, pRobot->x, pRobot->y))
			HubVisionInput(CMD_BOT_COUNT_HALF+pRobot->robot_id, pRobot->x, pRobot->y, pRobot->orientation, tCapture, camId);
	}

	return 0;
}

void VisionInit()
{
	memset(&vision, 0, sizeof(Vision));

	vision.allocator.alloc = &allocMem;
	vision.allocator.free = &freeMem;
	vision.allocator.allocator_data = 0;

	for(uint16_t i = 0; i < VISION_MAX_CAMS; i++)
	{
		vision.cams[i].minX = -32767;
		vision.cams[i].minY = -32767;
		vision.cams[i].maxX = 32767;
		vision.cams[i].maxY = 32767;
	}
}

void VisionUpdateViewport(const BaseStationCamViewport* pViewport)
{
	if(pViewport->camId >= VISION_MAX_CAMS)
		return;

	VisionViewport* pView = &vision.cams[pViewport->camId];

	pView->minX = pViewport->minX;
	pView->minY = pViewport->minY;
	pView->maxX = pViewport->maxX;
	pView->maxY = pViewport->maxY;
}
