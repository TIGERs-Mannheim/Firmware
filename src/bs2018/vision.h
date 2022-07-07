/*
 * vision.h
 *
 *  Created on: 23.10.2017
 *      Author: AndreR
 */

#pragma once

#include <stdint.h>
#include "constants.h"
#include "commands.h"

#include "protobuf-c/protobuf-c.h"

typedef struct _VisionViewport
{
	// units in [mm]
	int16_t minX;
	int16_t minY;
	int16_t maxX;
	int16_t maxY;
} VisionViewport;

typedef struct _Vision
{
	uint8_t pAllocBuf[VISION_ALLOC_BUF_SIZE];
	size_t allocUsed;

	ProtobufCAllocator allocator;

	VisionViewport cams[VISION_MAX_CAMS];
} Vision;

extern Vision vision;

void 	VisionInit();
int16_t VisionInput(uint8_t* pData, uint16_t length);
void	VisionUpdateViewport(const BaseStationCamViewport* pViewport);
