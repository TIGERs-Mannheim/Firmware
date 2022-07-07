/*
 * robot_pi.h
 *
 *  Created on: 08.11.2020
 *      Author: Andre
 */

#pragma once

#include "commands.h"
#include "util/config.h"

typedef struct _RobotPi
{
	ExtRobotPiVersion robotPiVersion;

	ExtCameraConfig cameraConfig;
	ExtCameraPreviewConfig previewConfig;
	ExtCameraControl cameraControl;
	ExtCameraStats cameraStats;
	ExtCameraIntrinsics cameraIntrinsics;
	ExtCameraExtrinsics cameraExtrinsics;

	ExtBallDetectionConfig ballDetectionConfig;
	ExtBallTrackerConfig ballTrackerConfig;

	ExtColorThresholds colorThresholdsOrange;

	ExtFieldInfo fieldInfo;

	ConfigFile* pConfigFileCameraConfig;

	ExtBallDetections ballDetections;
	uint8_t ballDetectionsUpdated;
	uint32_t currentTrackerId;
	int32_t lastDetectionLatency;

	ExtPositionDetection positionDetection;
	uint8_t positionDetectionUpdated;
	int32_t lastPositionLatency;

	ExtGoalDetections goalDetections;
	int32_t lastGoalLatency;
	uint8_t goalDetectionUpdated;
} RobotPi;

extern RobotPi robotPi;

void RobotPiInit();
void RobotPiTask(void* params);
void RobotPiHandleExtPacket(const PacketHeader* pHeader, const uint8_t* pData, uint32_t dataLength);
void RobotPiTriggerImageCapture();
void RobotPiUpdateBallSensorData(RobotSensors* pSensors);
