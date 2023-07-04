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
	ExtUpdateProgress updateProgress;

	ExtCameraConfig cameraConfig;
	ExtCameraControl cameraControl;
	ExtCameraStats cameraStats;
	ExtCameraCalibration cameraCalibration;

	ExtColorClassifierConfig colorClassifierConfig;
	ExtBallLocalisationConfig ballLocalisationConfig;
	ExtPointDistanceSensorConfig pointDistConfig;

	uint8_t enablePreview;
	uint32_t enabledSteps;
	uint32_t debugSteps;
	uint8_t debugLevel;

	ConfigFile* pConfigFileCameraConfig;
	ConfigFile* pConfigFileBallLocalisationConfig;
	ConfigFile* pConfigFileColorClassifierConfig;

	ExtBallDetections ballDetections;
	uint8_t ballDetectionsUpdated;
	uint32_t currentTrackerId;
	int32_t lastDetectionLatency;

	ExtPointDistSensor pointDistanceSensor;
	uint8_t pointDistanceSensorUpdated;
} RobotPi;

extern RobotPi robotPi;

void RobotPiInit();
void RobotPiTask(void* params);
void RobotPiHandleExtPacket(const PacketHeader* pHeader, const uint8_t* pData, uint32_t dataLength);
void RobotPiTriggerImageCapture();
void RobotPiUpdateBallSensorData(RobotSensors* pSensors);
void RobotPiUpdatePointDistanceSensorData(RobotSensors* pSensors);
