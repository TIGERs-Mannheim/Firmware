/*
 * robot_pi.cpp
 *
 *  Created on: 08.11.2020
 *      Author: AndreR
 */

#include "robot_pi.h"
#include "struct_ids.h"
#include "ext.h"
#include "util/log.h"
#include "util/sys_time.h"
#include "hal/tft.h"
#include "main/robot.h"
#include "gfx.h"
#include "src/gdisp/gdisp_driver.h"

RobotPi robotPi = {
	.cameraConfig = {
		.expAutoMode = 1,
		.expAnalogGain = 1.0f,
		.expDigitalGain = 1.0f,
		.expTimeUs = 10000,
		.wbAutoMode = 1,
		.wbRedGain = 1.0f,
		.wbBlueGain = 1.0f,
	},
	.previewConfig = {
		.enable = 0,
	},
	.cameraControl = {
		.resolution = 1,
		.recording = 0,
	},
	.cameraIntrinsics = {
		.resolution = { 2592, 1944 },
		.focalLength = 1342.0f,
		.principalPoint = { 1305.39f, 971.45f },
		.radialDistortion = { -0.279324740320405f, 0.0581955384718963f },
	},
	.cameraExtrinsics = {
		.cameraTiltDeg = 20.0f,
		.cameraPos = { 0, 0.0729f, 0.0701f },
		.groundClearance = 0.01f,
	},
	.ballDetectionConfig = {
		.topImageSkipFactor = 0.25f,
		.minBlobArea = 5,
		.ballDiameter = 0.043f,
		.greedyCircleFactor = 1.25f,
		.maxTrackers = 5,
		.trackerTimeout = 1.0f,
	},
	.ballTrackerConfig = {
		.modelError = 0.1f,
		.measError = 10.0f,
		.maxVelocity = 10.0f,
		.historySize = 10,
	},
	.colorThresholdsOrange = {
		.colorId = EXT_COLOR_THRESHOLDS_ID_ORANGE,
		.y = { 50, 255 },
		.u = { 0, 150 },
		.v = { 160, 255 },
	},
};

static const ConfigFileDesc configFileDescCameraConfig =
{ SID_CFG_CAMERA, 1, "camera", 7, (ElementDesc[]) {
	{  UINT8, "exp_auto", "bool", "Auto Exposure" },
	{  FLOAT, "analog_gain", "1.0-8.0", "Analog Gain" },
	{  FLOAT, "digital_gain", "1.0-2.0", "Digital Gain" },
	{ UINT32, "exp_time", "us", "Exposure Time" },
	{  UINT8, "wb_auto", "bool", "Auto White Balance" },
	{  FLOAT, "wb_gain_red", "0.9-1.9", "WB Gain Red" },
	{  FLOAT, "wb_gain_blue", "0.9-1.9", "WB Gain Blue" },
}};

void RobotPiInit()
{
	robotPi.currentTrackerId = 0xFFFFFFFF;
	robotPi.pConfigFileCameraConfig = ConfigOpenOrCreate(&configFileDescCameraConfig, &robotPi.cameraConfig, sizeof(ExtCameraConfig), 0, 0);
}

void RobotPiTask(void* params)
{
	(void)params;

	chRegSetThreadName("RobotPi");

	systime_t prev = chVTGetSystemTimeX();
	systime_t next = prev+US2ST(1000);

	uint8_t counter = 0;

	while(1)
	{
		PacketHeader header;
		header.section = SECTION_EXT;

		ExtRemoteTime remoteTime;
		remoteTime.timestampUs = SysTimeUSec();
		header.cmd = CMD_EXT_REMOTE_TIME;
		ExtSendPacket(&header, &remoteTime, sizeof(ExtRemoteTime));

		ExtRobotState state;
		state.timestampUs = robot.state.header.timestamp;
		memcpy(state.posGlobal, robot.state.pos, sizeof(float)*3);
		memcpy(state.velGlobal, robot.state.vel, sizeof(float)*3);
		memcpy(state.accGlobal, robot.state.acc, sizeof(float)*3);
		header.cmd = CMD_EXT_ROBOT_STATE;
		ExtSendPacket(&header, &state, sizeof(ExtRobotState));

		if(counter >= 10)
		{
			counter = 0;

			header.cmd = CMD_EXT_CAMERA_CONFIG;
			ExtSendPacket(&header, &robotPi.cameraConfig, sizeof(ExtCameraConfig));

			header.cmd = CMD_EXT_CAMERA_PREVIEW_CONFIG;
			ExtSendPacket(&header, &robotPi.previewConfig, sizeof(ExtCameraPreviewConfig));

			header.cmd = CMD_EXT_CAMERA_CONTROL;
			ExtSendPacket(&header, &robotPi.cameraControl, sizeof(ExtCameraControl));

			header.cmd = CMD_EXT_CAMERA_INTRINSICS;
			ExtSendPacket(&header, &robotPi.cameraIntrinsics, sizeof(ExtCameraIntrinsics));

			header.cmd = CMD_EXT_CAMERA_EXTRINSICS;
			ExtSendPacket(&header, &robotPi.cameraExtrinsics, sizeof(ExtCameraExtrinsics));

			header.cmd = CMD_EXT_BALL_DETECTION_CONFIG;
			ExtSendPacket(&header, &robotPi.ballDetectionConfig, sizeof(ExtBallDetectionConfig));

			header.cmd = CMD_EXT_BALL_TRACKER_CONFIG;
			ExtSendPacket(&header, &robotPi.ballTrackerConfig, sizeof(ExtBallTrackerConfig));

			header.cmd = CMD_EXT_COLOR_THRESHOLDS;
			ExtSendPacket(&header, &robotPi.colorThresholdsOrange, sizeof(ExtColorThresholds));
		}

		counter++;

		chThdSleepUntilWindowed(prev, next);
		prev = next;
		next += US2ST(1000);
	}
}

void RobotPiTriggerImageCapture()
{
	PacketHeader header;
	header.section = SECTION_EXT;
	header.cmd = CMD_EXT_CAMERA_TRIGGER_CAPTURE;

	ExtSendPacket(&header, 0, 0);
}

void RobotPiUpdateBallSensorData(RobotSensors* pSensors)
{
	if(robotPi.ballDetectionsUpdated)
	{
		pSensors->ball.time = robotPi.ballDetections.timestampUs;

		if(robotPi.ballDetections.numBalls == 0)
		{
			pSensors->ball.pos[0] = NAN;
			pSensors->ball.pos[1] = NAN;
			pSensors->ball.pos[2] = NAN;
			pSensors->ball.vel[0] = NAN;
			pSensors->ball.vel[1] = NAN;
			pSensors->ball.vel[2] = NAN;
			pSensors->ball.linePos[0] = NAN;
			pSensors->ball.linePos[1] = NAN;
			pSensors->ball.lineDir[0] = NAN;
			pSensors->ball.lineDir[1] = NAN;
		}
		else
		{
			// look for the current tracker id. If it is not found, select first tracker (closest to robot)
			uint8_t index = 0;
			for(uint8_t i = 0; i < robotPi.ballDetections.numBalls; i++)
			{
				if(robotPi.ballDetections.balls[i].trackerId == robotPi.currentTrackerId)
				{
					index = i;
					break;
				}
			}

			memcpy(pSensors->ball.pos, robotPi.ballDetections.balls[index].pos, sizeof(float)*3);
			memcpy(pSensors->ball.vel, robotPi.ballDetections.balls[index].vel, sizeof(float)*3);
			memcpy(pSensors->ball.linePos, robotPi.ballDetections.balls[index].linePos, sizeof(float)*2);
			memcpy(pSensors->ball.lineDir, robotPi.ballDetections.balls[index].lineDir, sizeof(float)*2);

			robotPi.currentTrackerId = robotPi.ballDetections.balls[index].trackerId;
		}

		pSensors->ball.updated = 1;
		robotPi.ballDetectionsUpdated = 0;
	}
	else
	{
		pSensors->ball.updated = 0;
	}
}

void RobotPiHandleExtPacket(const PacketHeader* pHeader, const uint8_t* pData, uint32_t dataLength)
{
	if(pHeader->section == SECTION_EXT && pHeader->cmd == CMD_EXT_BALL_DETECTIONS)
	{
		if(dataLength < sizeof(ExtBallDetections))
		{
			LogWarnC("Invalid ball container size from EXT", dataLength);
		}
		else
		{
			ExtBallDetections* pBallContainer = (ExtBallDetections*) pData;

			robotPi.lastDetectionLatency = SysTimeUSec() - pBallContainer->timestampUs;
			robotPi.ballDetections = *pBallContainer;
			robotPi.ballDetectionsUpdated = 1;
		}
	}

	if(pHeader->section == SECTION_EXT && pHeader->cmd == CMD_EXT_ROBOT_PI_VERSION)
	{
		if(dataLength < sizeof(ExtRobotPiVersion))
		{
			LogWarnC("Invalid ext RobotPi version", dataLength);
		}
		else
		{
			memcpy(&robotPi.robotPiVersion, pData, sizeof(ExtRobotPiVersion));
		}
	}

	if(pHeader->section == SECTION_EXT && pHeader->cmd == CMD_EXT_CAMERA_STATS)
	{
		if(dataLength < sizeof(ExtCameraStats))
		{
			LogWarnC("Invalid CameraStats message", dataLength);
		}
		else
		{
			memcpy(&robotPi.cameraStats, pData, sizeof(ExtCameraStats));
		}
	}

	if(pHeader->section == SECTION_EXT && pHeader->cmd == CMD_EXT_CAMERA_PREVIEW_LINE_160)
	{
		if(dataLength < sizeof(ExtCameraPreviewLine160))
		{
			LogWarnC("Invalid CameraPreviewLine160 message", dataLength);
		}
		else
		{
			ExtCameraPreviewLine160* pLine = (ExtCameraPreviewLine160*)pData;

			if(!robotPi.previewConfig.enable)
				return;

			// stream directly to display
			gfxMutexEnter(&GDISP->mutex);

			*TFT_CMD_REG = 0x2A;
			*TFT_DATA_REG = 0;
			*TFT_DATA_REG = 0;
			*TFT_DATA_REG = 0;
			*TFT_DATA_REG = 239;

			uint16_t y = pLine->row + 200;

			*TFT_CMD_REG = 0x2B;
			*TFT_DATA_REG = y >> 8;
			*TFT_DATA_REG = y & 0xFF;
			*TFT_DATA_REG = (y+1) >> 8;
			*TFT_DATA_REG = (y+1) & 0xFF;

			*TFT_CMD_REG = 0x2C;

			for(uint16_t x = 0; x < 160; x++)
				*TFT_DATA_REG = pLine->data[x];

			gfxMutexExit(&GDISP->mutex);
		}
	}
}
