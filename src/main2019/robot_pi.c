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
#include "robot/robot.h"
#include "gfx.h"
#include "src/gdisp/gdisp_driver.h"
#include "robot/ctrl_tigga.h"
#include "util/arm_mat_util_f32.h"
#include "util/angle_math.h"

#include "util/console.h"

RobotPi robotPi = {
	.cameraConfig = {
		.expAutoMode = 1,
		.expAnalogGain = 1.0f,
		.expDigitalGain = 1.0f,
		.expTimeUs = 10000,
		.wbAutoMode = 1,
		.wbRedGain = 1.0f,
		.wbBlueGain = 1.0f,
		.forcedResolution = -1,
	},
	.cameraControl = {
		.resolution = 2,
		.recording = 0,
	},
	.colorClassifierConfig = {
		.thresholds = {
			{ // Orange
				.y = {  40, 220 },
				.u = {   0, 150 },
				.v = { 140, 255 },
			},
			{ // White
				.y = { 144, 255 },
				.u = {  80, 192 },
				.v = {  64, 176 },
			},
			{ // Black
				.y = {   0,  32 },
				.u = { 112, 144 },
				.v = { 112, 160 },
			},
		},
	},
	.ballLocalisationConfig = {
		.topImageSkipFactor = 0.28f,
		.greedyCircleFactor = 1.25f,
		.minBlobArea = 20,
		.maxTrackers = 5,
		.trackerTimeoutMs = 1000,
		.historySize = 10,
		.modelError = 0.1f,
		.measError = 10.0f,
		.maxVelocity = 5.0f,
		.usePlaneIntersection = 0,
	},
	.pointDistConfig = {
		.blackThreshold = 50,
		.whiteThreshold = 105,
		.tooWhiteThreshold = 180,
		.bottomSkipPercent = 0.3f,
		.topSkipPercent = 0.26f,
		.centerCoverPercent = 0.08f,
	},
	.enablePreview = 0,
	.enabledSteps = EXT_STEP_MASK_BALL_LOC,
};

static const ConfigFileDesc configFileDescCameraConfig =
{ SID_CFG_CAMERA, 2, "camera", 8, (ElementDesc[]) {
	{  UINT8, "exp_auto", "bool", "Auto Exposure" },
	{  FLOAT, "analog_gain", "1.0-8.0", "Analog Gain" },
	{  FLOAT, "digital_gain", "1.0-2.0", "Digital Gain" },
	{ UINT32, "exp_time", "us", "Exposure Time" },
	{  UINT8, "wb_auto", "bool", "Auto White Balance" },
	{  FLOAT, "wb_gain_red", "0.9-1.9", "WB Gain Red" },
	{  FLOAT, "wb_gain_blue", "0.9-1.9", "WB Gain Blue" },
	{   INT8, "forced_res", "-1, 0-2", "Force camera resolution" },
}};

static const ConfigFileDesc configFileDescBallLocalisation =
{ SID_CFG_BALL_LOCALISATION, 0, "ext/ball_loc", 10, (ElementDesc[]) {
	{  FLOAT, "top_skip", "%", "Skip Image Top" },
	{  FLOAT, "greedy_circle", "-", "Greedy Circle Factor" },
	{ UINT16, "A_min_blob", "px", "Min. Blob Area" },
	{ UINT16, "max_trackers", "-", "Max. Ball Trackers" },
	{ UINT16, "tracker_timeout", "ms", "Tracker Timeout" },
	{ UINT16, "hist_size", "-", "Line Fit Samples" },
	{  FLOAT, "err_model", "m", "Model Error" },
	{  FLOAT, "err_meas", "m", "Measurement Error" },
	{  FLOAT, "max_vel", "m/s", "Outlier Max. Velocity" },
	{  UINT8, "use_plane_intersect", "bool", "Use Plane Intersection for Dist." },
}};

static const ConfigFileDesc configFileDescColorClassifier =
{ SID_CFG_COLOR_CLASSIFIER, 0, "ext/color_lut", 18, (ElementDesc[]) {
	{  UINT8, "orange_y_min", "-", "orange/y_min" },
	{  UINT8, "orange_y_max", "-", "orange/y_max" },
	{  UINT8, "orange_u_min", "-", "orange/u_min" },
	{  UINT8, "orange_u_max", "-", "orange/u_max" },
	{  UINT8, "orange_v_min", "-", "orange/v_min" },
	{  UINT8, "orange_v_max", "-", "orange/v_max" },
	{  UINT8, "white_y_min", "-", "white/y_min" },
	{  UINT8, "white_y_max", "-", "white/y_max" },
	{  UINT8, "white_u_min", "-", "white/u_min" },
	{  UINT8, "white_u_max", "-", "white/u_max" },
	{  UINT8, "white_v_min", "-", "white/v_min" },
	{  UINT8, "white_v_max", "-", "white/v_max" },
	{  UINT8, "black_y_min", "-", "black/y_min" },
	{  UINT8, "black_y_max", "-", "black/y_max" },
	{  UINT8, "black_u_min", "-", "black/u_min" },
	{  UINT8, "black_u_max", "-", "black/u_max" },
	{  UINT8, "black_v_min", "-", "black/v_min" },
	{  UINT8, "black_v_max", "-", "black/v_max" },
}};

void RobotPiInit()
{
	robotPi.currentTrackerId = 0xFFFFFFFF;
	robotPi.pConfigFileCameraConfig = ConfigOpenOrCreate(&configFileDescCameraConfig, &robotPi.cameraConfig, sizeof(ExtCameraConfig), 0, 0);
	robotPi.pConfigFileBallLocalisationConfig = ConfigOpenOrCreate(&configFileDescBallLocalisation, &robotPi.ballLocalisationConfig, sizeof(ExtBallLocalisationConfig), 0, 0);
	robotPi.pConfigFileColorClassifierConfig = ConfigOpenOrCreate(&configFileDescColorClassifier, &robotPi.colorClassifierConfig, sizeof(ExtColorClassifierConfig), 0, 0);
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

		if(robotPi.cameraConfig.forcedResolution >= 0 && robotPi.cameraConfig.forcedResolution <= 2)
			robotPi.cameraControl.resolution = robotPi.cameraConfig.forcedResolution;

		if(counter >= 100) // 10 Hz
		{
			counter = 0;

			header.cmd = CMD_EXT_STEP_CONFIG;
			ExtStepConfig cfg;
			cfg.debugSteps = robotPi.debugSteps;
			cfg.debugLevel = robotPi.debugLevel;
			cfg.enabledSteps = robotPi.enabledSteps | EXT_STEP_MASK_YPRESUMMER | EXT_STEP_MASK_COLOR_CLASSIFIER | EXT_STEP_MASK_REGION_EXTRACTOR | EXT_STEP_MASK_RLE_RECORDER;
			if(robotPi.enablePreview)
				cfg.enabledSteps |= EXT_STEP_MASK_MINI_PREVIEW;

			ExtSendPacket(&header, &cfg, sizeof(ExtStepConfig));

			header.cmd = CMD_EXT_CAMERA_CONFIG;
			ExtSendPacket(&header, &robotPi.cameraConfig, sizeof(ExtCameraConfig));

			header.cmd = CMD_EXT_CAMERA_CONTROL;
			ExtSendPacket(&header, &robotPi.cameraControl, sizeof(ExtCameraControl));

			header.cmd = CMD_EXT_BALL_LOC_CONFIG;
			ExtSendPacket(&header, &robotPi.ballLocalisationConfig, sizeof(ExtBallLocalisationConfig));

			header.cmd = CMD_EXT_COLOR_THRESHOLDS;
			ExtColorThresholds thresh;
			for(uint8_t i = 0; i < 3; i++)
			{
				thresh.colorId = i;
				memcpy(thresh.y, robotPi.colorClassifierConfig.thresholds[i].y, sizeof(uint8_t)*6);
				ExtSendPacket(&header, &thresh, sizeof(ExtColorThresholds));
			}

			header.cmd = CMD_EXT_POINT_DIST_SENSOR_CFG;
			ExtSendPacket(&header, &robotPi.pointDistConfig, sizeof(ExtPointDistanceSensorConfig));
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

void RobotPiUpdatePointDistanceSensorData(RobotSensors* pSensors)
{
	if(robotPi.pointDistanceSensorUpdated)
	{
		pSensors->pointDist.time = robotPi.pointDistanceSensor.timestampUs;
		pSensors->pointDist.avgHeight = robotPi.pointDistanceSensor.avgHeight;
		pSensors->pointDist.avgYBottom = robotPi.pointDistanceSensor.avgYBottom;
		pSensors->pointDist.validColumns = robotPi.pointDistanceSensor.validColumns;
		pSensors->pointDist.isMostlyWhite = robotPi.pointDistanceSensor.isMostlyWhite;

		pSensors->pointDist.updated = 1;
		robotPi.pointDistanceSensorUpdated = 0;
	}
	else
	{
		pSensors->pointDist.updated = 0;
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

	if(pHeader->section == SECTION_EXT && pHeader->cmd == CMD_EXT_POINT_DIST_SENSOR)
	{
		if(dataLength < sizeof(ExtPointDistSensor))
		{
			LogWarnC("Invalid point dist sensor msg", dataLength);
		}
		else
		{
			ExtPointDistSensor* pPoint = (ExtPointDistSensor*)pData;

			robotPi.pointDistanceSensor = *pPoint;
			robotPi.pointDistanceSensorUpdated = 1;
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

	if(pHeader->section == SECTION_EXT && pHeader->cmd == CMD_EXT_UPDATE_PROGRESS)
	{
		if(dataLength < sizeof(ExtUpdateProgress))
		{
			LogWarnC("Invalid ext update progress", dataLength);
		}
		else
		{
			memcpy(&robotPi.updateProgress, pData, sizeof(ExtUpdateProgress));
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

	if(pHeader->section == SECTION_EXT && pHeader->cmd == CMD_EXT_CAMERA_CALIBRATION)
	{
		if(dataLength < sizeof(ExtCameraCalibration))
		{
			LogWarnC("Invalid CameraCalibration message", dataLength);
		}
		else
		{
			memcpy(&robotPi.cameraCalibration, pData, sizeof(ExtCameraCalibration));
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

			if(!robotPi.enablePreview)
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
