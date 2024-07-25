#pragma once

#include "commands.h"
#include "util/config.h"
#include "util/shell_cmd.h"
#include "drv/mpu_ext.h"

#define ROBOT_PI_CAMERA_PREVIEW_LINE_RECEIVED	EVENT_MASK(0)

typedef struct _RobotPiData
{
	// Interface to RPi
	MpuExt* pExt;

	// Configs
	ConfigHeader cameraCfgHeader;
	ExtCameraConfig* pCameraCfg;

	ConfigHeader colorClassifierCfgHeader;
	ExtColorClassifierConfig* pColorClassifierCfg;

	ConfigHeader ballLocalisationCfgHeader;
	ExtBallLocalisationConfig* pBallLocalisationCfg;

	ExtPointDistanceSensorConfig* pPointDistCfg; // TODO: this has no config file yet
} RobotPiData;

typedef struct _RobotPi
{
	// Interface
	MpuExt* pExt;

	// OS data
	THD_WORKING_AREA(waTask, 256);
	thread_t* pTask;

	event_source_t eventSource;

	virtual_timer_t timer1kHz;
	virtual_timer_t timer10Hz;

	ShellCmdHandler cmdHandler;

	// Configs
	ExtCameraConfig* pCameraCfg;
	ConfigFileDesc cameraCfgDesc;
	ConfigFile* pCameraCfgFile;

	ExtColorClassifierConfig* pColorClassifierCfg;
	ConfigFileDesc colorClassifierCfgDesc;
	ConfigFile* pColorClassifierCfgFile;

	ExtBallLocalisationConfig* pBallLocalisationCfg;
	ConfigFileDesc ballLocalisationCfgDesc;
	ConfigFile* pBallLocalisationCfgFile;

	ExtPointDistanceSensorConfig* pPointDistCfg;

	// Transmitted data
	ExtCameraControl cameraControl;

	uint8_t enablePreview;
	uint32_t enabledSteps;
	uint32_t debugSteps;
	uint8_t debugLevel;

	ExtRobotState robotState;
	mutex_t robotStateMutex;

	// Received data
	ExtRobotPiVersion robotPiVersion;
	MpuExtPacketHandler robotPiVersionHandler;

	ExtUpdateProgress updateProgress;
	MpuExtPacketHandler updateProgressHandler;

	ExtCameraCalibration cameraCalibration;
	MpuExtPacketHandler cameraCalibrationHandler;

	ExtCameraStats cameraStats;
	MpuExtPacketHandler cameraStatsHandler;

	ExtBallDetections ballDetections;
	MpuExtPacketHandler ballDetectionsHandler;
	uint32_t currentTrackerId;

	ExtPointDistSensor pointDistanceSensor;
	MpuExtPacketHandler pointDistanceSensorHandler;

	ExtCameraPreviewLine160 cameraPreviewLine;
	MpuExtPacketHandler cameraPreviewLineHandler;
} RobotPi;

void RobotPiInit(RobotPi* pPi, RobotPiData* pInit, tprio_t prio);
void RobotPiSetRobotState(RobotPi* pPi, const RobotCtrlState* pState);
void RobotPiTriggerImageCapture(RobotPi* pPi);
void RobotPiUpdateSensorData(RobotPi* pPi, RobotSensors* pSensors);
