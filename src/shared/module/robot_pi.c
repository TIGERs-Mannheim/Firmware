#include "robot_pi.h"
#include "hal/sys_time.h"
#include "math/vector.h"
#include <string.h>
#include <math.h>

#define EVENT_MASK_EXT			EVENT_MASK(0)
#define EVENT_MASK_TRIGGER_1KHZ	EVENT_MASK(1)
#define EVENT_MASK_TRIGGER_10HZ	EVENT_MASK(2)

static void robotPiTask(void* params);
static void timer1kHzTrigger(virtual_timer_t* pTimer, void* pUser);
static void timer10HzTrigger(virtual_timer_t* pTimer, void* pUser);
static void registerShellCommands(ShellCmdHandler* pHandler);

static const ElementDesc cameraCfgElements[] =
{
	{  UINT8, "exp_auto", "bool", "Auto Exposure" },
	{  FLOAT, "analog_gain", "1.0-8.0", "Analog Gain" },
	{  FLOAT, "digital_gain", "1.0-2.0", "Digital Gain" },
	{ UINT32, "exp_time", "us", "Exposure Time" },
	{  UINT8, "wb_auto", "bool", "Auto White Balance" },
	{  FLOAT, "wb_gain_red", "0.9-1.9", "WB Gain Red" },
	{  FLOAT, "wb_gain_blue", "0.9-1.9", "WB Gain Blue" },
	{   INT8, "forced_res", "-1, 0-2", "Force camera resolution" },
};

static const ElementDesc ballLocalisationCfgElements[] =
{
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
};

static const ElementDesc colorClassifierCfgElements[] =
{
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
};

void RobotPiInit(RobotPi* pPi, RobotPiData* pInit, tprio_t prio)
{
	chMtxObjectInit(&pPi->robotStateMutex);
	chEvtObjectInit(&pPi->eventSource);

	pPi->pExt = pInit->pExt;

	// Configs
	pPi->pCameraCfg = pInit->pCameraCfg;
	pPi->cameraCfgDesc.cfgId = pInit->cameraCfgHeader.configId;
	pPi->cameraCfgDesc.version = pInit->cameraCfgHeader.configVersion;
	pPi->cameraCfgDesc.pName = pInit->cameraCfgHeader.pConfigName;
	pPi->cameraCfgDesc.numElements = sizeof(cameraCfgElements) / sizeof(cameraCfgElements[0]);
	pPi->cameraCfgDesc.elements = cameraCfgElements;
	pPi->pCameraCfgFile = ConfigOpenOrCreate(&pPi->cameraCfgDesc, pPi->pCameraCfg, sizeof(ExtCameraConfig), 0, 0);

	pPi->pColorClassifierCfg = pInit->pColorClassifierCfg;
	pPi->colorClassifierCfgDesc.cfgId = pInit->colorClassifierCfgHeader.configId;
	pPi->colorClassifierCfgDesc.version = pInit->colorClassifierCfgHeader.configVersion;
	pPi->colorClassifierCfgDesc.pName = pInit->colorClassifierCfgHeader.pConfigName;
	pPi->colorClassifierCfgDesc.numElements = sizeof(colorClassifierCfgElements) / sizeof(colorClassifierCfgElements[0]);
	pPi->colorClassifierCfgDesc.elements = colorClassifierCfgElements;
	pPi->pColorClassifierCfgFile = ConfigOpenOrCreate(&pPi->colorClassifierCfgDesc, pPi->pColorClassifierCfg, sizeof(ExtColorClassifierConfig), 0, 0);

	pPi->pBallLocalisationCfg = pInit->pBallLocalisationCfg;
	pPi->ballLocalisationCfgDesc.cfgId = pInit->ballLocalisationCfgHeader.configId;
	pPi->ballLocalisationCfgDesc.version = pInit->ballLocalisationCfgHeader.configVersion;
	pPi->ballLocalisationCfgDesc.pName = pInit->ballLocalisationCfgHeader.pConfigName;
	pPi->ballLocalisationCfgDesc.numElements = sizeof(ballLocalisationCfgElements) / sizeof(ballLocalisationCfgElements[0]);
	pPi->ballLocalisationCfgDesc.elements = ballLocalisationCfgElements;
	pPi->pColorClassifierCfgFile = ConfigOpenOrCreate(&pPi->ballLocalisationCfgDesc, pPi->pBallLocalisationCfg, sizeof(ExtBallLocalisationConfig), 0, 0);

	pPi->pPointDistCfg = pInit->pPointDistCfg;

	// Control
	pPi->cameraControl.resolution = 2;
	pPi->cameraControl.recording = 0;
	pPi->enablePreview = 0;
	pPi->enabledSteps = EXT_STEP_MASK_BALL_LOC;
	pPi->currentTrackerId = 0xFFFFFFFF;

	// Receive handler setup
	pPi->robotPiVersionHandler.cmd = (PacketHeader){{ CMD_EXT_ROBOT_PI_VERSION, SECTION_EXT }};
	pPi->robotPiVersionHandler.pStorage = &pPi->robotPiVersion;
	pPi->robotPiVersionHandler.storageSize = sizeof(pPi->robotPiVersion);
	MpuExtAddPacketHandler(pPi->pExt, &pPi->robotPiVersionHandler);

	pPi->updateProgressHandler.cmd = (PacketHeader){{ CMD_EXT_UPDATE_PROGRESS, SECTION_EXT }};
	pPi->updateProgressHandler.pStorage = &pPi->updateProgress;
	pPi->updateProgressHandler.storageSize = sizeof(pPi->updateProgress);
	MpuExtAddPacketHandler(pPi->pExt, &pPi->updateProgressHandler);

	pPi->cameraCalibrationHandler.cmd = (PacketHeader){{ CMD_EXT_CAMERA_CALIBRATION, SECTION_EXT }};
	pPi->cameraCalibrationHandler.pStorage = &pPi->cameraCalibration;
	pPi->cameraCalibrationHandler.storageSize = sizeof(pPi->cameraCalibration);
	MpuExtAddPacketHandler(pPi->pExt, &pPi->cameraCalibrationHandler);

	pPi->cameraStatsHandler.cmd = (PacketHeader){{ CMD_EXT_CAMERA_STATS, SECTION_EXT }};
	pPi->cameraStatsHandler.pStorage = &pPi->cameraStats;
	pPi->cameraStatsHandler.storageSize = sizeof(pPi->cameraStats);
	MpuExtAddPacketHandler(pPi->pExt, &pPi->cameraStatsHandler);

	pPi->ballDetectionsHandler.cmd = (PacketHeader){{ CMD_EXT_BALL_DETECTIONS, SECTION_EXT }};
	pPi->ballDetectionsHandler.pStorage = &pPi->ballDetections;
	pPi->ballDetectionsHandler.storageSize = sizeof(pPi->ballDetections);
	MpuExtAddPacketHandler(pPi->pExt, &pPi->ballDetectionsHandler);

	pPi->pointDistanceSensorHandler.cmd = (PacketHeader){{ CMD_EXT_POINT_DIST_SENSOR, SECTION_EXT }};
	pPi->pointDistanceSensorHandler.pStorage = &pPi->pointDistanceSensor;
	pPi->pointDistanceSensorHandler.storageSize = sizeof(pPi->pointDistanceSensor);
	MpuExtAddPacketHandler(pPi->pExt, &pPi->pointDistanceSensorHandler);

	pPi->cameraPreviewLineHandler.cmd = (PacketHeader){{ CMD_EXT_CAMERA_PREVIEW_LINE_160, SECTION_EXT }};
	pPi->cameraPreviewLineHandler.pStorage = &pPi->cameraPreviewLine;
	pPi->cameraPreviewLineHandler.storageSize = sizeof(pPi->cameraPreviewLine);
	MpuExtAddPacketHandler(pPi->pExt, &pPi->cameraPreviewLineHandler);

	ShellCmdHandlerInit(&pPi->cmdHandler, pPi);
	registerShellCommands(&pPi->cmdHandler);

	pPi->pTask = chThdCreateStatic(pPi->waTask, sizeof(pPi->waTask), prio, &robotPiTask, pPi);

	chVTObjectInit(&pPi->timer1kHz);
	chVTObjectInit(&pPi->timer10Hz);

	chVTSetContinuous(&pPi->timer1kHz, TIME_MS2I(1), &timer1kHzTrigger, pPi);
	chVTSetContinuous(&pPi->timer10Hz, TIME_MS2I(100), &timer10HzTrigger, pPi);
}

void RobotPiSetRobotState(RobotPi* pPi, const RobotCtrlState* pState)
{
	chMtxLock(&pPi->robotStateMutex);

	pPi->robotState.timestampUs = pState->header.timestamp;
	memcpy(pPi->robotState.posGlobal, pState->pos, sizeof(float)*3);
	memcpy(pPi->robotState.velGlobal, pState->vel, sizeof(float)*3);
	memcpy(pPi->robotState.accGlobal, pState->acc, sizeof(float)*3);

	chMtxUnlock(&pPi->robotStateMutex);
}

void RobotPiTriggerImageCapture(RobotPi* pPi)
{
	PacketHeader header;
	header.section = SECTION_EXT;
	header.cmd = CMD_EXT_CAMERA_TRIGGER_CAPTURE;

	MpuExtSendPacket(pPi->pExt, &header, 0, 0);
}

void RobotPiUpdateSensorData(RobotPi* pPi, RobotSensors* pSensors)
{
	// Ball detections
	if(pPi->ballDetectionsHandler.updated)
	{
		chMtxLock(&pPi->ballDetectionsHandler.storageMutex);

		pSensors->ball.time = pPi->ballDetections.timestampUs;

		if(pPi->ballDetections.numBalls == 0)
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
			for(uint8_t i = 0; i < pPi->ballDetections.numBalls; i++)
			{
				if(pPi->ballDetections.balls[i].trackerId == pPi->currentTrackerId)
				{
					index = i;
					break;
				}
			}

			memcpy(pSensors->ball.pos, pPi->ballDetections.balls[index].pos, sizeof(float)*3);
			memcpy(pSensors->ball.vel, pPi->ballDetections.balls[index].vel, sizeof(float)*3);
			memcpy(pSensors->ball.linePos, pPi->ballDetections.balls[index].linePos, sizeof(float)*2);
			memcpy(pSensors->ball.lineDir, pPi->ballDetections.balls[index].lineDir, sizeof(float)*2);

			pPi->currentTrackerId = pPi->ballDetections.balls[index].trackerId;
		}

		pSensors->ball.updated = 1;
		pPi->ballDetectionsHandler.updated = 0;

		chMtxUnlock(&pPi->ballDetectionsHandler.storageMutex);
	}
	else
	{
		pSensors->ball.updated = 0;
	}

	// Point distance sensor
	if(pPi->pointDistanceSensorHandler.updated)
	{
		chMtxLock(&pPi->pointDistanceSensorHandler.storageMutex);

		pSensors->pointDist.time = pPi->pointDistanceSensor.timestampUs;
		pSensors->pointDist.avgHeight = pPi->pointDistanceSensor.avgHeight;
		pSensors->pointDist.avgYBottom = pPi->pointDistanceSensor.avgYBottom;
		pSensors->pointDist.validColumns = pPi->pointDistanceSensor.validColumns;
		pSensors->pointDist.isMostlyWhite = pPi->pointDistanceSensor.isMostlyWhite;

		pSensors->pointDist.updated = 1;
		pPi->pointDistanceSensorHandler.updated = 0;

		chMtxUnlock(&pPi->pointDistanceSensorHandler.storageMutex);
	}
	else
	{
		pSensors->pointDist.updated = 0;
	}
}

static void timer1kHzTrigger(virtual_timer_t*, void* pUser)
{
	RobotPi* pPi = (RobotPi*)pUser;

	chSysLockFromISR();
	chEvtSignalI(pPi->pTask, EVENT_MASK_TRIGGER_1KHZ);
	chSysUnlockFromISR();
}

static void timer10HzTrigger(virtual_timer_t*, void* pUser)
{
	RobotPi* pPi = (RobotPi*)pUser;

	chSysLockFromISR();
	chEvtSignalI(pPi->pTask, EVENT_MASK_TRIGGER_10HZ);
	chSysUnlockFromISR();
}

static void robotPiTask(void* params)
{
	RobotPi* pPi = (RobotPi*)params;

	chRegSetThreadName("ROBOT_PI");

	event_listener_t extListener;
	chEvtRegisterMask(&pPi->pExt->eventSource, &extListener, EVENT_MASK_EXT);

	PacketHeader header;
	header.section = SECTION_EXT;

	while(1)
	{
		eventmask_t events = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_INFINITE);

		if(events & EVENT_MASK_EXT) // Some packet was updated
		{
			if(pPi->cameraPreviewLineHandler.updated)
			{
				pPi->cameraPreviewLineHandler.updated = 0;
				chEvtBroadcastFlags(&pPi->eventSource, ROBOT_PI_CAMERA_PREVIEW_LINE_RECEIVED);
			}
		}

		if(events & EVENT_MASK_TRIGGER_1KHZ)
		{
			ExtRemoteTime remoteTime;
			remoteTime.timestampUs = SysTimeUSec();
			header.cmd = CMD_EXT_REMOTE_TIME;
			MpuExtSendPacket(pPi->pExt, &header, &remoteTime, sizeof(ExtRemoteTime));

			header.cmd = CMD_EXT_ROBOT_STATE;
			chMtxLock(&pPi->robotStateMutex);
			MpuExtSendPacket(pPi->pExt, &header, &pPi->robotState, sizeof(ExtRobotState));
			chMtxUnlock(&pPi->robotStateMutex);
		}

		if(events & EVENT_MASK_TRIGGER_10HZ)
		{
			if(pPi->pCameraCfg->forcedResolution >= 0 && pPi->pCameraCfg->forcedResolution <= 2)
				pPi->cameraControl.resolution = pPi->pCameraCfg->forcedResolution;

			header.cmd = CMD_EXT_STEP_CONFIG;
			ExtStepConfig cfg;
			cfg.debugSteps = pPi->debugSteps;
			cfg.debugLevel = pPi->debugLevel;
			cfg.enabledSteps = pPi->enabledSteps | EXT_STEP_MASK_YPRESUMMER | EXT_STEP_MASK_COLOR_CLASSIFIER | EXT_STEP_MASK_REGION_EXTRACTOR | EXT_STEP_MASK_RLE_RECORDER;
			if(pPi->enablePreview)
				cfg.enabledSteps |= EXT_STEP_MASK_MINI_PREVIEW;

			MpuExtSendPacket(pPi->pExt, &header, &cfg, sizeof(ExtStepConfig));

			header.cmd = CMD_EXT_CAMERA_CONFIG;
			MpuExtSendPacket(pPi->pExt, &header, pPi->pCameraCfg, sizeof(ExtCameraConfig));

			header.cmd = CMD_EXT_CAMERA_CONTROL;
			MpuExtSendPacket(pPi->pExt, &header, &pPi->cameraControl, sizeof(ExtCameraControl));

			header.cmd = CMD_EXT_BALL_LOC_CONFIG;
			MpuExtSendPacket(pPi->pExt, &header, pPi->pBallLocalisationCfg, sizeof(ExtBallLocalisationConfig));

			header.cmd = CMD_EXT_COLOR_THRESHOLDS;
			ExtColorThresholds thresh;
			for(uint8_t i = 0; i < 3; i++)
			{
				thresh.colorId = i;
				memcpy(thresh.y, pPi->pColorClassifierCfg->thresholds[i].y, sizeof(uint8_t)*6);
				MpuExtSendPacket(pPi->pExt, &header, &thresh, sizeof(ExtColorThresholds));
			}

			header.cmd = CMD_EXT_POINT_DIST_SENSOR_CFG;
			MpuExtSendPacket(pPi->pExt, &header, pPi->pPointDistCfg, sizeof(ExtPointDistanceSensorConfig));
		}
	}
}

SHELL_CMD(config, "Show current configuration");

SHELL_CMD_IMPL(config)
{
	(void)argc; (void)argv;
	RobotPi* pPi = (RobotPi*)pUser;

	printf("Auto exposure:      %s\r\n", pPi->pCameraCfg->expAutoMode ? "On" : "Off");
	printf("  Analog gain:      %.2f\r\n  Digital gain:     %.2f\r\n", pPi->pCameraCfg->expAnalogGain, pPi->pCameraCfg->expDigitalGain);
	printf("  Exposure time:    %uus\r\n", pPi->pCameraCfg->expTimeUs);
	printf("Auto white-balance: %s\r\n", pPi->pCameraCfg->wbAutoMode ? "On" : "Off");
	printf("  Red gain:         %.2f\r\n  Blue gain:        %.2f\r\n", pPi->pCameraCfg->wbRedGain, pPi->pCameraCfg->wbBlueGain);
	printf("Preview:            %s\r\n", pPi->enablePreview ? "On" : "Off");
	printf("Resolution:         %hu (forced: %hd)\r\n", pPi->cameraControl.resolution, (int16_t)pPi->pCameraCfg->forcedResolution);
	printf("Recording:          %s\r\n", pPi->cameraControl.recording ? "On" : "Off");
}

SHELL_CMD(stats, "Show statistics");

SHELL_CMD_IMPL(stats)
{
	(void)argc; (void)argv;
	RobotPi* pPi = (RobotPi*)pUser;

	printf("Processing time used: %.2f%%\r\n", pPi->cameraStats.rtAvg / pPi->cameraStats.dtAvg * 100.0f);
	printf("Frame delta times:\r\n");
	printf("  min: %.5f\r\n", pPi->cameraStats.dtMin);
	printf("  avg: %.5f (%.2ffps)\r\n", pPi->cameraStats.dtAvg, 1.0f/pPi->cameraStats.dtAvg);
	printf("  max: %.5f\r\n", pPi->cameraStats.dtMax);
	printf("  dev: %.5f\r\n", pPi->cameraStats.dtDev);
	printf("Algorithm run times:\r\n");
	printf("  min: %.5f\r\n", pPi->cameraStats.rtMin);
	printf("  avg: %.5f\r\n", pPi->cameraStats.rtAvg);
	printf("  max: %.5f\r\n", pPi->cameraStats.rtMax);
	printf("  dev: %.5f\r\n", pPi->cameraStats.rtDev);
	printf("Recording: %hu\r\n", (uint16_t)pPi->cameraStats.recording);
	printf("  file: %s\r\n", pPi->cameraStats.recordFilename);
	printf("  size: %.3fMB\r\n", pPi->cameraStats.recordSize * 1.0f/1024.0f * 1.0f/1024.0f);
	printf("  duration: %.1fs\r\n", pPi->cameraStats.recordDuration);
	printf("Images: %u\r\n", pPi->cameraStats.imagesTaken);
	printf("  file: %s\r\n", pPi->cameraStats.imageFilename);

	MpuExt* pExt = pPi->pExt;
	UartDma* pUart = pExt->pUart;

	printf("--- UART6 ---\r\n");
	printf("RX payload/wire/usage: %u / %u / %3.2f%%\r\n", pExt->stats.rxPayload, pUart->stats.rxBytes, pUart->stats.rxUsage * 100.0f);
	printf("TX payload/wire/usage: %u / %u / %3.2f%%\r\n", pExt->stats.txPayload, pUart->stats.txBytes, pUart->stats.txUsage * 100.0f);
	printf("Installed: %hu\r\n", (uint16_t)pExt->isInstalled);
}

SHELL_CMD(exp, "Set exposure time",
	SHELL_ARG(duration, "Exposure [us] or auto")
);

SHELL_CMD_IMPL(exp)
{
	(void)argc;
	RobotPi* pPi = (RobotPi*)pUser;

	if(strcmp(argv[1], "auto") == 0)
	{
		printf("Enabling auto exposure mode\r\n");
		pPi->pCameraCfg->expAutoMode = 1;
		ConfigNotifyUpdate(pPi->pCameraCfgFile);
	}
	else
	{
		int duration = atoi(argv[1]);

		if(duration < 1)
		{
			fprintf(stderr, "Invalid argument\r\n");
			return;
		}

		printf("Setting camera exposure to %dus\r\n", duration);
		pPi->pCameraCfg->expTimeUs = duration;
		pPi->pCameraCfg->expAutoMode = 0;
		ConfigNotifyUpdate(pPi->pCameraCfgFile);
	}
}

SHELL_CMD(gain, "Set analog and digital gain",
	SHELL_ARG(analog, "Analog gain (1.0-8.0)"),
	SHELL_ARG(digital, "Digital gain (1.0-2.0)")
);

SHELL_CMD_IMPL(gain)
{
	(void)argc;
	RobotPi* pPi = (RobotPi*)pUser;

	float analog = atof(argv[1]);
	float digital = atof(argv[2]);

	if(analog < 1.0f || analog > 8.0f)
	{
		fprintf(stderr, "Invalid analog gain\r\n");
		return;
	}

	if(digital < 1.0f || digital > 2.0f)
	{
		fprintf(stderr, "Invalid digital gain\r\n");
		return;
	}

	printf("Setting camera gains. Analog: %.2f, Digital: %.2f\r\n", analog, digital);
	pPi->pCameraCfg->expAnalogGain = analog;
	pPi->pCameraCfg->expDigitalGain = digital;
	ConfigNotifyUpdate(pPi->pCameraCfgFile);
}

SHELL_CMD(awb, "Set auto white-balance mode",
	SHELL_ARG(mode, "0=off, 1=on")
);

SHELL_CMD_IMPL(awb)
{
	(void)argc;
	RobotPi* pPi = (RobotPi*)pUser;

	int mode = atoi(argv[1]);

	printf("Setting AWB mode to %d\r\n", mode);
	pPi->pCameraCfg->wbAutoMode = mode;
	ConfigNotifyUpdate(pPi->pCameraCfgFile);
}

SHELL_CMD(wb, "Set manual white-balance",
	SHELL_ARG(red, "0.0-8.0, typical 0.9-1.9"),
	SHELL_ARG(blue, "0.0-8.0, typical 0.9-1.9")
);

SHELL_CMD_IMPL(wb)
{
	(void)argc;
	RobotPi* pPi = (RobotPi*)pUser;

	float red = atof(argv[1]);
	float blue = atof(argv[2]);

	if(red < 0.0f || red > 8.0f)
	{
		fprintf(stderr, "Invalid red value\r\n");
		return;
	}

	if(blue < 0.0f || blue > 8.0f)
	{
		fprintf(stderr, "Invalid blue value\r\n");
		return;
	}

	printf("Setting white balance. Red: %.2f, Blue: %.2f\r\n", red, blue);
	pPi->pCameraCfg->wbRedGain = red;
	pPi->pCameraCfg->wbBlueGain = blue;
	ConfigNotifyUpdate(pPi->pCameraCfgFile);
}

SHELL_CMD(preview, "Enable streaming of camera preview image to display",
	SHELL_ARG(mode, "0=off, 1=on")
);

SHELL_CMD_IMPL(preview)
{
	(void)argc;
	RobotPi* pPi = (RobotPi*)pUser;

	int mode = atoi(argv[1]);

	printf("Camera preview: %d\r\n", mode);
	pPi->enablePreview = mode;
}

SHELL_CMD(steps, "Set enabled camera processing steps",
	SHELL_ARG(mask, "Bitmask of enabled steps, can use hex input")
);

SHELL_CMD_IMPL(steps)
{
	(void)argc;
	RobotPi* pPi = (RobotPi*)pUser;

	uint32_t mask = strtoul(argv[1], 0, 0);

	printf("Enabled steps: 0x%08X\r\n", mask);
	pPi->enabledSteps = mask;
}

SHELL_CMD(dbg, "Enable debugging for specified camera processing steps",
	SHELL_ARG(mask, "Bitmask of debugging steps, can use hex input"),
	SHELL_ARG(level, "Debug level")
);

SHELL_CMD_IMPL(dbg)
{
	(void)argc;
	RobotPi* pPi = (RobotPi*)pUser;

	uint32_t mask = strtoul(argv[1], 0, 0);
	int level = atoi(argv[2]);

	printf("Debug steps: 0x%08X\r\nLevel: %d\r\n", mask, level);
	pPi->debugSteps = mask;
	pPi->debugLevel = level;
}

SHELL_CMD(point_dist, "Show point distance sensor output");

SHELL_CMD_IMPL(point_dist)
{
	(void)argc; (void)argv;
	RobotPi* pPi = (RobotPi*)pUser;

	printf("validColumns: %u\r\n", pPi->pointDistanceSensor.validColumns);
	printf("color: %s\r\n", pPi->pointDistanceSensor.isMostlyWhite ? "White" : "Black");
	printf("avgYBottom: %.3f\r\n", pPi->pointDistanceSensor.avgYBottom);
	printf("avgHeight: %.3f\r\n", pPi->pointDistanceSensor.avgHeight);
}

SHELL_CMD(res, "Set camera resolution (may be overridden by forced camera resolution)",
	SHELL_ARG(mode, "0=low res, 1=mid res, 2=high res")
);

SHELL_CMD_IMPL(res)
{
	(void)argc;
	RobotPi* pPi = (RobotPi*)pUser;

	int mode = atoi(argv[1]);

	if(mode < 0 || mode > 2)
	{
		fprintf(stderr, "Invalud argument\r\n");
		return;
	}

	printf("Camera res: %d\r\n", mode);
	pPi->cameraControl.resolution = mode;
}

SHELL_CMD(rec, "Enable recording of camera stream on RPi, fixes resolution to mid res",
	SHELL_ARG(mode, "0=off, 1=on")
);

SHELL_CMD_IMPL(rec)
{
	(void)argc;
	RobotPi* pPi = (RobotPi*)pUser;

	int mode = atoi(argv[1]);

	printf("Set recording: %hu\r\n", mode);
	pPi->cameraControl.recording = mode;
}

SHELL_CMD(trigger, "Trigger capture of a full resolution image, stored on RPi");

SHELL_CMD_IMPL(trigger)
{
	(void)argc; (void)argv;
	RobotPi* pPi = (RobotPi*)pUser;

	RobotPiTriggerImageCapture(pPi);
	printf("Triggered camera capture\r\n");
}

SHELL_CMD(col_show, "Show color thresholds of a specific color",
	SHELL_ARG(col, "o=orange, w=white, b=black")
);

static ExtColorClassifierConfigThresholds* getThresholdsByColorChar(RobotPi* pPi, char col)
{
	ExtColorClassifierConfigThresholds* pThresh;

	switch(col)
	{
		case 'o': pThresh = &pPi->pColorClassifierCfg->thresholds[EXT_COLOR_THRESHOLDS_ID_ORANGE]; break;
		case 'w': pThresh = &pPi->pColorClassifierCfg->thresholds[EXT_COLOR_THRESHOLDS_ID_WHITE]; break;
		case 'b': pThresh = &pPi->pColorClassifierCfg->thresholds[EXT_COLOR_THRESHOLDS_ID_BLACK]; break;
		default: fprintf(stderr, "Unknown color\r\n"); return 0;
	}

	return pThresh;
}

SHELL_CMD_IMPL(col_show)
{
	(void)argc;
	RobotPi* pPi = (RobotPi*)pUser;

	char col = *argv[1];
	ExtColorClassifierConfigThresholds* pThresh = getThresholdsByColorChar(pPi, col);
	if(!pThresh)
		return;

	printf("Y: % 3hu % 3hu\r\n", (uint16_t)pThresh->y[0], (uint16_t)pThresh->y[1]);
	printf("U: % 3hu % 3hu\r\n", (uint16_t)pThresh->u[0], (uint16_t)pThresh->u[1]);
	printf("V: % 3hu % 3hu\r\n", (uint16_t)pThresh->v[0], (uint16_t)pThresh->v[1]);
}

SHELL_CMD(col_set, "Set thresholds for a component (YUV) of a color",
	SHELL_ARG(col, "color, see col_show for a list"),
	SHELL_ARG(comp, "y, u, or v component"),
	SHELL_ARG(min, "Lower threshold (0-255)"),
	SHELL_ARG(max, "Upper threshold (0-255)")
);

SHELL_CMD_IMPL(col_set)
{
	(void)argc;
	RobotPi* pPi = (RobotPi*)pUser;

	char col = *argv[1];
	ExtColorClassifierConfigThresholds* pThresh = getThresholdsByColorChar(pPi, col);
	if(!pThresh)
		return;

	char comp = *argv[2];
	uint8_t* pMinMax;

	switch(comp)
	{
		case 'y': pMinMax = pThresh->y; break;
		case 'u': pMinMax = pThresh->u; break;
		case 'v': pMinMax = pThresh->v; break;
		default: fprintf(stderr, "Unknown component\r\n"); return;
	}

	int min = atoi(argv[3]);
	if(min < 0 || min > 255)
	{
		fprintf(stderr, "Invalid lower threshold\r\n");
		return;
	}

	int max = atoi(argv[4]);
	if(max < 0 || max > 255)
	{
		fprintf(stderr, "Invalid upper threshold\r\n");
		return;
	}

	pMinMax[0] = min;
	pMinMax[1] = max;

	printf("%c: % 3hu % 3hu\r\n", comp, (uint16_t)pMinMax[0], (uint16_t)pMinMax[1]);
}

SHELL_CMD(balls, "List detected ball trackers in global and local frame");

SHELL_CMD_IMPL(balls)
{
	(void)argc; (void)argv;
	RobotPi* pPi = (RobotPi*)pUser;

	float pos[3];
	float robotPos[3];

	memcpy(robotPos, pPi->ballDetections.robotPos, sizeof(float)*3);

	printf("ID      X_map     Y_map     Z_map    X_base    Y_base\r\n");

	for(uint16_t i = 0; i < pPi->ballDetections.numBalls; i++)
	{
		memcpy(pos, pPi->ballDetections.balls[i].pos, sizeof(float)*3);

		float posLocal[3] = { 0, 0, pos[2] };
		posLocal[0] = pos[0] - robotPos[0];
		posLocal[1] = pos[1] - robotPos[1];
		Vector2fRotatePtr(-robotPos[2], posLocal[0], posLocal[1], posLocal);

		printf("%3hu  %8.3f  %8.3f  %8.3f  %8.3f  %8.3f\r\n",
				pPi->ballDetections.balls[i].trackerId,
				pos[0], pos[1], pos[2],
				posLocal[0], posLocal[1]);
	}
}

static void robotPiPrintCameraCalibrationStatus(RobotPi* pPi)
{
	printf("Status: %s\r\nScore: %fm\r\nResolution: %i %i\r\nPrincipal point: %f %f\r\nFocal length: %f\r\nDistortion: %f %f\r\nHeight: %f\r\nRotation: %f %f %f\r\n",
				 pPi->cameraCalibration.status,
				 pPi->cameraCalibration.score,
				 pPi->cameraCalibration.resolutionX, pPi->cameraCalibration.resolutionY,
				 pPi->cameraCalibration.principalPointX, pPi->cameraCalibration.principalPointY,
				 pPi->cameraCalibration.focalLength,
				 pPi->cameraCalibration.distortionCoeff0, pPi->cameraCalibration.distortionCoeff1,
				 pPi->cameraCalibration.height,
				 pPi->cameraCalibration.rotationY, pPi->cameraCalibration.rotationP, pPi->cameraCalibration.rotationR);
}

SHELL_CMD(calib_run, "Run camera geometry calibration, this command may take up to 10s");

SHELL_CMD_IMPL(calib_run)
{
	(void)argc; (void)argv;
	RobotPi* pPi = (RobotPi*)pUser;

	pPi->enabledSteps |= EXT_STEP_MASK_CALIBRATION;

	uint16_t i;
	char status[sizeof(pPi->cameraCalibration.status)];
	status[0] = 0;
	for(i = 0; i < 20; i++)
	{
		chThdSleepMilliseconds(500);

		if(!strncmp(pPi->cameraCalibration.status, status, sizeof(status)))
			continue; // No change since last print

		if(!strcmp(pPi->cameraCalibration.status, "Calibrated"))
		{
			robotPiPrintCameraCalibrationStatus(pPi);
			break; // Calibration has finished
		}

		printf("Status: %s\r\n", pPi->cameraCalibration.status);
		strncpy(status, pPi->cameraCalibration.status, sizeof(status));
	}

	if(i == 20)
		fprintf(stderr, "Camera calibration timeout\r\n");

	pPi->enabledSteps &= ~EXT_STEP_MASK_CALIBRATION;
}

SHELL_CMD(calib_show, "Show camera geometry calibration status");

SHELL_CMD_IMPL(calib_show)
{
	(void)argc; (void)argv;
	RobotPi* pPi = (RobotPi*)pUser;

	robotPiPrintCameraCalibrationStatus(pPi);
}

static void registerShellCommands(ShellCmdHandler* pHandler)
{
	ShellCmdAdd(pHandler, stats_command);
	ShellCmdAdd(pHandler, config_command);
	ShellCmdAdd(pHandler, exp_command);
	ShellCmdAdd(pHandler, gain_command);
	ShellCmdAdd(pHandler, awb_command);
	ShellCmdAdd(pHandler, wb_command);
	ShellCmdAdd(pHandler, preview_command);
	ShellCmdAdd(pHandler, steps_command);
	ShellCmdAdd(pHandler, dbg_command);
	ShellCmdAdd(pHandler, point_dist_command);
	ShellCmdAdd(pHandler, res_command);
	ShellCmdAdd(pHandler, rec_command);
	ShellCmdAdd(pHandler, trigger_command);
	ShellCmdAdd(pHandler, col_show_command);
	ShellCmdAdd(pHandler, col_set_command);
	ShellCmdAdd(pHandler, balls_command);
	ShellCmdAdd(pHandler, calib_run_command);
	ShellCmdAdd(pHandler, calib_show_command);
}
