#pragma once

#include "module/kicker.h"
#include "module/pattern_ident.h"
#include "module/robot_pi.h"
#include "module/power_control.h"
#include "module/radio/radio_bot.h"
#include "util/fw_updater.h"
#include "util/fw_loader_fatfs.h"

typedef struct _TigerBot
{
	// High-Level modules
	PowerControl powerControl;

	Kicker kicker;
	KickerConfig kickerConfig;

	PatternIdent patternIdent;
	PatternIdentConfig patternIdentConfig;

	RobotPi robotPi;
	ExtCameraConfig cameraConfig;
	ExtColorClassifierConfig colorClassifierConfig;
	ExtBallLocalisationConfig ballLocalisationConfig;
	ExtPointDistanceSensorConfig pointDistanceSensorConfig;

	FwUpdater fwUpdater;
	FwLoaderFatFs fwLoaderUsb;
	FwLoaderFatFs fwLoaderSdCard;

    FEMInterface femInterface;
	RadioPhy radioPhy;
	RadioModule radioModule;
	RadioBot radioBot;

	// High-Level Task
	THD_WORKING_AREA(waTask, 2048);
	thread_t* pTask;
} TigerBot;

extern TigerBot tigerBot;

void TigerBotInit();
