/*
 * wifi.h
 *
 *  Created on: 07.07.2014
 *      Author: AndreR
 */

#pragma once

#include "util/flash_fs.h"
#include "util/rf_queue.h"
#include "util/ema_filter.h"
#include "util/wifi_module.h"
#include "constants.h"
#include "commands.h"

#include "ch.h"

#define WIFI_MAX_BOTS		CMD_BOT_COUNT_TOTAL
#define WIFI_MAX_BOT_ID		(WIFI_MAX_BOTS-1)

typedef void(*BotDataAvailCallback)(uint8_t);

typedef struct _WifiConfig
{
	uint8_t channel;		// = frequency
	uint8_t speed;
	uint8_t maxBots;
	uint8_t fixedRuntime;
	uint32_t timeout;	// in [ms]
} WifiConfig;

typedef struct _WifiRobot
{
	uint8_t txData[WIFI_TX_SIZE];
	uint8_t rxData[WIFI_RX_SIZE];

	RFQueue queue;
	uint8_t id;
	EMAFilter emaRssi[2];
	EMAFilter botRssi;

	uint8_t online;
	uint32_t lastResponseTime;

	uint8_t antTableRow;
	uint8_t antTableColumn;
} WifiRobot;

typedef void(*BotProcessHook)(WifiRobot*);

typedef struct _WifiGlobal
{
	WifiModule module;

	WifiRobot bots[WIFI_MAX_BOTS];

	uint32_t runtime;	// in [us]

	binary_semaphore_t timerSem;

	WifiConfig cfg;
	FlashFile* pCfgFile;

	BotDataAvailCallback cbBotData;
	BotProcessHook hookBotProcess;

	uint32_t lastCall;
}WifiGlobal;

extern WifiGlobal wifi;

void	WifiInit();
void	WifiTask(void* params);
void	WifiSetBotDataCallback(BotDataAvailCallback cb);
void	WifiSetHookBotProcess(BotProcessHook hook);

void	WifiSetChannel(uint8_t channel);
void	WifiSetMaxBot(uint8_t maxBots);
void	WifiSetFixedRuntime(uint8_t enable);
void	WifiSetTimeout(uint32_t ms);
void	WifiSaveConfig();
void	WifiPrintConfig();

uint8_t WifiGetNumBotsOnline();
float WifiGetLinkQuality();
