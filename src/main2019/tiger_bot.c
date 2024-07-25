#include "tiger_bot.h"
#include "constants.h"
#include "struct_ids.h"
#include "robot/robot.h"
#include "robot/network.h"
#include "util/log_file.h"
#include "util/log.h"
#include "presenter.h"
#include "module/radio/radio_settings.h"
#include "sx1280_def.h"

#include "sys/i2c5.h"
#include "sys/tim13.h"
#include "sys/tim14.h"
#include "sys/usart2.h"

#include "dev/adc_kicker.h"
#include "dev/buzzer.h"
#include "dev/drib_ir.h"
#include "dev/power_mon.h"
#include "dev/raspberry_pi.h"
#include "dev/motors.h"
#include "dev/sky_fem.h"
#include "dev/sx1280.h"

#include <stdio.h>

TigerBot tigerBot = {
	.kickerConfig = {
		.maxVoltage_V = 200,
		.chgHysteresis_V = 5.0f,
		.cooldown_ms = 80,
		.kickVoltageLimit = 0.9f,
		.straightCoeffs = { 1.281f, -0.085f, 0.096f },
		.straightMinTime_ms = 1.0f,
		.chipCoeffs = { 0.166f, 0.919f, 0.0f },
		.chipMinTime_ms = 2.0f,
		.dribbleStraightCoeffs = { 0.0f, 0.0f },
		.dribbleChipCoeffs = { 0.0f, 0.0f },
	},

	.patternIdentConfig = {
		.changeThreshold = 0.2f,
		.minIntensity = 1000.0f,
		.centerThreshold = 0.06f,
		.outerThreshold = 0.04f,
	},

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
	.pointDistanceSensorConfig = {
		.blackThreshold = 50,
		.whiteThreshold = 105,
		.tooWhiteThreshold = 180,
		.bottomSkipPercent = 0.3f,
		.topSkipPercent = 0.26f,
		.centerCoverPercent = 0.08f,
	},
};

static void tigerBotTask(void*);
static void waitForFwUpdates();
static void shutdownSequence();
static void processMotorBreak(McuMotor* pMot);

CH_IRQ_HANDLER(Vector64) // EXTI3
{
	CH_IRQ_PROLOGUE();
	EXTI_D1->PR1 |= EXTI_PR1_PR3;
	PowerControlButtonIrq(&tigerBot.powerControl);
	CH_IRQ_EPILOGUE();
}

CH_IRQ_HANDLER(Vector1C4) // SPDIFRX, used as low prio radio IRQ to get back to OS-level
{
	CH_IRQ_PROLOGUE();
	RadioBotLowPrioIRQ(&tigerBot.radioBot);
	CH_IRQ_EPILOGUE();
}

void TigerBotInit()
{
	// ### Power Control ###
	PowerControlData powerControlData;
	powerControlData.usbPoweredPin = (GPIOPin){ GPIOG, GPIO_PIN_15 };
	powerControlData.powerDownRequestPin = (GPIOPin){ GPIOD, GPIO_PIN_3 };
	powerControlData.powerKillPin = (GPIOPin){ GPIOD, GPIO_PIN_6 };
	powerControlData.pPwrMon = &devPowerMon;
	powerControlData.cellInternalResistance = 0.02f;

	PowerControlInit(&tigerBot.powerControl, &powerControlData, TASK_PRIO_POWER_CONTROL);

	NVICEnableIRQ(EXTI3_IRQn, IRQL_EXTI3);

	// ### Kicker ###
	KickerData kickerData;
	kickerData.pAdcKicker =  &devAdcKicker;
	kickerData.pMcuDribbler = &devDribIr;
	kickerData.pTimerStraight = &tim13;
	kickerData.pTimerChip = &tim14;
	kickerData.chargePin = (GPIOPin){ GPIOG, GPIO_PIN_11 };
	kickerData.pConfig = &tigerBot.kickerConfig;
	kickerData.configId = SID_CFG_KICK;
	kickerData.configVersion = 9;
	kickerData.pConfigName = "kick/cfg";

	KickerInit(&tigerBot.kicker, &kickerData, TASK_PRIO_KICKER);

	// ### Pattern Ident ###
	PatternIdentData patternIdentData;
	patternIdentData.pI2C = &i2c5;
	patternIdentData.vccPin = (GPIOPin){ GPIOG, GPIO_PIN_2 };
	patternIdentData.gndPin = (GPIOPin){ GPIOG, GPIO_PIN_8 };
	patternIdentData.ledTlBr = (GPIOPin){ GPIOG, GPIO_PIN_5 };
	patternIdentData.ledTrBl = (GPIOPin){ GPIOG, GPIO_PIN_6 };
	patternIdentData.ledCenter = (GPIOPin){ GPIOG, GPIO_PIN_7 };
	patternIdentData.pConfig = &tigerBot.patternIdentConfig;
	patternIdentData.configId = SID_CFG_PATTERN_IDENT;
	patternIdentData.configVersion = 1;
	patternIdentData.pConfigName = "pattern";

	PatternIdentInit(&tigerBot.patternIdent, &patternIdentData, TASK_PRIO_PATTERN_IDENT);

	// ### RobotPi ###
	RobotPiData robotPiData;
	robotPiData.pExt = &devRaspberryPi;
	robotPiData.cameraCfgHeader = (ConfigHeader){ SID_CFG_CAMERA, 2, "camera" };
	robotPiData.pCameraCfg = &tigerBot.cameraConfig;
	robotPiData.colorClassifierCfgHeader = (ConfigHeader){ SID_CFG_COLOR_CLASSIFIER, 0, "ext/color_lut" };
	robotPiData.pColorClassifierCfg = &tigerBot.colorClassifierConfig;
	robotPiData.ballLocalisationCfgHeader = (ConfigHeader){ SID_CFG_BALL_LOCALISATION, 0, "ext/ball_loc" };
	robotPiData.pBallLocalisationCfg = &tigerBot.ballLocalisationConfig;
	robotPiData.pPointDistCfg = &tigerBot.pointDistanceSensorConfig;

	RobotPiInit(&tigerBot.robotPi, &robotPiData, TASK_PRIO_ROBOT_PI);

	// ### Firmware Updater ###
	FwUpdaterProgram fwProgram;
	fwProgram.execAddr = 0x08100000;
	fwProgram.loadAddr = 0x08040000;
	fwProgram.maxSize = 768*1024;
	fwProgram.flashWriteGranularity = 32;

	FwUpdaterInit(&tigerBot.fwUpdater, &fwProgram);

	FwLoaderFatFsInit(&tigerBot.fwLoaderUsb, "0:/main2019.bin");
	FwLoaderFatFsInit(&tigerBot.fwLoaderSdCard, "1:/main2019.bin");

	// ### Radio ###
	SKY66112CreateInterface(&devSkyFem, &tigerBot.femInterface);

	RadioPhyData radioPhyInit;
	radioPhyInit.pSX = &devSX1280;
	radioPhyInit.pFem = &tigerBot.femInterface;
	radioPhyInit.timeouts = radioSettings.phyTimeouts;

	RadioPhyInit(&tigerBot.radioPhy, &radioPhyInit);

	RadioModuleData radioModuleInit;
	radioModuleInit.pPhy = &tigerBot.radioPhy;
	radioModuleInit.pwrPin = (GPIOPin){ GPIOB, GPIO_PIN_2 };
	radioModuleInit.packetType = radioSettings.packetType;
	radioModuleInit.settingsCommon = radioSettings.settingsCommon;

	if(radioSettings.packetType == PACKET_TYPE_FLRC)
		radioModuleInit.settings.flrc = radioSettings.settingsFlrc;
	else
		radioModuleInit.settings.gfsk = radioSettings.settingsGfsk;

	RadioModuleInit(&tigerBot.radioModule, &radioModuleInit);

	RadioBotData radioBotInit;
	radioBotInit.pModule = &tigerBot.radioModule;
	radioBotInit.clientId = network.config.botId;
	radioBotInit.lowPrioIRQn = SPDIF_RX_IRQn;
	radioBotInit.baseTimeout_us = radioSettings.connectionTimeout_us;

	RadioBotInit(&tigerBot.radioBot, &radioBotInit);

	NVICEnableIRQ(SPDIF_RX_IRQn, IRQL_WIFI_LOW_PRIO);

	// ### Tiger Bot Task ###
	tigerBot.pTask = chThdCreateStatic(tigerBot.waTask, sizeof(tigerBot.waTask), TASK_PRIO_TIGER_BOT, &tigerBotTask, 0);
}

#define EVENT_MASK_PATTERN_IDENT	EVENT_MASK(0)
#define EVENT_MASK_ROBOT_PI			EVENT_MASK(1)
#define EVENT_MASK_CLI_UART			EVENT_MASK(2)
#define EVENT_MASK_POWER_CONTROL	EVENT_MASK(3)
#define EVENT_MASK_MOTOR			EVENT_MASK(4)

static void tigerBotTask(void*)
{
	chRegSetThreadName("TIGER_BOT");

	event_listener_t patternIdentListener;
	event_listener_t robotPiListener;
	event_listener_t cliUartListener;
	event_listener_t powerControlListener;
	event_listener_t motorListener[4];

	chEvtRegisterMask(&tigerBot.patternIdent.eventSource, &patternIdentListener, EVENT_MASK_PATTERN_IDENT);
	chEvtRegisterMask(&tigerBot.robotPi.eventSource, &robotPiListener, EVENT_MASK_ROBOT_PI);
	chEvtRegisterMaskWithFlags(&usart2.eventSource, &cliUartListener, EVENT_MASK_CLI_UART, UART_DMA_EVENT_BREAK_DETECTED);
	chEvtRegisterMaskWithFlags(&tigerBot.powerControl.eventSource, &powerControlListener, EVENT_MASK_POWER_CONTROL, POWER_CONTROL_EVENT_SHUTDOWN_REQUESTED);

	for(size_t i = 0; i < 4; i++)
		chEvtRegisterMaskWithFlags(&devMotors.mcu[i].eventSource, &motorListener[i], EVENT_MASK_MOTOR, MCU_MOTOR_EVENT_BREAK_DETECTED);

	uint8_t lastPatternIdentBadBlobs = 0xFF;

	printf("\f--- Main v2019 ---\r\n");

	waitForFwUpdates();

	while(1)
	{
		eventmask_t events = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_INFINITE);

		if(events & EVENT_MASK_PATTERN_IDENT)
		{
			PatternIdentDetection det;
			PatternIdentGet(&tigerBot.patternIdent, &det);

			if(det.isSystemPresent && det.badBlobs == 0 && lastPatternIdentBadBlobs)
			{
				network.config.botId = det.detectedId;
				NetworkSaveConfig();

				BuzzerPlay(&devBuzzer, &buzzSeqTada);
			}

			lastPatternIdentBadBlobs = det.badBlobs;
		}

		if(events & EVENT_MASK_ROBOT_PI)
		{
			eventflags_t flags = chEvtGetAndClearFlags(&robotPiListener);
			if((flags & ROBOT_PI_CAMERA_PREVIEW_LINE_RECEIVED) && tigerBot.robotPi.enablePreview)
			{
				PresenterSendPreviewLineToDisplay(&tigerBot.robotPi.cameraPreviewLine);
			}
		}

		if(events & EVENT_MASK_POWER_CONTROL)
		{
			eventflags_t flags = chEvtGetAndClearFlags(&powerControlListener);
			if(flags & POWER_CONTROL_EVENT_SHUTDOWN_REQUESTED)
			{
				shutdownSequence();
			}
		}

		if((events & EVENT_MASK_CLI_UART) && robot.mode == 0)
		{
			NVIC_SystemReset();
		}

		if(events & EVENT_MASK_MOTOR)
		{
			for(size_t i = 0; i < 4; i++)
				processMotorBreak(&devMotors.mcu[i]);
		}
	}
}

static void processMotorBreak(McuMotor* pMot)
{
	if(!pMot->breakTriggered)
		return;

	BuzzerPlay(&devBuzzer, &buzzSeqUp20);
	LogWarnC("Active break on motor", pMot->motorId);

	char filename[64];
	FIL file;
	FRESULT fresult;

	unsigned int id = 0;

	do
	{
		snprintf(filename, 64, "ovr%hu_%u.bin", (uint16_t)pMot->motorId, id++);
		fresult = f_open(&file, filename, FA_WRITE | FA_CREATE_NEW);
	} while(fresult == FR_EXIST);

	if(fresult == FR_OK)
	{
		UINT bw;
		f_write(&file, &pMot->recordCounter, sizeof(pMot->recordCounter), &bw);
		f_write(&file, pMot->pRecordData, pMot->recordsMax*sizeof(int16_t)*2, &bw);
		f_close(&file);
	}

	pMot->breakTriggered = 0;
	McuMotorDebugRecord(pMot, MOTOR_EXCHANGE_MOSI_RECORD_NONE, 0);
}

static void waitForFwUpdates()
{
	uint8_t fwUpdatePending = 1;
	while(fwUpdatePending)
	{
		fwUpdatePending = 0;

		for(uint16_t i = 0; i < 5; i++)
			fwUpdatePending += devMotors.mcu[i].doFwUpdate;

		fwUpdatePending += devDribIr.doFwUpdate;

		chThdSleepMilliseconds(10);
	}

	printf("    Req  Upd  Res     Size    Blocks  Time\r\n");
	for(uint16_t i = 0; i < 5; i++)
	{
		McuMotor* pMotor = &devMotors.mcu[i];
		printf("M%hu  %hu    %hu    0x%04hX  %-6u%4u     %ums\r\n", i,
				(uint16_t)pMotor->flashResult.updateRequired, (uint16_t)pMotor->flashResult.updated,
				pMotor->flashFuncResult, pMotor->flashResult.programSize, pMotor->flashResult.numBlocks,
				pMotor->flashResult.timeUs/1000);
	}

	printf("IR  %hu    %hu    0x%04hX  %-6u%4u     %ums\r\n",
			(uint16_t)devDribIr.flashResult.updateRequired, (uint16_t)devDribIr.flashResult.updated,
			devDribIr.flashFuncResult, devDribIr.flashResult.programSize, devDribIr.flashResult.numBlocks,
			devDribIr.flashResult.timeUs/1000);
}

static void shutdownSequence()
{
	systime_t extShutdownStart = 0;

	if(devRaspberryPi.isInstalled)
	{
		ExtShutdown shutdown;
		shutdown.key = EXT_SHUTDOWN_KEY;

		PacketHeader header;
		header.section = SECTION_EXT;
		header.cmd = CMD_EXT_SHUTDOWN;

		for(uint8_t i = 0; i < 10; i++)
		{
			MpuExtSendPacket(&devRaspberryPi, &header, &shutdown, sizeof(ExtShutdown));
			chThdSleepMilliseconds(10);
		}

		extShutdownStart = chVTGetSystemTimeX();
	}

	LogFileClose();

	if((tigerBot.kicker.errorFlags & KICKER_ERROR_KICK_NOT_INSTALLED) == 0)
	{
		KickerSetChargeMode(&tigerBot.kicker, KICKER_CHG_MODE_AUTO_DISCHARGE);
		chThdSleepMilliseconds(10);

		while(tigerBot.kicker.charger.mode == KICKER_CHG_MODE_AUTO_DISCHARGE)
			chThdSleepMilliseconds(10);
	}

	if(extShutdownStart)
	{
		RobotImplBuzzerPlay(BUZZ_DOUBLE_SLOW);

		chThdSleepUntilWindowed(extShutdownStart, extShutdownStart + TIME_S2I(2));

		RobotImplBuzzerPlay(BUZZ_TADA);
	}

	chThdSleepMilliseconds(100);

	PowerControlKill(&tigerBot.powerControl);

	RobotSetIdleMode();
	BuzzerPlay(&devBuzzer, 0);
}
