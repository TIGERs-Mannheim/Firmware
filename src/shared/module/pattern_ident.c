#include "pattern_ident.h"
#include "util/log.h"
#include "commands.h"
#include "hal/sys_time.h"
#include <math.h>
#include <string.h>

#define LED_TL_BR	1
#define LED_TR_BL	2
#define LED_CENTER	4

static const ElementDesc patternIdentCfgElements[] =
{
	{ FLOAT, "change_thr", "%", "Change threshold" },
	{ FLOAT, "min_intens", "", "Min. intensity" },
	{ FLOAT, "center_thr", "", "Center blob threshold" },
	{ FLOAT, "outer_thr", "", "Outer blobs threshold" },
};

static const uint8_t color2Id[] = {
		// BR BL TR TL
	 8, // G G G G
	14, // G G G M
	12, // G G M G
	10, // G G M M
	 6, // G M G G
	 7, // G M G M
	 5, // G M M G
	 4, // G M M M
	 2, // M G G G
	 3, // M G G M
	 1, // M G M G
	 0, // M G M M
	11, // M M G G
	15, // M M G M
	13, // M M M G
	 9, // M M M M
};

// reorder sensors from 0 - 4 to C, TL, TR, BL, BR
static const uint8_t reorderTable[] = { 1, 0, 4, 2, 3 };

static void ledSet(PatternIdent* pPat, uint8_t state);
static void patternIdentTask(void* pParam);
static void registerShellCommands(ShellCmdHandler* pHandler);

void PatternIdentInit(PatternIdent* pPat, PatternIdentData* pInit, tprio_t prio)
{
	chMtxObjectInit(&pPat->detectionMutex);
	chEvtObjectInit(&pPat->eventSource);

	ShellCmdHandlerInit(&pPat->cmdHandler, pPat);
	registerShellCommands(&pPat->cmdHandler);

	pPat->pI2C = pInit->pI2C;
	pPat->ledTlBr = pInit->ledTlBr;
	pPat->ledTrBl = pInit->ledTrBl;
	pPat->ledCenter = pInit->ledCenter;

	pPat->pConfig = pInit->pConfig;
	pPat->cfgDesc.cfgId = pInit->configId;
	pPat->cfgDesc.version = pInit->configVersion;
	pPat->cfgDesc.pName = pInit->pConfigName;
	pPat->cfgDesc.numElements = sizeof(patternIdentCfgElements) / sizeof(patternIdentCfgElements[0]);
	pPat->cfgDesc.elements = patternIdentCfgElements;

	pPat->pConfigFile = ConfigOpenOrCreate(&pPat->cfgDesc, pPat->pConfig, sizeof(PatternIdentConfig), 0, CONFIG_FILE_FLAG_INTERNAL);

	pPat->detection.isSystemPresent = 1;
	pPat->detection.badBlobs = 0x1F;

	DeviceProfilerInit(&pPat->profiler, 5.0f);

	GPIOPinReset(pInit->gndPin);
	GPIOPinSet(pInit->vccPin);

	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_OUTPUT;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.pupd = GPIO_PUPD_NONE;

	GPIOInit(pInit->gndPin.pPort, pInit->gndPin.pin, &gpioInit); // GND, never change!
	GPIOInit(pInit->vccPin.pPort, pInit->vccPin.pin, &gpioInit); // +3V3, never change!

	// LEDs
	GPIOInit(pPat->ledTlBr.pPort, pPat->ledTlBr.pin, &gpioInit);
	GPIOInit(pPat->ledTrBl.pPort, pPat->ledTrBl.pin, &gpioInit);
	GPIOInit(pPat->ledCenter.pPort, pPat->ledCenter.pin, &gpioInit);

	ledSet(pPat, 0);

	TCA9548AInit(&pPat->multiplexer, pPat->pI2C);

	for(uint8_t i = 0; i < PATTERN_IDENT_NUM_SENSORS; i++)
		TCS3472Init(&pPat->sensors[i], pPat->pI2C);

	pPat->pTask = chThdCreateStatic(pPat->waTask, sizeof(pPat->waTask), prio, &patternIdentTask, pPat);
}

void PatternIdentGet(PatternIdent* pPat, PatternIdentDetection* pMeas)
{
	chMtxLock(&pPat->detectionMutex);
	memcpy(pMeas, &pPat->detection, sizeof(pPat->detection));
	chMtxUnlock(&pPat->detectionMutex);
}

static void ledSet(PatternIdent* pPat, uint8_t state)
{
	if(state & LED_TL_BR)
		GPIOPinReset(pPat->ledTlBr);
	else
		GPIOPinSet(pPat->ledTlBr);

	if(state & LED_TR_BL)
		GPIOPinReset(pPat->ledTrBl);
	else
		GPIOPinSet(pPat->ledTrBl);

	if(state & LED_CENTER)
		GPIOPinReset(pPat->ledCenter);
	else
		GPIOPinSet(pPat->ledCenter);
}

static void takeSample(PatternIdent* pPat, PatternIdentMeasurement* pMeas)
{
	int16_t result = 0;

	DeviceProfilerBegin(&pPat->profiler);

	// enable ADC on all devices at the same time, starts taking a sample
	result |= TCA9548ASetChannels(&pPat->multiplexer, 0x1F);
	result |= TCS3472SetRgbcEnable(&pPat->sensors[0], 1);

	// wait until exposure time is over
	chThdSleepMilliseconds(110);

	// get results from all sensors
	for(uint8_t i = 0; i < PATTERN_IDENT_NUM_SENSORS; i++)
	{
		uint8_t orderedId = reorderTable[i];

		result |= TCA9548ASetChannels(&pPat->multiplexer, (1 << orderedId));
		result |= TCS3472GetSample(&pPat->sensors[i], &pMeas->samples[i], &pMeas->updated[i]);
		pMeas->timestamp_us = SysTimeUSec();
	}

	// disable ADC, stop sampling
	result |= TCA9548ASetChannels(&pPat->multiplexer, 0x1F);
	result |= TCS3472SetRgbcEnable(&pPat->sensors[0], 0);

	DeviceProfilerEnd(&pPat->profiler);

	if(result)
		pPat->i2cError = 1;
}

static void determinePatternId(PatternIdent* pPat)
{
	const PatternIdentMeasurement* pLedOn = &pPat->measLedOn;
	const PatternIdentMeasurement* pLedOff = &pPat->measLedOff;
	PatternIdentDetection* pDet = &pPat->detection;

	uint8_t badBlobs = 0;
	uint8_t colorId = 0;
	uint8_t blueTeam = 0;

	chMtxLock(&pPat->detectionMutex);

	pDet->isSystemPresent = !pPat->i2cError;

	for(uint8_t i = 0; i < PATTERN_IDENT_NUM_SENSORS; i++)
	{
		RGBCSamplef diff;
		RGBCSamplef diffNormalized;

		for(uint8_t j = 0; j < 4; j++)
			diff.data[j] = fabsf((float)pLedOn->samples[i].data[j] - (float)pLedOff->samples[i].data[j]);

		for(uint8_t j = 0; j < 4; j++)
			diffNormalized.data[j] = diff.data[j] / diff.clear;

		float magenta = (diffNormalized.red + diffNormalized.blue) * 0.5f;
		float yellow = (diffNormalized.red + diffNormalized.green) * 0.5f;

		pDet->blobs[i].yellowness = yellow - diffNormalized.blue;
		pDet->blobs[i].magentaness = magenta - diffNormalized.green;
		pDet->blobs[i].intensity = diff.clear;

		if(pDet->blobs[i].intensity < pPat->pConfig->minIntensity)
			badBlobs |= (1 << i);
	}

	if(pDet->blobs[0].yellowness > pPat->pConfig->centerThreshold)
		blueTeam = 1; // we detected yellow, so the blob is blue on the top side
	else if(pDet->blobs[0].yellowness < -pPat->pConfig->centerThreshold)
		blueTeam = 0;
	else
		badBlobs |= (1 << 0);

	for(uint8_t i = 1; i < PATTERN_IDENT_NUM_SENSORS; i++)
	{
		if(pDet->blobs[i].magentaness > pPat->pConfig->outerThreshold)
			colorId |= (1 << (i-1));
		else if(pDet->blobs[i].magentaness < -pPat->pConfig->outerThreshold)
			colorId &= ~(1 << (i-1));
		else
			badBlobs |= (1 << i);
	}

	if(badBlobs == 0)
	{
		uint8_t botId = color2Id[colorId];

		if(blueTeam)
			botId += CMD_BOT_COUNT_HALF;

		pDet->detectedId = botId;
	}

	pDet->badBlobs = badBlobs;
	pDet->timestamp_us = SysTimeUSec();

	chMtxUnlock(&pPat->detectionMutex);

	chEvtBroadcast(&pPat->eventSource);
}

static void patternIdentTask(void* pParam)
{
	PatternIdent* pPat = (PatternIdent*)pParam;
	int16_t result;

	chRegSetThreadName("PAT_IDENT");

	chThdSleepMilliseconds(10);

	for(uint32_t i = 0; i < PATTERN_IDENT_NUM_SENSORS; i++)
	{
		result = TCA9548ASetChannels(&pPat->multiplexer, (1 << i));
		if(result)
		{
			LogErrorC("TCA9548 I2C error", result | (i << 16));
		}

		result = TCS3472SetPowerOn(&pPat->sensors[i], 1);
		if(result)
		{
			LogErrorC("TCS3472 I2C error", result | (i << 16));
		}
	}

	float lastAvgClear = 0.0f;

	systime_t prev = chVTGetSystemTimeX();
	systime_t next = prev + TIME_MS2I(1000);

	while(1)
	{
		takeSample(pPat, &pPat->measLedOff);

		float avgClear = 0.0f;
		for(uint8_t i = 0; i < PATTERN_IDENT_NUM_SENSORS; i++)
			avgClear += pPat->measLedOff.samples[i].clear;

		avgClear *= 1.0f/PATTERN_IDENT_NUM_SENSORS;

		const float lowerLimit = 1.0f - pPat->pConfig->changeThreshold;
		const float upperLimit = 1.0f + pPat->pConfig->changeThreshold;

		if(avgClear < lastAvgClear*lowerLimit || avgClear >= lastAvgClear*upperLimit)
		{
			lastAvgClear = avgClear;

			ledSet(pPat, LED_TL_BR | LED_TR_BL | LED_CENTER);
			takeSample(pPat, &pPat->measLedOn);
			ledSet(pPat, 0);
		}

		determinePatternId(pPat);

		// wait until next loop
		prev = chThdSleepUntilWindowed(prev, next);
		next += TIME_MS2I(1000);
	}
}

SHELL_CMD(show, "Shot pattern ident summary");

SHELL_CMD_IMPL(show)
{
	(void)argc; (void)argv;
	PatternIdent* pPat = (PatternIdent*)pUser;

	static const char* sensorLocationName[] = {" C", "TL", "TR", "BL", "BR"};

	PatternIdentMeasurement* pLedOff = &pPat->measLedOff;
	PatternIdentMeasurement* pLedOn = &pPat->measLedOn;
	PatternIdentDetection* pDet = &pPat->detection;

	printf("         Red            Green           Blue            Clear\r\n");

	for(uint8_t i = 0; i < PATTERN_IDENT_NUM_SENSORS; i++)
	{
		printf("%s: %5hu / %5hu   %5hu / %5hu   %5hu / %5hu   %5hu / %5hu   M:% .4f Y:% .4f   I:% .4f\r\n",
			sensorLocationName[i],
			pLedOn->samples[i].red, pLedOff->samples[i].red,
			pLedOn->samples[i].green, pLedOff->samples[i].green,
			pLedOn->samples[i].blue, pLedOff->samples[i].blue,
			pLedOn->samples[i].clear, pLedOff->samples[i].clear,
			pDet->blobs[i].magentaness, pDet->blobs[i].yellowness, pDet->blobs[i].intensity);
	}

	printf("Detected ID: %hu,  bad blobs: 0x%02hX, I2C error: %hu\r\n",
		(uint16_t)pDet->detectedId, (uint16_t)pDet->badBlobs, (uint16_t)pPat->i2cError);
}

static void registerShellCommands(ShellCmdHandler* pHandler)
{
	ShellCmdAdd(pHandler, show_command);
}
