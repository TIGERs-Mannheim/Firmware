/*
 * pattern_ident.c
 *
 *  Created on: 12.03.2020
 *      Author: AndreR
 */

/**
 * Pinout:
 * PG2: +3V3
 * PG3: SDA
 * PG4: SCL
 * PG5: LED top left & bottom right
 * PG6: LED top right & bottom left
 * PG7: LED center
 * PG8: GND
 *
 * I2C:
 * - TCA9548A (multiplexer): 0x70
 * - TCS34725 (color sensor): 0x29
 *
 * Multiplexer:
 * CH0: top left
 * CH1: center
 * CH2: bottom left
 * CH3: bottom right
 * CH4: top right
 */

#include "pattern_ident.h"
#include "util/init_hal.h"
#include "ch.h"
#include "constants.h"
#include "commands.h"
#include "struct_ids.h"

#include "hal/buzzer.h"
#include "util/console.h"
#include "util/sys_time.h"
#include "util/log.h"
#include "main/network.h"
#include <math.h>

#define LED_TL_BR	1
#define LED_TR_BL	2
#define LED_CENTER	4

#define ENABLE_I2C_CLOCK() TIM12->CR1 |= TIM_CR1_CEN
#define DISABLE_I2C_CLOCK() TIM12->CR1 &= ~TIM_CR1_CEN

PatternIdent patternIdent = {
	.config = {
		.changeThreshold = 0.2f,
		.minIntensity = 1000.0f,
		.centerThreshold = 0.06f,
		.outerThreshold = 0.04f,
	},
};

static const ConfigFileDesc configFileDescPatternIdent =
	{ SID_CFG_PATTERN_IDENT, 1, "pattern", 4, (ElementDesc[]) {
		{ FLOAT, "change_thr", "%", "Change threshold" },
		{ FLOAT, "min_intens", "", "Min. intensity" },
		{ FLOAT, "center_thr", "", "Center blob threshold" },
		{ FLOAT, "outer_thr", "", "Outer blobs threshold" },
	} };


// reorder sensors from 0 - 4 to C, TL, TR, BL, BR
static const uint8_t reorderTable[] = { 1, 0, 4, 2, 3 };
static const char* sensorLocationName[] = {" C", "TL", "TR", "BL", "BR"};
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

static void determinePatternId();
static void ledSet(uint8_t state);
static void takeSample(uint8_t ledOn);

void TIM8_BRK_TIM12_IRQHandler()
{
	CH_IRQ_PROLOGUE();

	TIM12->SR = 0;

	I2CSoftTick(&patternIdent.i2c);

	CH_IRQ_EPILOGUE();
}

void PatternIdentInit()
{
	patternIdent.badBlobs = 0x1F;

	GPIOReset(GPIOG, GPIO_PIN_8);
	GPIOSet(GPIOG, GPIO_PIN_2);

	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_OUTPUT;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.pupd = GPIO_PUPD_NONE;

	GPIOInit(GPIOG, GPIO_PIN_2 | GPIO_PIN_8, &gpioInit); // +3V3 and GND, never change!

	GPIOInit(GPIOG, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, &gpioInit); // LEDs

	ledSet(0);

	I2CSoftInit(&patternIdent.i2c, (GPIOPin){GPIOG, GPIO_PIN_4}, (GPIOPin){GPIOG, GPIO_PIN_3});

	RCC->APB1LENR |= RCC_APB1LENR_TIM12EN;
	__DSB();

	// 100MHz base clk
	TIM12->CR1 = 0;
	TIM12->CR2 = 0;
	TIM12->CNT = 0;
	TIM12->PSC = 0; // => 100MHz
	TIM12->ARR = 399; // => 250kHz => 83.333kHz I2C frequency
	TIM12->SR = 0;
	TIM12->DIER = TIM_DIER_UIE;
	TIM12->CR1 |= TIM_CR1_CEN;

	NVICEnableIRQ(TIM8_BRK_TIM12_IRQn, IRQL_I2C_SOFT);

	TCA9548AInit(&patternIdent.multiplexer, &patternIdent.i2c);

	for(uint8_t i = 0; i < PATTERN_IDENT_NUM_SENSORS; i++)
	{
		TCS3472Init(&patternIdent.detectors[i].sensor, &patternIdent.i2c);
	}

	DISABLE_I2C_CLOCK();

	patternIdent.pConfigFile = ConfigOpenOrCreate(&configFileDescPatternIdent, &patternIdent.config, sizeof(PatternIdentConfig), 0, 0);
}

void PatternIdentTask(void* params)
{
	(void)params;
	int16_t result;

	chRegSetThreadName("Pattern");

	ENABLE_I2C_CLOCK();

	for(uint32_t i = 0; i < PATTERN_IDENT_NUM_SENSORS; i++)
	{
		result = TCA9548ASetChannels(&patternIdent.multiplexer, (1 << i));
		if(result)
			LogErrorC("TCA9548 I2C error", result | (i << 16));
		result = TCS3472SetPowerOn(&patternIdent.detectors[i].sensor, 1);
		if(result)
			LogErrorC("TCS3472 I2C error", result | (i << 16));
	}

	DISABLE_I2C_CLOCK();

	float lastAvgClear = 0.0f;

	while(1)
	{
		takeSample(0);

		float avgClear = 0.0f;
		for(uint8_t i = 0; i < PATTERN_IDENT_NUM_SENSORS; i++)
			avgClear += patternIdent.detectors[i].ledOffSample.clear;

		avgClear *= 1.0f/PATTERN_IDENT_NUM_SENSORS;

		const float lowerLimit = 1.0f - patternIdent.config.changeThreshold;
		const float upperLimit = 1.0f + patternIdent.config.changeThreshold;

		if(avgClear < lastAvgClear*lowerLimit || avgClear >= lastAvgClear*upperLimit)
		{
			lastAvgClear = avgClear;
			takeSample(1);

			determinePatternId();
		}

		chThdSleepMilliseconds(1000);
	}
}

void PatternIdentPrintMeasurements()
{
	ConsolePrint("\r\n         Red            Green           Blue            Clear\r\n");

	for(uint8_t i = 0; i < PATTERN_IDENT_NUM_SENSORS; i++)
	{
		PatternIdentDetector* pDet = &patternIdent.detectors[i];

		ConsolePrint("%s: %5hu / %5hu   %5hu / %5hu   %5hu / %5hu   %5hu / %5hu   M:% .4f Y:% .4f   I:% .4f\r\n",
			sensorLocationName[i],
			pDet->ledOnSample.red, pDet->ledOffSample.red,
			pDet->ledOnSample.green, pDet->ledOffSample.green,
			pDet->ledOnSample.blue, pDet->ledOffSample.blue,
			pDet->ledOnSample.clear, pDet->ledOffSample.clear,
			pDet->magentaness, pDet->yellowness, pDet->intensity);
	}

	ConsolePrint("Detected ID: %hu,  bad blobs: 0x%02hX, I2C error: %hu\r\n",
		(uint16_t)patternIdent.detectedId, (uint16_t)patternIdent.badBlobs, (uint16_t)patternIdent.i2cError);
}

uint32_t PatternIdentGetAndClearFlags()
{
	uint32_t flags = patternIdent.flags | patternIdent.badBlobs;
	if(!patternIdent.i2cError)
		flags |= PATTERN_IDENT_FLAGS_SYSTEM_PRESENT;

	patternIdent.flags = 0;

	return flags;
}

static void ledSet(uint8_t state)
{
	if(state & LED_TL_BR)
		GPIOReset(GPIOG, GPIO_PIN_5);
	else
		GPIOSet(GPIOG, GPIO_PIN_5);

	if(state & LED_TR_BL)
		GPIOReset(GPIOG, GPIO_PIN_6);
	else
		GPIOSet(GPIOG, GPIO_PIN_6);

	if(state & LED_CENTER)
		GPIOReset(GPIOG, GPIO_PIN_7);
	else
		GPIOSet(GPIOG, GPIO_PIN_7);
}

static void takeSample(uint8_t ledOn)
{
	int16_t result = 0;

	if(ledOn)
		ledSet(LED_TL_BR | LED_TR_BL | LED_CENTER);
	else
		ledSet(0);

	ENABLE_I2C_CLOCK();

	// enable ADC on all devices at the same time, starts taking a sample
	result |= TCA9548ASetChannels(&patternIdent.multiplexer, 0x1F);
	result |= TCS3472SetRgbcEnable(&patternIdent.detectors[0].sensor, 1);

	// wait until exposure time is over
	chThdSleepMilliseconds(110);

	// get results from all sensors
	for(uint8_t i = 0; i < PATTERN_IDENT_NUM_SENSORS; i++)
	{
		uint8_t orderedId = reorderTable[i];

		result |= TCA9548ASetChannels(&patternIdent.multiplexer, (1 << orderedId));

		PatternIdentDetector* pDet = &patternIdent.detectors[i];

		if(ledOn)
			result |= TCS3472GetSample(&pDet->sensor, &pDet->ledOnSample, &pDet->updated);
		else
			result |= TCS3472GetSample(&pDet->sensor, &pDet->ledOffSample, &pDet->updated);
	}

	// disable ADC, stop sampling
	result |= TCA9548ASetChannels(&patternIdent.multiplexer, 0x1F);
	result |= TCS3472SetRgbcEnable(&patternIdent.detectors[0].sensor, 0);

	DISABLE_I2C_CLOCK();

	if(result || patternIdent.i2c.timeoutOccured)
		patternIdent.i2cError = 1;

	ledSet(0);
}

static void determinePatternId()
{
	uint8_t badBlobs = 0;
	uint8_t colorId = 0;
	uint8_t blueTeam = 0;

	for(uint8_t i = 0; i < PATTERN_IDENT_NUM_SENSORS; i++)
	{
		PatternIdentDetector* pDet = &patternIdent.detectors[i];

		RGBCSamplef diff;
		RGBCSamplef diffNormalized;

		for(uint8_t j = 0; j < 4; j++)
		{
			diff.data[j] = fabsf((float)pDet->ledOnSample.data[j] - (float)pDet->ledOffSample.data[j]);
		}

		for(uint8_t j = 0; j < 4; j++)
		{
			diffNormalized.data[j] = diff.data[j] / diff.clear;
		}

		float magenta = (diffNormalized.red + diffNormalized.blue) * 0.5f;
		float yellow = (diffNormalized.red + diffNormalized.green) * 0.5f;

		pDet->yellowness = yellow - diffNormalized.blue;
		pDet->magentaness = magenta - diffNormalized.green;
		pDet->intensity = diff.clear;

		if(pDet->intensity < patternIdent.config.minIntensity)
		{
			badBlobs |= (1 << i);
		}
	}

	PatternIdentDetector* pCenter = &patternIdent.detectors[0];
	if(pCenter->yellowness > patternIdent.config.centerThreshold)
		blueTeam = 1; // we detected yellow, so the blob is blue on the top side
	else if(pCenter->yellowness < -patternIdent.config.centerThreshold)
		blueTeam = 0;
	else
		badBlobs |= (1 << 0);

	for(uint8_t i = 1; i < PATTERN_IDENT_NUM_SENSORS; i++)
	{
		PatternIdentDetector* pDet = &patternIdent.detectors[i];

		if(pDet->magentaness > patternIdent.config.outerThreshold)
			colorId |= (1 << (i-1));
		else if(pDet->magentaness < -patternIdent.config.outerThreshold)
			colorId &= ~(1 << (i-1));
		else
			badBlobs |= (1 << i);
	}

	if(badBlobs == 0)
	{
		uint8_t botId = color2Id[colorId];

		if(blueTeam)
			botId += CMD_BOT_COUNT_HALF;

		patternIdent.detectedId = botId;

		if(patternIdent.badBlobs)
		{
			patternIdent.flags |= PATTERN_IDENT_FLAGS_COVER_DETECTED;

			network.config.botId = botId;
			NetworkSaveConfig();

			BuzzerPlay(&buzzSeqTada);
		}
	}

	if(patternIdent.badBlobs == 0 && badBlobs)
	{
		patternIdent.flags |= PATTERN_IDENT_FLAGS_COVER_LOST;
	}

	patternIdent.badBlobs = badBlobs;
}
