/*
 * pattern_ident.h
 *
 *  Created on: 12.03.2020
 *      Author: AndreR
 */

#pragma once

#include "util/i2c_soft.h"
#include "util/tca9548a.h"
#include "util/tcs3472.h"
#include "util/config.h"

#define PATTERN_IDENT_NUM_SENSORS 5

typedef struct _PatternIdentConfig
{
	float changeThreshold;
	float minIntensity;
	float centerThreshold;
	float outerThreshold;
} PatternIdentConfig;

typedef struct _PatternIdentDetector
{
	TCS3472 sensor;
	RGBCSample ledOffSample;
	RGBCSample ledOnSample;
	uint8_t updated;

	float yellowness; // if positive this blob is more yellow than blue
	float magentaness; // if positive this blob is more magenta than green
	float intensity;

	int8_t magenta;
	int8_t yellow;
	uint8_t valid;
} PatternIdentDetector;

typedef struct _PatternIdent
{
	I2CSoft i2c;
	uint8_t i2cError;

	TCA9548A multiplexer;
	PatternIdentDetector detectors[PATTERN_IDENT_NUM_SENSORS];

	uint8_t detectedId;
	uint8_t badBlobs;
	uint32_t flags;

	PatternIdentConfig config;
	ConfigFile* pConfigFile;
} PatternIdent;

extern PatternIdent patternIdent;

void PatternIdentInit();
void PatternIdentTask(void* params);
void PatternIdentPrintMeasurements();
uint32_t PatternIdentGetAndClearFlags();

