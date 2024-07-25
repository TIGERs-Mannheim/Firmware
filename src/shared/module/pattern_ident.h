#pragma once

#include "hal/i2c_lld_soft.h"
#include "drv/tca9548a.h"
#include "drv/tcs3472.h"
#include "util/config.h"
#include "util/device_profiler.h"
#include "util/shell_cmd.h"

#define PATTERN_IDENT_NUM_SENSORS 5

typedef struct _PatternIdentConfig
{
	float changeThreshold;
	float minIntensity;
	float centerThreshold;
	float outerThreshold;
} PatternIdentConfig;

typedef struct _PatternIdentMeasurement
{
	uint32_t timestamp_us;
	RGBCSample samples[PATTERN_IDENT_NUM_SENSORS];
	uint8_t updated[PATTERN_IDENT_NUM_SENSORS];
} PatternIdentMeasurement;

typedef struct _PatternIdentDetection
{
	uint32_t timestamp_us;

	uint8_t isSystemPresent;
	uint8_t detectedId;
	uint8_t badBlobs;

	struct
	{
		float yellowness; // if positive this blob is more yellow than blue
		float magentaness; // if positive this blob is more magenta than green
		float intensity;
	} blobs[PATTERN_IDENT_NUM_SENSORS];
} PatternIdentDetection;

typedef struct _PatternIdentData
{
	I2C* pI2C;
	GPIOPin vccPin;
	GPIOPin gndPin;
	GPIOPin ledTlBr;
	GPIOPin ledTrBl;
	GPIOPin ledCenter;

	PatternIdentConfig* pConfig;
	uint16_t configId;
	uint16_t configVersion;
	const char* pConfigName;
} PatternIdentData;

typedef struct _PatternIdent
{
	I2C* pI2C;
	uint8_t i2cError;

	GPIOPin ledTlBr;
	GPIOPin ledTrBl;
	GPIOPin ledCenter;

	TCA9548A multiplexer;
	TCS3472 sensors[PATTERN_IDENT_NUM_SENSORS];

	PatternIdentMeasurement measLedOff;
	PatternIdentMeasurement measLedOn;

	PatternIdentDetection detection;
	mutex_t detectionMutex;

	event_source_t eventSource;

	DeviceProfiler profiler;

	ShellCmdHandler cmdHandler;

	THD_WORKING_AREA(waTask, 512);
	thread_t* pTask;

	PatternIdentConfig* pConfig;
	ConfigFileDesc cfgDesc;
	ConfigFile* pConfigFile;
} PatternIdent;

void PatternIdentInit(PatternIdent* pPat, PatternIdentData* pInit, tprio_t prio);
void PatternIdentGet(PatternIdent* pPat, PatternIdentDetection* pMeas);
