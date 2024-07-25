#pragma once

#include "radio_phy.h"

#define RADIO_MODULE_TRACE_BUF_SIZE 16

typedef enum RadioMode
{
	RADIO_MODE_OFF,
	RADIO_MODE_SNIFFER,
	RADIO_MODE_ACTIVE,
} RadioMode;

typedef struct _RadioSettingsGFSK
{
	// Modulation Params
	uint8_t bitrateAndBandwidth; // One of GFSK_BR_...
	uint8_t modulationIndex;
	uint8_t modulationShaping;

	// Packet Params
	uint8_t preambleLength;
	uint8_t crcLength; // 0..2
	uint8_t enableWhitening;

	// Sync Params
	uint32_t syncWord;
	uint8_t syncWordTolerance;
} RadioSettingsGFSK;

typedef struct _RadioSettingsFLRC
{
	// Modulation Params
	uint8_t bitrateAndBandwidth;
	uint8_t codingRate;
	uint8_t modulationShaping;

	// Packet Params
	uint8_t preambleLength;
	uint8_t crcLength;
	uint8_t enableWhitening;

	// Sync Params
	uint32_t syncWord;
	uint8_t syncWordTolerance;
} RadioSettingsFLRC;

typedef struct _RadioSettingsCommon
{
	uint32_t frequency; // in [Hz]
	uint8_t txPower; // 0 - 31 => -18 - 13dBm (be very careful not to exceed PA input limits!)
	uint8_t paRampTime;
	uint8_t highSensitivityMode;
} RadioSettingsCommon;

typedef struct _RadioModuleOpTrace
{
	RadioPhyOpTrace opTrace;
	uint8_t id;
} RadioModuleOpTrace;

typedef struct _RadioModuleData
{
	RadioPhy* pPhy;
	GPIOPin pwrPin;

	uint8_t packetType;

	RadioSettingsCommon settingsCommon;
	union
	{
		RadioSettingsGFSK gfsk;
		RadioSettingsFLRC flrc;
	} settings;
} RadioModuleData;

typedef struct _RadioModule
{
	RadioModuleData data;

	uint32_t setupStep;

	RadioModuleOpTrace traces[RADIO_MODULE_TRACE_BUF_SIZE];
	volatile uint32_t tracePos;
} RadioModule;

void RadioModuleInit(RadioModule* pModule, RadioModuleData* pInit);
RadioModuleOpTrace* RadioModuleGetNextTrace(RadioModule* pRadio);
void RadioModulePrintTrace(RadioModule* pRadio);
