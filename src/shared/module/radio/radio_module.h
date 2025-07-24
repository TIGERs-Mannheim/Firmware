#pragma once

#include "radio_phy.h"
#include "radio_settings.h"

#define RADIO_MODULE_TRACE_BUF_SIZE 16

typedef enum RadioMode
{
	RADIO_MODE_OFF,
	RADIO_MODE_SNIFFER,
	RADIO_MODE_ACTIVE,
} RadioMode;

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
