#pragma once

#include "gfx.h"
#include "../base_station.h"

typedef struct _SetupData
{
	struct
	{
		uint8_t channel;
		uint8_t maxBots;
		uint8_t fixedRuntime;
	} wifi;

	struct
	{
		IPv4Address ip;
		uint16_t port;
		uint8_t useDhcp;
	} eth;

	struct
	{
		IPv4Address ip;
		uint16_t port;
	} vision;
} SetupData;

typedef void(*SetupChangedCallback)(const SetupData*);

GHandle SetupCreate();
void SetupUpdate(BaseStationConfig* pBaseConfig, RadioBaseConfig* pRadioConfig, NetIf* pIf);
void SetupSetCallback(SetupChangedCallback cb);
