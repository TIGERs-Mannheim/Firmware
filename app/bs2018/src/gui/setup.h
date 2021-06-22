/*
 * setup.h
 *
 *  Created on: 18.11.2017
 *      Author: AndreR
 */

#pragma once

#include "gfx.h"
#include "intercom_constants.h"
#include "../wifi.h"
#include "../network.h"

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
		uint8_t mac;
	} eth;

	struct
	{
		IPv4Address ip;
		uint16_t port;
	} vision;
} SetupData;

typedef void(*SetupChangedCallback)(const SetupData*);

GHandle SetupCreate();
void SetupUpdate(WifiConfig* pConfig, NetworkConfig* pNetworkConfig);
void SetupSetCallback(SetupChangedCallback cb);
