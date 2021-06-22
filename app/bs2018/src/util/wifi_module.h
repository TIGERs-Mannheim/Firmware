/*
 * wifi_module.h
 *
 *  Created on: 24.10.2017
 *      Author: AndreR
 */

#pragma once

#include "util/sx1280.h"
#include "util/sky66112.h"

typedef struct _WifiModuleData
{
	SX1280Data* pRadioInit;
	SX1280SettingsFLRC* pRadioSettings;

	SKY66112* pFEMInit;

	GPIOPin pwrPin;
} WifiModuleData;

typedef struct _WifiModule
{
	SX1280 radio;
	SX1280SettingsFLRC radioSettings;

	SKY66112 fem;

	GPIOPin pwrPin;
} WifiModule;

void WifiModuleInit(WifiModule* pModule, WifiModuleData* pData);
int16_t WifiModuleTransmitAndReceive(WifiModule* pModule,  uint8_t antenna, uint8_t address,
		uint8_t numBytes, void* pTxRxData, SX1280RxResult* pRxResult);
