#pragma once

#include "gfx.h"
#include "robot/network.h"

typedef struct _PresenterWifiStat
{
	NetworkMode networkMode;
	uint8_t sumatraOnline;
	uint8_t bsOnline;
	uint8_t channel;
	uint8_t botId;
	uint16_t updateFreq;  // [Hz]
	uint16_t visionDelay; // [us]
	float rssi;			  // [dBm]
} PresenterWifiStat;

GHandle WifiCreate();
void WifiUpdateStatus(PresenterWifiStat* pStat);
