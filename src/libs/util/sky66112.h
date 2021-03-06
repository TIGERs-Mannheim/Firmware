/*
 * sky66112.h
 *
 *  Created on: 24.10.2017
 *      Author: AndreR
 *
 * SKY66112 RF Front End Module
 */

#pragma once

#include "util/init_hal.h"

typedef struct _SKY66112
{
	GPIOPin antPin;
	GPIOPin cpsPin;
	GPIOPin crxPin;
	GPIOPin ctxPin;
} SKY66112;

#define SKY66112_MODE_OFF 0
#define SKY66112_MODE_RX 1
#define SKY66112_MODE_TX 2
#define SKY66112_MODE_RX_BYPASS 3
#define SKY66112_MODE_TX_BYPASS 4

void SKY66112Init(SKY66112* pSky);
void SKY66112SetMode(SKY66112* pSky, uint8_t mode);
void SKY66112UseAntenna(SKY66112* pSky, uint8_t ant);
