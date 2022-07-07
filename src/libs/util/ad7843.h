/*
 * ad7843.h
 *
 *  Created on: 02.11.2017
 *      Author: AndreR
 *
 * Driver for 4-wire resistive touchscreen controller.
 * Also works for ADS7843 and XPT2046.
 */

#pragma once

#include "util/spi_sync.h"
#include "util/init_hal.h"

#define AD7843_TOUCH_SAMPLES 16

// this structure contains DMA data fields and should be put into DTC memory!
typedef struct _AD7843
{
	uint8_t tx[2*2*AD7843_TOUCH_SAMPLES+2];
	uint8_t rx[2*2*AD7843_TOUCH_SAMPLES+2];

	SPISync* pBus;
	SPISyncSlave slave;

	uint8_t pressed;
	uint16_t x;
	uint16_t y;

	uint8_t inv;

	GPIOPin irqPin;

	uint32_t sampleTime;
} AD7843;

void AD7843Init(AD7843* pAd, SPISync* pBus, GPIOPin csPin, GPIOPin irqPin);
void AD7843Update(AD7843* pAd);
