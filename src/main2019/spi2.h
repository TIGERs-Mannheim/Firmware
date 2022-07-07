/*
 * spi2.h
 *
 *  Created on: 03.02.2019
 *      Author: AndreR
 */

#pragma once

#include "util/spi_sync.h"

#define SPI2_KICK_SAMPLES 16

typedef struct _SPI2Global
{
	SPISync bus;

	struct _kicker
	{
		SPISyncSlave slave;

		uint8_t tx[4*SPI2_KICK_SAMPLES];
		uint16_t rx[2*SPI2_KICK_SAMPLES];

		uint32_t sampleTime;
	} kicker;
} SPI2Global;

extern SPI2Global spi2;

void SPI2Init();
void SPI2Task(void* params);
