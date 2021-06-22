/*
 * spi4.h
 *
 *  Created on: 07.01.2019
 *      Author: AndreR
 */

#pragma once

#include "util/spi_sync.h"
#include "util/ad7843.h"

typedef struct _SPI4Global
{
	SPISync bus;

	AD7843 touch;

	struct _imu
	{
		SPISyncSlave slave;

		uint8_t tx[16];
		uint8_t rx[16];

		uint32_t sampleTime;

		float acc[3];	// [m/s^2]
		float temp; 	// [°C]
		float gyr[3];	// [rad/s]
		uint8_t updated;

		uint32_t numUpdates;
		float updateRate;
	} imu;

	struct _mag
	{
		SPISyncSlave slave;

		uint8_t tx[16];
		uint8_t rx[16];

		uint32_t sampleTime;

		float strength[3];	// [uT]
		float temp;			// [°C]
		uint8_t updated;

		uint32_t numUpdates;
		float updateRate;
	} mag;
} SPI4Global;

extern SPI4Global spi4;

void SPI4Init();
void SPI4Task(void* params);
