/*
 * spi4.h
 *
 *  Created on: 31.10.2015
 *      Author: AndreR
 */

#pragma once

#include "util/spi_sync.h"
#include "util/ad7843.h"
#include "log_msgs.h"

#define SPI4_TOUCH_SAMPLES 16
#define SPI4_MOTOR_ADC_SAMPLES 32

typedef struct _SPI4Global
{
	SPISync bus;

	AD7843 touch;

	uint16_t leftBoardType;
	uint16_t rightBoardType;

	uint8_t motCurrentSenseInstalled;

	struct _motors
	{
		SPISyncSlave slave;

		uint8_t tx[SPI4_MOTOR_ADC_SAMPLES*4*2];
		uint16_t rx[SPI4_MOTOR_ADC_SAMPLES*4];

		uint32_t sampleTime;	// [us]

		float raw[4];
		float biasCurrent[4];
		float current[4];

		uint32_t measTimestamp;
	} motors;
} SPI4Global;

extern SPI4Global spi4;

void SPI4Init();
void SPI4Task(void* params);
void SPI4CurrentBiasCalibration();
