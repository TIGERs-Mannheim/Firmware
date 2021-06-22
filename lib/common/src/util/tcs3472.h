/*
 * tcs3472.h
 *
 *  Created on: 13.03.2020
 *      Author: AndreR
 *
 * TCS34725 and TCS34727 color light sensor.
 */

#pragma once

#include "util/i2c_soft.h"

typedef union _RGBCSample
{
	uint16_t data[4];
	struct
	{
		uint16_t clear;
		uint16_t red;
		uint16_t green;
		uint16_t blue;
	};
} RGBCSample;

typedef union _RGBCSamplef
{
	float data[4];
	struct
	{
		float clear;
		float red;
		float green;
		float blue;
	};
} RGBCSamplef;

typedef struct _TCS3472
{
	I2CSoft* pI2C;
	uint8_t status; // last read from status register
	uint8_t regEnable; // current config of enable register
} TCS3472;

void TCS3472Init(TCS3472* pTcs, I2CSoft* pI2C);
int16_t TCS3472GetId(TCS3472* pTcs, uint8_t* pId);
int16_t TCS3472SetPowerOn(TCS3472* pTcs, uint8_t enable);
int16_t TCS3472SetRgbcEnable(TCS3472* pTcs, uint8_t enable);
int16_t TCS3472GetSample(TCS3472* pTcs, RGBCSample* pSample, uint8_t* pUpdated);
