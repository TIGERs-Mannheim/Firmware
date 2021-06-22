/*
 * tcs3472.c
 *
 *  Created on: 13.03.2020
 *      Author: AndreR
 */

#include "util/tcs3472.h"
#include <string.h>

#define I2C_ADDR 0x29

#define REG_ENABLE 0x00
#define REG_ATIME 0x01
#define REG_CONFIG 0x0D
#define REG_CONTROL 0x0F
#define REG_ID 0x12
#define REG_STATUS 0x13
#define REG_CDATAL 0x14

void TCS3472Init(TCS3472* pTcs, I2CSoft* pI2C)
{
	pTcs->pI2C = pI2C;
}

static int16_t readReg(TCS3472* pTcs, uint8_t reg, uint8_t* pValue)
{
	reg |= 0x80;

	int16_t result = I2CSoftWrite(pTcs->pI2C, I2C_ADDR, 1, &reg);
	if(result)
		return result;

	result = I2CSoftRead(pTcs->pI2C, I2C_ADDR, 1, pValue);

	return result;
}

static int16_t writeReg(TCS3472* pTcs, uint8_t reg, uint8_t value)
{
	uint8_t data[2] = { reg | 0x80, value };

	return I2CSoftWrite(pTcs->pI2C, I2C_ADDR, 2, data);
}

int16_t TCS3472GetId(TCS3472* pTcs, uint8_t* pId)
{
	return readReg(pTcs, REG_ID, pId);
}

int16_t TCS3472SetPowerOn(TCS3472* pTcs, uint8_t enable)
{
	if(!enable)
	{
		pTcs->regEnable = 0;
		return writeReg(pTcs, REG_ENABLE, 0);
	}

	pTcs->regEnable = 0x01;
	int16_t result = writeReg(pTcs, REG_ENABLE, 0x01);
	if(result)
		return result;

	chThdSleepMilliseconds(5);

	result = writeReg(pTcs, REG_ATIME, 0xD5); // 101ms
	if(result)
		return result;

	result = writeReg(pTcs, REG_CONTROL, 0x00); // 1x gain
	if(result)
		return result;

	return 0;
}

int16_t TCS3472SetRgbcEnable(TCS3472* pTcs, uint8_t enable)
{
	if(enable)
		pTcs->regEnable |= 0x02;
	else
		pTcs->regEnable &= ~0x02;

	return writeReg(pTcs, REG_ENABLE, pTcs->regEnable);
}

int16_t TCS3472GetSample(TCS3472* pTcs, RGBCSample* pSample, uint8_t* pUpdated)
{
	*pUpdated = 0;

	uint8_t reg = REG_STATUS | 0x80 | 0x20; // write command reg, auto-increment

	int16_t result = I2CSoftWrite(pTcs->pI2C, I2C_ADDR, 1, &reg);
	if(result)
		return result;

	uint8_t data[9];

	result = I2CSoftRead(pTcs->pI2C, I2C_ADDR, 9, data);
	if(result)
		return result;

	pTcs->status = data[0];

	if(pTcs->status & 0x10)
	{
		memcpy(pSample, data+1, 8);
		*pUpdated = 1;

		reg = 0x80 | 0x60 | 0x6;
		result = I2CSoftWrite(pTcs->pI2C, I2C_ADDR, 1, &reg);
		if(result)
			return result;
	}

	return 0;
}
