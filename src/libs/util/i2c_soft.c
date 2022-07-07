/*
 * i2c_soft.c
 *
 *  Created on: 13.03.2020
 *      Author: AndreR
 */

#include "util/i2c_soft.h"
#include "errors.h"

#define SDA_HIGH() GPIOSet(pI2C->pSDAPort, pI2C->sdaPin)
#define SCL_HIGH() GPIOSet(pI2C->pSCLPort, pI2C->sclPin)
#define SDA_LOW() GPIOReset(pI2C->pSDAPort, pI2C->sdaPin)
#define SCL_LOW() GPIOReset(pI2C->pSCLPort, pI2C->sclPin)


void I2CSoftTick(I2CSoft* pI2C)
{
	chSysLockFromISR();
	chBSemSignalI(&pI2C->semTick);
	chSysUnlockFromISR();
}

void I2CSoftInit(I2CSoft* pI2C, GPIOPin sclPin, GPIOPin sdaPin)
{
	chBSemObjectInit(&pI2C->semTick, TRUE);

	pI2C->pSCLPort = sclPin.pPort;
	pI2C->sclPin = sclPin.pin;
	pI2C->pSDAPort = sdaPin.pPort;
	pI2C->sdaPin = sdaPin.pin;

	SDA_HIGH();
	SCL_HIGH();

	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_OUTPUT;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.otype = GPIO_OTYPE_OPEN_DRAIN;
	gpioInit.pupd = GPIO_PUPD_NONE;

	GPIOInit(sclPin.pPort, sclPin.pin, &gpioInit);
	GPIOInit(sdaPin.pPort, sdaPin.pin, &gpioInit);
}

static void waitForTick(I2CSoft* pI2C)
{
	if(chBSemWaitTimeout(&pI2C->semTick, MS2ST(10)) != MSG_OK)
		pI2C->timeoutOccured = 1;

	if(chBSemWaitTimeout(&pI2C->semTick, MS2ST(10)) != MSG_OK)
		pI2C->timeoutOccured = 1;
}

static int16_t start(I2CSoft* pI2C)
{
	// generate start condition
	waitForTick(pI2C);
	SCL_HIGH();
	SDA_HIGH();

	waitForTick(pI2C);

	// SDA and SCL are not high some slave is broken, we lost arbitration
	uint8_t sdaState = (pI2C->pSDAPort->IDR & pI2C->sdaPin) ? 1 : 0;
	uint8_t sclState = (pI2C->pSCLPort->IDR & pI2C->sclPin) ? 1 : 0;
	if(!sdaState || !sclState)
		return ERROR_I2C_SOFT_ARBITRATION_LOST;

	SDA_LOW();

	waitForTick(pI2C);
	SCL_LOW();

	return 0;
}

static void stop(I2CSoft* pI2C)
{
	// generate stop condition
	waitForTick(pI2C);
	SCL_LOW();
	SDA_LOW();

	waitForTick(pI2C);
	SCL_HIGH();

	waitForTick(pI2C);
	SDA_HIGH();
}

static int16_t writeByte(I2CSoft* pI2C, uint8_t data)
{
	// clock out bits, MSB first
	for(int8_t i = 7; i >= 0; i--)
	{
		waitForTick(pI2C);
		if(data & (1 << i))
			SDA_HIGH();
		else
			SDA_LOW();

		waitForTick(pI2C);
		SCL_HIGH();

		waitForTick(pI2C);
		SCL_LOW();
	}

	// read in ACK
	waitForTick(pI2C);
	SDA_HIGH();

	waitForTick(pI2C);
	SCL_HIGH();

	waitForTick(pI2C);
	uint8_t ack = (pI2C->pSDAPort->IDR & pI2C->sdaPin) ? 0 : 1;
	SCL_LOW();

	if(!ack)
		return ERROR_I2C_SOFT_NACK;

	return 0;
}

static void readByte(I2CSoft* pI2C, uint8_t* pData, uint8_t ack)
{
	// read in data, MSB first
	*pData = 0;
	for(int8_t i = 7; i >= 0; i--)
	{
		waitForTick(pI2C);
		SDA_HIGH();

		waitForTick(pI2C);
		SCL_HIGH();

		waitForTick(pI2C);
		uint8_t read = (pI2C->pSDAPort->IDR & pI2C->sdaPin) ? 1 : 0;
		*pData |= (read << i);
		SCL_LOW();
	}

	// send ACK (pull SDA low) or NACK (leave SDA high)
	waitForTick(pI2C);
	if(ack)
		SDA_LOW();
	else
		SDA_HIGH();

	waitForTick(pI2C);
	SCL_HIGH();

	waitForTick(pI2C);
	SCL_LOW();
}

int16_t I2CSoftWrite(I2CSoft* pI2C, uint8_t addr, uint8_t length, uint8_t* pData)
{
	int16_t result;

	pI2C->timeoutOccured = 0;

	result = start(pI2C);
	if(result)
	{
		stop(pI2C);
		return result;
	}

	result = writeByte(pI2C, (addr << 1));
	if(result)
	{
		stop(pI2C);
		return result;
	}

	for(uint8_t i = 0; i < length; i++)
	{
		writeByte(pI2C, pData[i]);
	}

	stop(pI2C);

	return 0;
}

int16_t I2CSoftRead(I2CSoft* pI2C, uint8_t addr, uint8_t length, uint8_t* pData)
{
	int16_t result;

	pI2C->timeoutOccured = 0;

	result = start(pI2C);
	if(result)
	{
		stop(pI2C);
		return result;
	}

	result = writeByte(pI2C, (addr << 1) | 1);
	if(result)
	{
		stop(pI2C);
		return result;
	}

	for(uint8_t i = 0; i < length; i++)
	{
		readByte(pI2C, pData+i, i != (length-1));
	}

	stop(pI2C);

	return 0;
}
