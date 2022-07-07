/*
 * sky66112.c
 *
 *  Created on: 24.10.2017
 *      Author: AndreR
 */

#include "sky66112.h"

void SKY66112Init(SKY66112* pSky)
{
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_OUTPUT;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.pupd = GPIO_PUPD_NONE;

	if(pSky->antPin.pPort != 0)
	{
		GPIOReset(pSky->antPin.pPort, pSky->antPin.pin);
		GPIOInit(pSky->antPin.pPort, pSky->antPin.pin, &gpioInit);
	}

	GPIOReset(pSky->cpsPin.pPort, pSky->cpsPin.pin);
	GPIOInit(pSky->cpsPin.pPort, pSky->cpsPin.pin, &gpioInit);

	GPIOReset(pSky->crxPin.pPort, pSky->crxPin.pin);
	GPIOInit(pSky->crxPin.pPort, pSky->crxPin.pin, &gpioInit);

	GPIOReset(pSky->ctxPin.pPort, pSky->ctxPin.pin);
	GPIOInit(pSky->ctxPin.pPort, pSky->ctxPin.pin, &gpioInit);
}

void SKY66112SetMode(SKY66112* pSky, uint8_t mode)
{
	switch(mode)
	{
		case SKY66112_MODE_RX:
		{
			GPIOReset(pSky->cpsPin.pPort, pSky->cpsPin.pin);
			GPIOSet(pSky->crxPin.pPort, pSky->crxPin.pin);
			GPIOReset(pSky->ctxPin.pPort, pSky->ctxPin.pin);
		}
		break;
		case SKY66112_MODE_TX:
		{
			GPIOReset(pSky->cpsPin.pPort, pSky->cpsPin.pin);
			GPIOSet(pSky->ctxPin.pPort, pSky->ctxPin.pin);
			GPIOReset(pSky->crxPin.pPort, pSky->crxPin.pin);
		}
		break;
		case SKY66112_MODE_RX_BYPASS:
		{
			GPIOSet(pSky->cpsPin.pPort, pSky->cpsPin.pin);
			GPIOSet(pSky->crxPin.pPort, pSky->crxPin.pin);
			GPIOReset(pSky->ctxPin.pPort, pSky->ctxPin.pin);
		}
		break;
		case SKY66112_MODE_TX_BYPASS:
		{
			GPIOSet(pSky->cpsPin.pPort, pSky->cpsPin.pin);
			GPIOSet(pSky->ctxPin.pPort, pSky->ctxPin.pin);
			GPIOReset(pSky->crxPin.pPort, pSky->crxPin.pin);
		}
		break;
		default:
		{
			GPIOReset(pSky->cpsPin.pPort, pSky->cpsPin.pin);
			GPIOReset(pSky->crxPin.pPort, pSky->crxPin.pin);
			GPIOReset(pSky->ctxPin.pPort, pSky->ctxPin.pin);
		}
		break;
	}
}

void SKY66112UseAntenna(SKY66112* pSky, uint8_t ant)
{
	if(pSky->antPin.pPort == 0)
		return;

	if(ant == 0)
		GPIOReset(pSky->antPin.pPort, pSky->antPin.pin);
	else
		GPIOSet(pSky->antPin.pPort, pSky->antPin.pin);
}
