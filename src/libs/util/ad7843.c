/*
 * ad7843.c
 *
 *  Created on: 02.11.2017
 *      Author: AndreR
 */

#include "ad7843.h"
#include "util/sys_time.h"

void AD7843Init(AD7843* pAd, SPISync* pBus, GPIOPin csPin, GPIOPin irqPin)
{
	GPIOInitData gpioInit;

	pAd->pBus = pBus;
	pAd->pressed = 0;
	pAd->x = 0;
	pAd->y = 0;
	pAd->inv = 0;
	pAd->irqPin = irqPin;

	pAd->slave.cpha = 0;
	pAd->slave.cpol = 0;
	pAd->slave.csPin = csPin.pin;
	pAd->slave.pCSPort = csPin.pPort;
#ifdef STM32H7XX
	pAd->slave.prescaler = SPI_CFG1_BRDIV128;
#else
	pAd->slave.prescaler = SPI_CR1_BRDIV64;
#endif
	pAd->slave.timeoutTicks = MS2ST(20);
	SPISyncSlaveInit(pBus, &pAd->slave);

	// init touch IRQ
	gpioInit.mode = GPIO_MODE_INPUT;
	gpioInit.alternate = 0;
	gpioInit.pupd = GPIO_PUPD_UP;
	GPIOInit(irqPin.pPort, irqPin.pin, &gpioInit);

	for(uint16_t i = 0; i < 2*AD7843_TOUCH_SAMPLES; i += 2)
		pAd->tx[i] = 0x90;	// X sample

	for(uint16_t i = 2*AD7843_TOUCH_SAMPLES; i < 4*AD7843_TOUCH_SAMPLES; i += 2)
		pAd->tx[i] = 0xD0;	// Y sample
}

void AD7843Update(AD7843* pAd)
{
	if(pAd->irqPin.pPort->IDR & pAd->irqPin.pin)
	{
		pAd->pressed = 0;
		return;
	}

	pAd->pressed = 1;

	uint32_t start = SysTimeUSec();

	SPISyncTransfer(&pAd->slave, pAd->tx, pAd->rx, 4*AD7843_TOUCH_SAMPLES+1);

	uint32_t x = 0;
	uint32_t y = 0;

	uint32_t val;

	for(uint16_t i = 1*AD7843_TOUCH_SAMPLES+1; i < 2*AD7843_TOUCH_SAMPLES+1; i += 2)
	{
		if(pAd->inv)
			val = (((uint16_t)pAd->rx[i+1]) << 5) | (pAd->rx[i] >> 3);
		else
			val = (((uint16_t)pAd->rx[i]) << 5) | (pAd->rx[i+1] >> 3);

		if(val > 4096)
			pAd->inv = 1;

		x += val;
	}

	for(uint16_t i = 3*AD7843_TOUCH_SAMPLES+1; i < 4*AD7843_TOUCH_SAMPLES+1; i += 2)
	{
		if(pAd->inv)
			val = (((uint16_t)pAd->rx[i+1]) << 5) | (pAd->rx[i] >> 3);
		else
			val = (((uint16_t)pAd->rx[i]) << 5) | (pAd->rx[i+1] >> 3);

		if(val > 4096)
			pAd->inv = 1;

		y += val;
	}

	x /= AD7843_TOUCH_SAMPLES/2;
	y /= AD7843_TOUCH_SAMPLES/2;

	pAd->x = x;
	pAd->y = y;

	uint32_t end = SysTimeUSec();
	pAd->sampleTime = end-start;
}
