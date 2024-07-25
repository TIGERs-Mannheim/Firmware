#include "touch_ad7843.h"
#include "hal/sys_time.h"
#include <string.h>

static void touchTask(void* pParam);

void TouchAD7843Init(TouchAD7843* pTouch, SPI* pSPI, GPIOPin csPin, GPIOPin irqPin, uint32_t prescaler, tprio_t prio)
{
	chMtxObjectInit(&pTouch->measMutex);

	SPISlave* pSlave = &pTouch->spiSlave;
	pSlave->cpol = 0;
	pSlave->cpha = 0;
	pSlave->csPin = csPin;
	pSlave->prescaler = prescaler;
	pSlave->timeoutTicks = TIME_MS2I(20);

	SPISlaveInit(pSPI, &pTouch->spiSlave);

	if(!SPIHasEnoughMemory(pSlave, 2*2*AD7843_TOUCH_SAMPLES+2))
	{
		chSysHalt("Not enough SPI memory for AD7843 driver");
	}

	pTouch->irqPin = irqPin;

	GPIOInitData gpioInit;

	// init touch IRQ (used as regular input)
	gpioInit.mode = GPIO_MODE_INPUT;
	gpioInit.alternate = 0;
	gpioInit.pupd = GPIO_PUPD_UP;
	GPIOInit(irqPin.pPort, irqPin.pin, &gpioInit);

	for(uint16_t i = 0; i < 2*AD7843_TOUCH_SAMPLES; i += 2)
		pTouch->tx[i] = 0x90;	// X sample

	for(uint16_t i = 2*AD7843_TOUCH_SAMPLES; i < 4*AD7843_TOUCH_SAMPLES; i += 2)
		pTouch->tx[i] = 0xD0;	// Y sample

	DeviceProfilerInit(&pTouch->profiler, 1.0f);

	pTouch->pTask = chThdCreateStatic(pTouch->waTask, sizeof(pTouch->waTask), prio, &touchTask, pTouch);
}

void TouchAD7843Get(TouchAD7843* pTouch, TouchAD7843Measurement* pMeas)
{
	chMtxLock(&pTouch->measMutex);
	memcpy(pMeas, &pTouch->meas, sizeof(pTouch->meas));
	chMtxUnlock(&pTouch->measMutex);
}

static void update(TouchAD7843* pTouch)
{
	if(pTouch->irqPin.pPort->IDR & pTouch->irqPin.pin)
	{
		DeviceProfilerBegin(&pTouch->profiler);
		DeviceProfilerEnd(&pTouch->profiler);

		chMtxLock(&pTouch->measMutex);
		pTouch->meas.pressed = 0;
		pTouch->meas.timestamp_us = SysTimeUSec();
		chMtxUnlock(&pTouch->measMutex);
	}
	else
	{
		uint8_t* pTx;
		uint8_t* pRx;

		SPIAcquire(&pTouch->spiSlave, &pTx, &pRx);

		DeviceProfilerBegin(&pTouch->profiler);

		memcpy(pTx, pTouch->tx, sizeof(pTouch->tx));
		SPITransfer(&pTouch->spiSlave, 4*AD7843_TOUCH_SAMPLES+1);

		uint32_t x = 0;
		uint32_t y = 0;

		uint32_t val;

		for(uint16_t i = 1*AD7843_TOUCH_SAMPLES+1; i < 2*AD7843_TOUCH_SAMPLES+1; i += 2)
		{
			if(pTouch->inv)
				val = (((uint16_t)pRx[i+1]) << 5) | (pRx[i] >> 3);
			else
				val = (((uint16_t)pRx[i]) << 5) | (pRx[i+1] >> 3);

			if(val > 4096)
				pTouch->inv = 1;

			x += val;
		}

		for(uint16_t i = 3*AD7843_TOUCH_SAMPLES+1; i < 4*AD7843_TOUCH_SAMPLES+1; i += 2)
		{
			if(pTouch->inv)
				val = (((uint16_t)pRx[i+1]) << 5) | (pRx[i] >> 3);
			else
				val = (((uint16_t)pRx[i]) << 5) | (pRx[i+1] >> 3);

			if(val > 4096)
				pTouch->inv = 1;

			y += val;
		}

		SPIRelease(&pTouch->spiSlave);

		DeviceProfilerEnd(&pTouch->profiler);

		chMtxLock(&pTouch->measMutex);

		pTouch->meas.pressed = 1;
		pTouch->meas.timestamp_us = SysTimeUSec();

		x /= AD7843_TOUCH_SAMPLES/2;
		y /= AD7843_TOUCH_SAMPLES/2;

		pTouch->meas.x = x;
		pTouch->meas.y = y;

		chMtxUnlock(&pTouch->measMutex);
	}
}

static void touchTask(void* pParam)
{
	TouchAD7843* pTouch = (TouchAD7843*)pParam;

	chRegSetThreadName("TOUCH_AD7843");

	systime_t prev = chVTGetSystemTimeX();
	systime_t next = prev + TIME_US2I(1000);

	while(1)
	{
		update(pTouch);

		// wait until next loop
		prev = chThdSleepUntilWindowed(prev, next);
		next += TIME_US2I(2000);
	}
}
