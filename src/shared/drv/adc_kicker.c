#include "adc_kicker.h"
#include "util/log.h"
#include "hal/sys_time.h"
#include <string.h>

static void adcTask(void* pParam);

void ADCKickerInit(ADCKicker* pAdc, SPI* pSPI, GPIOPin csPin, uint32_t prescaler, tprio_t prio)
{
	chMtxObjectInit(&pAdc->measMutex);
	chEvtObjectInit(&pAdc->eventSource);

	SPISlave* pSlave = &pAdc->spiSlave;
	pSlave->cpol = 1;
	pSlave->cpha = 1;
	pSlave->csPin = csPin;
	pSlave->prescaler = prescaler;
	pSlave->timeoutTicks = TIME_MS2I(100);

	SPISlaveInit(pSPI, &pAdc->spiSlave);

	if(!SPIHasEnoughMemory(pSlave, sizeof(pAdc->tx)))
	{
		chSysHalt("Not enough SPI memory for ADCKicker driver");
	}

	for(uint16_t i = 0; i < ADC_KICKER_SAMPLES; i++)
	{
		pAdc->tx[i*2] = 0 << 3;
		pAdc->tx[i*2+1] = 0;
	}
	for(uint16_t i = ADC_KICKER_SAMPLES; i < 2*ADC_KICKER_SAMPLES; i++)
	{
		pAdc->tx[i*2] = 1 << 3;
		pAdc->tx[i*2+1] = 0;
	}

	DeviceProfilerInit(&pAdc->profiler, 1.0f);

	pAdc->pTask = chThdCreateStatic(pAdc->waTask, sizeof(pAdc->waTask), prio, &adcTask, pAdc);
}

void ADCKickerGet(ADCKicker* pAdc, ADCKickerMeasurement* pMeas)
{
	chMtxLock(&pAdc->measMutex);
	memcpy(pMeas, &pAdc->meas, sizeof(pAdc->meas));
	chMtxUnlock(&pAdc->measMutex);
}

static void update(ADCKicker* pAdc)
{
	uint8_t* pTx;
	uint16_t* pRx;

	SPIAcquire(&pAdc->spiSlave, &pTx, &pRx);

	DeviceProfilerBegin(&pAdc->profiler);

	memcpy(pTx, pAdc->tx, sizeof(pAdc->tx));

	int16_t result = SPITransfer(&pAdc->spiSlave, sizeof(pAdc->tx));
	if(result)
	{
		SPIRelease(&pAdc->spiSlave);
		return;
	}

	uint32_t ch1Sum = 0;
	uint32_t ch2Sum = 0;
	uint8_t validCh1Samples = 0;
	uint8_t validCh2Samples = 0;

	// sum up measurements, we ignore the first two measurements
	// the sampling capacitor has to settle in that time
	for(uint16_t i = 2; i < ADC_KICKER_SAMPLES; i++)
	{
		uint16_t sample = __REV16(pRx[i]);
		if(sample < 4096)
		{
			ch1Sum += sample;
			++validCh1Samples;
		}
	}
	for(uint16_t i = ADC_KICKER_SAMPLES+2; i < 2*ADC_KICKER_SAMPLES; i++)
	{
		uint16_t sample = __REV16(pRx[i]);
		if(sample < 4096)
		{
			ch2Sum += sample;
			++validCh2Samples;
		}
	}

	SPIRelease(&pAdc->spiSlave);

	DeviceProfilerEnd(&pAdc->profiler);

	// we need at least 4 valid samples for a reasonable measurement
	if(validCh1Samples > 3 && validCh2Samples > 3)
	{
		float ch1 = ch1Sum/(validCh1Samples*4096.0f);
		float ch2 = ch2Sum/(validCh2Samples*4096.0f);

		chMtxLock(&pAdc->measMutex);

		pAdc->meas.timestamp_us = SysTimeUSec();
		pAdc->meas.capLevel_V = 251.42f*ch2;
		pAdc->meas.boardTemp_degC = (6600.0f*ch1-800.0f)*0.02564102564102564f;

		chMtxUnlock(&pAdc->measMutex);

		chEvtBroadcast(&pAdc->eventSource);
	}
	else
	{
		LogWarnC("Not enough ADC samples.", validCh1Samples | ((uint16_t)validCh2Samples) << 8);
	}
}

static void adcTask(void* pParam)
{
	ADCKicker* pAdc = (ADCKicker*)pParam;

	chRegSetThreadName("ADC_KICKER");

	systime_t prev = chVTGetSystemTimeX();
	systime_t next = prev + TIME_US2I(1000);

	while(1)
	{
		update(pAdc);

		// wait until next loop
		prev = chThdSleepUntilWindowed(prev, next);
		next += TIME_US2I(1000);
	}
}
