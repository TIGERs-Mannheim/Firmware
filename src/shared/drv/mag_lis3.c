#include "mag_lis3.h"
#include "hal/sys_time.h"
#include <string.h>

#define LIS3_READ 0x80
#define LIS3_AUTO_INC 0x40

#define LIS3_CTRL_REG1 0x20
#define LIS3_STATUS_REG 0x27
#define LIS3_TEMP_OUT_L 0x2E

#define MAG_UNIT_TO_UT (0.01220703125f)

static void magTask(void* pParam);

void MagLIS3Init(MagLIS3* pMag, SPI* pSPI, GPIOPin csPin, uint32_t prescaler, tprio_t prio)
{
	chMtxObjectInit(&pMag->measMutex);

	SPISlave* pSlave = &pMag->spiSlave;
	pSlave->cpol = 0;
	pSlave->cpha = 0;
	pSlave->csPin = csPin;
	pSlave->prescaler = prescaler;
	pSlave->timeoutTicks = TIME_MS2I(100);

	SPISlaveInit(pSPI, &pMag->spiSlave);

	if(!SPIHasEnoughMemory(pSlave, 16))
	{
		chSysHalt("Not enough SPI memory for LIS3 driver");
	}

	DeviceProfilerInit(&pMag->profiler, 1.0f);

	pMag->pTask = chThdCreateStatic(pMag->waTask, sizeof(pMag->waTask), prio, &magTask, pMag);
}

void MagLIS3Get(MagLIS3* pMag, MagLIS3Measurement* pMeas)
{
	chMtxLock(&pMag->measMutex);
	memcpy(pMeas, &pMag->meas, sizeof(pMag->meas));
	chMtxUnlock(&pMag->measMutex);
}

static void configure(MagLIS3* pMag)
{
	uint8_t* pTx;
	SPIAcquire(&pMag->spiSlave, &pTx, 0);

	chThdSleepMilliseconds(20);

	pTx[0] = LIS3_CTRL_REG1 | LIS3_AUTO_INC;
	pTx[1] = 0b11111110; // 155Hz output datarate, UHP mode on XY
	pTx[2] = 0; // +-4 gauss full-scale
	pTx[3] = 0; // continuous conversion
	pTx[4] = 0b00001100; // UHP mode on Z
	pTx[5] = 0x40; // block data update
	SPITransfer(&pMag->spiSlave, 6);
	chThdSleepMilliseconds(20);

	SPIRelease(&pMag->spiSlave);
}

static void update(MagLIS3* pMag)
{
	uint8_t* pTx;
	uint8_t* pRx;

	SPIAcquire(&pMag->spiSlave, &pTx, &pRx);

	DeviceProfilerBegin(&pMag->profiler);

	memset(pTx, 0, 8);
	pTx[0] = LIS3_STATUS_REG | LIS3_AUTO_INC | LIS3_READ;
	SPITransfer(&pMag->spiSlave, 8);

	// check if there is new data ready
	if((pRx[1] & 0x07) == 0)
	{
		SPIRelease(&pMag->spiSlave);
		return;
	}

	int16_t strength[3];
	strength[0] = *((int16_t*)&pRx[2]);
	strength[1] = *((int16_t*)&pRx[4]);
	strength[2] = *((int16_t*)&pRx[6]);

	// strangely, the above auto increment does not pass on beyond OUT registers
	// an extra read is required for temperature
	pTx[0] = LIS3_TEMP_OUT_L | LIS3_AUTO_INC | LIS3_READ;
	SPITransfer(&pMag->spiSlave, 3);

	int16_t temp = *((int16_t*)&pRx[1]);

	SPIRelease(&pMag->spiSlave);

	DeviceProfilerEnd(&pMag->profiler);

	chMtxLock(&pMag->measMutex);

	pMag->meas.timestamp_us = SysTimeUSec();

	for(uint8_t i = 0; i < 3; i++)
	{
		pMag->meas.strength_uT[i] = strength[i]*MAG_UNIT_TO_UT;
	}

	pMag->meas.temp_degC = temp*0.125f + 25.0f;

	chMtxUnlock(&pMag->measMutex);
}

static void magTask(void* pParam)
{
	MagLIS3* pMag = (MagLIS3*)pParam;

	chRegSetThreadName("MAG_LIS3");

	configure(pMag);

	systime_t prev = chVTGetSystemTimeX();
	systime_t next = prev + TIME_US2I(1000);

	while(1)
	{
		update(pMag);

		// wait until next loop
		prev = chThdSleepUntilWindowed(prev, next);
		next += TIME_US2I(2500);
	}
}
