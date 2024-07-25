#include "pwr_ina226.h"
#include "hal/sys_time.h"
#include <string.h>

#define I2C_ADDR_INA226	0x40

#define INA226_REG_CONFIGURATION	0x00
#define INA226_REG_SHUNT_VOLTAGE	0x01
#define INA226_REG_BUS_VOLTAGE		0x02

static void pwrTask(void* pParam);

void PwrINA226Init(PwrINA226* pPwr, I2C* pBus, tprio_t prio)
{
	chMtxObjectInit(&pPwr->measMutex);
	chEvtObjectInit(&pPwr->eventSource);

	pPwr->pBus = pBus;

	DeviceProfilerInit(&pPwr->profiler, 1.0f);

	pPwr->pTask = chThdCreateStatic(pPwr->waTask, sizeof(pPwr->waTask), prio, &pwrTask, pPwr);
}

void PwrINA226Get(PwrINA226* pPwr, PwrINA226Measurement* pMeas)
{
	chMtxLock(&pPwr->measMutex);
	memcpy(pMeas, &pPwr->meas, sizeof(pPwr->meas));
	chMtxUnlock(&pPwr->measMutex);
}

static void configure(PwrINA226* pPwr)
{
	uint8_t* pTx;
	I2CAcquire(pPwr->pBus, &pTx, 0);

	// sample time 588us and 4x averaging on bus and shunt => 4704us per conversion
	pTx[0] = 0x00;
	pTx[1] = 0b01000010;
	pTx[2] = 0b11011111;
	I2CWrite(pPwr->pBus, I2C_ADDR_INA226, 3);

	chThdSleepMilliseconds(10);

	I2CRelease(pPwr->pBus);
}

void update(PwrINA226* pPwr)
{
	uint8_t* pTx;
	uint8_t* pRx;

	I2CAcquire(pPwr->pBus, &pTx, &pRx);

	DeviceProfilerBegin(&pPwr->profiler);

	pTx[0] = INA226_REG_SHUNT_VOLTAGE;
	I2CTransfer(pPwr->pBus, I2C_ADDR_INA226, 1, 2);
	int16_t shunt = *((int16_t*)pRx);

	pTx[0] = INA226_REG_BUS_VOLTAGE;
	I2CTransfer(pPwr->pBus, I2C_ADDR_INA226, 1, 2);
	uint16_t bus = *((uint16_t*)pRx);

	bus = __REV16(bus);
	shunt = __REV16(shunt);

	I2CRelease(pPwr->pBus);

	DeviceProfilerEnd(&pPwr->profiler);

	chMtxLock(&pPwr->measMutex);

	pPwr->meas.timestamp_us = SysTimeUSec();
	pPwr->meas.bus_V = bus * 1.25e-3f;
	pPwr->meas.shunt_mV = shunt * 2.5e-3f;

	chMtxUnlock(&pPwr->measMutex);

	chEvtBroadcastFlags(&pPwr->eventSource, PWR_INA226_EVENT_MEAS_UPDATED);
}

static void pwrTask(void* pParam)
{
	PwrINA226* pPwr = (PwrINA226*)pParam;

	chRegSetThreadName("PWR_INA226");

	configure(pPwr);

	systime_t prev = chVTGetSystemTimeX();
	systime_t next = prev + TIME_US2I(1000);

	while(1)
	{
		update(pPwr);

		// wait until next loop
		prev = chThdSleepUntilWindowed(prev, next);
		next += TIME_US2I(2000);
	}
}
