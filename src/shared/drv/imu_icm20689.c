#include "imu_icm20689.h"
#include "hal/sys_time.h"
#include <string.h>

#define ICM20689_READ	0x80
#define ICM20689_WRITE	0x00

#define ICM20689_CONFIG				26
#define ICM20689_GYRO_CONFIG		27
#define ICM20689_ACCEL_CONFIG		28
#define ICM20689_ACCEL_CONFIG2		29
#define ICM20689_FIFO_ENABLE		35
#define ICM20689_INT_STATUS			58
#define ICM20689_ACCEL_XOUT_H		59
#define ICM20689_USER_CONTROL		106
#define ICM20689_PWR_MGMT_1			107
#define ICM20689_PWR_MGMT_2			108
#define ICM20689_SIGNAL_PATH_RESET	104
#define ICM20689_FIFO_COUNT			114
#define ICM20689_FIFO_READ			116
#define ICM20689_WHOAMI				117

#define ACC_UNIT_TO_MS2 (0.00239501953125f)
#define GYR_UNIT_TO_RADS (0.0010652644360317f)

static void imuTask(void* pParam);

void ImuICM20689Init(ImuICM20689* pImu, SPI* pSPI, GPIOPin csPin, uint32_t prescaler, tprio_t prio)
{
	chMtxObjectInit(&pImu->measMutex);

	SPISlave* pSlave = &pImu->spiSlave;
	pSlave->cpol = 0;
	pSlave->cpha = 0;
	pSlave->csPin = csPin;
	pSlave->prescaler = prescaler;
	pSlave->timeoutTicks = TIME_MS2I(100);

	SPISlaveInit(pSPI, &pImu->spiSlave);

	if(!SPIHasEnoughMemory(pSlave, 16))
	{
		chSysHalt("Not enough SPI memory for ICM20689 driver");
	}

	DeviceProfilerInit(&pImu->profiler, 1.0f);

	pImu->pTask = chThdCreateStatic(pImu->waTask, sizeof(pImu->waTask), prio, &imuTask, pImu);
}

void ImuICM20689Get(ImuICM20689* pImu, ImuICM20689Measurement* pMeas)
{
	chMtxLock(&pImu->measMutex);
	memcpy(pMeas, &pImu->meas, sizeof(pImu->meas));
	chMtxUnlock(&pImu->measMutex);
}

static void configure(ImuICM20689* pImu)
{
	chThdSleepMilliseconds(100);

	uint8_t* pTx;
	SPIAcquire(&pImu->spiSlave, &pTx, 0);

	pTx[0] = ICM20689_PWR_MGMT_1 | ICM20689_WRITE;
	pTx[1] = 0x80; // DEVICE_RESET
	SPITransfer(&pImu->spiSlave, 2);
	chThdSleepMilliseconds(100);

	pTx[0] = ICM20689_USER_CONTROL | ICM20689_WRITE;
	pTx[1] = 0x11;	// reset all digital signal paths, disable I2C interface
	SPITransfer(&pImu->spiSlave, 2);
	chThdSleepMilliseconds(100);

	pTx[0] = ICM20689_PWR_MGMT_1 | ICM20689_WRITE;
	pTx[1] = 0x01; // auto-select best clock source, PLL if available
	SPITransfer(&pImu->spiSlave, 2);
	chThdSleepMilliseconds(100);

	pTx[0] = ICM20689_GYRO_CONFIG | ICM20689_WRITE;
	pTx[1] = 0x18; // +-2000dps full scale
	SPITransfer(&pImu->spiSlave, 2);
	chThdSleepMilliseconds(5);

	pTx[0] = ICM20689_ACCEL_CONFIG | ICM20689_WRITE;
	pTx[1] = 0x10; // +-8g full scale
	SPITransfer(&pImu->spiSlave, 2);
	chThdSleepMilliseconds(5);

	pTx[0] = ICM20689_ACCEL_CONFIG2 | ICM20689_WRITE;
//	pTx[1] = 0x08; // Acc BW 1046Hz, 4kHz rate
	pTx[1] = 0x07; // Acc BW 420Hz, 1kHz rate
	SPITransfer(&pImu->spiSlave, 2);
	chThdSleepMilliseconds(5);

	pTx[0] = ICM20689_CONFIG | ICM20689_WRITE;
//	pTx[1] = 0x07; // Gyr BW 3281Hz, 8kHz rate, Temp BW 4000Hz
	pTx[1] = 0x01; // Gyr BW 176Hz, 1kHz rate, Temp BW 188Hz
	SPITransfer(&pImu->spiSlave, 2);
	chThdSleepMilliseconds(5);

	SPIRelease(&pImu->spiSlave);
}

static void update(ImuICM20689* pImu)
{
	uint8_t* pTx;
	uint8_t* pRx;

	SPIAcquire(&pImu->spiSlave, &pTx, &pRx);

	DeviceProfilerBegin(&pImu->profiler);

	memset(pTx, 0, 2);

	pTx[0] = ICM20689_INT_STATUS | ICM20689_READ;
	SPITransfer(&pImu->spiSlave, 2);

	// check if there is new data ready
	if((pRx[1] & 0x01) == 0)
	{
		SPIRelease(&pImu->spiSlave);
		return;
	}

	pTx[0] = ICM20689_ACCEL_XOUT_H | ICM20689_READ; // read out accel, temp, gyro
	SPITransfer(&pImu->spiSlave, 15);

	int16_t acc[3];
	acc[0] = (int16_t)__REV16(*((uint16_t*)&pRx[1]));
	acc[1] = (int16_t)__REV16(*((uint16_t*)&pRx[3]));
	acc[2] = (int16_t)__REV16(*((uint16_t*)&pRx[5]));

	int16_t temp;
	temp = (int16_t)__REV16(*((uint16_t*)&pRx[7]));

	int16_t gyr[3];
	gyr[0] = (int16_t)__REV16(*((uint16_t*)&pRx[9]));
	gyr[1] = (int16_t)__REV16(*((uint16_t*)&pRx[11]));
	gyr[2] = (int16_t)__REV16(*((uint16_t*)&pRx[13]));

	SPIRelease(&pImu->spiSlave);

	DeviceProfilerEnd(&pImu->profiler);

	chMtxLock(&pImu->measMutex);

	pImu->meas.timestamp_us = SysTimeUSec();

	for(uint8_t i = 0; i < 3; i++)
	{
		pImu->meas.gyr_radDs[i] = gyr[i]*GYR_UNIT_TO_RADS;
		pImu->meas.acc_mDs2[i] = acc[i]*ACC_UNIT_TO_MS2;
	}

	// conversion according to datasheet
	pImu->meas.temp_degC = temp * (1.0f/326.8f) + 25.0f;

	chMtxUnlock(&pImu->measMutex);
}

static void imuTask(void* pParam)
{
	ImuICM20689* pImu = (ImuICM20689*)pParam;

	chRegSetThreadName("IMU_ICM20689");

	configure(pImu);

	systime_t prev = chVTGetSystemTimeX();
	systime_t next = prev + TIME_US2I(1000);

	while(1)
	{
		update(pImu);

		// wait until next loop
		prev = chThdSleepUntilWindowed(prev, next);
		next += TIME_US2I(500);
	}
}
