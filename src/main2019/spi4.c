/*
 * spi4.c
 *
 *  Created on: 06.01.2019
 *      Author: AndreR
 */

#include "spi4.h"
#include "util/init_hal.h"
#include "util/sys_time.h"
#include "util/console.h"
#include "util/log.h"
#include "constants.h"
#include "gfx.h"
#include "src/ginput/ginput_driver_mouse.h"
#include <string.h>

#define SPI4_ACC_UNIT_TO_MS2 (0.00239501953125f)
#define SPI4_GYR_UNIT_TO_RADS (0.0010652644360317f)
#define SPI4_MAG_UNIT_TO_UT (0.01220703125f)

SPI4Global spi4 __attribute__((aligned(1024), section(".sram1")));

void DMA1_Stream0_IRQHandler()
{
	CH_IRQ_PROLOGUE();

	SPISyncDmaRxInterrupt(&spi4.bus);

	CH_IRQ_EPILOGUE();
}

#define ICM20689_READ	0x80
#define ICM20689_WRITE	0x00

#define ICM20689_CONFIG				26
#define ICM20689_GYRO_CONFIG		27
#define ICM20689_ACCEL_CONFIG		28
#define ICM20689_ACCEL_CONFIG2		29
#define ICM20689_FIFO_ENABLE		35
#define ICM20689_USER_CONTROL		106
#define ICM20689_PWR_MGMT_1			107
#define ICM20689_PWR_MGMT_2			108
#define ICM20689_SIGNAL_PATH_RESET	104
#define ICM20689_FIFO_COUNT			114
#define ICM20689_FIFO_READ			116
#define ICM20689_WHOAMI				117

static void imuInit()
{
	chThdSleepMilliseconds(100);

	spi4.imu.tx[0] = ICM20689_PWR_MGMT_1 | ICM20689_WRITE;
	spi4.imu.tx[1] = 0x80; // DEVICE_RESET
	SPISyncTransfer(&spi4.imu.slave, spi4.imu.tx, 0, 2);
	chThdSleepMilliseconds(100);

	spi4.imu.tx[0] = ICM20689_USER_CONTROL | ICM20689_WRITE;
	spi4.imu.tx[1] = 0x11;	// reset all digital signal paths, disable I2C interface
	SPISyncTransfer(&spi4.imu.slave, spi4.imu.tx, 0, 2);
	chThdSleepMilliseconds(100);

	spi4.imu.tx[0] = ICM20689_PWR_MGMT_1 | ICM20689_WRITE;
	spi4.imu.tx[1] = 0x01; // auto-select best clock source, PLL if available
	SPISyncTransfer(&spi4.imu.slave, spi4.imu.tx, 0, 2);
	chThdSleepMilliseconds(100);

	spi4.imu.tx[0] = ICM20689_GYRO_CONFIG | ICM20689_WRITE;
	spi4.imu.tx[1] = 0x18; // +-2000dps full scale
	SPISyncTransfer(&spi4.imu.slave, spi4.imu.tx, 0, 2);
	chThdSleepMilliseconds(5);

	spi4.imu.tx[0] = ICM20689_ACCEL_CONFIG | ICM20689_WRITE;
	spi4.imu.tx[1] = 0x10; // +-8g full scale
	SPISyncTransfer(&spi4.imu.slave, spi4.imu.tx, 0, 2);
	chThdSleepMilliseconds(5);

	spi4.imu.tx[0] = ICM20689_ACCEL_CONFIG2 | ICM20689_WRITE;
//	spi4.imu.tx[1] = 0x08; // Acc BW 1046Hz, 4kHz rate
	spi4.imu.tx[1] = 0x07; // Acc BW 420Hz, 1kHz rate
	SPISyncTransfer(&spi4.imu.slave, spi4.imu.tx, 0, 2);
	chThdSleepMilliseconds(5);

	spi4.imu.tx[0] = ICM20689_CONFIG | ICM20689_WRITE;
//	spi4.imu.tx[1] = 0x07; // Gyr BW 3281Hz, 8kHz rate, Temp BW 4000Hz
	spi4.imu.tx[1] = 0x01; // Gyr BW 176Hz, 1kHz rate, Temp BW 188Hz
	SPISyncTransfer(&spi4.imu.slave, spi4.imu.tx, 0, 2);
	chThdSleepMilliseconds(5);
}

static void imuUpdate()
{
	spi4.imu.tx[0] = 58 | ICM20689_READ; // start at INTERRUPT_STATUS
	SPISyncTransfer(&spi4.imu.slave, spi4.imu.tx, spi4.imu.rx, 2);

	// check if there is new data ready
	if((spi4.imu.rx[1] & 0x01) == 0)
		return;

	++spi4.imu.numUpdates;

	spi4.imu.tx[0] = 59 | ICM20689_READ; // start at ACCEL_XOUT_H
	SPISyncTransfer(&spi4.imu.slave, spi4.imu.tx, spi4.imu.rx, 15);

	int16_t acc[3];
	acc[0] = (int16_t)__REV16(*((uint16_t*)&spi4.imu.rx[1]));
	acc[1] = (int16_t)__REV16(*((uint16_t*)&spi4.imu.rx[3]));
	acc[2] = (int16_t)__REV16(*((uint16_t*)&spi4.imu.rx[5]));

	int16_t temp;
	temp = (int16_t)__REV16(*((uint16_t*)&spi4.imu.rx[7]));

	int16_t gyr[3];
	gyr[0] = (int16_t)__REV16(*((uint16_t*)&spi4.imu.rx[9]));
	gyr[1] = (int16_t)__REV16(*((uint16_t*)&spi4.imu.rx[11]));
	gyr[2] = (int16_t)__REV16(*((uint16_t*)&spi4.imu.rx[13]));

	for(uint8_t i = 0; i < 3; i++)
	{
		spi4.imu.gyr[i] = gyr[i]*SPI4_GYR_UNIT_TO_RADS;
		spi4.imu.acc[i] = acc[i]*SPI4_ACC_UNIT_TO_MS2;
	}

	// conversion according to datasheet
	spi4.imu.temp = temp * (1.0f/326.8f) + 25.0f;
	spi4.imu.updated = 1;
}

#define LIS3_READ 0x80
#define LIS3_AUTO_INC 0x40

static void magInit()
{
	chThdSleepMilliseconds(20);

	// start at CTRL_REG1
	spi4.mag.tx[0] = 0x20 | LIS3_AUTO_INC;
	spi4.mag.tx[1] = 0b11111110; // 155Hz output datarate, UHP mode on XY
	spi4.mag.tx[2] = 0; // +-4 gauss full-scale
	spi4.mag.tx[3] = 0; // continuous conversion
	spi4.mag.tx[4] = 0b00001100; // UHP mode on Z
	spi4.mag.tx[5] = 0x40; // block data update
	SPISyncTransfer(&spi4.mag.slave, spi4.mag.tx, 0, 6);
	chThdSleepMilliseconds(20);
}

static void magUpdate()
{
	spi4.mag.tx[0] = 0x27 | LIS3_AUTO_INC | LIS3_READ;
	SPISyncTransfer(&spi4.mag.slave, spi4.mag.tx, spi4.mag.rx, 8);

	// check if there is new data ready
	if((spi4.mag.rx[1] & 0x07) == 0)
		return;

	++spi4.mag.numUpdates;

	int16_t strength[3];
	strength[0] = *((int16_t*)&spi4.mag.rx[2]);
	strength[1] = *((int16_t*)&spi4.mag.rx[4]);
	strength[2] = *((int16_t*)&spi4.mag.rx[6]);

	// strangely, the above auto increment does not pass on beyond OUT registers
	// an extra read is required for temperature
	spi4.mag.tx[0] = 0x2E | LIS3_AUTO_INC | LIS3_READ;
	SPISyncTransfer(&spi4.mag.slave, spi4.mag.tx, spi4.mag.rx, 3);

	int16_t temp = *((int16_t*)&spi4.mag.rx[1]);

	for(uint8_t i = 0; i < 3; i++)
	{
		spi4.mag.strength[i] = strength[i]*SPI4_MAG_UNIT_TO_UT;
	}

	spi4.mag.temp = temp*0.125f + 25.0f;
	spi4.mag.updated = 1;
}

void SPI4Init()
{
	RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;
	__DSB();

	// Data in SRAM1 is not initialized
	memset(&spi4, 0x00, sizeof(SPI4Global));

	DMAMUX1_Channel0->CCR = 83; // spi4_rx_dma
	DMAMUX1_Channel1->CCR = 84; // spi4_tx_dma

	SPISyncData spiData;
	spiData.dmaChRx = 0;
	spiData.dmaChTx = 1;
	spiData.dmaPrio = 1;
	spiData.pDma = DMA1;
	spiData.pRegister = SPI4;
	SPISyncInit(&spi4.bus, &spiData);

	AD7843Init(&spi4.touch, &spi4.bus, (GPIOPin){GPIOG, GPIO_PIN_0}, (GPIOPin){GPIOA, GPIO_PIN_4});

	spi4.imu.slave.cpol = 0;
	spi4.imu.slave.cpha = 0;
	spi4.imu.slave.csPin = GPIO_PIN_7;
	spi4.imu.slave.pCSPort = GPIOB;
	spi4.imu.slave.timeoutTicks = MS2ST(20);
	spi4.imu.slave.prescaler = SPI_CFG1_BRDIV16;
	SPISyncSlaveInit(&spi4.bus, &spi4.imu.slave);

	spi4.mag.slave.cpol = 0;
	spi4.mag.slave.cpha = 0;
	spi4.mag.slave.csPin = GPIO_PIN_13;
	spi4.mag.slave.pCSPort = GPIOC;
	spi4.mag.slave.timeoutTicks = MS2ST(20);
	spi4.mag.slave.prescaler = SPI_CFG1_BRDIV16;
	SPISyncSlaveInit(&spi4.bus, &spi4.mag.slave);

	// init GPIO
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.alternate = 5;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_25MHZ;
	gpioInit.pupd = GPIO_PUPD_DOWN;
	GPIOInit(GPIOE, GPIO_PIN_2 | GPIO_PIN_5 | GPIO_PIN_6, &gpioInit); // SCK, MISO, MOSI

	NVICEnableIRQ(DMA1_Stream0_IRQn, IRQL_SPI4);
}

void SPI4Task(void* params)
{
	(void)params;

	uint32_t start;
	uint32_t end;

	chRegSetThreadName("SPI4");

	imuInit();
	magInit();

	systime_t prev = chVTGetSystemTimeX();
	systime_t next = prev+US2ST(10000);

	uint32_t lastRateUpdate = SysTimeUSec();

	while(1)
	{
		AD7843Update(&spi4.touch);

		start = SysTimeUSec();
		imuUpdate();
		end = SysTimeUSec();
		spi4.imu.sampleTime = end-start;

		start = SysTimeUSec();
		magUpdate();
		end = SysTimeUSec();
		spi4.mag.sampleTime = end-start;

		if(SysTimeUSec() - lastRateUpdate > 1000000)
		{
			lastRateUpdate += 1000000;

			spi4.imu.updateRate = (float)spi4.imu.numUpdates;
			spi4.imu.numUpdates = 0;

			spi4.mag.updateRate = (float)spi4.mag.numUpdates;
			spi4.mag.numUpdates = 0;
		}

		chThdSleepUntilWindowed(prev, next);
		prev = next;
		next += US2ST(1000);
	}
}


static bool_t mouseGetXYZ(GMouse* m, GMouseReading* pdr)
{
	(void)m;

	// No buttons
	pdr->buttons = 0;
	pdr->z = 0;

	if(spi4.touch.pressed)
	{
		pdr->z = 1;
		pdr->x = spi4.touch.x;
		pdr->y = spi4.touch.y;
	}

	return TRUE;
}

static void calSave(GMouse *m, const void *buf, size_t sz)
{
	(void)m;
	(void)sz;

	float* pCalib = (float*)buf;

	for(uint8_t i = 0; i < 6; i++)
		ConsolePrint("%.8f\r\n", pCalib[i]);
}

static const float calib[] = {
	0.00114166f, 0.06780271f, -19.17998314f,
	0.09104055f, 0.00010331f, -15.59686184f,
};

static bool_t calLoad(GMouse *m, void *buf, size_t sz)
{
	(void)m;

	if(sz != sizeof(calib))
		return FALSE;

	memcpy(buf, calib, sz);

	return TRUE;
}

static bool_t mouseInit(GMouse* m, unsigned driverinstance)
{
	(void)m;
	(void)driverinstance;

	return TRUE;
}

// Resolution and Accuracy Settings
#define GMOUSE_ADS7843_PEN_CALIBRATE_ERROR		8
#define GMOUSE_ADS7843_PEN_CLICK_ERROR			6
#define GMOUSE_ADS7843_PEN_MOVE_ERROR			4
#define GMOUSE_ADS7843_FINGER_CALIBRATE_ERROR	14
#define GMOUSE_ADS7843_FINGER_CLICK_ERROR		18
#define GMOUSE_ADS7843_FINGER_MOVE_ERROR		14

const GMouseVMT const GMOUSE_DRIVER_VMT[1] = {{
	{
		GDRIVER_TYPE_TOUCH,
		GMOUSE_VFLG_TOUCH | GMOUSE_VFLG_CALIBRATE | /*GMOUSE_VFLG_CAL_TEST |*/ GMOUSE_VFLG_ONLY_DOWN | /*GMOUSE_VFLG_POORUPDOWN |*/ GMOUSE_VFLG_CAL_EXTREMES,
		sizeof(GMouse),
		_gmouseInitDriver,
		_gmousePostInitDriver,
		_gmouseDeInitDriver
	},
	1,				// z_max - (currently?) not supported
	0,				// z_min - (currently?) not supported
	1,				// z_touchon
	0,				// z_touchoff
	{				// pen_jitter
		GMOUSE_ADS7843_PEN_CALIBRATE_ERROR,			// calibrate
		GMOUSE_ADS7843_PEN_CLICK_ERROR,				// click
		GMOUSE_ADS7843_PEN_MOVE_ERROR				// move
	},
	{				// finger_jitter
		GMOUSE_ADS7843_FINGER_CALIBRATE_ERROR,		// calibrate
		GMOUSE_ADS7843_FINGER_CLICK_ERROR,			// click
		GMOUSE_ADS7843_FINGER_MOVE_ERROR			// move
	},
	mouseInit,	 	// init
	0,				// deinit
	mouseGetXYZ,	// get
	calSave,		// calsave
	calLoad			// calload
}};
