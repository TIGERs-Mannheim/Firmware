/*
 * spi4.c
 *
 *  Created on: 31.10.2015
 *      Author: AndreR
 */

#include "spi4.h"

#include "gfx.h"
#include "src/ginput/ginput_driver_mouse.h"
#include "util/init_hal.h"
#include "util/console.h"
#include "util/sys_time.h"
#include "util/log_file.h"
#include "hal/kicker.h"
#include "struct_ids.h"
#include "constants.h"
#include <string.h>

#define ADC_SHUNT_TO_CURRENT 0.00322265625f

SPI4Global spi4 __attribute__((aligned(1024), section(".dtc")));

static uint8_t identifyBoard(GPIOPin pin);

void DMA2_Stream0_IRQHandler()
{
	CH_IRQ_PROLOGUE();

	SPISyncDmaRxInterrupt(&spi4.bus);

	CH_IRQ_EPILOGUE();
}

void SPI4Init()
{
	RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;

	SPISyncData spiData;
	spiData.dmaChRx = 0;
	spiData.dmaChTx = 1;
	spiData.dmaPrio = 1;
	spiData.pDma = DMA2;
	spiData.pRegister = SPI4;
	SPISyncInit(&spi4.bus, &spiData);

	AD7843Init(&spi4.touch, &spi4.bus, (GPIOPin){GPIOG, GPIO_PIN_4}, (GPIOPin){GPIOF, GPIO_PIN_12});

	spi4.leftBoardType = identifyBoard((GPIOPin){GPIOF, GPIO_PIN_2});
	spi4.rightBoardType = identifyBoard((GPIOPin){GPIOF, GPIO_PIN_14});

	if(spi4.leftBoardType == 1)
		spi4.motCurrentSenseInstalled = 1;
	else
		spi4.motCurrentSenseInstalled = 0;

	spi4.motors.slave.cpha = 0;
	spi4.motors.slave.cpol = 0;
	spi4.motors.slave.csPin = GPIO_PIN_2;
	spi4.motors.slave.pCSPort = GPIOF;
	spi4.motors.slave.prescaler = SPI_CR1_BRDIV8;
	spi4.motors.slave.timeoutTicks = MS2ST(20);

	SPISyncSlaveInit(&spi4.bus, &spi4.motors.slave);

	// init GPIO
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.alternate = 5;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_25MHZ;
	gpioInit.pupd = GPIO_PUPD_DOWN;
	GPIOInit(GPIOE, GPIO_PIN_2 | GPIO_PIN_5 | GPIO_PIN_6, &gpioInit); // SCK, MISO, MOSI

	// configure NVIC
	NVICEnableIRQ(DMA2_Stream0_IRQn, IRQL_SPI6);

	// setup Motor ADC tx data
	for(uint16_t ch = 0; ch < 4; ch++)
	{
		for(uint16_t i = ch*SPI4_MOTOR_ADC_SAMPLES; i < (ch+1)*SPI4_MOTOR_ADC_SAMPLES; i++)
		{
			spi4.motors.tx[i*2] = ch << 3;
			spi4.motors.tx[i*2+1] = 0;
		}

		spi4.motors.biasCurrent[ch] = 2048.0f * ADC_SHUNT_TO_CURRENT;
	}
}

static void motorsUpdate()
{
	SPISyncTransfer(&spi4.motors.slave, spi4.motors.tx, (uint8_t*)spi4.motors.rx, SPI4_MOTOR_ADC_SAMPLES*4*2);

	spi4.motors.measTimestamp = SysTimeUSec();

	for(uint16_t ch = 0; ch < 4; ch++)
	{
		uint32_t sum = 0;
		uint8_t validSamples = 0;

		// sum up measurements, we ignore the first two measurements
		// the sampling capacitor has to settle in that time
		for(uint16_t i = ch*SPI4_MOTOR_ADC_SAMPLES; i < (ch+1)*SPI4_MOTOR_ADC_SAMPLES; i++)
		{
			uint16_t sample = __REV16(spi4.motors.rx[i]);
			if(sample < 4096)
			{
				sum += sample;
				++validSamples;
			}
		}

		if(validSamples > 3)
			spi4.motors.raw[ch] = (((float)sum)/((float)validSamples))*ADC_SHUNT_TO_CURRENT;

		spi4.motors.current[ch] = spi4.motors.raw[ch] - spi4.motors.biasCurrent[ch];
	}
}

void SPI4CurrentBiasCalibration()
{
	float bias[4];
	for(uint16_t runs = 0; runs < 16; runs++)
	{
		motorsUpdate();
		for(uint8_t ch = 0; ch < 4; ch++)
		{
			bias[ch] += spi4.motors.raw[ch];
		}
		chThdSleepMilliseconds(2);
	}

	for(uint8_t ch = 0; ch < 4; ch++)
	{
		spi4.motors.biasCurrent[ch] = bias[ch]/16.0f;
	}
}

void SPI4Task(void* params)
{
	(void)params;

	uint32_t start;
	uint32_t end;

	chRegSetThreadName("SPI4");

	SPI4CurrentBiasCalibration();

	systime_t prev = chVTGetSystemTimeX();
	systime_t next = prev+US2ST(10000);

	while(1)
	{
		if(spi4.leftBoardType)
		{
			start = SysTimeUSec();
			motorsUpdate();
			end = SysTimeUSec();
			spi4.motors.sampleTime = end-start;
		}

		AD7843Update(&spi4.touch);

		chThdSleepUntilWindowed(prev, next);
		prev = next;
		next += US2ST(1000);
	}
}

static uint8_t identifyBoard(GPIOPin pin)
{
	GPIOInitData gpioInit;
	uint8_t hasPullUp = 0;
	uint8_t hasPullDown = 0;

	// enable weak pull-down
	gpioInit.mode = GPIO_MODE_INPUT;
	gpioInit.pupd = GPIO_PUPD_DOWN;
	GPIOInit(pin.pPort, pin.pin, &gpioInit);

	chThdSleepMilliseconds(10);
	if(pin.pPort->IDR & pin.pin)	// is the input high?
	{
		hasPullUp = 1;
	}

	// enable weak pull-up
	gpioInit.mode = GPIO_MODE_INPUT;
	gpioInit.pupd = GPIO_PUPD_UP;
	GPIOInit(pin.pPort, pin.pin, &gpioInit);

	chThdSleepMilliseconds(10);
	if((pin.pPort->IDR & pin.pin) == 0)	// is the input low?
	{
		hasPullDown = 2;
	}

	return hasPullUp + hasPullDown;
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
