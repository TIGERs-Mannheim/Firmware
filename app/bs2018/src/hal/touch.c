/*
 * touch.c
 *
 *  Created on: 24.10.2017
 *      Author: AndreR
 */

#include "gfx.h"
#include "src/ginput/ginput_driver_mouse.h"
#include "util/console.h"
#include <string.h>
#include "touch.h"
#include "../constants.h"
#include "util/flash_fs.h"

Touch touch __attribute__((aligned(16), section(".dtc")));

void DMA1_Stream3_IRQHandler()
{
	CH_IRQ_PROLOGUE();

	SPISyncDmaRxInterrupt(&touch.bus);

	CH_IRQ_EPILOGUE();
}

void TouchInit()
{
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

	SPISyncData spiData;
	spiData.dmaChRx = 3;
	spiData.dmaChTx = 4;
	spiData.dmaPrio = 1;
	spiData.pDma = DMA1;
	spiData.pRegister = SPI2;
	SPISyncInit(&touch.bus, &spiData);

	AD7843Init(&touch.sensor, &touch.bus, (GPIOPin){GPIOC, GPIO_PIN_6}, (GPIOPin){GPIOD, GPIO_PIN_12});

	// init GPIO
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.alternate = 5;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_25MHZ;
	gpioInit.pupd = GPIO_PUPD_DOWN;
	GPIOInit(GPIOB, GPIO_PIN_14 | GPIO_PIN_15, &gpioInit); // MISO, MOSI
	GPIOInit(GPIOA, GPIO_PIN_12, &gpioInit); // SCK

	// configure NVIC
	NVICEnableIRQ(DMA1_Stream3_IRQn, IRQL_TOUCH);
}

void TouchUpdate()
{
	AD7843Update(&touch.sensor);
}

void TouchTask(void* params)
{
	(void)params;

	chRegSetThreadName("Touch");

	while(1)
	{
		TouchUpdate();

		chThdSleepMilliseconds(5);
	}
}

static bool_t mouseGetXYZ(GMouse* m, GMouseReading* pdr)
{
	(void)m;

	// No buttons
	pdr->buttons = 0;
	pdr->z = 0;

	if(touch.sensor.pressed)
	{
		pdr->z = 1;
		pdr->x = touch.sensor.x;
		pdr->y = touch.sensor.y;
	}

	return TRUE;
}

static void calSave(GMouse *m, const void *buf, size_t sz)
{
	(void)m;
	(void)sz;

	float* pCalib = (float*)buf;

	ConsolePrint("%d %d\n", sz, sizeof(float));
	for(uint8_t i = 0; i < 6; i++)
		ConsolePrint("%.8f\r\n", pCalib[i]);

	FlashFile* calib;
	if(FlashFSOpenOrCreate("touch/calib", 1, pCalib, 24, &calib) == FLASH_FS_NOMEM)
		ConsolePrint("No memory to save calibration data!\n");
}

static bool_t calLoad(GMouse *m, void *buf, size_t sz)
{
	(void)m;

	FlashFile* calib = FlashFSOpen("touch/calib", 1);
	if(!calib)
	{
		return FALSE;
	}
	else
	{
		float calibData[6];
		FlashFSRead(calib, calibData);
		memcpy(buf, calibData, sz);
		return TRUE;
	}
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
		GMOUSE_VFLG_TOUCH | GMOUSE_VFLG_CALIBRATE | GMOUSE_VFLG_ONLY_DOWN | GMOUSE_VFLG_POORUPDOWN | GMOUSE_VFLG_CAL_EXTREMES,
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
