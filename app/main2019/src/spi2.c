/*
 * spi2.c
 *
 *  Created on: 03.02.2019
 *      Author: AndreR
 */

#include "spi2.h"
#include "util/init_hal.h"
#include "util/sys_time.h"
#include "util/log.h"
#include "kicker.h"
#include "constants.h"
#include <string.h>

SPI2Global spi2 __attribute__((aligned(1024), section(".sram1")));

void DMA1_Stream2_IRQHandler()
{
	CH_IRQ_PROLOGUE();

	SPISyncDmaRxInterrupt(&spi2.bus);

	CH_IRQ_EPILOGUE();
}

void SPI2Init()
{
	RCC->APB1LENR |= RCC_APB1LENR_SPI2EN;
	__DSB();

	// Data in SRAM1 is not initialized
	memset(&spi2, 0x00, sizeof(SPI2Global));

	DMAMUX1_Channel2->CCR = 39; // spi2_rx_dma
	DMAMUX1_Channel3->CCR = 40; // spi2_tx_dma

	SPISyncData spiData;
	spiData.dmaChRx = 2;
	spiData.dmaChTx = 3;
	spiData.dmaPrio = 1;
	spiData.pDma = DMA1;
	spiData.pRegister = SPI2;
	SPISyncInit(&spi2.bus, &spiData);

	spi2.kicker.slave.cpol = 1;
	spi2.kicker.slave.cpha = 1;
	spi2.kicker.slave.csPin = GPIO_PIN_12;
	spi2.kicker.slave.pCSPort = GPIOG;
	spi2.kicker.slave.timeoutTicks = MS2ST(20);
	spi2.kicker.slave.prescaler = SPI_CFG1_BRDIV32;
	SPISyncSlaveInit(&spi2.bus, &spi2.kicker.slave);

	// init GPIO
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.alternate = 5;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_25MHZ;
	gpioInit.pupd = GPIO_PUPD_UP;
	GPIOInit(GPIOB, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, &gpioInit); // SCK, MISO, MOSI

	NVICEnableIRQ(DMA1_Stream2_IRQn, IRQL_SPI2);

	// setup kicker ADC tx data
	for(uint16_t i = 0; i < SPI2_KICK_SAMPLES; i++)
	{
		spi2.kicker.tx[i*2] = 0 << 3;
	}
	for(uint16_t i = SPI2_KICK_SAMPLES; i < 2*SPI2_KICK_SAMPLES; i++)
	{
		spi2.kicker.tx[i*2] = 1 << 3;
	}
}

static void kickUpdate()
{
	SPISyncTransfer(&spi2.kicker.slave, spi2.kicker.tx, (uint8_t*)spi2.kicker.rx, 4*SPI2_KICK_SAMPLES);

	uint32_t ch1Sum = 0;
	uint32_t ch2Sum = 0;
	uint8_t validCh1Samples = 0;
	uint8_t validCh2Samples = 0;

	// sum up measurements, we ignore the first two measurements
	// the sampling capacitor has to settle in that time
	for(uint16_t i = 2; i < SPI2_KICK_SAMPLES; i++)
	{
		uint16_t sample = __REV16(spi2.kicker.rx[i]);
		if(sample < 4096)
		{
			ch1Sum += sample;
			++validCh1Samples;
		}
	}
	for(uint16_t i = SPI2_KICK_SAMPLES+2; i < 2*SPI2_KICK_SAMPLES; i++)
	{
		uint16_t sample = __REV16(spi2.kicker.rx[i]);
		if(sample < 4096)
		{
			ch2Sum += sample;
			++validCh2Samples;
		}
	}

	// we need at least 4 valid samples for a reasonable measurement
	if(validCh1Samples > 3 && validCh2Samples > 3)
	{
		float ch1 = ch1Sum/(validCh1Samples*4096.0f);
		float ch2 = ch2Sum/(validCh2Samples*4096.0f);

		KickerADCUpdate(ch2, ch1);
	}
	else
	{
		LogWarnC("Not enough ADC samples.", validCh1Samples | ((uint16_t)validCh2Samples) << 8);
	}
}

void SPI2Task(void* params)
{
	(void)params;

	chRegSetThreadName("SPI2");

	systime_t prev = chVTGetSystemTimeX();
	systime_t next = prev+US2ST(1000);

	while(1)
	{
		uint32_t start = SysTimeUSec();
		kickUpdate();
		uint32_t end = SysTimeUSec();
		spi2.kicker.sampleTime = end-start;

		chThdSleepUntilWindowed(prev, next);
		prev = next;
		next += US2ST(1000);
	}
}
