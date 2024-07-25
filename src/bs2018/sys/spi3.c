#include "spi3.h"
#include "ch.h"

SPILLD spi3;

static uint8_t spi3Tx[512] __attribute__((aligned(4), section(".nocache")));
static uint8_t spi3Rx[512] __attribute__((aligned(4), section(".nocache")));

CH_FAST_IRQ_HANDLER(Vector6C) // DMA1_Stream0, SPI3 RX
{
	SPILLDDmaRxInterrupt(&spi3);
}

void SPI3Init(uint32_t irqLevel)
{
	RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;

	SPILLDData spiInit;
	spiInit.pRegister = SPI3;
	spiInit.pDma = DMA1;
	spiInit.dmaChTx = 7;
	spiInit.dmaChRx = 0;
	spiInit.dmaPrio = DMA_PL_VERY_HIGH;
	spiInit.pDmaBufTx = spi3Tx;
	spiInit.pDmaBufRx = spi3Rx;
	spiInit.dmaBufSize = sizeof(spi3Tx);
	SPILLDInit(&spi3, &spiInit);

	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.alternate = 6;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_25MHZ;
	gpioInit.pupd = GPIO_PUPD_DOWN;
	GPIOInit(GPIOC, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12, &gpioInit);  // SCK, MISO, MOSI

	NVICEnableIRQ(DMA1_Stream0_IRQn, irqLevel);
}
