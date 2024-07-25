#include "spi4.h"

SPI spi4;

static SPILLD spi4Driver;
static uint8_t spi4Tx[128] __attribute__((aligned(4), section(".nocache")));
static uint8_t spi4Rx[128] __attribute__((aligned(4), section(".nocache")));

CH_IRQ_HANDLER(Vector6C) // DMA1_STR0, SPI4 RX
{
	CH_IRQ_PROLOGUE();
	SPILLDDmaRxInterrupt(&spi4Driver);
	CH_IRQ_EPILOGUE();
}

void SPI4Init(uint32_t irqLevel)
{
	RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;
	__DSB();

	DMAMUX1_Channel0->CCR = 83; // spi4_rx_dma
	DMAMUX1_Channel1->CCR = 84; // spi4_tx_dma

	SPILLDData spiInit;
	spiInit.pRegister = SPI4;
	spiInit.pDma = DMA1;
	spiInit.dmaChTx = 1;
	spiInit.dmaChRx = 0;
	spiInit.dmaPrio = DMA_PL_MEDIUM;
	spiInit.pDmaBufTx = spi4Tx;
	spiInit.pDmaBufRx = spi4Rx;
	spiInit.dmaBufSize = sizeof(spi4Tx);
	SPILLDInit(&spi4Driver, &spiInit);
	SPIInit(&spi4, &spi4Driver);

	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.alternate = 5;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_25MHZ;
	gpioInit.pupd = GPIO_PUPD_DOWN;
	GPIOInit(GPIOE, GPIO_PIN_2 | GPIO_PIN_5 | GPIO_PIN_6, &gpioInit); // SCK, MISO, MOSI

	NVICEnableIRQ(DMA1_Stream0_IRQn, irqLevel);
}
