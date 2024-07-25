#include "spi1.h"
#include "ch.h"

SPILLD spi1;

static uint8_t spi1Tx[512] __attribute__((aligned(4), section(".nocache")));
static uint8_t spi1Rx[512] __attribute__((aligned(4), section(".nocache")));

CH_FAST_IRQ_HANDLER(Vector120) // DMA2_STR0, SPI1 RX
{
	SPILLDDmaRxInterrupt(&spi1);
}

void SPI1Init(uint32_t irqLevel)
{
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	__DSB();

	DMAMUX1_Channel8->CCR = 37; // spi1_rx_dma => DMA2-CH0
	DMAMUX1_Channel9->CCR = 38; // spi1_tx_dma => DMA2-CH1

	SPILLDData spiInit;
	spiInit.pRegister = SPI1;
	spiInit.pDma = DMA2;
	spiInit.dmaChTx = 1;
	spiInit.dmaChRx = 0;
	spiInit.dmaPrio = DMA_PL_VERY_HIGH;
	spiInit.pDmaBufTx = spi1Tx;
	spiInit.pDmaBufRx = spi1Rx;
	spiInit.dmaBufSize = sizeof(spi1Tx);
	SPILLDInit(&spi1, &spiInit);

	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.alternate = 5;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_25MHZ;
	gpioInit.pupd = GPIO_PUPD_UP;
	GPIOInit(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, &gpioInit); // SCK, MISO, MOSI

	NVICEnableIRQ(DMA2_Stream0_IRQn, irqLevel);
}
