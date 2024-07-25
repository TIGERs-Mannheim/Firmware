#include "spi2.h"

SPI spi2;

static SPILLD spi2Driver;
static uint8_t spi2Tx[64] __attribute__((aligned(4), section(".nocache")));
static uint8_t spi2Rx[64] __attribute__((aligned(4), section(".nocache")));

CH_IRQ_HANDLER(Vector74) // DMA1_STR2, SPI2 RX
{
	CH_IRQ_PROLOGUE();
	SPILLDDmaRxInterrupt(&spi2Driver);
	CH_IRQ_EPILOGUE();
}

void SPI2Init(uint32_t irqLevel)
{
	RCC->APB1LENR |= RCC_APB1LENR_SPI2EN;
	__DSB();

	DMAMUX1_Channel2->CCR = 39; // spi2_rx_dma
	DMAMUX1_Channel3->CCR = 40; // spi2_tx_dma

	SPILLDData spiInit;
	spiInit.pRegister = SPI2;
	spiInit.pDma = DMA1;
	spiInit.dmaChTx = 3;
	spiInit.dmaChRx = 2;
	spiInit.dmaPrio = DMA_PL_MEDIUM;
	spiInit.pDmaBufTx = spi2Tx;
	spiInit.pDmaBufRx = spi2Rx;
	spiInit.dmaBufSize = sizeof(spi2Tx);
	SPILLDInit(&spi2Driver, &spiInit);
	SPIInit(&spi2, &spi2Driver);

	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.alternate = 5;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_25MHZ;
	gpioInit.pupd = GPIO_PUPD_UP;
	GPIOInit(GPIOB, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, &gpioInit); // SCK, MISO, MOSI

	NVICEnableIRQ(DMA1_Stream2_IRQn, irqLevel);
}
