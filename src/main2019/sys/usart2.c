#include "usart2.h"

UartDma usart2;

static uint8_t usart2Tx[4096] __attribute__((aligned(4), section(".nocache")));
static uint8_t usart2Rx[4096] __attribute__((aligned(4), section(".nocache")));

CH_IRQ_HANDLER(VectorD8) // USART2
{
	CH_IRQ_PROLOGUE();
	UartDmaIrq(&usart2);
	CH_IRQ_EPILOGUE();
}

CH_IRQ_HANDLER(Vector12C) // DMA2_STR3 (USART2 TX)
{
	CH_IRQ_PROLOGUE();
	UartDmaTxDmaIrq(&usart2);
	CH_IRQ_EPILOGUE();
}

void USART2Init(uint32_t irqLevel, tprio_t taskPrio)
{
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_25MHZ;
	gpioInit.pupd = GPIO_PUPD_UP;
	gpioInit.alternate = 7;
	GPIOInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3, &gpioInit);
	RCC->APB1LENR |= RCC_APB1LENR_USART2EN; __DSB();

	DMAMUX1_Channel10->CCR = 43; // usart2_rx_dma => DMA2-CH2
	DMAMUX1_Channel11->CCR = 44; // usart2_tx_dma => DMA2-CH3

	UartDmaData uartInit;
	uartInit.pRegister = USART2;
	uartInit.pDma = DMA2;
	uartInit.dmaStreamTx = 3;
	uartInit.dmaStreamRx = 2;
	uartInit.dmaPrio = DMA_PL_LOW;
	uartInit.pDmaBufTx = usart2Tx;
	uartInit.pDmaBufRx = usart2Rx;
	uartInit.dmaBufSizeTx = sizeof(usart2Tx);
	uartInit.dmaBufSizeRx = sizeof(usart2Rx);
	uartInit.baudrate = 921600;
	uartInit.perClk = systemClockInfo.APB1PeriphClk;
	uartInit.swapRxTx = 1;
	uartInit.pTaskName = "USART2";
	UartDmaInit(&usart2, &uartInit, taskPrio);

	NVICEnableIRQ(USART2_IRQn, irqLevel);
	NVICEnableIRQ(DMA2_Stream3_IRQn, irqLevel);
}
