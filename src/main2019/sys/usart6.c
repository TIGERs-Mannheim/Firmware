#include "usart6.h"

UartDma usart6;

static uint8_t usart6Tx[2048] __attribute__((aligned(4), section(".nocache")));
static uint8_t usart6Rx[2048] __attribute__((aligned(4), section(".nocache")));

CH_IRQ_HANDLER(Vector15C) // USART6
{
	CH_IRQ_PROLOGUE();
	UartDmaIrq(&usart6);
	CH_IRQ_EPILOGUE();
}

CH_IRQ_HANDLER(VectorFC) // DMA1_STR7 (USART6 TX)
{
	CH_IRQ_PROLOGUE();
	UartDmaTxDmaIrq(&usart6);
	CH_IRQ_EPILOGUE();
}

void USART6Init(uint32_t irqLevel, tprio_t taskPrio)
{
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_25MHZ;
	gpioInit.pupd = GPIO_PUPD_UP;
	gpioInit.alternate = 7;
	GPIOInit(GPIOG, GPIO_PIN_9 | GPIO_PIN_14, &gpioInit);
	RCC->APB2ENR |= RCC_APB2ENR_USART6EN; __DSB();

	DMAMUX1_Channel6->CCR = 71; // usart6_rx_dma
	DMAMUX1_Channel7->CCR = 72; // usart6_tx_dma

	UartDmaData uartInit;
	uartInit.pRegister = USART6;
	uartInit.pDma = DMA1;
	uartInit.dmaStreamTx = 7;
	uartInit.dmaStreamRx = 6;
	uartInit.dmaPrio = DMA_PL_MEDIUM;
	uartInit.pDmaBufTx = usart6Tx;
	uartInit.pDmaBufRx = usart6Rx;
	uartInit.dmaBufSizeTx = sizeof(usart6Tx);
	uartInit.dmaBufSizeRx = sizeof(usart6Rx);
	uartInit.baudrate = 2500000;
	uartInit.perClk = systemClockInfo.APB2PeriphClk;
	uartInit.swapRxTx = 0;
	uartInit.pTaskName = "USART6";
	UartDmaInit(&usart6, &uartInit, taskPrio);

	NVICEnableIRQ(USART6_IRQn, irqLevel);
	NVICEnableIRQ(DMA1_Stream7_IRQn, irqLevel);
}
