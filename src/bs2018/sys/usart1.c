#include "usart1.h"

UartDma usart1;

static uint8_t usart1Tx[4096] __attribute__((aligned(4), section(".nocache")));
static uint8_t usart1Rx[4096] __attribute__((aligned(4), section(".nocache")));

CH_IRQ_HANDLER(VectorD4) // USART1
{
	CH_IRQ_PROLOGUE();
	UartDmaIrq(&usart1);
	CH_IRQ_EPILOGUE();
}

CH_IRQ_HANDLER(Vector158) // DMA2_STR7 (USART1 TX)
{
	CH_IRQ_PROLOGUE();
	UartDmaTxDmaIrq(&usart1);
	CH_IRQ_EPILOGUE();
}

void USART1Init(uint32_t irqLevel, tprio_t taskPrio)
{
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.alternate = 7;
	gpioInit.pupd = GPIO_PUPD_UP;
	GPIOInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, &gpioInit);

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	UartDmaData uartInit;
	uartInit.pRegister = USART1;
	uartInit.pDma = DMA2;
	uartInit.dmaStreamTx = 7;
	uartInit.dmaStreamRx = 5;
	uartInit.dmaPrio = DMA_PL_LOW;
	uartInit.pDmaBufTx = usart1Tx;
	uartInit.pDmaBufRx = usart1Rx;
	uartInit.dmaBufSizeTx = sizeof(usart1Tx);
	uartInit.dmaBufSizeRx = sizeof(usart1Rx);
	uartInit.baudrate = 921600;
	uartInit.perClk = systemClockInfo.APB2PeriphClk;
	uartInit.swapRxTx = 0;
	uartInit.pTaskName = "USART1";
	UartDmaInit(&usart1, &uartInit, taskPrio);

	NVICEnableIRQ(USART1_IRQn, irqLevel);
	NVICEnableIRQ(DMA2_Stream7_IRQn, irqLevel);
}
