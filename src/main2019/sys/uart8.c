#include "uart8.h"

UARTFifo uart8;

CH_IRQ_HANDLER(Vector18C) // UART8
{
	CH_IRQ_PROLOGUE();
	UARTFifoIRQ(&uart8);
	CH_IRQ_EPILOGUE();
}

void UART8Init(uint32_t irqLevel)
{
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_25MHZ;
	gpioInit.pupd = GPIO_PUPD_UP;
	gpioInit.alternate = 8;
	GPIOInit(GPIOE, GPIO_PIN_0 | GPIO_PIN_1, &gpioInit);

	RCC->APB1LENR |= RCC_APB1LENR_UART8EN; __DSB();

	UARTFifoInit(&uart8, UART8, 2000000, systemClockInfo.APB1PeriphClk, 1);

	NVICEnableIRQ(UART8_IRQn, irqLevel);
}
