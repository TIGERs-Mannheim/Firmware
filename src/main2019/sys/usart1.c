#include "usart1.h"

UARTFifo usart1;

CH_IRQ_HANDLER(VectorD4) // USART1, Motor 1 (FR)
{
	CH_IRQ_PROLOGUE();
	UARTFifoIRQ(&usart1);
	CH_IRQ_EPILOGUE();
}

void USART1Init(uint32_t irqLevel)
{
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_25MHZ;
	gpioInit.pupd = GPIO_PUPD_UP;
	gpioInit.alternate = 7;
	GPIOInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10, &gpioInit);

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN; __DSB();

	UARTFifoInit(&usart1, USART1, 2000000, systemClockInfo.APB2PeriphClk, 1);

	NVICEnableIRQ(USART1_IRQn, irqLevel);
}
