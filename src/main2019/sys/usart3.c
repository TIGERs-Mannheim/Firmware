#include "usart3.h"

UARTFifo usart3;

CH_IRQ_HANDLER(VectorDC) // USART3, Motor 4 (RR)
{
	CH_IRQ_PROLOGUE();
	UARTFifoIRQ(&usart3);
	CH_IRQ_EPILOGUE();
}

void USART3Init(uint32_t irqLevel)
{
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_25MHZ;
	gpioInit.pupd = GPIO_PUPD_UP;
	gpioInit.alternate = 7;
	GPIOInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, &gpioInit);

	RCC->APB1LENR |= RCC_APB1LENR_USART3EN; __DSB();

	UARTFifoInit(&usart3, USART3, 2000000, systemClockInfo.APB1PeriphClk, 1);

	NVICEnableIRQ(USART3_IRQn, irqLevel);
}
