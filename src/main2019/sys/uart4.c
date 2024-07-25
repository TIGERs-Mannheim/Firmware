#include "uart4.h"

UARTFifo uart4;

CH_IRQ_HANDLER(Vector110) // UART4, Motor 3 (RL)
{
	CH_IRQ_PROLOGUE();
	UARTFifoIRQ(&uart4);
	CH_IRQ_EPILOGUE();
}

void UART4Init(uint32_t irqLevel)
{
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_25MHZ;
	gpioInit.pupd = GPIO_PUPD_UP;
	gpioInit.alternate = 8;
	GPIOInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1, &gpioInit);

	RCC->APB1LENR |= RCC_APB1LENR_UART4EN; __DSB();

	UARTFifoInit(&uart4,  UART4, 2000000, systemClockInfo.APB1PeriphClk, 1);

	NVICEnableIRQ(UART4_IRQn, irqLevel);
}
