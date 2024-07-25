#include "uart7.h"

UARTFifo uart7;

CH_IRQ_HANDLER(Vector188) // UART7, Motor 2 (FL)
{
	CH_IRQ_PROLOGUE();
	UARTFifoIRQ(&uart7);
	CH_IRQ_EPILOGUE();
}

void UART7Init(uint32_t irqLevel)
{
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_25MHZ;
	gpioInit.pupd = GPIO_PUPD_UP;
	gpioInit.alternate = 7;
	GPIOInit(GPIOF, GPIO_PIN_6 | GPIO_PIN_7, &gpioInit);

	RCC->APB1LENR |= RCC_APB1LENR_UART7EN; __DSB();

	UARTFifoInit(&uart7,  UART7, 2000000, systemClockInfo.APB1PeriphClk, 1);

	NVICEnableIRQ(UART7_IRQn, irqLevel);
}
