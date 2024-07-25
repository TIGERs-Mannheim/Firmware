#include "uart5.h"

UARTFifo uart5;

CH_IRQ_HANDLER(Vector114) // UART5, Motor 5 (Dribbler)
{
	CH_IRQ_PROLOGUE();
	UARTFifoIRQ(&uart5);
	CH_IRQ_EPILOGUE();
}

void UART5Init(uint32_t irqLevel)
{
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_25MHZ;
	gpioInit.pupd = GPIO_PUPD_UP;
	gpioInit.alternate = 14;
	GPIOInit(GPIOB, GPIO_PIN_5 | GPIO_PIN_6, &gpioInit);

	RCC->APB1LENR |= RCC_APB1LENR_UART5EN; __DSB();

	UARTFifoInit(&uart5,  UART5, 2000000, systemClockInfo.APB1PeriphClk, 1);

	NVICEnableIRQ(UART5_IRQn, irqLevel);
}
