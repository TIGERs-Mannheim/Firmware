/*
 * usart3.c
 *
 *  Created on: 24.10.2015
 *      Author: AndreR
 */

#include "util/init_hal.h"
#include "util/bits.h"
#include "usart1.h"

USART1Data usart1;

#define ClearTXEI()	BITS_CLEAR(USART1->CR1, USART_CR1_TXEIE)
#define SetTXEI()	BITS_SET(USART1->CR1, USART_CR1_TXEIE)
#define ClearRXI()	BITS_CLEAR(USART1->CR1, USART_CR1_RXNEIE)
#define SetRXI()	BITS_SET(USART1->CR1, USART_CR1_RXNEIE)

void USART1_IRQHandler()
{
	CH_IRQ_PROLOGUE();

	uint32_t sr = USART1->ISR;

	if(sr & USART_ISR_TXE)	// enabled by TXEIE
	{
		// Transmitter empty, cleared by write to DR
		chSysLockFromISR();
		uint32_t data = chOQGetI(&usart1.outQueue);
		chSysUnlockFromISR();

		if(data == Q_EMPTY)
			ClearTXEI();
		else
			USART1->TDR = data;
	}

	if(sr & USART_ISR_RXNE)	// enabled by RXNEIE
	{
		// read data register not empty, cleared by: read DR
		uint8_t data = USART1->RDR;

		chSysLockFromISR();
		chIQPutI(&usart1.inQueue, data);
		chSysUnlockFromISR();
	}

	CH_IRQ_EPILOGUE();
}

static void txReadyCb(output_queue_t* pOut)
{
	(void)pOut;

	SetTXEI();
}

void USART1Init()
{
	chIQObjectInit(&usart1.inQueue, usart1.rxData, USART1_RX_SIZE, 0, 0);
	chOQObjectInit(&usart1.outQueue, usart1.txData, USART1_TX_SIZE, &txReadyCb, 0);

	// init port
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.alternate = 7;
	gpioInit.pupd = GPIO_PUPD_UP;
	GPIOInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, &gpioInit);

	// USART1
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	USART1->CR1 = USART_CR1_TE | USART_CR1_RE;
	USART1->CR2 = 0;
	USART1->CR3 = 0;
	USART1->BRR = USARTCalcBRR(921600, systemClockInfo.APB2PeriphClk, 0);

	NVICEnableIRQ(USART1_IRQn, IRQL_USART1);

	USART1->CR1 |= USART_CR1_UE;

	SetRXI();
}
