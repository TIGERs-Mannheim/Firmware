/*
 * usart2.c
 *
 *  Created on: 01.01.2019
 *      Author: AndreR
 */

#include "util/init_hal.h"
#include "util/bits.h"
#include "usart2.h"

USART2Data usart2;

#define ClearTXEI()	BITS_CLEAR(USART2->CR1, USART_CR1_TXEIE)
#define SetTXEI()	BITS_SET(USART2->CR1, USART_CR1_TXEIE)
#define ClearRXI()	BITS_CLEAR(USART2->CR1, USART_CR1_RXNEIE)
#define SetRXI()	BITS_SET(USART2->CR1, USART_CR1_RXNEIE)

void USART2_IRQHandler()
{
	CH_IRQ_PROLOGUE();

	uint32_t sr = USART2->ISR;

	if(sr & USART_ISR_TXE)	// enabled by TXEIE
	{
		// Transmitter empty, cleared by write to DR
		chSysLockFromISR();
		uint32_t data = chOQGetI(&usart2.outQueue);
		chSysUnlockFromISR();

		if(data == Q_EMPTY)
			ClearTXEI();
		else
			USART2->TDR = data;
	}

	if(sr & USART_ISR_RXNE)	// enabled by RXNEIE
	{
		// read data register not empty, cleared by: read DR
		uint8_t data = USART2->RDR;

		chSysLockFromISR();
		chIQPutI(&usart2.inQueue, data);
		chSysUnlockFromISR();
	}

	CH_IRQ_EPILOGUE();
}

static void txReadyCb(output_queue_t* pOut)
{
	(void)pOut;

	SetTXEI();
}

void USART2Init()
{
	chIQObjectInit(&usart2.inQueue, usart2.rxData, USART2_RX_SIZE, 0, 0);
	chOQObjectInit(&usart2.outQueue, usart2.txData, USART2_TX_SIZE, &txReadyCb, 0);

	// init port
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.alternate = 7;
	gpioInit.pupd = GPIO_PUPD_UP;
	GPIOInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3, &gpioInit);

	// USART3
	RCC->APB1LENR |= RCC_APB1LENR_USART2EN;
	__DSB();

	USART2->CR1 = USART_CR1_TE | USART_CR1_RE;
	USART2->CR2 = USART_CR2_SWAP;
	USART2->CR3 = 0;
	USART2->BRR = USARTCalcBRR(921600, systemClockInfo.APB1PeriphClk, 0);

	NVICEnableIRQ(USART2_IRQn, IRQL_USART2);

	USART2->CR1 |= USART_CR1_UE;

	SetRXI();
}
