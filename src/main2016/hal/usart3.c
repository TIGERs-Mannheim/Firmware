/*
 * usart3.c
 *
 *  Created on: 24.10.2015
 *      Author: AndreR
 */

#include "usart3.h"

#include "util/init_hal.h"
#include "util/bits.h"

USART3Data usart3;

#define ClearTXEI()	BITS_CLEAR(USART3->CR1, USART_CR1_TXEIE)
#define SetTXEI()	BITS_SET(USART3->CR1, USART_CR1_TXEIE)
#define ClearRXI()	BITS_CLEAR(USART3->CR1, USART_CR1_RXNEIE)
#define SetRXI()	BITS_SET(USART3->CR1, USART_CR1_RXNEIE)

void USART3_IRQHandler()
{
	CH_IRQ_PROLOGUE();

	uint32_t sr = USART3->ISR;

	if(sr & USART_ISR_TXE)	// enabled by TXEIE
	{
		// Transmitter empty, cleared by write to DR
		chSysLockFromISR();
		uint32_t data = chOQGetI(&usart3.outQueue);
		chSysUnlockFromISR();

		if(data == Q_EMPTY)
			ClearTXEI();
		else
			USART3->TDR = data;
	}

	if(sr & USART_ISR_RXNE)	// enabled by RXNEIE
	{
		// read data register not empty, cleared by: read DR
		uint8_t data = USART3->RDR;

		chSysLockFromISR();
		chIQPutI(&usart3.inQueue, data);
		chSysUnlockFromISR();
	}

	CH_IRQ_EPILOGUE();
}

static void txReadyCb(output_queue_t* pOut)
{
	(void)pOut;

	SetTXEI();
}

void USART3Init()
{
	chIQObjectInit(&usart3.inQueue, usart3.rxData, USART3_RX_SIZE, 0, 0);
	chOQObjectInit(&usart3.outQueue, usart3.txData, USART3_TX_SIZE, &txReadyCb, 0);

	// init port
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.alternate = 7;
	gpioInit.pupd = GPIO_PUPD_UP;
	GPIOInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_11, &gpioInit);

	// USART3
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

	USART3->CR1 = USART_CR1_TE | USART_CR1_RE;
	USART3->CR2 = 0;
	USART3->CR3 = 0;
	USART3->BRR = USARTCalcBRR(921600, systemClockInfo.APB1PeriphClk, 0);

	NVICEnableIRQ(USART3_IRQn, IRQL_USART3);

	USART3->CR1 |= USART_CR1_UE;

	SetRXI();
}
