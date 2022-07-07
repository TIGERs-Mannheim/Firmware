/*
 * usart1.c
 *
 *  Created on: 15.01.2019
 *      Author: AndreR
 */

#include "system_init.h"

static volatile uint8_t txBusy = 0;

void USART1_IRQHandler()
{
	USART1->CR1 &= ~USART_CR1_TCIE;
	DMA1_Channel2->CCR &= ~DMA_CCR_EN;
	txBusy = 0;
}

void USART1Init()
{
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.alternate = 1;
	gpioInit.ospeed = GPIO_OSPEED_10MHZ;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.pupd = GPIO_PUPD_UP;
	GPIOInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10, &gpioInit);

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	// 8,E,1 configuration
	USART1->CR1 = USART_CR1_M | USART_CR1_PCE | USART_CR1_TE | USART_CR1_RE;
	USART1->CR2 = 0;
	USART1->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;
	USART1->BRR = 24; // => 2Mbaud/s, approx. 181kB/s with 8,E,1
	USART1->CR1 |= USART_CR1_UE;

	// USART1_TX, medium priority, memory increment, memory to peripheral
	DMA1_Channel2->CCR = DMA_CCR_PL_0 | DMA_CCR_MINC | DMA_CCR_DIR;
	DMA1_Channel2->CMAR = 0;
	DMA1_Channel2->CPAR = (uint32_t)&USART1->TDR;
	DMA1_Channel2->CNDTR = 0;

	// USART1_RX, low priority, memory increment, peripheral to memory
	DMA1_Channel3->CCR = DMA_CCR_MINC;
	DMA1_Channel3->CMAR = 0;
	DMA1_Channel3->CPAR = (uint32_t)&USART1->RDR;
	DMA1_Channel3->CNDTR = 0;

	DMA1->IFCR = DMA_IFCR_CGIF2 | DMA_IFCR_CGIF3;

	NVIC_SetPriority(USART1_IRQn, 10);
	NVIC_EnableIRQ(USART1_IRQn);
}

void USART1Transmit(const void* pData, uint16_t length)
{
	if(txBusy)
		return;

	txBusy = 1;

	DMA1->IFCR = DMA_IFCR_CGIF2;
	DMA1_Channel2->CMAR = (uint32_t)pData;
	DMA1_Channel2->CNDTR = length;

	USART1->ICR = USART_ICR_TCCF;
	USART1->CR1 |= USART_CR1_TCIE;

	DMA1_Channel2->CCR |= DMA_CCR_EN;
}
