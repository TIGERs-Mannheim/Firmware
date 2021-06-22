/*
 * usart.c
 *
 *  Created on: 04.11.2012
 *      Author: Andre
 */

#include "util/usart.h"

#include "util/bits.h"
#include "util/log.h"
#include "util/init_hal.h"
#include "util/cobs.h"
#include "errors.h"

#include <string.h>

// uses DMA_StreamTypeDef
#if defined(STM32F4XX) || defined(STM32F7XX) || defined(STM32H7XX)
#define DMAClearEn(pDMA)	BITS_CLEAR(pDMA->CR, DMA_SxCR_EN)
#define DMASetEn(pDMA)		BITS_SET(pDMA->CR, DMA_SxCR_EN)
#define DMAGetEn(pDMA)		(pDMA->CR & DMA_SxCR_EN)
#else
#define DMAClearEn(pDMA)	BITS_CLEAR(pDMA->CCR, DMA_CCR_EN)
#define DMASetEn(pDMA)		BITS_SET(pDMA->CCR, DMA_CCR_EN)
#define DMAGetEn(pDMA)		(pDMA->CCR & DMA_CCR_EN)
#endif

// uses USART_TypeDef
#define USARTClearEIE(pRegister)	BITS_CLEAR(pRegister->CR3, USART_CR3_EIE)
#define USARTSetEIE(pRegister)		BITS_SET(pRegister->CR3, USART_CR3_EIE)
#define USARTGetEIE(pRegister)		(pRegister->CR3 & USART_CR3_EIE)

#define USARTClearTCIE(pRegister)	BITS_CLEAR(pRegister->CR1, USART_CR1_TCIE)
#define USARTSetTCIE(pRegister)		BITS_SET(pRegister->CR1, USART_CR1_TCIE)
#define USARTGetTCIE(pRegister)		(pRegister->CR1, USART_CR1_TCIE)

#define USARTClearTE(pRegister)		BITS_CLEAR(pRegister->CR1, USART_CR1_TE)
#define USARTSetTE(pRegister)		BITS_SET(pRegister->CR1, USART_CR1_TE)

#define USARTClearRE(pRegister)		BITS_CLEAR(pRegister->CR1, USART_CR1_RE)
#define USARTSetRE(pRegister)		BITS_SET(pRegister->CR1, USART_CR1_RE)

#define EVENT_TX_START			0x0001
#define EVENT_RX_ERROR			0x0002

void USARTIRQ(USART* pUSART)
{
	CH_IRQ_PROLOGUE();

#ifdef STM32F4XX
	uint16_t sr = pUSART->pRegister->SR;

	if((sr & USART_SR_TC) && USARTGetTCIE(pUSART->pRegister))	// transfer complete
#else
	uint32_t sr = pUSART->pRegister->ISR;

	if((sr & USART_ISR_TC) && USARTGetTCIE(pUSART->pRegister))	// transfer complete
#endif
	{
		USARTClearTCIE(pUSART->pRegister);	// disable TC IRQ on USART
		DMAClearEn(pUSART->pTxStream);	// disable DMA
		while(DMAGetEn(pUSART->pTxStream));	// wait until stream is disabled

		chSysLockFromISR();
		if(chMBPostI(&pUSART->eventQueue, EVENT_TX_START) == MSG_TIMEOUT)
			LogWarn("Post EVENT_TX_DONE timed out");
		chSysUnlockFromISR();
	}

#ifdef STM32F4XX
	if((sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) && USARTGetEIE(pUSART->pRegister))
#else
	if((sr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE)) && USARTGetEIE(pUSART->pRegister))
#endif
	{
		// clear interrupt enable
		USARTClearEIE(pUSART->pRegister);

		chSysLockFromISR();
		if(chMBPostI(&pUSART->eventQueue, EVENT_RX_ERROR) == MSG_TIMEOUT)
			LogWarn("Post EVENT_RX_* timed out");
		chSysUnlockFromISR();
	}

	CH_IRQ_EPILOGUE();
}

#ifdef STM32F30X
/**
 * pDMA: DMA1 or DMA2
 * stream: 0 - 7
 */
static __inline DMA_Channel_TypeDef* DMAGenerateStreamTypeDef(DMA_TypeDef* pDMA, uint32_t stream)
{
	return (DMA_Channel_TypeDef*)( ((uint32_t*)pDMA) +  5*(stream-1) + 2);
}
#endif

void USARTInit(USART* pUSART, USART_TypeDef* pRegister, USARTData data)
{
	pUSART->data = data;
	pUSART->pRegister = pRegister;
	pUSART->callback = 0;
	pUSART->rxProcBufUsed = 0;

	// Init action queue
	chMBObjectInit(&pUSART->eventQueue, pUSART->eventQueueData, 10);

	pUSART->rxDmaBufSize = data.baudrate/10000;
	if(pUSART->rxDmaBufSize < 10)
		pUSART->rxDmaBufSize = 10;
	pUSART->rxProcBufSize = COBSMaxStuffedSize(data.maxPacketSize);
	uint32_t rxFifoSize = data.rxSize - pUSART->rxDmaBufSize - pUSART->rxProcBufSize;

	pUSART->pRxDMABuf = data.pDataRx;
	pUSART->pRxProcBuf = data.pDataRx + pUSART->rxDmaBufSize;
	uint8_t* pFifoRxData = data.pDataRx + pUSART->rxDmaBufSize + pUSART->rxProcBufSize;

	pUSART->txDmaBufSize = COBSMaxStuffedSize(data.maxPacketSize)+1;
	pUSART->pTxDmaBuf = data.pDataTx;
	uint8_t* pFifoTxData = data.pDataTx + pUSART->txDmaBufSize;
	uint32_t txFifoSize = data.txSize - pUSART->txDmaBufSize;

	// Fifo init
	FifoLinInit(&pUSART->rxFifo, pFifoRxData, rxFifoSize);
	FifoLinInit(&pUSART->txFifo, pFifoTxData, txFifoSize);

	// find interrupt flag clear register and generate mask
	pUSART->pRxStream = DMAGenerateStreamTypeDef(data.pDMA, data.rxStream);
	pUSART->pTxStream = DMAGenerateStreamTypeDef(data.pDMA, data.txStream);

#if defined(STM32F4XX) || defined(STM32F7XX) || defined(STM32H7XX)
	pUSART->pDMARxIFCR = DMAGenerateIFCR(data.pDMA, data.rxStream);
	pUSART->pDMATxIFCR = DMAGenerateIFCR(data.pDMA, data.txStream);
	pUSART->pDMARxISR = DMAGenerateISR(data.pDMA, data.rxStream);
	pUSART->pDMATxISR = DMAGenerateISR(data.pDMA, data.txStream);
	pUSART->IFCRRxMask = DMAGenerateMaskAll(data.rxStream);
	pUSART->IFCRTxMask = DMAGenerateMaskAll(data.txStream);

	// DMA TX setup
	pUSART->pTxStream->CR = DMA_CHSEL(data.dmaChannel) | DMA_DIR_MEM2PERIPH | DMA_SxCR_MINC |
			DMA_PSIZE_1BYTE | DMA_MSIZE_1BYTE | DMA_PL_MEDIUM | DMA_MBURST_SINGLE | DMA_PBURST_SINGLE;
	pUSART->pTxStream->FCR = DMA_SxFCR_DMDIS | DMA_FTH_FULL;
#ifdef STM32F4XX
	pUSART->pTxStream->PAR = (uint32_t)&pRegister->DR;
#else
	pUSART->pTxStream->PAR = (uint32_t)&pRegister->TDR;
#endif
	pUSART->pTxStream->NDTR = 0;
	pUSART->pTxStream->M0AR = 0;

	// DMA RX setup
	pUSART->pRxStream->CR = DMA_CHSEL(data.dmaChannel) | DMA_DIR_PERIPH2MEM | DMA_SxCR_MINC |
			DMA_PSIZE_1BYTE | DMA_MSIZE_1BYTE | DMA_PL_HIGH | DMA_MBURST_SINGLE | DMA_PBURST_SINGLE | DMA_SxCR_CIRC;
	pUSART->pRxStream->FCR = 0;
#ifdef STM32F4XX
	pUSART->pRxStream->PAR = (uint32_t)&pRegister->DR;
#else
	pUSART->pRxStream->PAR = (uint32_t)&pRegister->RDR;
#endif
	pUSART->pRxStream->NDTR = pUSART->rxDmaBufSize;
	pUSART->pRxStream->M0AR = (uint32_t)pUSART->pRxDMABuf;
#else
	pUSART->pDMARxIFCR = &data.pDMA->IFCR;
	pUSART->pDMATxIFCR = &data.pDMA->IFCR;
	pUSART->pDMARxISR = &data.pDMA->ISR;
	pUSART->pDMATxISR = &data.pDMA->ISR;
	pUSART->IFCRRxMask = 0xF << (data.rxStream-1)*4;
	pUSART->IFCRTxMask = 0xF << (data.rxStream-1)*4;

	// DMA TX setup
	pUSART->pTxStream->CCR = DMA_PL_MEDIUM | DMA_MSIZE_1BYTE | DMA_PSIZE_1BYTE | DMA_DIR_MEM2PERIPH	| DMA_CCR_MINC;
	pUSART->pTxStream->CPAR = (uint32_t)&pRegister->TDR;

	// DMA RX setup
	pUSART->pRxStream->CCR = DMA_PL_HIGH | DMA_MSIZE_1BYTE | DMA_PSIZE_1BYTE | DMA_DIR_PERIPH2MEM | DMA_CCR_MINC | DMA_CCR_CIRC;
	pUSART->pRxStream->CPAR = (uint32_t)&pRegister->RDR;
	pUSART->pRxStream->CMAR = (uint32_t)pUSART->pRxDMABuf;
	pUSART->pRxStream->CNDTR = pUSART->rxDmaBufSize;
#endif

	// USART setup
	pRegister->CR1 = USART_CR1_OVER8;
	pRegister->CR2 = 0;
	pRegister->CR3 = USART_CR3_ONEBIT;
	pRegister->BRR = USARTCalcBRR(data.baudrate, data.peripheralClock, 1);
#ifdef STM32F4XX
	pRegister->SR = 0; // clear all flags
#else
	pRegister->ICR = 0xFFFFFFFF; // clear all flags
#endif

	USARTClearEIE(pRegister);
	USARTClearTCIE(pRegister);

	// NVIC setup
	NVICEnableIRQ(data.usartIRQn, data.IRQL);

	// clear DMA flags
	*pUSART->pDMARxIFCR = pUSART->IFCRRxMask;
	*pUSART->pDMATxIFCR = pUSART->IFCRTxMask;

	// finally enable USART
	pRegister->CR1 |= USART_CR1_UE | USART_CR1_TE;

	// enable DMA for TX and RX
	pRegister->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR;

	// enable RX DMA
	DMASetEn(pUSART->pRxStream);
	USARTSetEIE(pUSART->pRegister);
	USARTSetRE(pUSART->pRegister);
}

int16_t USARTSend(USART* pUSART, uint16_t length, uint8_t* pData)
{
	uint8_t* pDst;
	int16_t result;

	result = FifoLinReserve(&pUSART->txFifo, length, &pDst);
	if(result)
		return result;

	memcpy(pDst, pData, length);

	result = FifoLinCommit(&pUSART->txFifo, length);
	if(result)
		return result;

	if(chMBPost(&pUSART->eventQueue, EVENT_TX_START, MS2ST(500)) != MSG_OK)
		return ERROR_USART_QUEUE_FULL;

	return 0;
}

int16_t USARTStartTransfer(USART* pUSART)
{
	if(DMAGetEn(pUSART->pTxStream)) // already transmitting?
		return 0;

	if(chMBPost(&pUSART->eventQueue, EVENT_TX_START, MS2ST(500)) != MSG_OK)
		return ERROR_USART_QUEUE_FULL;

	return 0;
}

void USARTTask(void* params)
{
	msg_t event;
	int32_t osResult;
	USART* pUSART = (USART*)params;

	chRegSetThreadName("USART");

	uint16_t lastWrPos = 0;
	uint32_t last1sTime = 0;

	while(42)
	{
		osResult = chMBFetch(&pUSART->eventQueue, &event, MS2ST(1));
		if(osResult == MSG_OK)
		{
			LogDebugC("Received event", event);
			switch(event)
			{
				case EVENT_TX_START:
				{
					if(DMAGetEn(pUSART->pTxStream) == 0)
					{
						uint8_t* pTxData;
						uint32_t txSize;
						if(FifoLinGet(&pUSART->txFifo, &pTxData, &txSize) == 0)
						{
							pUSART->liveStats.txPayload += txSize;

							uint32_t encodedSize;
							if(COBSEncode(pTxData, txSize, pUSART->pTxDmaBuf, pUSART->txDmaBufSize-1, &encodedSize) == 0)
							{
								pUSART->liveStats.txWire += encodedSize+1;

								pUSART->pTxDmaBuf[encodedSize] = 0;

#if defined(STM32F4XX) || defined(STM32F7XX) || defined(STM32H7XX)
								pUSART->pTxStream->M0AR = (uint32_t)pUSART->pTxDmaBuf;
								pUSART->pTxStream->NDTR = (uint16_t)(encodedSize+1);
								*pUSART->pDMATxIFCR = pUSART->IFCRTxMask;	// clear flags
#ifdef STM32F4XX
								pUSART->pRegister->SR &= ~USART_SR_TC;	// clear TC flag in USART
#else
								pUSART->pRegister->ICR = USART_ICR_TCCF;
#endif
#else
								pUSART->pTxStream->CMAR = (uint32_t)pUSART->pTxDmaBuf;
								pUSART->pTxStream->CNDTR = (uint16_t)(encodedSize+1);
								*pUSART->pDMATxIFCR = pUSART->IFCRTxMask;	// clear flags
								pUSART->pRegister->ICR = USART_ICR_TCCF;
#endif
								DMASetEn(pUSART->pTxStream);	// enable DMA
								USARTSetTCIE(pUSART->pRegister);
							}
							else
							{
								LogWarn("COBS Error");
							}

							FifoLinDelete(&pUSART->txFifo);
						}
					}
				}
				break;
				case EVENT_RX_ERROR:
				{
					DMAClearEn(pUSART->pRxStream);	// disable DMA
					while(DMAGetEn(pUSART->pRxStream));	// wait until stream is disabled
					USARTClearRE(pUSART->pRegister);	// disable receiver

					pUSART->rxProcBufUsed = 0;

					LogWarn("RX_ERROR");

					// clear flags
#ifdef STM32F4XX
					pUSART->pRegister->SR &= ~(USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_RXNE);
					pUSART->pRegister->DR;
					pUSART->pRegister->DR;
#else
					pUSART->pRegister->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NCF;
					pUSART->pRegister->RDR;
					pUSART->pRegister->RDR;
#endif
					*pUSART->pDMARxIFCR = pUSART->IFCRRxMask;

					DMASetEn(pUSART->pRxStream);
					USARTSetEIE(pUSART->pRegister);
					USARTSetRE(pUSART->pRegister);
				}
				break;
			}
		}

#if defined(STM32F4XX) || defined(STM32F7XX) || defined(STM32H7XX)
		uint16_t wrPos = pUSART->rxDmaBufSize - pUSART->pRxStream->NDTR;
#else
		uint16_t wrPos = pUSART->rxDmaBufSize - pUSART->pRxStream->CNDTR;
#endif

		uint8_t* pRead = &pUSART->pRxDMABuf[lastWrPos];

		uint16_t rxSize;
		if(wrPos >= lastWrPos)
			rxSize = wrPos-lastWrPos;
		else
			rxSize = pUSART->rxDmaBufSize - lastWrPos + wrPos;

		lastWrPos = (lastWrPos + rxSize)%pUSART->rxDmaBufSize;

		pUSART->liveStats.rxWire += rxSize;

		for(uint16_t i = 0; i < rxSize; i++)
		{
			uint8_t c = *pRead;

			if(c == 0)
			{
				// packet complete
				uint8_t* pDst;
				if(FifoLinReserve(&pUSART->rxFifo, pUSART->data.maxPacketSize, &pDst) == 0)
				{
					uint32_t decodedSize;
					if(COBSDecode(pUSART->pRxProcBuf, pUSART->rxProcBufUsed, pDst, pUSART->data.maxPacketSize, &decodedSize) == 0)
					{
						FifoLinCommit(&pUSART->rxFifo, decodedSize);

						pUSART->liveStats.rxPayload += decodedSize;

						if(pUSART->callback)
							(*pUSART->callback)(pUSART->pUserData);
					}
					else
					{
						LogWarn("COBS decode error");
					}
				}
				else
				{
					LogWarnC("Not enough memory", pUSART->rxProcBufUsed);
				}

				pUSART->rxProcBufUsed = 0;
			}
			else
			{
				if(pUSART->rxProcBufUsed == pUSART->rxProcBufSize)
				{
					// we filled the whole buffer without delimiter => start again
					pUSART->rxProcBufUsed = 0;
					LogWarn("rx proc buf overrun");
				}

				pUSART->pRxProcBuf[pUSART->rxProcBufUsed++] = c;
			}

			++pRead;
			if(pRead == &pUSART->pRxDMABuf[pUSART->rxDmaBufSize])
				pRead = pUSART->pRxDMABuf;
		}

		if(chVTTimeElapsedSinceX(last1sTime) > S2ST(1))
		{
			last1sTime = chVTGetSystemTimeX();

			memcpy(&pUSART->stats, &pUSART->liveStats, sizeof(USARTStats));
			memset(&pUSART->liveStats, 0, sizeof(USARTStats));
			pUSART->stats.rxUsage = ((float)pUSART->stats.rxWire)/(pUSART->data.baudrate*0.1f);
			pUSART->stats.txUsage = ((float)pUSART->stats.txWire)/(pUSART->data.baudrate*0.1f);
		}
	}
}
