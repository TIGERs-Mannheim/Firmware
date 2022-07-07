/*
 * spi_sync.h
 *
 *  Created on: 28.09.2013
 *      Author: AndreR
 */

#pragma once

#include "ch.h"

#ifndef STM32F30X

typedef void(*SPILowDoneCallback)(void*);

typedef struct _SPILowData
{
	SPI_TypeDef* pRegister;

	DMA_TypeDef* pDma;
	uint16_t dmaChTx;
	uint16_t dmaChRx;
	uint32_t dmaPrio;

	GPIO_TypeDef* pCSPort;
	uint16_t csPin;

	uint32_t prescaler;
	uint32_t cpol;
	uint32_t cpha;
} SPILowData;

typedef struct _SPILow
{
	SPILowData data;

	DMA_Stream_TypeDef* pDmaTx;
	DMA_Stream_TypeDef* pDmaRx;

	volatile uint32_t* pDMARxIFCR;
	volatile uint32_t* pDMATxIFCR;
	uint32_t IFCRRxMask;
	uint32_t IFCRTxMask;

	binary_semaphore_t doneSem;
} SPILow;

void	SPILowDmaRxInterrupt(SPILow* pSPI);
void	SPILowInit(SPILow* pSPI, SPILowData* pInit);
void	SPILowTransfer(SPILow* pSPI, const uint8_t* pSrc, uint8_t* pDst, uint32_t size);

#endif
