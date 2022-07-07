/*
 * spi_sync.h
 *
 *  Created on: 28.09.2013
 *      Author: AndreR
 */

#pragma once

#include "ch.h"

typedef struct _SPISyncData
{
	SPI_TypeDef* pRegister;

	DMA_TypeDef* pDma;
	uint16_t dmaChTx;
	uint16_t dmaChRx;
	uint32_t dmaPrio;
} SPISyncData;

typedef struct _SPISync
{
	SPISyncData data;

	uint8_t dummy;

	DMA_Stream_TypeDef* pDmaTx;
	DMA_Stream_TypeDef* pDmaRx;

	volatile uint32_t* pDMARxIFCR;
	volatile uint32_t* pDMATxIFCR;
	volatile uint32_t* pDMARxISR;
	volatile uint32_t* pDMATxISR;
	uint32_t IFCRRxMask;
	uint32_t IFCRTxMask;

	binary_semaphore_t transferSem;
	mutex_t taskMutex;
} SPISync;

typedef struct _SPISyncSlave
{
	SPISync* pBus;

	GPIO_TypeDef* pCSPort;
	uint16_t csPin;

	uint32_t prescaler;
	uint32_t cpol;
	uint32_t cpha;

	uint32_t timeoutTicks;
} SPISyncSlave;

void	SPISyncDmaRxInterrupt(SPISync* pSPI);
void	SPISyncInit(SPISync* pSPI, SPISyncData* pInit);
void	SPISyncSlaveInit(SPISync* pSPI, SPISyncSlave* pSlave);

void	SPISyncStartTransfer(SPISyncSlave* pSlave, uint8_t* pSrc, uint8_t* pDst, uint32_t size);
int16_t SPISyncWait(SPISyncSlave* pSlave);
int16_t SPISyncTransfer(SPISyncSlave* pSlave, uint8_t* pSrc, uint8_t* pDst, uint32_t size);
