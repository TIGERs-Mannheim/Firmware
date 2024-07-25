/*
 * Low-Level SPI HAL.
 *
 * Does not use any OS functions.
 */
#pragma once

#include "hal/init_hal.h"

typedef void(*SPIDoneCallback)(void*);

typedef struct _SPILLDData
{
	SPI_TypeDef* pRegister;

	DMA_TypeDef* pDma;
	uint16_t dmaChTx;
	uint16_t dmaChRx;
	uint32_t dmaPrio;

	uint8_t* pDmaBufTx; // must be in non-cachable memory
	uint8_t* pDmaBufRx; // must be in non-cachable memory
	uint32_t dmaBufSize;
} SPILLDData;

typedef struct _SPILLD
{
	SPILLDData data;

	uint32_t prescaler;
	uint32_t cpol;
	uint32_t cpha;

	SPIDoneCallback doneCallback; // called from IRQ level
	void* pCallbackData;

	DMA_Stream_TypeDef* pDmaTx;
	DMA_Stream_TypeDef* pDmaRx;

	volatile uint32_t* pDMARxIFCR;
	volatile uint32_t* pDMATxIFCR;
	uint32_t IFCRRxMask;
	uint32_t IFCRTxMask;

	volatile GPIOPin activeCsPin;
	volatile uint32_t transferCompletionTime_us;
} SPILLD;

void	SPILLDDmaRxInterrupt(SPILLD* pSPI);
void	SPILLDInit(SPILLD* pSPI, SPILLDData* pInit);
void	SPILLDConfigureCsPin(GPIOPin csPin);
int16_t	SPILLDReconfigure(SPILLD* pSPI, uint32_t prescaler, uint32_t cpol, uint32_t cpha);
int16_t SPILLDStartTransferSelect(SPILLD* pSPI, GPIOPin csPin, uint32_t size);
void	SPILLDStartTransferExec(SPILLD* pSPI);
int16_t	SPILLDStartTransfer(SPILLD* pSPI, GPIOPin csPin, uint32_t size);
void	SPILLDAbortTransfer(SPILLD* pSPI);
uint8_t	SPILLDIsTransferActive(SPILLD* pSPI);

static inline void* SPILLDGetTxBuf(SPILLD* pSPI) { return pSPI->data.pDmaBufTx; }
static inline void* SPILLDGetRxBuf(SPILLD* pSPI) { return pSPI->data.pDmaBufRx; }
