#pragma once

#include "hal/init_hal.h"
#include "util/fifo.h"
#include "ch.h"

#define UART_DMA_EVENT_DATA_RECEIVED	EVENT_MASK(0)
#define UART_DMA_EVENT_DATA_SENT		EVENT_MASK(1)
#define UART_DMA_EVENT_BREAK_DETECTED	EVENT_MASK(2)

typedef struct _UartDmaStats
{
	uint32_t rxBytes;
	uint32_t rxLost;
	uint32_t txBytes;
	float rxUsage;
	float txUsage;
} UartDmaStats;

typedef struct _UartDmaData
{
	USART_TypeDef* pRegister;

	DMA_TypeDef* pDma;
	uint16_t dmaStreamTx;
	uint16_t dmaStreamRx;
	uint32_t dmaPrio;

	uint8_t* pDmaBufTx; // must be in non-cachable memory
	uint8_t* pDmaBufRx; // must be in non-cachable memory
	uint32_t dmaBufSizeTx;
	uint32_t dmaBufSizeRx;

	uint32_t baudrate;
	uint32_t perClk;
	uint8_t swapRxTx;

	const char* pTaskName;
} UartDmaData;

typedef struct _UartDma
{
	UartDmaData data;

	DMA_Stream_TypeDef* pDmaTx;
	DMA_Stream_TypeDef* pDmaRx;

	Fifo txFifo;

	UartDmaStats liveStats;
	UartDmaStats stats; // rates and usage per second

	uint32_t txTransferSize;
	uint8_t* pRxRead;
	uint8_t* pRxWriteLast;
	uint32_t rxMinEmptySize;

	event_source_t eventSource;

	volatile uint32_t* pDMARxIFCR;
	volatile uint32_t* pDMATxIFCR;
	volatile uint32_t* pDMATxISR;
	uint32_t IFCRRxMask;
	uint32_t IFCRTxMask;
	uint32_t ISRTxTCMask;

	mutex_t rxDataMutex;
	mutex_t txFifoMutex;

	THD_WORKING_AREA(waTask, 256);
	thread_t* pTask;
} UartDma;

void UartDmaInit(UartDma* pUart, UartDmaData* pInit, tprio_t prio);
void UartDmaIrq(UartDma* pUart);
void UartDmaTxDmaIrq(UartDma* pUart);
size_t UartDmaWrite(UartDma* pUart, const void* pData, size_t dataSize);
size_t UartDmaRead(UartDma* pUart, void* pTarget, size_t targetSize);
