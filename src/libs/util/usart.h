/*
 * usart.h
 *
 *  Created on: 04.11.2012
 *      Author: AndreR
 */

#ifndef USART_H_
#define USART_H_

#if defined(STM32F30X)
#include "stm32f30x.h"
#elif defined(STM32F4XX)
#include "stm32f407xx.h"
#elif defined(STM32F7XX)
#include "stm32f746xx.h"
#endif
#include "util/fifo_lin.h"

#include "ch.h"

typedef struct _USARTData
{
	uint8_t* pDataTx;
	uint16_t txSize;
	uint8_t* pDataRx;
	uint16_t rxSize;

	uint32_t maxPacketSize;

#if defined(STM32F4XX) || defined(STM32F7XX)
	uint32_t dmaChannel;
#endif
	uint32_t rxStream;
	uint32_t txStream;

	DMA_TypeDef* pDMA;

	uint8_t usartIRQn;
	uint8_t IRQL;

	uint32_t baudrate;
	uint32_t peripheralClock;
} USARTData;

typedef struct _USARTStats
{
	uint32_t rxWire;
	uint32_t rxPayload;
	uint32_t txWire;
	uint32_t txPayload;
	float rxUsage;
	float txUsage;
} USARTStats;

typedef void(*USARTCallback)(void*);

typedef struct _USART
{
	USART_TypeDef* pRegister;

	USARTData data;

	FifoLin txFifo;
	FifoLin rxFifo;

	uint8_t* pRxDMABuf;	// circular DMA buffer
	uint8_t* pRxProcBuf; // linear processing buffer
	uint16_t rxProcBufSize;
	uint16_t rxDmaBufSize;
	uint16_t rxProcBufUsed;

	uint8_t* pTxDmaBuf;
	uint16_t txDmaBufSize;

#if defined(STM32F4XX) || defined(STM32F7XX) || defined(STM32H7XX)
	DMA_Stream_TypeDef* pRxStream;
	DMA_Stream_TypeDef* pTxStream;
#else
	DMA_Channel_TypeDef* pRxStream;
	DMA_Channel_TypeDef* pTxStream;
#endif

	volatile uint32_t* pDMARxIFCR;
	volatile uint32_t* pDMATxIFCR;
	volatile uint32_t* pDMARxISR;
	volatile uint32_t* pDMATxISR;
	uint32_t IFCRRxMask;
	uint32_t IFCRTxMask;

	USARTCallback callback;
	void* pUserData;

	mailbox_t eventQueue;
	msg_t eventQueueData[10];

	USARTStats liveStats;
	USARTStats stats;
} USART;

void	USARTInit(USART* pUSART, USART_TypeDef* pRegister, USARTData data);
void	USARTTask(void* params);
int16_t USARTSend(USART* pUSART, uint16_t length, uint8_t* pData);
int16_t USARTStartTransfer(USART* pUSART);

// IRQ functions
void 	USARTIRQ(USART* pUSART);

#endif
