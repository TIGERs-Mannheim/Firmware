/*
 * uart_fifo.h
 *
 *  Created on: 13.01.2019
 *      Author: AndreR
 */

#pragma once

#include "ch.h"
#include "hal/init_hal.h"

#define UART_FIFO_RX_SLOTS 5
#define UART_FIFO_MAX_RX_SIZE 330

#define UART_FIFO_STATUS_ERR_MASK		0x1F
#define UART_FIFO_STATUS_ERR_PARITY		0x01 // Parity Error
#define UART_FIFO_STATUS_ERR_FRAMING	0x02 // Framing Error (break received)
#define UART_FIFO_STATUS_ERR_NOISE		0x04 // noise error
#define UART_FIFO_STATUS_ERR_FIFO_OVRN	0x08 // embedded FIFO overrun
#define UART_FIFO_STATUS_ERR_BUF_OVRN	0x10 // maximum rx message size exceeded, data truncated
#define UART_FIFO_STATUS_COMPLETE		0x20

#define UART_FIFO_ERROR_TIMEOUT 1
#define UART_FIFO_ERROR_NO_DATA 2
#define UART_FIFO_ERROR_TRUNCATED 3

typedef void(*UARTFifoRxIrqHook)();

typedef struct _UARTFifoSlot
{
	volatile uint16_t status;
	volatile uint16_t length;
	volatile uint8_t data[UART_FIFO_MAX_RX_SIZE];
	volatile uint32_t rxTime;
} UARTFifoSlot;

typedef struct _UARTFifo
{
	USART_TypeDef* pRegister;
	binary_semaphore_t rxDataAvailable;
	binary_semaphore_t txComplete;
	mutex_t txInUse;
	uint32_t baudrate;
	uint32_t perClk;
	uint8_t parityEnabled;
	uint8_t sendBreakAfterTx;

	UARTFifoSlot rxSlots[UART_FIFO_RX_SLOTS];
	UARTFifoSlot* pActiveSlot;

	volatile const uint8_t* pTxData;
	volatile const uint8_t* pTxDataEnd;

	UARTFifoRxIrqHook rxIrqHook;
} UARTFifo;

void UARTFifoIRQ(UARTFifo* pUART);
void UARTFifoInit(UARTFifo* pUART, USART_TypeDef* pRegister, uint32_t baudrate, uint32_t perClk, uint8_t enableParity);
void UARTFifoFlush(UARTFifo* pUART);
void UARTFifoSetBaudrate(UARTFifo* pUART, uint32_t baudrate);
int16_t UARTFifoRead(UARTFifo* pUART, uint8_t* pData, uint16_t* dataSize, uint16_t* pStatus, uint32_t* pRxTime);
msg_t UARTFifoReadWait(UARTFifo* pUART, systime_t timeout);
void UARTFifoWriteStart(UARTFifo* pUART, const uint8_t* pData, uint16_t size);
int16_t UARTFifoWriteWait(UARTFifo* pUART, systime_t timeout);
int16_t UARTFifoWrite(UARTFifo* pUART, const uint8_t* pData, uint16_t size, systime_t timeout);
