/*
 * uart_fifo.c
 *
 *  Created on: 13.01.2019
 *      Author: AndreR
 */

#include "uart_fifo.h"
#include "util/init_hal.h"
#include "util/sys_time.h"
#include <string.h>

void UARTFifoIRQ(UARTFifo* pUART)
{
	const uint32_t sr = pUART->pRegister->ISR;

	// we will handle all flags which are set, so clear them here
	// If we blindly clear all flags we might miss some which are set right at this moment (other IRQ)
	pUART->pRegister->ICR = sr;

	// handle receive events
	if(sr & (USART_ISR_RXFT | USART_ISR_IDLE | USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE))
	{
		UARTFifoSlot* pSlot = pUART->pActiveSlot;

		// read data, if present
		while(pUART->pRegister->ISR & USART_ISR_RXNE)
		{
			if(pSlot->length < UART_FIFO_MAX_RX_SIZE)
			{
				pSlot->data[pSlot->length++] = pUART->pRegister->RDR;
			}
			else
			{
				// too much data, drain extra data (lost) and tag slot with overrun error
				pUART->pRegister->RDR;
				pSlot->status |= UART_FIFO_STATUS_ERR_BUF_OVRN;
			}
		}

		// check for errors
		if(sr & USART_ISR_PE)
			pSlot->status |= UART_FIFO_STATUS_ERR_PARITY;

		if(sr & USART_ISR_FE)
			pSlot->status |= UART_FIFO_STATUS_ERR_FRAMING;

		if(sr & USART_ISR_NE)
			pSlot->status |= UART_FIFO_STATUS_ERR_NOISE;

		if(sr & USART_ISR_ORE)
			pSlot->status |= UART_FIFO_STATUS_ERR_FIFO_OVRN;

		// check for idle line => message complete
		if(sr & USART_ISR_IDLE)
		{
			pSlot->status |= UART_FIFO_STATUS_COMPLETE;
			pSlot->rxTime = SysTimeUSec();

			++pUART->pActiveSlot;
			if(pUART->pActiveSlot == &pUART->rxSlots[UART_FIFO_RX_SLOTS])
				pUART->pActiveSlot = &pUART->rxSlots[0];

			pUART->pActiveSlot->status = 0;
			pUART->pActiveSlot->length = 0;

			chSysLockFromISR();
			chBSemSignalI(&pUART->rxDataAvailable);
			chSysUnlockFromISR();

			if(pUART->rxIrqHook)
				(*pUART->rxIrqHook)();
		}
	}

	if(pUART->pTxData)
	{
		// transfer complete?
		if(sr & USART_ISR_TC)
		{
			// is there still data to transfer?
			if(pUART->pTxData != pUART->pTxDataEnd)
			{
				// yes! so we came here because fifo drained! that shouldn't happen (long IRQ blocking)
				pUART->pRegister->CR1 &= ~USART_CR1_TCIE;
				pUART->pRegister->CR3 |= USART_CR3_TXFTIE;
			}
			else
			{
				// transfer is really complete :)
				pUART->pRegister->CR1 &= ~USART_CR1_TCIE;
				pUART->pTxData = 0;

				chSysLockFromISR();
				chBSemSignalI(&pUART->txComplete);
				chSysUnlockFromISR();
			}
		}
		else if(sr & USART_ISR_TXFT) // TX fifo going empty
		{
			// fill it up!
			while(pUART->pRegister->ISR & USART_ISR_TXE)
			{
				// all data to transmit in fifo?
				if(pUART->pTxData == pUART->pTxDataEnd)
				{
					// disable the fifo interrupt and wait for transfer complete
					pUART->pRegister->CR3 &= ~USART_CR3_TXFTIE;
					pUART->pRegister->CR1 |= USART_CR1_TCIE;
					break;
				}

				pUART->pRegister->TDR = *pUART->pTxData++;
			}
		}
	}
}

void UARTFifoInit(UARTFifo* pUART, USART_TypeDef* pRegister, uint32_t baudrate, uint32_t perClk, uint8_t enableParity)
{
	pUART->pRegister = pRegister;
	pUART->baudrate = baudrate;
	pUART->perClk = perClk;
	pUART->parityEnabled = enableParity;
	pUART->rxIrqHook = 0;
	chBSemObjectInit(&pUART->rxDataAvailable, 1);
	chBSemObjectInit(&pUART->txComplete, 1);
	chMtxObjectInit(&pUART->txInUse);

	pUART->pActiveSlot = &pUART->rxSlots[0];

	pRegister->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_FIFOEN;

	if(enableParity)
		pRegister->CR1 |= USART_CR1_PCE | USART_CR1_M0;

	pRegister->CR2 = 0;
	pRegister->CR3 = USART_CR3_RXFTCFG_1 | USART_CR3_TXFTCFG_1;
	pRegister->BRR = USARTCalcBRR(baudrate, perClk, 0);
	pRegister->CR1 |= USART_CR1_UE;
	pRegister->ICR = 0xFFFFFFFF;

	pRegister->CR1 |= USART_CR1_IDLEIE;

	if(enableParity)
		pRegister->CR1 |= USART_CR1_PEIE;

	pRegister->CR3 |= USART_CR3_EIE | USART_CR3_RXFTIE;
}

void UARTFifoSetBaudrate(UARTFifo* pUART, uint32_t baudrate)
{
	chMtxLock(&pUART->txInUse);

	pUART->pRegister->CR1 &= ~USART_CR1_UE;
	pUART->pRegister->BRR = USARTCalcBRR(baudrate, pUART->perClk, 0);
	pUART->pRegister->CR1 |= USART_CR1_UE;
	pUART->baudrate = baudrate;

	chMtxUnlock(&pUART->txInUse);
}

void UARTFifoFlush(UARTFifo* pUART)
{
	chMtxLock(&pUART->txInUse);

	chBSemReset(&pUART->rxDataAvailable, 1);

	pUART->pRegister->CR1 &= ~(USART_CR1_PEIE | USART_CR1_IDLEIE | USART_CR1_TCIE);
	pUART->pRegister->CR3 &= ~(USART_CR3_EIE | USART_CR3_RXFTIE | USART_CR3_TXFTIE);

	pUART->pRegister->ICR = 0xFFFFFFFF;
	pUART->pRegister->RQR = USART_RQR_TXFRQ | USART_RQR_RXFRQ;

	memset(pUART->rxSlots, 0x00, sizeof(pUART->rxSlots));

	pUART->pRegister->CR1 |= USART_CR1_IDLEIE;

	if(pUART->parityEnabled)
		pUART->pRegister->CR1 |= USART_CR1_PEIE;

	pUART->pRegister->CR3 |= USART_CR3_EIE | USART_CR3_RXFTIE;

	chMtxUnlock(&pUART->txInUse);
}

int16_t UARTFifoRead(UARTFifo* pUART, uint8_t* pData, uint16_t* dataSize, uint16_t* pStatus, uint32_t* pRxTime)
{
	int16_t result = 0;

	UARTFifoSlot* pSlot = pUART->pActiveSlot;
	while((pSlot->status & UART_FIFO_STATUS_COMPLETE) == 0)
	{
		++pSlot;
		if(pSlot == &pUART->rxSlots[UART_FIFO_RX_SLOTS])
			pSlot = &pUART->rxSlots[0];

		if(pSlot == pUART->pActiveSlot)
			return UART_FIFO_ERROR_NO_DATA;
	}

	if(pStatus)
		*pStatus = pSlot->status;

	if(pRxTime)
		*pRxTime = pSlot->rxTime;

	if(*dataSize < pSlot->length)
		result = UART_FIFO_ERROR_TRUNCATED;
	else
		*dataSize = pSlot->length;

	memcpy(pData, (void*)pSlot->data, *dataSize);

	pSlot->length = 0;
	pSlot->status = 0;

	return result;
}

msg_t UARTFifoReadWait(UARTFifo* pUART, systime_t timeout)
{
	return chBSemWaitTimeout(&pUART->rxDataAvailable, timeout);
}

void UARTFifoWriteStart(UARTFifo* pUART, const uint8_t* pData, uint16_t size)
{
	chMtxLock(&pUART->txInUse);

	pUART->pTxData = 0;

	pUART->pRegister->CR1 &= ~USART_CR1_TCIE;
	pUART->pRegister->CR3 &= ~USART_CR3_TXFTIE;

	chBSemReset(&pUART->txComplete, 1);

	pUART->pRegister->RQR = USART_RQR_TXFRQ;
	pUART->pRegister->ICR = USART_ICR_TCCF;

	pUART->pTxDataEnd = pData+size;
	pUART->pTxData = pData;

	pUART->pRegister->CR3 |= USART_CR3_TXFTIE;
}

int16_t UARTFifoWriteWait(UARTFifo* pUART, systime_t timeout)
{
	if(pUART->txInUse.m_owner == NULL)
		return 0; // no transfer pending

	msg_t waitResult = chBSemWaitTimeout(&pUART->txComplete, timeout);

	chMtxUnlock(&pUART->txInUse);

	return waitResult == MSG_OK ? 0 : -1;
}

int16_t UARTFifoWrite(UARTFifo* pUART, const uint8_t* pData, uint16_t size, systime_t timeout)
{
	UARTFifoWriteStart(pUART, pData, size);

	return UARTFifoWriteWait(pUART, timeout);
}
