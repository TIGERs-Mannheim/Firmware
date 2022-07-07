/*
 * ext.c
 *
 *  Created on: 25.06.2019
 *      Author: AndreR
 */

#include "ext.h"
#include "robot/robot.h"
#include "util/init_hal.h"
#include "util/log.h"
#include "util/sys_time.h"
#include "util/crc.h"
#include "constants.h"
#include "errors.h"
#include "commands.h"
#include "robot_pi.h"
#include <string.h>

/**
 * TX procedure:
 * - disable TX DMA
 * - shift out transmitted data
 * - do COBS encoding directly to TX DMA buffer
 * - re-enable TX DMA
 *
 * RX procedure:
 * - Regularly (~5ms):
 * - disable RX DMA
 * - store number of received bytes
 * - set RX DMA memory address to other buffer
 * - re-enable RX DMA
 * - process received bytes by COBS
 * - if packet complete: process it
 */

static void processReceivedData(const uint8_t* pData, uint32_t dataLength);

Ext ext __attribute__((aligned(1024), section(".sram1")));;

void USART6_IRQHandler()
{
	CH_IRQ_PROLOGUE();

	const uint32_t sr = USART6->ISR;
	USART6->ICR = sr;

	if(sr & USART_ISR_IDLE)
	{
		chSysLockFromISR();
		chBSemSignalI(&ext.dataAvailable);
		chSysUnlockFromISR();
	}

	CH_IRQ_EPILOGUE();
}

void ExtInit(uint32_t baudrate)
{
	memset(&ext, 0, sizeof(Ext)); // Data in SRAM1 is not zero-initialized

	ext.baudrate = baudrate;

	chBSemObjectInit(&ext.dataAvailable, 1);
	chMtxObjectInit(&ext.txMutex);

	DMAMUX1_Channel6->CCR = 71; // usart6_rx_dma
	DMAMUX1_Channel7->CCR = 72; // usart6_tx_dma

	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.alternate = 7;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_25MHZ;
	gpioInit.pupd = GPIO_PUPD_UP;
	GPIOInit(GPIOG, GPIO_PIN_9 | GPIO_PIN_14, &gpioInit); // USART6_RX, USART6_TX
	RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
	__DSB();

	USART6->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_FIFOEN;
	USART6->CR2 = 0;
	USART6->CR3 = USART_CR3_DMAR | USART_CR3_DMAT;
	USART6->BRR = USARTCalcBRR(baudrate, systemClockInfo.APB2PeriphClk, 0);
	USART6->CR1 |= USART_CR1_UE;
	USART6->ICR = 0xFFFFFFFF;

	USART6->CR1 |= USART_CR1_IDLEIE;

//	USART6->CR3 |= USART_CR3_EIE;

	ext.pRxStream = DMA1_Stream6;
	ext.pTxStream = DMA1_Stream7;
	ext.pRxIFCR = DMAGenerateIFCR(DMA1, 6);
	ext.pTxIFCR = DMAGenerateIFCR(DMA1, 7);
	ext.rxIfcrClearAll = DMAGenerateMaskAll(6);
	ext.txIfcrClearAll = DMAGenerateMaskAll(7);

	ext.pRxStream->CR = DMA_DIR_PERIPH2MEM | (1 << 20) | DMA_PL_MEDIUM | DMA_SxCR_MINC;
	ext.pRxStream->FCR = 0;
	ext.pRxStream->M0AR = (uint32_t)ext.rxBuf[0];
	ext.pRxStream->PAR = (uint32_t)&USART6->RDR;
	ext.pRxStream->NDTR = EXT_RX_BUF_SIZE;

	ext.pTxStream->CR = DMA_DIR_MEM2PERIPH | (1 << 20) | DMA_PL_MEDIUM | DMA_SxCR_MINC;
	ext.pTxStream->FCR = 0;
	ext.pTxStream->M0AR = (uint32_t)ext.txBuf;
	ext.pTxStream->PAR = (uint32_t)&USART6->TDR;
	ext.pTxStream->NDTR = 0;

	*ext.pRxIFCR = ext.rxIfcrClearAll;
	*ext.pTxIFCR = ext.txIfcrClearAll;

	ext.pRxStream->CR |= DMA_SxCR_EN;

	NVICEnableIRQ(USART6_IRQn, IRQL_EXT_COMM);
}

void ExtTask(void* params)
{
	(void)params;

	chRegSetThreadName("EXT");

	uint32_t last1sTime = 0;

	while(1)
	{
		chBSemWaitTimeout(&ext.dataAvailable, MS2ST(2));

		// disable RX DMA
		ext.pRxStream->CR &= ~DMA_SxCR_EN;
		while(ext.pRxStream->CR & DMA_SxCR_EN);

		// store information about received data
		uint32_t bytesReceived = EXT_RX_BUF_SIZE - ext.pRxStream->NDTR;
		volatile uint8_t* pRxBuf = (volatile uint8_t*)ext.pRxStream->M0AR;

		// swap RX buffers
		if(ext.pRxStream->M0AR == (uint32_t)ext.rxBuf[0])
			ext.pRxStream->M0AR = (uint32_t)ext.rxBuf[1];
		else
			ext.pRxStream->M0AR = (uint32_t)ext.rxBuf[0];

		// re-enable RX DMA
		*ext.pRxIFCR = ext.rxIfcrClearAll;
		ext.pRxStream->NDTR = EXT_RX_BUF_SIZE;
		ext.pRxStream->CR |= DMA_SxCR_EN;

		ext.liveStats.rxWire += bytesReceived;

		// process data
		for(uint32_t i = 0; i < bytesReceived; i++)
		{
			uint8_t c = pRxBuf[i];

			if(c == 0)
			{
				// packet complete
				uint32_t decodedSize;
				if(COBSDecode(ext.rxProcessingBuf, ext.rxProcBufUsed, ext.rxDecodeBuf, EXT_MAX_PACKET_SIZE, &decodedSize) == 0)
				{
					ext.liveStats.rxPayload += decodedSize;

					if(CRC32CalcChecksum(ext.rxDecodeBuf, decodedSize) == CRC32_MAGIC_NUMBER)
					{
						ext.installed = 1;

						processReceivedData(ext.rxDecodeBuf, decodedSize-sizeof(uint32_t));
					}
					else
					{
						LogWarn("CRC error");
					}
				}
				else
				{
					LogWarn("COBS decode error");
				}

				ext.rxProcBufUsed = 0;
			}
			else
			{
				if(ext.rxProcBufUsed == sizeof(ext.rxProcessingBuf))
				{
					// we filled the whole buffer without delimiter => start again
					ext.rxProcBufUsed = 0;
					LogWarn("rx proc buf overrun");
				}

				ext.rxProcessingBuf[ext.rxProcBufUsed++] = c;
			}
		}

		if(chVTTimeElapsedSinceX(last1sTime) > S2ST(1))
		{
			last1sTime = chVTGetSystemTimeX();

			memcpy(&ext.stats, &ext.liveStats, sizeof(EXTStats));
			memset(&ext.liveStats, 0, sizeof(EXTStats));
			ext.stats.rxUsage = ((float)ext.stats.rxWire)/(ext.baudrate*0.1f);
			ext.stats.txUsage = ((float)ext.stats.txWire)/(ext.baudrate*0.1f);
		}
	}
}


int16_t ExtSendPacket(PacketHeader* pHeader, const void* _pData, uint16_t dataLength)
{
	uint8_t* pData = (uint8_t*)_pData;
	int16_t result = 0;

	chMtxLock(&ext.txMutex);

	if(ext.pTxStream->CR & DMA_SxCR_EN)
	{
		ext.pTxStream->CR &= ~DMA_SxCR_EN;
		while(ext.pTxStream->CR & DMA_SxCR_EN);
	}

	int32_t curTransferBytesRemaining = ext.pTxStream->NDTR;
	int32_t curTransferBytesTransmitted = ext.curTxTransferSize - curTransferBytesRemaining;

	memmove(ext.txBuf, ext.txBuf+curTransferBytesTransmitted, curTransferBytesRemaining);
	int32_t bytesFree = EXT_TX_BUF_SIZE - curTransferBytesRemaining;

	uint32_t bytesWritten = 0;

	if(bytesFree >= COBSMaxStuffedSize(dataLength+sizeof(PacketHeader))+1+4)
	{
		ext.liveStats.txPayload += dataLength+sizeof(PacketHeader);

		memcpy(ext.txAssemblyBuf, pHeader, sizeof(PacketHeader));
		memcpy(ext.txAssemblyBuf+sizeof(PacketHeader), pData, dataLength);
		uint32_t crc32 = CRC32CalcChecksum(ext.txAssemblyBuf, dataLength+sizeof(PacketHeader));
		*((uint32_t*)(ext.txAssemblyBuf+sizeof(PacketHeader)+dataLength)) = crc32;

		uint8_t* pWrite = ext.txBuf + curTransferBytesRemaining;
		COBSEncode(ext.txAssemblyBuf, sizeof(PacketHeader)+dataLength+4, pWrite, bytesFree, &bytesWritten);
		pWrite[bytesWritten++] = 0;

		ext.liveStats.txWire += bytesWritten;
	}
	else
	{
		result = ERROR_NOT_ENOUGH_MEMORY;
	}

	ext.curTxTransferSize = curTransferBytesRemaining + bytesWritten;
	if(ext.curTxTransferSize > 0)
	{
		ext.pTxStream->NDTR = ext.curTxTransferSize;
		*ext.pTxIFCR = ext.txIfcrClearAll;
		ext.pTxStream->CR |= DMA_SxCR_EN;
	}

	chMtxUnlock(&ext.txMutex);

	return result;
}

static void processReceivedData(const uint8_t* pData, uint32_t dataLength)
{
	if(dataLength < 2)
	{
		LogErrorC("Packet too short", dataLength);
		return;
	}

	PacketHeader* pHeader = (PacketHeader*)pData;
	pData += sizeof(PacketHeader);
	dataLength -= sizeof(PacketHeader);

	if(pHeader->section == SECTION_SYSTEM && pHeader->cmd == CMD_SYSTEM_MATCH_CTRL)
	{
		if(dataLength < sizeof(SystemMatchCtrl)-SYSTEM_MATCH_CTRL_USER_DATA_SIZE)
		{
			LogWarnC("Invalid match ctrl size from EXT", dataLength);
		}
		else
		{
			memcpy(&robot.ext.matchCtrl, pData, dataLength);
			robot.ext.lastMatchCtrlTime = SysTimeUSec();
		}
	}

	RobotPiHandleExtPacket(pHeader, pData, dataLength);
}
