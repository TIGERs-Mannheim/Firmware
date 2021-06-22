/*
 * spi_sync.c
 *
 *  Created on: 28.09.2013
 *      Author: AndreR
 */

#include "spi_sync.h"
#include "errors.h"
#include "util/bits.h"
#include "util/init_hal.h"
#include "util/log.h"

#define DMAClearEn(pDMA)	BITS_CLEAR(pDMA->CR, DMA_SxCR_EN)
#define DMASetEn(pDMA)		BITS_SET(pDMA->CR, DMA_SxCR_EN)

static void stopTransfer(SPISync* pSPI)
{
	DMAClearEn(pSPI->pDmaRx);
	DMAClearEn(pSPI->pDmaTx);

	*pSPI->pDMARxIFCR |= pSPI->IFCRRxMask; // clear rx flags
	*pSPI->pDMATxIFCR |= pSPI->IFCRTxMask; // clear tx flags
}

// IRQ -> no task switch can occur
void SPISyncDmaRxInterrupt(SPISync* pSPI)
{
	stopTransfer(pSPI);

	chSysLockFromISR();
	chBSemSignalI(&pSPI->transferSem);
	chSysUnlockFromISR();
}

void SPISyncInit(SPISync* pSPI, SPISyncData* pInit)
{
	pSPI->data = *pInit;

	chBSemObjectInit(&pSPI->transferSem, FALSE);
	chMtxObjectInit(&pSPI->taskMutex);

	pSPI->pDmaRx = DMAGenerateStreamTypeDef(pSPI->data.pDma, pSPI->data.dmaChRx);
	pSPI->pDmaTx = DMAGenerateStreamTypeDef(pSPI->data.pDma, pSPI->data.dmaChTx);
	pSPI->pDMARxIFCR = DMAGenerateIFCR(pSPI->data.pDma, pSPI->data.dmaChRx);
	pSPI->pDMATxIFCR = DMAGenerateIFCR(pSPI->data.pDma, pSPI->data.dmaChTx);
	pSPI->IFCRRxMask = DMAGenerateMaskAll(pSPI->data.dmaChRx);
	pSPI->IFCRTxMask = DMAGenerateMaskAll(pSPI->data.dmaChTx);
	pSPI->pDMARxISR = DMAGenerateISR(pSPI->data.pDma, pSPI->data.dmaChRx);
	pSPI->pDMATxISR = DMAGenerateISR(pSPI->data.pDma, pSPI->data.dmaChTx);

	*pSPI->pDMARxIFCR |= pSPI->IFCRRxMask; // clear rx flags
	*pSPI->pDMATxIFCR |= pSPI->IFCRTxMask; // clear tx flags

#ifndef STM32H7XX
	uint32_t chTx;
	uint32_t chRx;

	// Select correct channel
	if(pInit->pRegister == SPI1) // DMA2 Channel3
	{
		chRx = 3;
		chTx = 3;
	}
	if(pInit->pRegister == SPI2 || pInit->pRegister == SPI3) // DMA1 Channel0
	{
		chRx = 0;
		chTx = 0;
	}
#ifdef STM32F7XX
	if(pInit->pRegister == SPI4) // DMA CH4 (ST0, ST1) or CH5 (ST3,ST4)
	{
		if(pSPI->data.dmaChRx == 0)
			chRx = 4;
		else
			chRx = 5;

		if(pSPI->data.dmaChTx == 1)
			chTx = 4;
		else
			chRx = 5;
	}
	if(pInit->pRegister == SPI5) // DMA2 CH2 (ST3,ST4) or CH7 (ST5,ST6)
	{
		if(pSPI->data.dmaChRx == 3)
			chRx = 2;
		else
			chRx = 7;

		if(pSPI->data.dmaChTx == 4)
			chTx = 2;
		else
			chRx = 7;
	}
	if(pInit->pRegister == SPI6) // DMA2 Channel1
	{
		chRx = 1;
		chTx = 1;
	}
#endif

	pSPI->pDmaRx->CR = (chRx << 25);
	pSPI->pDmaTx->CR = (chTx << 25);
#endif

	// configure DMA
	// Medium Priority, Memory Increment, Memory-to-peripheral
	pSPI->pDmaTx->CR |= (pInit->dmaPrio << 16) | DMA_SxCR_MINC | DMA_SxCR_DIR_0;
	pSPI->pDmaTx->FCR = DMA_SxFCR_FTH_0 | DMA_SxFCR_DMDIS; // FIFO 1/2 threshold, direct mode disabled, FIFO error IRQ enabled
	pSPI->pDmaTx->NDTR = 0;	// Number of data to transfer
#ifdef STM32H7XX
	pSPI->pDmaTx->PAR = (uint32_t)&pSPI->data.pRegister->TXDR;	// peripheral address
#else
	pSPI->pDmaTx->PAR = (uint32_t)&pSPI->data.pRegister->DR;	// peripheral address
#endif
	pSPI->pDmaTx->M0AR = 0;	// configured on transfer

	// Medium Priority, Memory Increment, Peripheral-to-Memory, Transfer complete  interrupt enable
	pSPI->pDmaRx->CR |= (pInit->dmaPrio << 16) | DMA_SxCR_MINC | DMA_SxCR_TCIE;
	pSPI->pDmaRx->FCR = DMA_SxFCR_FTH_0 | DMA_SxFCR_DMDIS | DMA_SxFCR_FEIE; // FIFO 1/2 threshold, direct mode disabled, FIFO error IRQ enabled
	pSPI->pDmaRx->NDTR = 0;
#ifdef STM32H7XX
	pSPI->pDmaRx->PAR = (uint32_t)&pSPI->data.pRegister->RXDR;
#else
	pSPI->pDmaRx->PAR = (uint32_t)&pSPI->data.pRegister->DR;
#endif
	pSPI->pDmaRx->M0AR = 0; // configured on transfer

#ifdef STM32H7XX
	pSPI->data.pRegister->CR1 = SPI_CR1_SSI;
	pSPI->data.pRegister->CFG2 = SPI_CFG2_MASTER | SPI_CFG2_SSM;
	pSPI->data.pRegister->CFG1 = SPI_CFG1_DSIZE_0 | SPI_CFG1_DSIZE_1 | SPI_CFG1_DSIZE_2 |
			SPI_CFG1_CRCSIZE_0 | SPI_CFG1_CRCSIZE_1 | SPI_CFG1_CRCSIZE_2;
#else
	pSPI->data.pRegister->SR = 0;
	pSPI->data.pRegister->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM;
	pSPI->data.pRegister->CR2 = SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2 | SPI_CR2_FRXTH;
#endif
}

void SPISyncSlaveInit(SPISync* pSPI, SPISyncSlave* pSlave)
{
	GPIOSet(pSlave->pCSPort, pSlave->csPin);

	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_OUTPUT;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_25MHZ;
	gpioInit.pupd = GPIO_PUPD_NONE;
	GPIOInit(pSlave->pCSPort, pSlave->csPin, &gpioInit);

	pSlave->pBus = pSPI;
}

void SPISyncStartTransfer(SPISyncSlave* pSlave, uint8_t* pSrc, uint8_t* pDst, uint32_t size)
{
	SPISync* pSPI = pSlave->pBus;

	// Block multiple tasks entering this function
	chMtxLock(&pSPI->taskMutex);

	// configure SPI
#ifdef STM32H7XX
	pSPI->data.pRegister->CFG1 = pSlave->prescaler | SPI_CFG1_DSIZE_0 | SPI_CFG1_DSIZE_1 | SPI_CFG1_DSIZE_2 |
			SPI_CFG1_CRCSIZE_0 | SPI_CFG1_CRCSIZE_1 | SPI_CFG1_CRCSIZE_2;
	pSPI->data.pRegister->CFG2 = (pSlave->cpol << 25) | (pSlave->cpha << 24) | SPI_CFG2_MASTER | SPI_CFG2_SSM | SPI_CFG2_AFCNTR;
#else
	pSPI->data.pRegister->CR2 = SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2 | SPI_CR2_FRXTH;	// set 8 bit data size
	pSPI->data.pRegister->CR1 = pSlave->prescaler | pSlave->cpol << 1 | pSlave->cpha | SPI_CR1_MSTR | SPI_CR1_SSM;
#endif

	// Clear semaphore which indicates transfer is complete
	chBSemWaitTimeout(&pSPI->transferSem, TIME_IMMEDIATE);

#ifdef STM32H7XX
	pSPI->data.pRegister->IFCR = 0x0FF8;
#else
	// enable SPI interface to set data lines to the correct level
	pSPI->data.pRegister->CR1 |= SPI_CR1_SPE | SPI_CR1_SSI;
#endif

	// configure DMA
	if(pSrc == 0)
	{
		pSPI->pDmaTx->CR &= ~DMA_SxCR_MINC;
		pSPI->pDmaTx->M0AR = (uint32_t)&pSPI->dummy;
	}
	else
	{
		pSPI->pDmaTx->CR |= DMA_SxCR_MINC;
		pSPI->pDmaTx->M0AR = (uint32_t)pSrc;
	}

	if(pDst == 0)
	{
		pSPI->pDmaRx->CR &= ~DMA_SxCR_MINC;
		pSPI->pDmaRx->M0AR = (uint32_t)&pSPI->dummy;
	}
	else
	{
		pSPI->pDmaRx->CR |= DMA_SxCR_MINC;
		pSPI->pDmaRx->M0AR = (uint32_t)pDst;
	}

	pSPI->pDmaTx->NDTR = size;
	pSPI->pDmaRx->NDTR = size;
#ifdef STM32H7XX
	pSPI->data.pRegister->CR2 = size;
#endif

	// select slave
	GPIOReset(pSlave->pCSPort, pSlave->csPin);

	// enable DMA RX buffer
#ifdef STM32H7XX
	pSPI->data.pRegister->CFG1 |= SPI_CFG1_RXDMAEN;
#else
	pSPI->data.pRegister->CR2 |= SPI_CR2_RXDMAEN;
#endif

	// enable DMA
	DMASetEn(pSPI->pDmaTx);
	DMASetEn(pSPI->pDmaRx);

#ifdef STM32H7XX
	pSPI->data.pRegister->CFG1 |= SPI_CFG1_TXDMAEN;
	pSPI->data.pRegister->CR1 |= SPI_CR1_SPE | SPI_CR1_SSI;
	pSPI->data.pRegister->CR1 |= SPI_CR1_CSTART;
#else
	// enable DMA TX buffer => will start transfer
	pSPI->data.pRegister->CR2 |= SPI_CR2_TXDMAEN;
#endif
}

int16_t SPISyncWait(SPISyncSlave* pSlave)
{
	SPISync* pSPI = pSlave->pBus;

	// check if wait is called from the task which called start transfer
	if(pSPI->taskMutex.m_owner != chThdGetSelfX())
		return ERROR_SPI_SYNC_INVALID_TASK;

	// wait for transfer complete semaphore
	msg_t result = chBSemWaitTimeout(&pSPI->transferSem, pSlave->timeoutTicks);

	if(result != MSG_OK)
	{
		stopTransfer(pSPI);

		LogError("SPISyncTimeout");

		chMtxUnlock(&pSPI->taskMutex);

		return ERROR_SPI_SYNC_TIMEOUT;
	}

	GPIOSet(pSlave->pCSPort, pSlave->csPin);

#ifdef STM32H7XX
	// disable SPI (SPE cleared)
	pSPI->data.pRegister->CR1 &= ~SPI_CR1_SPE; // SPE disabled
	pSPI->data.pRegister->CFG1 = SPI_CFG1_DSIZE_0 | SPI_CFG1_DSIZE_1 | SPI_CFG1_DSIZE_2 |
			SPI_CFG1_CRCSIZE_0 | SPI_CFG1_CRCSIZE_1 | SPI_CFG1_CRCSIZE_2;
#else
	pSPI->data.pRegister->CR1 &= ~(SPI_CR1_SSI | SPI_CR1_SPE); // SPE disabled

	// flush RXFIFO of SPI
	while((pSPI->data.pRegister->SR & SPI_SR_FRLVL) != 0)
		pSPI->data.pRegister->DR;

	pSPI->data.pRegister->CR2 = SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2 | SPI_CR2_FRXTH;
#endif

	// this task is done
	chMtxUnlock(&pSPI->taskMutex);

	return result;
}

int16_t SPISyncTransfer(SPISyncSlave* pSlave, uint8_t* pSrc, uint8_t* pDst, uint32_t size)
{
	SPISyncStartTransfer(pSlave, pSrc, pDst, size);

	return SPISyncWait(pSlave);
}
