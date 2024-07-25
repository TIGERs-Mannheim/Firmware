#include "spi_lld.h"
#include "errors.h"
#include "util/bits.h"
#include "hal/sys_time.h"

#define DMAClearEn(pDMA)	BITS_CLEAR(pDMA->CR, DMA_SxCR_EN)
#define DMASetEn(pDMA)		BITS_SET(pDMA->CR, DMA_SxCR_EN)

// IRQ -> no task switch can occur
void SPILLDDmaRxInterrupt(SPILLD* pSPI)
{
	pSPI->transferCompletionTime_us = SysTimeUSec();

	if(pSPI->activeCsPin.pPort)
		GPIOPinSet(pSPI->activeCsPin);

	DMAClearEn(pSPI->pDmaRx);
	DMAClearEn(pSPI->pDmaTx);

	*pSPI->pDMARxIFCR |= pSPI->IFCRRxMask; // clear rx flags
	*pSPI->pDMATxIFCR |= pSPI->IFCRTxMask; // clear tx flags

	// Clear SPE
#ifdef STM32H7XX
	pSPI->data.pRegister->CR1 &= ~SPI_CR1_SPE;
#else
	pSPI->data.pRegister->CR1 &= ~(SPI_CR1_SSI | SPI_CR1_SPE);
#endif

#if defined(STM32F3XX) || defined(STM32F7XX)
	// flush RXFIFO of SPI
	while((pSPI->data.pRegister->SR & SPI_SR_FRLVL) != 0)
		pSPI->data.pRegister->DR;
#endif

#ifndef STM32H7XX
	pSPI->data.pRegister->CR2 = 0;
#endif

	pSPI->activeCsPin.pPort = 0;

	if(pSPI->doneCallback)
		(pSPI->doneCallback)(pSPI->pCallbackData);
}

void SPILLDInit(SPILLD* pSPI, SPILLDData* pInit)
{
	pSPI->data = *pInit;
	pSPI->activeCsPin.pPort = 0;

	pSPI->pDmaRx = DMAGenerateStreamTypeDef(pSPI->data.pDma, pSPI->data.dmaChRx);
	pSPI->pDmaTx = DMAGenerateStreamTypeDef(pSPI->data.pDma, pSPI->data.dmaChTx);
	pSPI->pDMARxIFCR = DMAGenerateIFCR(pSPI->data.pDma, pSPI->data.dmaChRx);
	pSPI->pDMATxIFCR = DMAGenerateIFCR(pSPI->data.pDma, pSPI->data.dmaChTx);
	pSPI->IFCRRxMask = DMAGenerateMaskAll(pSPI->data.dmaChRx);
	pSPI->IFCRTxMask = DMAGenerateMaskAll(pSPI->data.dmaChTx);

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
	pSPI->pDmaTx->CR |= pInit->dmaPrio | DMA_SxCR_MINC | DMA_SxCR_DIR_0;
	pSPI->pDmaTx->FCR = DMA_SxFCR_FTH_0 | DMA_SxFCR_DMDIS | DMA_SxFCR_FEIE; // FIFO 1/2 threshold, direct mode disabled, FIFO error IRQ enabled
	pSPI->pDmaTx->NDTR = 0;	// Number of data to transfer
#ifdef STM32H7XX
	pSPI->pDmaTx->PAR = (uint32_t)&pSPI->data.pRegister->TXDR;	// peripheral address
#else
	pSPI->pDmaTx->PAR = (uint32_t)&pSPI->data.pRegister->DR;	// peripheral address
#endif
	pSPI->pDmaTx->M0AR = (uint32_t)pSPI->data.pDmaBufTx;

	// Medium Priority, Memory Increment, Peripheral-to-Memory, Transfer complete  interrupt enable
	pSPI->pDmaRx->CR |= pInit->dmaPrio | DMA_SxCR_MINC | DMA_SxCR_TCIE;
	pSPI->pDmaRx->FCR = DMA_SxFCR_FTH_0 | DMA_SxFCR_DMDIS | DMA_SxFCR_FEIE; // FIFO 1/2 threshold, direct mode disabled, FIFO error IRQ enabled
	pSPI->pDmaRx->NDTR = 0;
#ifdef STM32H7XX
	pSPI->pDmaRx->PAR = (uint32_t)&pSPI->data.pRegister->RXDR;
#else
	pSPI->pDmaRx->PAR = (uint32_t)&pSPI->data.pRegister->DR;
#endif
	pSPI->pDmaRx->M0AR = (uint32_t)pSPI->data.pDmaBufRx;

#ifdef STM32H7XX
	pSPI->data.pRegister->CR1 = SPI_CR1_SSI;
#else
	pSPI->data.pRegister->SR = 0;
#endif
}

void SPILLDConfigureCsPin(GPIOPin csPin)
{
	GPIOPinSet(csPin);

	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_OUTPUT;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_25MHZ;
	gpioInit.pupd = GPIO_PUPD_NONE;
	GPIOInit(csPin.pPort, csPin.pin, &gpioInit);
}

int16_t	SPILLDReconfigure(SPILLD* pSPI, uint32_t prescaler, uint32_t cpol, uint32_t cpha)
{
	if(pSPI->activeCsPin.pPort)
		return ERROR_SPI_BUSY;

#ifdef STM32H7XX
	pSPI->data.pRegister->CFG1 = prescaler | SPI_CFG1_DSIZE_0 | SPI_CFG1_DSIZE_1 | SPI_CFG1_DSIZE_2 |
			SPI_CFG1_CRCSIZE_0 | SPI_CFG1_CRCSIZE_1 | SPI_CFG1_CRCSIZE_2;
	pSPI->data.pRegister->CFG2 = (cpol << 25) | (cpha << 24) | SPI_CFG2_MASTER | SPI_CFG2_SSM | SPI_CFG2_AFCNTR;
#else
	pSPI->data.pRegister->CR2 = SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2 | SPI_CR2_FRXTH;	// set 8 bit data size
	pSPI->data.pRegister->CR1 = prescaler | cpol << 1 | cpha | SPI_CR1_MSTR | SPI_CR1_SSM;
#endif

	pSPI->prescaler = prescaler;
	pSPI->cpol = cpol;
	pSPI->cpha = cpha;

	return 0;
}

int16_t SPILLDStartTransfer(SPILLD* pSPI, GPIOPin csPin, uint32_t size)
{
	int16_t result = SPILLDStartTransferSelect(pSPI, csPin, size);
	if(result)
		return result;

	SPILLDStartTransferExec(pSPI);

	return 0;
}

int16_t SPILLDStartTransferSelect(SPILLD* pSPI, GPIOPin csPin, uint32_t size)
{
	if(size == 0)
		return ERROR_INVALID_PARAMETER;

	if(size > pSPI->data.dmaBufSize)
		return ERROR_NOT_ENOUGH_MEMORY;

	if(pSPI->activeCsPin.pPort)
		return ERROR_SPI_BUSY;

	pSPI->activeCsPin = csPin;

	// configure SPI
#ifdef STM32H7XX
	pSPI->data.pRegister->IFCR = 0x0FF8;
#else
	pSPI->data.pRegister->CR1 |= SPI_CR1_SSI | SPI_CR1_SPE;
#endif

	// select slave
	GPIOPinReset(csPin);

	// configure DMA
	pSPI->pDmaTx->NDTR = size;
	pSPI->pDmaRx->NDTR = size;

#ifdef STM32H7XX
	pSPI->data.pRegister->CR2 = size;
#endif

#ifdef STM32F7XX
	pSPI->data.pRegister->CR2 |= SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2 | SPI_CR2_FRXTH;	// set 8 bit data size
#endif

	// enable DMA RX buffer
#ifdef STM32H7XX
	pSPI->data.pRegister->CFG1 |= SPI_CFG1_RXDMAEN;
#else
	pSPI->data.pRegister->CR2 |= SPI_CR2_RXDMAEN;
#endif

	return 0;
}

void SPILLDStartTransferExec(SPILLD* pSPI)
{
	// enable DMA
	DMASetEn(pSPI->pDmaRx);
	DMASetEn(pSPI->pDmaTx);

#ifdef STM32H7XX
	pSPI->data.pRegister->CFG1 |= SPI_CFG1_TXDMAEN;
	pSPI->data.pRegister->CR1 |= SPI_CR1_SPE | SPI_CR1_SSI;
	pSPI->data.pRegister->CR1 |= SPI_CR1_CSTART;
#else
	// enable DMA TX buffer => will start transfer
	pSPI->data.pRegister->CR2 |= SPI_CR2_TXDMAEN;
#endif
}

void SPILLDAbortTransfer(SPILLD* pSPI)
{
	if(!pSPI->activeCsPin.pPort)
		return;

	DMAClearEn(pSPI->pDmaRx);
	DMAClearEn(pSPI->pDmaTx);

	*pSPI->pDMARxIFCR |= pSPI->IFCRRxMask; // clear rx flags
	*pSPI->pDMATxIFCR |= pSPI->IFCRTxMask; // clear tx flags
}

uint8_t	SPILLDIsTransferActive(SPILLD* pSPI)
{
	return pSPI->activeCsPin.pPort != 0;
}
