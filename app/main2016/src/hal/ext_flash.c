/*
 * ext_flash.c
 *
 *  Created on: 12.11.2015
 *      Author: AndreR
 */

#include "ext_flash.h"

#include "util/init_hal.h"
#include "util/sys_time.h"
#include "util/console.h"
#include "util/log.h"
#include "../constants.h"
#include "errors.h"
#include <string.h>
								 // Addr, Dummy, Data
#define SST25_READ			0x03 // 3, 0, 1-n
#define SST25_READ_FAST		0x0B // 3, 1, 1-n
#define SST25_ERASE_4K		0x20 // 3, 0, 0
#define SST25_ERASE_32K		0x52 // 3, 0, 0
#define SST25_ERASE_64K		0xD8 // 3, 0, 0
#define SST25_ERASE_CHIP	0x60 // 0, 0, 0
#define SST25_AAI_PROGRAM	0xAD // 3, 0, 2-n
#define SST25_RDSR			0x05 // 0, 0, 1-n
#define SST25_EWSR			0x50 // 0, 0, 0
#define SST25_WRSR			0x01 // 0, 0, 1
#define SST25_WREN			0x06 // 0, 0, 0
#define SST25_WRDI			0x04 // 0, 0, 0

#define QSPI_DMA DMA2_Stream7

ExtFlash extFlash;

void QUAD_SPI_IRQHandler()
{
	CH_IRQ_PROLOGUE();

	// clear all IRQ sources
	QUADSPI->CR &= ~(QUADSPI_CR_TOIE | QUADSPI_CR_SMIE | QUADSPI_CR_FTIE | QUADSPI_CR_TCIE | QUADSPI_CR_TEIE);

	chSysLockFromISR();
	chBSemSignalI(&extFlash.irqSem);
	chSysUnlockFromISR();

	CH_IRQ_EPILOGUE();
}

void ExtFlashInit()
{
	chBSemObjectInit(&extFlash.irqSem, 1);

	GPIOInitData gpioInit;

	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.alternate = 9;
	gpioInit.ospeed = GPIO_OSPEED_50MHZ;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.pupd = GPIO_PUPD_NONE;
	GPIOInit(GPIOB, GPIO_PIN_2, &gpioInit);	// CLK

	gpioInit.alternate = 10;
	GPIOInit(GPIOB, GPIO_PIN_6, &gpioInit);	// CS

	GPIOInit(GPIOF, GPIO_PIN_8 | GPIO_PIN_9, &gpioInit);	// MOSI, MISO

	RCC->AHB3ENR |= RCC_AHB3ENR_QSPIEN;

	QUADSPI->CR = (4 << 24) | QUADSPI_CR_APMS | (3 << 8) | QUADSPI_CR_EN; // prescaler 5 = 43.2MHz, FTF set on 4 bytes
	QUADSPI->DCR = (20 << 16) | (2 << 8);	// 20bit addressing (2MB), 3 clocks CS high between CMDs

	// DMA2 ST7 CH3
	QSPI_DMA->FCR = DMA_FTH_HALF | DMA_SxFCR_DMDIS;
	QSPI_DMA->CR = (3 << 25) | DMA_PL_LOW | DMA_PSIZE_4BYTE | DMA_MSIZE_4BYTE | DMA_DIR_PERIPH2MEM | DMA_SxCR_MINC;
	QSPI_DMA->M0AR = 0;
	QSPI_DMA->PAR = (uint32_t)&QUADSPI->DR;

	NVICEnableIRQ(QUADSPI_IRQn, IRQL_QUADSPI);
}

uint8_t ExtFlashGetStatus()
{
	chBSemReset(&extFlash.irqSem, 1);

	QUADSPI->CR |= QUADSPI_CR_ABORT;	// flush FIFO
	QUADSPI->FCR = 0x1B;	// clear all flags

	QUADSPI->DLR = 0;	// 1 byte
	QUADSPI->CR |= QUADSPI_CR_TCIE;
	// Indirect read mode, single data line, 0 dummy cycle, no address, single instr line
	QUADSPI->CCR = QUADSPI_CCR_FMODE_0 | QUADSPI_CCR_DMODE_0 | QUADSPI_CCR_IMODE_0 | SST25_RDSR;

	chBSemWait(&extFlash.irqSem);

	uint8_t status = QUADSPI->DR;

	return status;
}

void ExtFlashSetProtected(uint8_t protect)
{
	chBSemReset(&extFlash.irqSem, 1);

	QUADSPI->CR |= QUADSPI_CR_ABORT;	// flush FIFO
	QUADSPI->FCR = 0x1B;	// clear all flags
	QUADSPI->CR |= QUADSPI_CR_TCIE;
	QUADSPI->CCR = QUADSPI_CCR_IMODE_0 | SST25_EWSR;

	chBSemWait(&extFlash.irqSem);

	QUADSPI->CR |= QUADSPI_CR_ABORT;	// flush FIFO
	QUADSPI->FCR = 0x1B;	// clear all flags
	QUADSPI->DLR = 0;	// 1 byte
	QUADSPI->CCR = QUADSPI_CCR_DMODE_0 | QUADSPI_CCR_IMODE_0 | SST25_WRSR;
	QUADSPI->CR |= QUADSPI_CR_TCIE;
	*((uint8_t*)&QUADSPI->DR) = protect ? 0x1C : 0;

	chBSemWait(&extFlash.irqSem);
}

void ExtFlashSetWrite(uint8_t enable)
{
	uint32_t cmd = enable ? SST25_WREN : SST25_WRDI;

	chBSemReset(&extFlash.irqSem, 1);

	QUADSPI->CR |= QUADSPI_CR_ABORT;	// flush FIFO
	QUADSPI->FCR = 0x1B;	// clear all flags
	QUADSPI->CR |= QUADSPI_CR_TCIE;
	QUADSPI->CCR = QUADSPI_CCR_IMODE_0 | cmd;

	chBSemWait(&extFlash.irqSem);
}

void ExtFlashWaitUntilReady()
{
	chBSemReset(&extFlash.irqSem, 1);

	QUADSPI->CR |= QUADSPI_CR_ABORT;	// flush FIFO
	QUADSPI->FCR = 0x1B;	// clear all flags
	QUADSPI->DLR = 0;	// 1 byte
	QUADSPI->PSMKR = 1;
	QUADSPI->PSMAR = 0;	// wait for busy == 0
	QUADSPI->CR |= QUADSPI_CR_SMIE;
	QUADSPI->CCR = QUADSPI_CCR_FMODE_1 | QUADSPI_CCR_DMODE_0 | QUADSPI_CCR_IMODE_0 | SST25_RDSR;

	chBSemWait(&extFlash.irqSem);
}

// takes approx. 35ms
void ExtFlashChipErase()
{
	uint8_t status = ExtFlashGetStatus();

	if(status & 0x1C)				// write-protected?
		ExtFlashSetProtected(0);	// disable!
	if((status & 0x02) == 0)		// write not enabled?
		ExtFlashSetWrite(1);		// enable!

	chBSemReset(&extFlash.irqSem, 1);

	QUADSPI->CR |= QUADSPI_CR_ABORT;	// flush FIFO
	QUADSPI->FCR = 0x1B;	// clear all flags
	QUADSPI->CR |= QUADSPI_CR_TCIE;
	QUADSPI->CCR = QUADSPI_CCR_IMODE_0 | SST25_ERASE_CHIP;

	chBSemWait(&extFlash.irqSem);

	ExtFlashWaitUntilReady();

	ExtFlashSetWrite(0);
	ExtFlashSetProtected(1);
}

int16_t ExtFlashRead(uint32_t flashAddr, void* pBuffer, uint32_t numBytes)
{
	if(numBytes%4 != 0)
	{
		LogErrorC("Number of bytes must be multiple of 4", numBytes);
		return ERROR_INVALID_PARAMETER;
	}

	if(ExtFlashGetStatus() & 0x02)	// is flash write-enabled?
		ExtFlashSetWrite(0);		// clear that to read data

	chBSemReset(&extFlash.irqSem, 1);

	// flush cache lines that will be updated in a moment by DMA
	SystemCleanInvalidateDCache(pBuffer, numBytes);

	DMA2->HIFCR = 0x3D << 22;
	QSPI_DMA->NDTR = numBytes/4;
	QSPI_DMA->M0AR = (uint32_t)pBuffer;

	QUADSPI->FCR = 0x1B;	// clear all flags
	QUADSPI->DLR = numBytes-1;
	// Indirect read mode, single data line, 1 dummy cycle, 24bit address, single addr line, single instr line
	QUADSPI->CCR = QUADSPI_CCR_FMODE_0 | QUADSPI_CCR_DMODE_0 | (8 << 18) | QUADSPI_CCR_ADMODE_0 | QUADSPI_CCR_ADSIZE_1 | QUADSPI_CCR_IMODE_0;
	QUADSPI->CR |= QUADSPI_CR_DMAEN | QUADSPI_CR_TCIE;
	QUADSPI->CCR |= SST25_READ_FAST;
	QUADSPI->AR = flashAddr;
	QSPI_DMA->CR |= DMA_SxCR_EN;

	chBSemWait(&extFlash.irqSem);

	QUADSPI->CR &= ~QUADSPI_CR_DMAEN;

	return 0;
}

int16_t ExtFlashWrite(uint32_t flashAddr, const void* pBuffer, uint32_t numBytes)
{
	if(numBytes%2 != 0)
	{
		LogErrorC("Odd number of bytes to write", numBytes);
		return ERROR_INVALID_PARAMETER;
	}

	uint8_t status = ExtFlashGetStatus();

	if(status & 0x1C)				// write-protected?
		ExtFlashSetProtected(0);	// disable!
	if((status & 0x02) == 0)		// write not enabled?
		ExtFlashSetWrite(1);		// enable!

	uint16_t* pData = (uint16_t*)pBuffer;
	const uint16_t* pEnd = ((uint16_t*)pBuffer)+numBytes/2;

	chBSemReset(&extFlash.irqSem, 1);

	QUADSPI->CR |= QUADSPI_CR_ABORT;	// flush FIFO
	QUADSPI->FCR = 0x1B;	// clear all flags
	QUADSPI->DLR = 1;
	// Indirect write mode, single data line, 24bit address, single addr line, single instr line
	QUADSPI->CCR = QUADSPI_CCR_DMODE_0 | QUADSPI_CCR_ADMODE_0 | QUADSPI_CCR_ADSIZE_1 | QUADSPI_CCR_IMODE_0 | SST25_AAI_PROGRAM;
	QUADSPI->CR |= QUADSPI_CR_TCIE;
	QUADSPI->AR = flashAddr;
	*((uint16_t*)&QUADSPI->DR) = *pData++;

	chBSemWait(&extFlash.irqSem);

	ExtFlashWaitUntilReady();

	while(pData < pEnd)
	{
		chBSemReset(&extFlash.irqSem, 1);

		QUADSPI->CR |= QUADSPI_CR_ABORT;	// flush FIFO
		QUADSPI->FCR = 0x1B;	// clear all flags
		QUADSPI->DLR = 1;
		// Indirect write mode, single data line, no addr, single instr line
		QUADSPI->CCR = QUADSPI_CCR_DMODE_0 | QUADSPI_CCR_IMODE_0 | SST25_AAI_PROGRAM;
		QUADSPI->CR |= QUADSPI_CR_TCIE;
		*((uint16_t*)&QUADSPI->DR) = *pData++;

		chBSemWait(&extFlash.irqSem);

		ExtFlashWaitUntilReady();
	}

	ExtFlashSetWrite(0);
	ExtFlashSetProtected(1);

	return 0;
}

static uint8_t dmaBuf[1024] __attribute__((aligned(16)));

void ExtFlashPrintPerformance()
{
	uint32_t start;
	uint32_t end;

	memset(dmaBuf, 0, sizeof(dmaBuf));

	start = SysTimeUSec();
	ExtFlashChipErase();
	end = SysTimeUSec();
	uint32_t eraseTime = end-start;

	start = SysTimeUSec();
	ExtFlashWrite(0x000000, dmaBuf, 1024);
	end = SysTimeUSec();
	uint32_t writeTime = end-start;

	start = SysTimeUSec();
	ExtFlashRead(0x000000, dmaBuf, 1024);
	end = SysTimeUSec();
	uint32_t readTime = end-start;

	ConsolePrint("Erase time (2MB): %uus\r\n", eraseTime);
	ConsolePrint("Write time (1k):  %uus\r\n", writeTime);
	ConsolePrint("Read time (1k):   %uus\r\n", readTime);

	ConsolePrint("Erase speed: %ukB/s\r\n", (2048*1000000)/eraseTime);
	ConsolePrint("Write speed: %ukB/s\r\n", 1000000/writeTime);
	ConsolePrint("Read speed:  %ukB/s\r\n", 1000000/readTime);
}
