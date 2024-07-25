#include "hal/flash.h"
#include "errors.h"
#include "stm32h743xx.h"
#include "ch.h"
#include "util/log.h"
#include <string.h>

static MUTEX_DECL(flashMutex);

typedef struct _FlashBank
{
	volatile uint32_t* SR;
	volatile uint32_t* CR;
	volatile uint32_t* CCR;
	volatile uint32_t* KEYR;
} FlashBank;

const FlashBank flashBank[2] = {
	{
		&FLASH->SR1,
		&FLASH->CR1,
		&FLASH->CCR1,
		&FLASH->KEYR1,
	},
	{
		&FLASH->SR2,
		&FLASH->CR2,
		&FLASH->CCR2,
		&FLASH->KEYR2,
	}
};

static int16_t waitForLastOperation(const FlashBank* pBank, uint8_t busyWait)
{
	if(busyWait)
	{
		while(*pBank->SR & (FLASH_SR_QW | FLASH_SR_BSY | FLASH_SR_WBNE));
	}
	else
	{
		do
		{
			chThdSleep(1);
		}
		while(*pBank->SR & (FLASH_SR_QW | FLASH_SR_BSY | FLASH_SR_WBNE));
	}

	uint32_t status = *pBank->SR;

	*pBank->CCR = status;	// clear flags

	if(status & FLASH_SR_INCERR)
		return ERROR_FLASH_PROG_INC_ERROR;

	if(status & FLASH_SR_OPERR)
		return ERROR_FLASH_PROG_OP_ERROR;

	if(status & FLASH_SR_PGSERR)
		return ERROR_FLASH_PROG_SEQ_ERROR;

	if(status & FLASH_SR_WRPERR)
		return ERROR_FLASH_WRITE_PROTECT;

	return 0;
}

static int16_t eraseSector(const FlashBank* pBank, uint32_t sector)
{
	int16_t result;

	// Wait for last operation to be completed
	result = waitForLastOperation(pBank, 0);

	if(result == 0)
	{
		// if the previous operation is completed, proceed to erase the sector
		*pBank->CR &= FLASH_CR_PSIZE;
		*pBank->CR |= FLASH_CR_PSIZE_1 | FLASH_CR_PSIZE_0;	// double word size
		*pBank->CR &= ~FLASH_CR_SNB;
		*pBank->CR |= FLASH_CR_SER | (sector << 8);
		*pBank->CR |= FLASH_CR_START;
		__DSB();

		// Wait for last operation to be completed
		result = waitForLastOperation(pBank, 0);

		// if the erase operation is completed, disable the SER Bit
		*pBank->CR &= (~FLASH_CR_SER);
		*pBank->CR &= ~FLASH_CR_SNB;
		__DSB();
	}

	return result;
}

int16_t FlashErase(uint32_t addr, uint32_t endAddr)
{
	int16_t result = 0;

	const FlashBank* pBank;
	uint32_t startSector;
	uint32_t endSector;
	uint32_t baseAddr;

	if(addr < 0x08100000) // flash bank 1?
	{
		pBank = &flashBank[0];
		baseAddr = FLASH_BANK1_BASE;
	}
	else
	{
		pBank = &flashBank[1];
		baseAddr = FLASH_BANK2_BASE;
	}

	startSector = (addr-baseAddr)/(128*1024);
	endSector = (endAddr-baseAddr-1)/(128*1024);

	chMtxLock(&flashMutex);

	// disable all IRQs
	asm volatile ("cpsid   i" : : : "memory");

	// unlock flash
	*pBank->KEYR = 0x45670123;
	*pBank->KEYR = 0xCDEF89AB;

	// enable IRQs again
	asm volatile ("cpsie   i" : : : "memory");

	// Clear pending flags (if any)
	*pBank->CCR = 0x0FEF0000;

	for(uint32_t sector = startSector; sector <= endSector; sector++)
	{
		volatile uint32_t* pSectorBegin = (volatile uint32_t*)(baseAddr + sector*128*1024);
		volatile uint32_t* pSectorEnd = (volatile uint32_t*)(baseAddr + (sector+1)*128*1024 - 4);

		uint8_t eraseRequired = 0;

		for(size_t i = 0; i < 256; i++)
		{
			if(*pSectorBegin++ != 0xFFFFFFFF)
			{
				eraseRequired = 1;
				break;
			}
		}

		for(size_t i = 0; i < 256; i++)
		{
			if(*pSectorEnd-- != 0xFFFFFFFF)
			{
				eraseRequired = 1;
				break;
			}
		}

		if(!eraseRequired)
			continue;

		result = eraseSector(pBank, sector);
		if(result)
			break;

		SCB_InvalidateDCache_by_Addr((void*)pSectorBegin, 128*1024);
	}

	*pBank->CR |= FLASH_CR_LOCK;

	chMtxUnlock(&flashMutex);

	return result;
}

// addr must be 256 bit aligned (32 Byte), length must be multiple of 8 (32 Byte)
int16_t FlashProgram(uint32_t addr, const uint32_t* pData, uint32_t length)
{
	int16_t result = 0;
	const uint32_t numBytes = length*sizeof(uint32_t);
	const uint32_t numDWords = numBytes/8;

	if(memcmp((void*)addr, pData, numBytes) == 0)
		return 0;

	if((length % 8) != 0)
		return ERROR_FLASH_INVALID_OPERATION;

	const FlashBank* pBank;
	if(addr < 0x08100000) // flash bank 1?
		pBank = &flashBank[0];
	else
		pBank = &flashBank[1];

	volatile uint64_t* pDst = (volatile uint64_t*)addr;
	volatile const uint64_t* pSrc = (volatile const uint64_t*)pData;

	chMtxLock(&flashMutex);

	// disable all IRQs
	asm volatile ("cpsid   i" : : : "memory");

	// unlock flash
	*pBank->KEYR = 0x45670123;
	*pBank->KEYR = 0xCDEF89AB;

	// enable IRQs again
	asm volatile ("cpsie   i" : : : "memory");

	waitForLastOperation(pBank, 0);

	// Clear pending flags (if any)
	*pBank->CCR = 0x0FEF0000;

	*pBank->CR |= FLASH_CR_PSIZE_1 | FLASH_CR_PSIZE_0;
	*pBank->CR |= FLASH_CR_PG;

	for(uint32_t i = 0; i < numDWords; i += 4)
	{
	    // if the previous operation is completed, proceed to program the new data
	    for(uint32_t j = 0; j < 4; j++)
	    {
	    	static volatile uint64_t __attribute__((aligned(32))) src;
	    	memcpy((void*)&src, (const void*)pSrc, sizeof(uint64_t)); // hack to workaround unaligned issues
	    	pSrc++;
	    	*pDst++ = src;
	    }

	    __DSB();

	    // Wait for last operation to be completed
	    result = waitForLastOperation(pBank, 1);
	    if(result)
	    	break;
	}

	// if the program operation is completed, disable the PG Bit
	*pBank->CR &= (~FLASH_CR_PG);
	*pBank->CR |= FLASH_CR_LOCK;

	chMtxUnlock(&flashMutex);

	return result;
}
