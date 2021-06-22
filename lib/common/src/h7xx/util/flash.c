/*
 * flash.c
 *
 *  Created on: 08.01.2019
 *      Author: AndreR
 */

#include "util/flash.h"
#include "errors.h"
#include "stm32h743xx.h"
#include "ch.h"
#include "util/console.h"
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

static int16_t waitForLastOperation(const FlashBank* pBank)
{
	do
	{
		chThdYield();
	}
	while(*pBank->SR & (FLASH_SR_QW | FLASH_SR_BSY | FLASH_SR_WBNE));

	uint32_t status = *pBank->SR;

	*pBank->CCR = status;	// clear flags

	if(status & (FLASH_SR_PGSERR | FLASH_SR_OPERR))
		return ERROR_FLASH_INVALID_OPERATION;

	if(status & FLASH_SR_WRPERR)
		return ERROR_FLASH_WRITE_PROTECT;

	return 0;
}

static int16_t eraseSector(const FlashBank* pBank, uint32_t sector)
{
	int16_t result;

	// Wait for last operation to be completed
	result = waitForLastOperation(pBank);

	if(result == 0)
	{
		// if the previous operation is completed, proceed to erase the sector
		*pBank->CR &= FLASH_CR_PSIZE;
		*pBank->CR |= FLASH_CR_PSIZE_1 | FLASH_CR_PSIZE_0;	// double word size
		*pBank->CR &= ~FLASH_CR_SNB;
		*pBank->CR |= FLASH_CR_SER | (sector << 8);
		*pBank->CR |= FLASH_CR_START;

		// Wait for last operation to be completed
		result = waitForLastOperation(pBank);

		// if the erase operation is completed, disable the SER Bit
		*pBank->CR &= (~FLASH_CR_SER);
		*pBank->CR &= ~FLASH_CR_SNB;
	}

	SCB_CleanInvalidateDCache();

	return result;
}

int16_t FlashErase(uint32_t addr, uint32_t endAddr)
{
	int16_t result = 0;

	const FlashBank* pBank;
	uint32_t startSector;
	uint32_t endSector;

	if(addr < 0x08100000) // flash bank 1?
	{
		pBank = &flashBank[0];
		startSector = (addr-0x08000000)/(128*1024);
		endSector = (endAddr-0x08000000-1)/(128*1024);
	}
	else
	{
		pBank = &flashBank[1];
		startSector = (addr-0x08100000)/(128*1024);
		endSector = (endAddr-0x08100000-1)/(128*1024);
	}

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
		result = eraseSector(pBank, sector);
		if(result)
			break;
	}

	*pBank->CR |= FLASH_CR_LOCK;

	chMtxUnlock(&flashMutex);

	return result;
}

// addr must be 256 bit aligned (32 Byte), length must be multiple of 8 (32 Byte)
int16_t FlashProgram(uint32_t addr, uint32_t* pData, uint32_t length)
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
	volatile uint64_t* pSrc = (volatile uint64_t*)pData;

	chMtxLock(&flashMutex);

	// disable all IRQs
	asm volatile ("cpsid   i" : : : "memory");

	// unlock flash
	*pBank->KEYR = 0x45670123;
	*pBank->KEYR = 0xCDEF89AB;

	// enable IRQs again
	asm volatile ("cpsie   i" : : : "memory");

	waitForLastOperation(pBank);

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
	    	memcpy((void*)&src, (void*)pSrc, sizeof(uint64_t)); // hack to workaround unaligned issues
	    	pSrc++;
	    	*pDst++ = src;
	    }

	    __DSB();

	    // Wait for last operation to be completed
	    result = waitForLastOperation(pBank);
	    if(result)
	    	break;
	}

	// if the program operation is completed, disable the PG Bit
	*pBank->CR &= (~FLASH_CR_PG);
	*pBank->CR |= FLASH_CR_LOCK;

	chMtxUnlock(&flashMutex);

	return result;
}
