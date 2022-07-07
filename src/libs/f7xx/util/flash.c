/*
 * flash.c
 *
 *  Created on: 12.06.2014
 *      Author: AndreR
 */

#include "util/flash.h"
#include "errors.h"
#include "stm32f746xx.h"
#include "ch.h"
#include "util/console.h"
#include "util/log.h"
#include <string.h>

static MUTEX_DECL(flashMutex);

static int16_t waitForLastOperation()
{
	for(uint16_t i = 0; i < 2160; i++)
		asm volatile("nop");

	do
	{
		asm volatile("nop");
	}
	while(FLASH->SR & FLASH_SR_BSY);

	uint32_t status = FLASH->SR;

	FLASH->SR = status;	// clear flags

//	LogWarnC("SR", status);

	if(status & (FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_ERSERR))
		return ERROR_FLASH_INVALID_OPERATION;

	if(status & FLASH_SR_WRPERR)
		return ERROR_FLASH_WRITE_PROTECT;

	return 0;
}

static int16_t eraseSector(uint32_t sector)
{
	int16_t result;

	// Wait for last operation to be completed
	result = waitForLastOperation();

	if(result == 0)
	{
		// if the previous operation is completed, proceed to erase the sector
		FLASH->CR &= FLASH_CR_PSIZE;
		FLASH->CR |= FLASH_CR_PSIZE_1;	// word size
		FLASH->CR &= ~FLASH_CR_SNB;
		FLASH->CR |= FLASH_CR_SER | (sector << 3);
		FLASH->CR |= FLASH_CR_STRT;

		// Wait for last operation to be completed
		result = waitForLastOperation();

		// if the erase operation is completed, disable the SER Bit
		FLASH->CR &= (~FLASH_CR_SER);
		FLASH->CR &= ~FLASH_CR_SNB;
	}

	SCB_CleanInvalidateDCache();

	return result;
}

int16_t FlashErase(uint32_t addr, uint32_t endAddr)
{
	int16_t result = 0;

	chMtxLock(&flashMutex);

	// disable all IRQs
	asm volatile ("cpsid   i" : : : "memory");

	// disable ART accelerator and flush it
	FLASH->ACR |= FLASH_ACR_ARTRST;
	FLASH->ACR &= ~FLASH_ACR_PRFTEN;

	// unlock flash
	FLASH->KEYR = 0x45670123;
	FLASH->KEYR = 0xCDEF89AB;

	// Clear pending flags (if any)
	FLASH->SR = 0xF3;

	do
	{
		if(addr < 0x08008000)
			result = eraseSector(0);	// 32k

		if(result)
			break;

		if(addr < 0x08010000 && endAddr >= 0x08008000)
			result = eraseSector(1);	// 32k

		if(result)
			break;

		if(addr < 0x08018000 && endAddr >= 0x08010000)
			result = eraseSector(2);	// 32k

		if(result)
			break;

		if(addr < 0x08020000 && endAddr >= 0x08018000)
			result = eraseSector(3);	// 32k

		if(result)
			break;

		if(addr < 0x08040000 && endAddr >= 0x08020000)
			result = eraseSector(4);	// 128k

		if(result)
			break;

		if(addr < 0x08080000 && endAddr > 0x08040000)
			result = eraseSector(5);	// 256k

		if(result)
			break;

		if(addr < 0x080C0000 && endAddr > 0x08080000)
			result = eraseSector(6);	// 256k

		if(result)
			break;

		if(addr < 0x08100000 && endAddr > 0x080C0000)
			result = eraseSector(7);	// 256k
	} while(0);

	FLASH->CR |= FLASH_CR_LOCK;

	FLASH->ACR |= FLASH_ACR_ARTEN;
	FLASH->ACR |= FLASH_ACR_PRFTEN;

	// enable IRQs again
	asm volatile ("cpsie   i" : : : "memory");

	chMtxUnlock(&flashMutex);

	return result;
}

int16_t FlashProgram(uint32_t addr,  uint32_t* pData, uint32_t length)
{
	int16_t result = 0;

	if(memcmp((void*)addr, pData, length*sizeof(uint32_t)) == 0)
		return 0;

	chMtxLock(&flashMutex);

	// disable all IRQs
	asm volatile ("cpsid   i" : : : "memory");

	// unlock flash
	FLASH->KEYR = 0x45670123;
	FLASH->KEYR = 0xCDEF89AB;

	waitForLastOperation();

	// Clear pending flags (if any)
	FLASH->SR = 0xF3;

	for(uint32_t i = 0; i < length; i++)
	{
	    // if the previous operation is completed, proceed to program the new data
	    FLASH->CR &= FLASH_CR_PSIZE;
	    FLASH->CR |= FLASH_CR_PSIZE_1;
	    FLASH->CR |= FLASH_CR_PG;

	    *(volatile uint32_t*)addr = pData[i];
		addr += 4;

		__DSB();

	    /* Wait for last operation to be completed */
	    result = waitForLastOperation();
	    if(result)
	    	break;

	    /* if the program operation is completed, disable the PG Bit */
	    FLASH->CR &= (~FLASH_CR_PG);
	}

	FLASH->CR |= FLASH_CR_LOCK;

	// enable IRQs again
	asm volatile ("cpsie   i" : : : "memory");

	chMtxUnlock(&flashMutex);

	return result;
}
