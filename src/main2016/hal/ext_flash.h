/*
 * ext_flash.h
 *
 *  Created on: 12.11.2015
 *      Author: AndreR
 */

#ifndef EXT_FLASH_H_
#define EXT_FLASH_H_

#include "ch.h"

typedef struct _ExtFlash
{
	binary_semaphore_t irqSem;
} ExtFlash;

extern ExtFlash extFlash;

void	ExtFlashInit();
uint8_t	ExtFlashGetStatus();
void	ExtFlashSetProtected(uint8_t protect);
void	ExtFlashSetWrite(uint8_t enable);
void	ExtFlashWaitUntilReady();
void	ExtFlashChipErase();
int16_t	ExtFlashRead(uint32_t flashAddr, void* pBuffer, uint32_t numBytes);
int16_t	ExtFlashWrite(uint32_t flashAddr, const void* pBuffer, uint32_t numBytes);
void	ExtFlashPrintPerformance();

#endif /* EXT_FLASH_H_ */
