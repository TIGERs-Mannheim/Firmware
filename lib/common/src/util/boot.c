/*
 * boot.c
 *
 *  Created on: 21.04.2013
 *      Author: AndreR
 */

#include "util/boot.h"
#include "util/flash.h"
#include "util/crc.h"

#ifdef STM32F30X
#include "stm32f30x.h"
#endif

#ifdef STM32F4XX
#include "stm32f407xx.h"
#endif

#ifdef STM32F7XX
#include "stm32f746xx.h"
#endif

#ifdef STM32H7XX
#include "stm32h743xx.h"
#endif

#define BOOT_MAGIC 0xA5E3C6F1

volatile uint32_t bootCode __attribute__ ((section (".boot")));
uint32_t applicationCrc32 = 0;

int16_t BootEraseProgramFlash()
{
	return FlashErase(BOOT_APP_ADDR, BOOT_APP_END);
}

int16_t BootProgramFlash(uint32_t* pData, uint32_t length, uint32_t offset)
{
	return FlashProgram(BOOT_APP_ADDR+offset, pData, length);
}

uint32_t BootGetApplicationCRC32()
{
	if(applicationCrc32 == 0)
		applicationCrc32 = CRC32CalcChecksum((const uint8_t*)BOOT_APP_ADDR, BOOT_APP_END - BOOT_APP_ADDR);

	return applicationCrc32;
}

uint32_t BootIsBootloaderSelected()
{
	if(bootCode == BOOT_MAGIC)
		return 1;

	return 0;
}

uint32_t BootGetBootCode()
{
	return bootCode;
}

void BootSetBootloaderSelected(uint8_t enable)
{
	if(enable)
		bootCode = BOOT_MAGIC;
	else
		bootCode = 0;
}

typedef void (*pFunction)(void);

void BootJumpToApplication()
{
	if(*((uint32_t*)BOOT_APP_ADDR) == 0xFFFFFFFF)
		return;

	volatile uint32_t JumpAddress;
	pFunction Jump_To_Application;

	JumpAddress = *(volatile uint32_t*) (BOOT_APP_ADDR + 4);
	Jump_To_Application = (pFunction) JumpAddress;
	__set_MSP(*(volatile uint32_t*) (BOOT_APP_ADDR));
	Jump_To_Application();
}

void BootReset()
{
	NVIC_SystemReset();
}
