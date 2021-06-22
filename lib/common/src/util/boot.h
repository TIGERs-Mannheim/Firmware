/*
 * boot.h
 *
 *  Created on: 21.04.2013
 *      Author: AndreR
 */

#ifndef BOOT_H_
#define BOOT_H_

#include <stdint.h>

#ifdef STM32F30X
#define BOOT_APP_OFFSET	0x00008800
#define BOOT_APP_ADDR	0x08008800
#define BOOT_APP_END	0x0801F800
#endif

#ifdef STM32F4XX
#define BOOT_APP_OFFSET	0x00020000
#define BOOT_APP_ADDR	0x08020000
#define BOOT_APP_END	0x08060000
#endif

#ifdef STM32F7XX
#define BOOT_APP_OFFSET	0x00020000
#define BOOT_APP_ADDR	0x08020000
#define BOOT_APP_END	0x08080000
#endif

#ifdef STM32H7XX
#define BOOT_APP_OFFSET	0x00100000
#define BOOT_APP_ADDR	0x08100000
#define BOOT_APP_END	0x08180000
#endif

uint32_t BootIsBootloaderSelected();
void BootSetBootloaderSelected(uint8_t enable);
uint32_t BootGetApplicationCRC32();
uint32_t BootGetBootCode();
void BootJumpToApplication();
void BootReset();

int16_t BootProgramFlash(uint32_t* pData, uint32_t length, uint32_t offset);
int16_t BootEraseProgramFlash();

#endif /* BOOT_H_ */
