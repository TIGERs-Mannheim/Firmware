/*
 * flash.h
 *
 *  Created on: 12.06.2014
 *      Author: AndreR
 */

#ifndef FLASH_H_
#define FLASH_H_

#include <stdint.h>

int16_t FlashErase(uint32_t addr, uint32_t endAddr);
int16_t FlashProgram(uint32_t addr,  uint32_t* pData, uint32_t length);

#endif /* FLASH_H_ */
