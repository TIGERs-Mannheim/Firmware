/*
 * crc.h
 *
 *  Created on: 06.06.2013
 *      Author: AndreR
 */

#pragma once

#include <stdint.h>

#define CRC32_MAGIC_NUMBER 0x2144DF1C	// result of CRC verification must be this value

uint32_t CRC32StartChecksum();
uint32_t CRC32FeedChecksum(uint32_t crc, const void* _pData, uint32_t dataLength);
uint32_t CRC32StopChecksum(uint32_t crc);

uint32_t CRC32CalcChecksum(const void* pData, uint32_t dataLength);
uint32_t CRC32VerifyChecksum(const void* pData, uint32_t dataLength, uint32_t crc);
