/*
 * st_bootloader.c
 *
 *  Created on: 13.01.2019
 *      Author: AndreR
 */

#include "st_bootloader.h"
#include "util/log.h"
#include "util/sys_time.h"
#include "util/crc.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

void STBootloaderInit(STBootloader* pBoot, UARTFifo* pUART, StartBootloaderCallback cbStartBL, uint8_t userArg)
{
	pBoot->pUART = pUART;
	pBoot->cbStartBL = cbStartBL;
	pBoot->userArg = userArg;
}

static int16_t sendReceive(STBootloader* pBoot, uint8_t txSize, uint8_t expectedRxSize, uint8_t attempts)
{
	int16_t result = 0;
	for(uint8_t attempt = 0; attempt < attempts; attempt++)
	{
		UARTFifoFlush(pBoot->pUART);

		UARTFifoWrite(pBoot->pUART, pBoot->tx, txSize, MS2ST(20));

		if(UARTFifoReadWait(pBoot->pUART, MS2ST(50)) != MSG_OK)
		{
			result = ERROR_STBOOTLOADER_TIMEOUT;
			LogWarn("read timeout");
			continue;
		}

		uint16_t datasize = STBOOTLOADER_BUFFER_SIZE;
		uint16_t fifoStatus = 0;
		int16_t result = UARTFifoRead(pBoot->pUART, pBoot->rx, &datasize, &fifoStatus, 0);
		if(fifoStatus & UART_FIFO_STATUS_ERR_MASK)
		{
			LogWarnC("fifo error", fifoStatus);
			result = ERROR_STBOOTLOADER_TIMEOUT;
			continue;
		}
		if(result)
		{
			LogWarnC("read error", result);
			result = ERROR_STBOOTLOADER_TIMEOUT;
			chThdSleepMilliseconds(2);
			continue;
		}
		if(datasize == 0 && expectedRxSize != 0)
		{
			LogWarn("timeout");
			result = ERROR_STBOOTLOADER_TIMEOUT;
			continue;
		}
		if(datasize < expectedRxSize)
		{
			LogWarnC("short rx", ((uint32_t)datasize << 16) | expectedRxSize);
			result = ERROR_STBOOTLOADER_TIMEOUT;
			continue;
		}
		if(pBoot->rx[0] != 0x79)
		{
			LogWarn("NACK");
			result = ERROR_STBOOTLOADER_NACK;
			continue;
		}

		return 0;
	}

	return result;
}

int16_t STBootloaderStart(STBootloader* pBoot)
{
	int16_t result = 0;

	// autobaud magic byte
	pBoot->tx[0] = 0x7F;

	for(uint8_t attempt = 0; attempt < 5; attempt++)
	{
		UARTFifoFlush(pBoot->pUART);

		(pBoot->cbStartBL)(pBoot->userArg);

		// wait until remote bootloader is ready
		chThdSleepMilliseconds(5);

		result = sendReceive(pBoot, 1, 1, 1);
		if(result)
		{
			LogWarnC("BL start error", result);
			continue;
		}

		return 0;
	}

	return result;
}

int16_t STBootloaderCmdGet(STBootloader* pBoot)
{
	pBoot->tx[0] = 0x00;
	pBoot->tx[1] = 0xFF;
	int16_t result = sendReceive(pBoot, 2, 15, 5);
	if(result)
		return result;

	pBoot->blVersion = pBoot->rx[2];
	pBoot->eraseCmd = pBoot->rx[9];

	if(pBoot->rx[14] != 0x79)
		return ERROR_STBOOTLOADER_NACK;

	return 0;
}

int16_t STBootloaderCmdGetId(STBootloader* pBoot)
{
	pBoot->tx[0] = 0x02;
	pBoot->tx[1] = 0xFD;
	int16_t result = sendReceive(pBoot, 2, 5, 5);
	if(result)
		return result;

	if(pBoot->rx[4] != 0x79)
		return ERROR_STBOOTLOADER_NACK;

	pBoot->productId = ((uint16_t)pBoot->rx[2]) << 8 | pBoot->rx[3];

	return 0;
}

static uint8_t xorChecksum(uint8_t* pData, uint8_t length)
{
	uint8_t checksum = 0;
	for(uint8_t i = 0; i < length; i++)
		checksum ^= pData[i];

	return checksum;
}

int16_t STBootloaderCmdReadMemory(STBootloader* pBoot, uint32_t address, uint8_t* pData, uint8_t length)
{
	if(length > 128)
		return ERROR_STBOOTLOADER_INVALID_ARGUMENT;

	// send command
	pBoot->tx[0] = 0x11;
	pBoot->tx[1] = 0xEE;
	int16_t result = sendReceive(pBoot, 2, 1, 1);
	if(result != 0)
	{
		LogErrorC("Send command", result);
		return result;
	}

	// send address
	pBoot->tx[0] = (address >> 24) & 0xFF;
	pBoot->tx[1] = (address >> 16) & 0xFF;
	pBoot->tx[2] = (address >> 8) & 0xFF;
	pBoot->tx[3] = address & 0xFF;
	pBoot->tx[4] = xorChecksum(pBoot->tx, 4);
	result = sendReceive(pBoot, 5, 1, 1);
	if(result != 0)
	{
		LogErrorC("Send address", result);
		return result;
	}

	// send length
	pBoot->tx[0] = (length-1);
	pBoot->tx[1] = (length-1) ^ 0xFF;
	result = sendReceive(pBoot, 2, length+1, 1);
	if(result != 0)
	{
		LogErrorC("receive data", result);
		return result;
	}
	if(pBoot->rx[0] != 0x79)
	{
		LogError("Read memory NACK");
		return ERROR_STBOOTLOADER_NACK;
	}

	memcpy(pData, &pBoot->rx[1], length);

	return 0;
}

int16_t STBootloaderCmdMassErase(STBootloader* pBoot)
{
	pBoot->tx[0] = 0x44;
	pBoot->tx[1] = 0xBB;
	int16_t result = sendReceive(pBoot, 2, 1, 1);
	if(result != 0)
	{
		LogErrorC("Send command", result);
		return result;
	}

	pBoot->tx[0] = 0xFF;
	pBoot->tx[1] = 0xFF;
	pBoot->tx[2] = 0x00;
	result = sendReceive(pBoot, 3, 1, 1);
	if(result != 0)
	{
		LogErrorC("Send erase", result);
		return result;
	}

	return 0;
}

int16_t STBootloaderCmdWriteMemory(STBootloader* pBoot, uint32_t address, const uint8_t* pData, uint8_t length)
{
	if(length > 128)
		return ERROR_STBOOTLOADER_INVALID_ARGUMENT;

	pBoot->tx[0] = 0x31;
	pBoot->tx[1] = 0xCE;
	int16_t result = sendReceive(pBoot, 2, 1, 1);
	if(result != 0)
	{
		LogErrorC("Send command", result);
		return result;
	}

	pBoot->tx[0] = (address >> 24) & 0xFF;
	pBoot->tx[1] = (address >> 16) & 0xFF;
	pBoot->tx[2] = (address >> 8) & 0xFF;
	pBoot->tx[3] = address & 0xFF;
	pBoot->tx[4] = xorChecksum(pBoot->tx, 4);
	result = sendReceive(pBoot, 5, 1, 1);
	if(result != 0)
	{
		LogErrorC("Send address", result);
		return result;
	}

	pBoot->tx[0] = length-1;
	memcpy(&pBoot->tx[1], pData, length);
	pBoot->tx[length+1] = xorChecksum(pBoot->tx, length+1);
	result = sendReceive(pBoot, length+2, 1, 1);
	if(result != 0)
	{
		LogErrorC("Send data", result);
		return result;
	}

	return 0;
}

int16_t STBootloaderCmdGo(STBootloader* pBoot, uint32_t address)
{
	pBoot->tx[0] = 0x21;
	pBoot->tx[1] = 0xDE;
	int16_t result = sendReceive(pBoot, 2, 1, 1);
	if(result != 0)
	{
		LogErrorC("Send command", result);
		return result;
	}

	pBoot->tx[0] = (address >> 24) & 0xFF;
	pBoot->tx[1] = (address >> 16) & 0xFF;
	pBoot->tx[2] = (address >> 8) & 0xFF;
	pBoot->tx[3] = address & 0xFF;
	pBoot->tx[4] = xorChecksum(pBoot->tx, 4);
	result = sendReceive(pBoot, 5, 1, 1);
	if(result != 0)
	{
		LogErrorC("Send address", result);
		return result;
	}

	return 0;
}

int16_t STBootloaderFlash(STBootloader* pBoot, const uint8_t* pProgram, uint32_t maxLength,
		uint32_t targetAddress, uint8_t forceUpdate, uint32_t configWord, STBootloaderFlashResult* pResult)
{
	int16_t result;

	uint32_t start = SysTimeUSec();

	pResult->updateRequired = forceUpdate;
	pResult->updated = 0;
	pResult->programSize = 0;
	pResult->numBlocks = 0;
	pResult->timeUs = 0;
	pResult->versionMajor = 0;
	pResult->versionMinor = 0;

	result = STBootloaderStart(pBoot);
	if(result)
		return result;

	result = STBootloaderCmdGet(pBoot);
	if(result)
		return result;

	result = STBootloaderCmdGetId(pBoot);
	if(result)
		return result;

	const uint8_t* pProgramEnd = pProgram+maxLength;
	for(; pProgramEnd > pProgram; pProgramEnd--)
	{
		if(*pProgramEnd != 0xFF)
			break;
	}

	++pProgramEnd;

	uint32_t programSize = (uint32_t)(pProgramEnd-pProgram);
	uint32_t programSizeAligned = (programSize+127)/128*128;
	uint32_t numBlocks = programSizeAligned/128;

	pResult->programSize = programSize;
	pResult->numBlocks = numBlocks;

	const uint32_t programCRC = CRC32CalcChecksum(pProgram, programSizeAligned);
	pResult->programCrc = programCRC;

	// extract firmware version number from program code
	for(const uint8_t* pSearch = pProgram; pSearch < pProgramEnd; ++pSearch)
	{
		if(*pSearch != 'F')
			continue;

		if(memcmp(pSearch, "Firmware Version: ", 18) == 0)
		{
			pSearch += 18;

			const uint8_t* pMajorStart = pSearch;
			const uint8_t* pMinorStart = pMajorStart;
			const uint8_t* pMinorEnd = pMajorStart;

			while(pSearch < pProgramEnd)
			{
				if(*pSearch == '.')
					pMinorStart = pSearch + 1;

				if(*pSearch == '\r')
				{
					pMinorEnd = pSearch;
					break;
				}

				++pSearch;
			}

			if(pMinorStart == pMajorStart || pMinorEnd == pMajorStart)
				break; // firmware string delimiters not found

			pResult->versionMajor = strtoul((const char*)pMajorStart, 0, 10);
			pResult->versionMinor = strtoul((const char*)pMinorStart, 0, 10);

			break;
		}
	}

	if(!forceUpdate)
	{
		uint32_t targetCRC;
		result = STBootloaderCmdReadMemory(pBoot, targetAddress+maxLength-4, (uint8_t*)&targetCRC, 4);
		if(result)
			return result;

		pResult->updateRequired = programCRC != targetCRC;

		if(!pResult->updateRequired)
		{
			result = STBootloaderCmdGo(pBoot, targetAddress);
			if(result)
				return result;

			uint32_t end = SysTimeUSec();
			pResult->timeUs = end-start;

			return 0;
		}
	}

	result = STBootloaderCmdMassErase(pBoot);
	if(result)
		return result;

	for(uint32_t block = 0; block < numBlocks; block++)
	{
		result = STBootloaderCmdWriteMemory(pBoot, targetAddress+block*128, pProgram+block*128, 128);
		if(result)
			return result;
	}

	result = STBootloaderCmdWriteMemory(pBoot, targetAddress+maxLength-4, (uint8_t*)&programCRC, 4);
	if(result)
		return result;

	result = STBootloaderCmdWriteMemory(pBoot, targetAddress+maxLength-8, (uint8_t*)&configWord, 4);
	if(result)
		return result;

	result = STBootloaderCmdGo(pBoot, targetAddress);
	if(result)
		return result;

	uint32_t end = SysTimeUSec();
	pResult->updated = 1;
	pResult->timeUs = end-start;

	return 0;
}
