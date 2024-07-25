#include "st_bootloader.h"
#include "util/log.h"
#include "hal/sys_time.h"
#include "util/crc.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

void STBootloaderInit(STBootloader* pBoot, UARTFifo* pUART, GPIOPinInterface* pRstPin, GPIOPinInterface* pBootPin)
{
	pBoot->pUART = pUART;
	pBoot->pRstPin = pRstPin;
	pBoot->pBootPin = pBootPin;
}

static int16_t sendReceive(STBootloader* pBoot, uint8_t txSize, uint8_t expectedRxSize, uint8_t attempts)
{
	int16_t result = 0;
	for(uint8_t attempt = 0; attempt < attempts; attempt++)
	{
		UARTFifoFlush(pBoot->pUART);

		UARTFifoWrite(pBoot->pUART, pBoot->tx, txSize, TIME_MS2I(20));

		if(UARTFifoReadWait(pBoot->pUART, TIME_MS2I(50)) != MSG_OK)
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

static void enterBootloader(STBootloader* pBoot)
{
	pBoot->pBootPin->write(pBoot->pBootPin, 1); // boot high
	pBoot->pRstPin->write(pBoot->pRstPin, 0); // reset low

	pBoot->pBootPin->configure(pBoot->pBootPin, 1); // boot output
	pBoot->pRstPin->configure(pBoot->pRstPin, 1); // reset output

	chThdSleepMilliseconds(50);

	pBoot->pRstPin->configure(pBoot->pRstPin, 0); // reset: input => goes high

	chThdSleepMilliseconds(50);

	pBoot->pBootPin->write(pBoot->pBootPin, 0); // boot low
}


int16_t STBootloaderStart(STBootloader* pBoot)
{
	int16_t result = 0;

	// autobaud magic byte
	pBoot->tx[0] = 0x7F;

	for(uint8_t attempt = 0; attempt < 5; attempt++)
	{
		UARTFifoFlush(pBoot->pUART);

		enterBootloader(pBoot);

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

int16_t STBootloaderFlash(STBootloader* pBoot, const uint8_t* pProgram, uint32_t programSize,
		uint32_t targetAddress, uint32_t targetSize, uint8_t forceUpdate, uint32_t configWord, STBootloaderFlashResult* pResult)
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

	if(targetSize < programSize + 8)
		return ERROR_STBOOTLOADER_INVALID_ARGUMENT;

	result = STBootloaderStart(pBoot);
	if(result)
		return result;

	result = STBootloaderCmdGet(pBoot);
	if(result)
		return result;

	result = STBootloaderCmdGetId(pBoot);
	if(result)
		return result;

	pResult->programSize = programSize;
	pResult->numBlocks = (programSize+127)/128;

	const uint32_t programCRC = CRC32CalcChecksum(pProgram, programSize);
	pResult->programCrc = programCRC;

	const uint8_t* pProgramEnd = pProgram + programSize;

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
		result = STBootloaderCmdReadMemory(pBoot, targetAddress+targetSize-4, (uint8_t*)&targetCRC, 4);
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

	uint32_t bytesLeft = programSize;

	while(bytesLeft > 0)
	{
		const uint32_t bytesWritten = programSize - bytesLeft;

		uint32_t writeSize = bytesLeft;
		if(writeSize > sizeof(pBoot->dataBlock))
			writeSize = sizeof(pBoot->dataBlock);
		else
			memset(pBoot->dataBlock, 0xFF, sizeof(pBoot->dataBlock));

		memcpy(pBoot->dataBlock, pProgram + bytesWritten, writeSize);

		result = STBootloaderCmdWriteMemory(pBoot, targetAddress + bytesWritten, pBoot->dataBlock, sizeof(pBoot->dataBlock));
		if(result)
			return result;

		bytesLeft -= writeSize;
	}

	result = STBootloaderCmdWriteMemory(pBoot, targetAddress+targetSize-4, (uint8_t*)&programCRC, 4);
	if(result)
		return result;

	result = STBootloaderCmdWriteMemory(pBoot, targetAddress+targetSize-8, (uint8_t*)&configWord, 4);
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
