#pragma once

#include "hal/uart_fifo.h"
#include "hal/gpio_pin.h"

#define ERROR_STBOOTLOADER_TIMEOUT 1
#define ERROR_STBOOTLOADER_NACK 2
#define ERROR_STBOOTLOADER_INVALID_ARGUMENT 3

#define STBOOTLOADER_BUFFER_SIZE 130

typedef struct _STBootloader
{
	uint8_t tx[STBOOTLOADER_BUFFER_SIZE];
	uint8_t rx[STBOOTLOADER_BUFFER_SIZE];
	uint8_t dataBlock[128];
	UARTFifo* pUART;
	GPIOPinInterface* pRstPin;
	GPIOPinInterface* pBootPin;

	// filled after a GET command
	uint8_t blVersion;
	uint8_t eraseCmd;

	// filled after Get ID command
	uint16_t productId;
} STBootloader;

typedef struct _STBootloaderFlashResult
{
	uint8_t updateRequired;
	uint8_t updated;
	uint32_t programSize;
	uint32_t numBlocks;
	uint32_t timeUs;
	uint32_t programCrc;
	uint16_t versionMajor;
	uint16_t versionMinor;
} STBootloaderFlashResult;

void STBootloaderInit(STBootloader* pBoot, UARTFifo* pUART, GPIOPinInterface* pRstPin, GPIOPinInterface* pBootPin);
int16_t STBootloaderStart(STBootloader* pBoot);
int16_t STBootloaderCmdGet(STBootloader* pBoot);
int16_t STBootloaderCmdGetId(STBootloader* pBoot);
int16_t STBootloaderCmdReadMemory(STBootloader* pBoot, uint32_t address, uint8_t* pData, uint8_t length);
int16_t STBootloaderCmdMassErase(STBootloader* pBoot);
int16_t STBootloaderCmdWriteMemory(STBootloader* pBoot, uint32_t address, const uint8_t* pData, uint8_t length);
int16_t STBootloaderCmdGo(STBootloader* pBoot, uint32_t address);
int16_t STBootloaderFlash(STBootloader* pBoot, const uint8_t* pProgram, uint32_t programSize,
		uint32_t targetAddress, uint32_t targetSize, uint8_t forceUpdate, uint32_t configWord, STBootloaderFlashResult* pResult);
