#pragma once

#include "hal/spi.h"

#define SX1280_MAX_PKT_LENGTH 100
#define SX1280_BUF_SIZE_RXTX 272

typedef struct _SX1280SettingsFLRC
{
	uint32_t frequency;
	uint32_t syncWord;
	uint8_t highSensitivityMode;
	uint8_t bitrate;
	uint8_t coderate;
	uint8_t syncWordEnable;
	uint8_t variableLength;
	uint8_t payloadLength;
	uint8_t crcSize;
	uint8_t txPower;
} SX1280SettingsFLRC;

typedef struct _SX1280PacketStatus
{
	uint8_t rfu;
	uint8_t rssiSync;
	uint8_t errors;
	uint8_t status;
	uint8_t sync;
} SX1280PacketStatus;

typedef struct _SX1280
{
	SPISlave spiSlave;

	uint8_t* pTx;
	uint8_t* pRx;

	GPIOPin busyPin;
	GPIOPin irqPin;

	uint8_t lastStatus;

	binary_semaphore_t busyIrqSem;
	binary_semaphore_t dataIrqSem;

	mutex_t dataMutex;

	uint8_t channel;
	uint8_t address;

	SX1280PacketStatus rxStatus;
} SX1280;

typedef struct _SX1280Data
{
	SPI* pSPI;
	GPIOPin csPin;
	uint32_t prescaler;

	// busy pin EXTI must be configured by caller
	GPIOPin busyPin;
	GPIOPin irqPin;
} SX1280Data;

typedef struct _SX1280RxResult
{
	uint8_t bytesReceived;

	uint8_t rssiSync;

	uint8_t syncCode;

	uint8_t syncError; // only with sync address detection enabled
	uint8_t lengthError; // only with dynamic payload length
	uint8_t crcError; // only with CRC enabled
	uint8_t abortError;
	uint8_t headerReceived; // received header of dynamic payload length packet
	uint8_t packetReceived; // reception complete
	uint8_t packetCtrlBusy;

	uint8_t rxPid; // only with dynamic payload length
	uint8_t noAck; // only with dynamic payload length
	uint8_t rxPidError; // only with dynamic payload length and rxpid_filter_enable
} SX1280RxResult;

void	SX1280Init(SX1280* pSX, SX1280Data* pData);
int16_t SX1280Setup(SX1280* pSX, const SX1280SettingsFLRC* pSettings);
void	SX1280BusyIRQ(SX1280* pSX);
void	SX1280DataIRQ(SX1280* pSX);

int16_t	SX1280GetStatus(SX1280* pSX, uint16_t* pStatus);
int16_t	SX1280GetFirmwareVersion(SX1280* pSX, uint16_t* pVersion);

int16_t SX1280Receive(SX1280* pSX, uint8_t maxBytes, void* pData, SX1280RxResult* pResult, uint16_t timeoutTicks);
int16_t	SX1280Transmit(SX1280* pSX, uint8_t numBytes, const void* pData);

int16_t SX1280FastRxTxReceive(SX1280* pSX, uint8_t txBytes, const void* pTxData, uint32_t rxTimeoutTicks);
int16_t SX1280FastRxTxTransmit(SX1280* pSX, SX1280RxResult* pResult, uint8_t rxBytes, void* pRxData);

int16_t SX1280SetRfFrequency(SX1280* pSX, uint32_t frequency);
int16_t SX1280SetChannel(SX1280* pSX, uint8_t channel);
int16_t SX1280SetAddress(SX1280* pSX, uint8_t address);
