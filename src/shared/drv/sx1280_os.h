#pragma once

#include "sx1280_lld.h"
#include "fem_interface.h"
#include "ch.h"

typedef struct _SX1280OSSettingsGFSK
{
	// Modulation Params
	uint8_t bitrateAndBandwidth; // One of GFSK_BR_...
	uint8_t modulationIndex;
	uint8_t modulationShaping;

	// Packet Params
	uint8_t preambleLength;
	uint8_t syncWordLength;
	uint8_t syncWordMatch;
	uint8_t crcLength; // 0..2
	uint8_t enableWhitening;

	// Sync Params
	uint32_t syncWords[3];
	uint8_t syncWordTolerance;
} SX1280OSSettingsGFSK;

typedef struct _SX1280OSSettingsFLRC
{
	// Modulation Params
	uint8_t bitrateAndBandwidth;
	uint8_t codingRate;
	uint8_t modulationShaping;

	// Packet Params
	uint8_t preambleLength;
	uint8_t syncWordLength;
	uint8_t syncWordMatch;
	uint8_t crcLength;
	uint8_t whitening;

	// Sync Params
	uint32_t syncWords[3];
	uint8_t syncWordTolerance;
} SX1280OSSettingsFLRC;

typedef struct _SX1280OSSettingsLoRa
{
	// Modulation Params
	uint8_t spreadingFactor;
	uint8_t bandwidth;
	uint8_t codingRate;

	// Packet Params
	uint8_t preambleLength;
	uint8_t crcEnable;
	uint8_t invertIq;
} SX1280OSSettingsLoRa;

typedef struct _SX1280OSSettingsCommon
{
	uint32_t frequency; // in [Hz]
	uint8_t txPower; // 0 - 31 => -18 - 13dBm (be very careful not to exceed PA input limits!)
	uint8_t paRampTime;
	uint8_t highSensitivityMode;
	uint8_t fixedLength;
	uint8_t payloadLength;
} SX1280OSSettingsCommon;

typedef struct _SX1280OSPacketStatus
{
	uint8_t rssiSync;
	uint8_t errors; // N/A in LoRa Mode
	uint8_t status; // N/A in LoRa Mode
	uint8_t sync; // N/A in LoRa Mode
	int8_t snr; // LoRa only
	uint8_t rfu; // reserved for future use
} SX1280OSPacketStatus;

typedef struct _SX1280OSRxResult
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
} SX1280OSRxResult;

typedef enum _SX1280OSState
{
    SX1280OS_STATE_OFF,
    SX1280OS_STATE_RX,
    SX1280OS_STATE_TX,
} SX1280OSState;

typedef struct _SX1280OS
{
	SX1280LLD* pDrv;
	FEMInterface* pFem; // TODO: not used yet

	IRQn_Type lowPrioIRQn;

	binary_semaphore_t transferDoneSem;
	mutex_t dataMutex;

	SX1280LLDCmd setup;

	SX1280LLDCmd cmdWriteBuffer;
	SX1280LLDCmd cmdSetDioIrqParams;
	SX1280LLDCmd cmdClearDio;
	SX1280LLDCmd cmdSetRx;
	SX1280LLDCmd cmdWaitDio;
	SX1280LLDCmd cmdSetFs;
	SX1280LLDCmd cmdGetPacketStatusRx;
	SX1280LLDCmd cmdClearIrqStatus;
	SX1280LLDCmd cmdSetTx;
	SX1280LLDCmd cmdGetRxBufferStatus;
	SX1280LLDCmd cmdReadBuffer;

	uint8_t channel;
	uint8_t address;
} SX1280OS;

typedef struct _SX1280OSData
{
	SX1280LLD* pDrv;
	FEMInterface* pFem;

	IRQn_Type lowPrioIRQn;
} SX1280OSData;

// TODO: methods to create and chain commands

void SX1280OSLowPrioIRQ(SX1280OS* pSX);
void SX1280OSInit(SX1280OS* pSX, SX1280OSData* pInit);
//int16_t SX1280OSSetupGFSK(SX1280OS* pSX, const SX1280OSSettingsCommon* pCommon, const SX1280OSSettingsGFSK* pSettingsGFSK);
int16_t SX1280OSSetupFLRC(SX1280OS* pSX, const SX1280OSSettingsCommon* pCommon, const SX1280OSSettingsFLRC* pSettingsFLRC);
//int16_t SX1280OSSetupLora(SX1280OS* pSX, const SX1280OSSettingsCommon* pCommon, const SX1280OSSettingsLoRa* pSettingsLora);

int16_t SX1280OSFastRxTxReceive(SX1280OS* pSX, uint8_t txBytes, const void* pTxData, uint32_t rxTimeout_us);
int16_t SX1280OSFastRxTxTransmit(SX1280OS* pSX, SX1280OSRxResult* pResult, uint8_t rxBytes, void* pRxData);
int16_t SX1280OSTransmit(SX1280OS* pSX, uint8_t numBytes, const void* pData);
int16_t SX1280OSReceive(SX1280OS* pSX, uint8_t maxBytes, void* pData, SX1280OSRxResult* pResult, uint16_t timeoutTicks);

int16_t SX1280OSSetChannel(SX1280OS* pSX, uint8_t channel);
int16_t SX1280OSSetAddress(SX1280OS* pSX, uint8_t address);
