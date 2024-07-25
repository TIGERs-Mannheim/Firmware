#pragma once

#include "radio_settings.h"
#include "ch.h"
#include <stddef.h>

typedef struct _RadioBufferStats
{
	uint32_t rxPacketsTotal;
	uint32_t rxBufferOverruns; // Lost packets due to unavailable RadioBufferEntry owned by radio to put application packet
	uint32_t rxBufferDataOverflows; // Received data exceeds array size
	uint32_t rxCobsDecodingErrors;
	uint32_t rxCrcErrors;
	uint32_t txBufferUnderrun; // No free TX buffer entry when attempting to enqueue application packet
	uint32_t rxAppBytes; // Application packet bytes processed after decoding
	uint32_t rxBytesGood; // Received bytes which are successfully decoded to application packets
	uint32_t rxBytesDiscarded;	// Bytes received but dropped due to packet loss, COBS decoding errors, or no free buffer entry
	uint32_t txPacketsTotal;
	uint32_t txBytesTotal;
} RadioBufferStats;

typedef struct _RadioBufferEntry
{
	uint8_t data[RADIO_APP_MAX_ENCODED_PACKET_SIZE_WITH_TRAILER];
	volatile size_t dataSize; // Size includes separating zero
	volatile size_t bytesProcessed;
	volatile uint8_t isOwnedByRadio;
} RadioBufferEntry;

typedef struct _RadioBuffer
{
	// A RadioBuffer is a simplex and can only be used for RX or TX
	RadioBufferEntry entries[RADIO_APP_MAX_BUFFERED_PACKETS];
	volatile size_t nextRadioEntry;
	size_t nextAppEntry;

	RadioBufferStats stats;

	mutex_t accessMtx;
	uint8_t isRxBuf;
} RadioBuffer;

void RadioBufInit(RadioBuffer* pBuf, uint8_t isRx);
uint8_t RadioBufferIsEmpty(RadioBuffer* pBuf);
uint32_t RadioBufPutRxAirPacket(RadioBuffer* pRxBuf, const uint8_t* pData, size_t dataSize);
uint32_t RadioBufGetTxAirPacket(RadioBuffer* pTxBuf, uint8_t* pData, size_t dataSize);
size_t RadioBufGetNextAppPacket(RadioBuffer* pRxBuf, uint8_t* pData, size_t dataSizeMax);
size_t RadioBufEnqueueAppPacket(RadioBuffer* pTxBuf, const uint8_t* pData, size_t dataSize);
