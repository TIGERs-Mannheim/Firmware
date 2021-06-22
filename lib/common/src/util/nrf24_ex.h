/*
 * nrf24_ex.h
 *
 *  Created on: 29.09.2013
 *      Author: AndreR
 *
 * NRF24 extended (v2) interface.
 */

#ifndef NRF24_EX_H_
#define NRF24_EX_H_

#ifndef STM32F30X

#ifdef STM32F7XX
#include "stm32f746xx.h"
#endif

#ifdef STM32F4XX
#include "stm32f407xx.h"
#endif

#include "util/spi_sync.h"
#include "util/fifo_lin.h"
#include "util/cobs.h"
#include "commands.h"

#include "ch.h"

#define NRF24_MAX_BOTS			24
#define NRF24_MAX_BOT_ID		(NRF24_MAX_BOTS-1)

#define NRF24_MAX_PKT_LENGTH	31	// low level packets via wireless link
#define NRF24_MAX_MSG_SIZE		68	// 64 user data + max. 4B extended header
#define NRF24_MAX_ENCODED_MSG_SIZE (COBSMaxStuffedSize(NRF24_MAX_MSG_SIZE))

#define NRF24_RXTX_BUF_SIZE (NRF24_MAX_PKT_LENGTH+NRF24_MAX_ENCODED_MSG_SIZE)

typedef struct PACKED _NRF24QueueStats // 26
{
	// LL traffic (NRF24 I/O)
	struct PACKED _nrfIO // 8
	{
		uint16_t txPackets;
		uint16_t txBytes;

		uint16_t rxPackets;
		uint16_t rxBytes;
	} nrfIO;

	// LL feedback, packet-based (lost RX due to sequence, tx ACK, rx MaxRT)
	struct PACKED _nrfFeedback // 6
	{
		uint16_t rxPacketsLost;	// identified by sequence gaps

		// only used by TX nRF24 (= base station)
		uint16_t txPacketsMaxRT;
		uint16_t txPacketsAcked;
	} nrfFeedback;

	// HL traffic (Queue I/O), measured at NRF processing, NOT at user input
	struct PACKED _queueIO // 8
	{
		uint16_t txPackets;
		uint16_t txBytes;

		uint16_t rxPackets;
		uint16_t rxBytes;
	} queueIO;

	// HL feedback, packet-based (queues full)
	struct PACKED _queueFeedback // 4
	{
		uint16_t txPacketsLost;
		uint16_t rxPacketsLost;
	} queueFeedback;
} NRF24QueueStats;

typedef void(*NRF24OutgoingCmdFunc)(uint8_t id, uint8_t* pData, uint32_t pktSize);

typedef struct _NRF24Queue
{
	FifoLin rxFifo;
	FifoLin txFifo;

	uint8_t rxBuf[NRF24_RXTX_BUF_SIZE];
	uint8_t txBuf[NRF24_RXTX_BUF_SIZE];

	uint32_t rxBufUsed;
	uint32_t txBufUsed;

	uint8_t txSeq;
	uint8_t rxSeq;

	NRF24QueueStats stats;

	uint8_t id;
	NRF24OutgoingCmdFunc pOutgoindCmdCb;
} NRF24Queue;

typedef struct _rwDelay
{
	uint32_t read2M;
	uint32_t write2M;
	uint32_t read1M;
	uint32_t write1M;
	uint32_t read250k;
	uint32_t write250k;
} NRF24ReadWriteDelay;

typedef struct _NRF24Ex
{
	GPIO_TypeDef* pCEPort; // one of GPIOx
	uint16_t cePin; // one of GPIO_Pin_x
	SPISync spi;
	SPISyncSlave slave;

	uint8_t enabled;
	uint8_t rx;
	uint8_t channel;
	uint8_t speed;
	uint8_t ackDelay;

	volatile uint8_t lastStatus;			// status response of any command

	NRF24Queue* pQueue;

	binary_semaphore_t irqSem;

	NRF24ReadWriteDelay rwDelay;
} NRF24Ex;

typedef struct _NRF24ExData
{
	// SPI exclusive config
	SPISyncData spiData;

	GPIO_TypeDef* pCEPort;
	uint16_t cePin;

	GPIO_TypeDef* pCSPort;
	uint16_t csPin;

	// RF
	uint8_t channel;
	uint8_t speed;

	uint8_t rx;
} NRF24ExData;

/**
 * Channel and address are initialized to defaults.
 * Use ChangeAddress and ChangeChannel after init.
 */
int16_t NRF24ExInit(NRF24Ex* pNRF24, NRF24ExData* pData);
int16_t NRF24ExSetup(NRF24Ex* pNRF24);

void 	NRF24ExIRQ(NRF24Ex* pNRF24);
int16_t NRF24ExWaitForIRQ(NRF24Ex* pNRF24, uint32_t waitTicks);

int16_t NRF24ExRxDownload(NRF24Ex* pNRF24);
int16_t NRF24ExRxUpload(NRF24Ex* pNRF24);
int16_t NRF24ExTxProcessOne(NRF24Ex* pNRF24);

int16_t NRF24ExChangeId(NRF24Ex* pNRF24, uint8_t id);
int16_t NRF24ExChangeChannel(NRF24Ex* pNRF24, uint8_t channel);
/**
 * @speed 0 = 250k, 1 = 1M, 2 = 2M
 */
int16_t NRF24ExChangeSpeed(NRF24Ex* pNRF24, uint8_t speed);
uint8_t NRF24ExGetSpeedId(NRF24Ex* pNRF24);
int16_t NRF24ExCheckChannel(NRF24Ex* pNRF24, uint8_t* pRPD);
int16_t NRF24ExRestart(NRF24Ex* pNRF24);
void	NRF24ExGetReadWriteDelay(NRF24Ex* pNRF24, uint32_t* pRead, uint32_t* pWrite);
void	NRF24ExSetReadWriteDelay(NRF24Ex* pNRF24, const NRF24ReadWriteDelay* pNew);

int16_t NRF24ExReadRegister(NRF24Ex* pNRF24, uint8_t reg, uint8_t* pValue);
int16_t NRF24ExWriteRegister(NRF24Ex* pNRF24, uint8_t reg, uint8_t value);
void	NRF24ExUseQueue(NRF24Ex* pNRF24, NRF24Queue* pQueue);

void	NRF24QueueInit(NRF24Queue* pQueue, uint8_t* pDataTx, uint16_t txSize, uint8_t* pDataRx, uint16_t rxSize);
void	NRF24QueueSetOutgoingCmdCallback(NRF24Queue* pQueue, NRF24OutgoingCmdFunc pFunc);
/**
 * Put data to tx queue.
 * Data is not transmitted immediately. Call NRF24Process to send data.
 */
int16_t NRF24QueueSend(NRF24Queue* pNRF24, uint8_t* pData, uint32_t dataSize);
int16_t NRF24QueueGetPacket(NRF24Queue* pNRF24, uint8_t* pDst, uint32_t dstSize, uint32_t* pPktSize);
void	NRF24QueueClear(NRF24Queue* pQueue);

#endif

#endif /* NRF24_H_ */
