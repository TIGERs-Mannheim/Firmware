#pragma once

#include "ch.h"
#include "net/inet.h"

#define ETH_RX_BUF_SIZE			128
#define ETH_TX_BUF_SIZE			128
#define ETH_RXBUFNB				64
#define ETH_TXBUFNB				64

#define ETH_MAX_PACKET_SIZE		1524	// ETH_HEADER + ETH_EXTRA + VLAN_TAG + MAX_ETH_PAYLOAD + ETH_CRC

#define ETH_EVENT_LINK_UP		EVENT_MASK(0)
#define ETH_EVENT_LINK_DOWN		EVENT_MASK(1)
#define ETH_EVENT_DATA_RECEIVED	EVENT_MASK(2)

typedef struct _EthStats
{
	// only successful rx/tx frames/bytes
	uint32_t txFramesProcessed;
	uint32_t rxFramesProcessed;
	uint32_t txBytesProcessed;
	uint32_t rxBytesProcessed;

	uint32_t txFrames;
	uint32_t rxFrames;

	uint32_t rxMissedApp;
	uint32_t rxMissedCtrl;
} EthStats;

typedef struct _LinkStatus
{
	uint8_t up;
	uint8_t fullDuplex;
	uint8_t speed100M;
	uint8_t MDI;
} LinkStatus;

typedef struct __attribute__((packed)) _ETHDMADesc
{
	volatile uint32_t Status;
	uint32_t ControlBufferSize;		// Control and Buffer1, Buffer2 lengths
	uint32_t Buffer1Addr;			// Buffer1 address pointer
	uint32_t Buffer2NextDescAddr;	// Buffer2 or next descriptor address pointer
	uint32_t ExtendedStatus;		// Extended status for PTP receive descriptor
	uint32_t _reserved;
	uint32_t TimeStampLow;			// Time Stamp Low value for transmit and receive
	uint32_t TimeStampHigh;			// Time Stamp High value for transmit and receive
} ETHDMADesc;

typedef struct _EthData
{
	ETH_TypeDef* pReg;
	uint32_t phyAddr;
} EthData;

typedef struct _Eth
{
	uint8_t rxBuffer[ETH_RXBUFNB*ETH_RX_BUF_SIZE] __attribute__((aligned(16)));
	uint8_t txBuffer[ETH_TXBUFNB*ETH_TX_BUF_SIZE] __attribute__((aligned(16)));
	ETHDMADesc rxDescTab[ETH_RXBUFNB] __attribute__((aligned(16)));
	ETHDMADesc txDescTab[ETH_TXBUFNB] __attribute__((aligned(16)));

	ETH_TypeDef* pReg;
	EthData data;

	ETHDMADesc* pCurTxDesc;
	ETHDMADesc* pCurRxDesc;

	EthStats stats;
	LinkStatus linkStatus;

	EthStats rates;

	mutex_t txMutex;
	event_source_t eventSource;

	THD_WORKING_AREA(waTask, 512);
	thread_t* pTask;
} Eth;

void        EthIRQ(Eth* pEth);
void        EthMiiIRQ(Eth* pEth);
void		EthInit(Eth* pEth, EthData* pInit, tprio_t prio);
int16_t		EthSendEthernetFrame(Eth* pEth, const uint8_t* pData, size_t length);
int16_t		EthGetEthernetFrame(Eth* pEth, uint8_t* pData, size_t dataSize, size_t* pBytesRead);
void		EthAllowMAC(Eth* pEth, uint8_t slot, const uint8_t* pMac, uint8_t enable, uint8_t saFiltering);
EthStats*	EthGetStats(Eth* pEth);
void 		EthPrintDebugOutput(Eth* pEth);
