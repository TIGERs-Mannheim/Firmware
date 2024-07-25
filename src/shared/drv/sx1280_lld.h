/*
 * Low-Level SX1280 driver.
 *
 * Does not use any OS functions.
 */

#pragma once

#include "hal/spi_lld.h"
#include "hal/timer_simple_lld.h"

typedef void(*SX1280LLDCmdDoneFunc)(void*);

typedef struct _SX1280LLDCmdWriteBuffer
{
	uint8_t offset;
	uint8_t size;
	uint8_t data[256];
} SX1280LLDCmdWriteBuffer;

typedef SX1280LLDCmdWriteBuffer SX1280LLDCmdReadBuffer;

typedef struct _SX1280LLDCmdWriteRegister
{
	uint16_t address;
	uint8_t size;
	uint8_t data[256];
} SX1280LLDCmdWriteRegister;

typedef SX1280LLDCmdWriteRegister SX1280LLDCmdReadRegister;

typedef struct _SX1280LLDCmdGetStatus
{
	uint8_t status;
} SX1280LLDCmdGetStatus;

typedef struct _SX1280LLDCmdSetSleep
{
	uint8_t sleepConfig;
} SX1280LLDCmdSetSleep;

typedef struct _SX1280LLDCmdSetStandby
{
	uint8_t standbyConfig;
} SX1280LLDCmdSetStandby;

typedef struct _SX1280LLDCmdSetTx
{
	uint8_t periodBase;
	uint16_t periodCount;
} SX1280LLDCmdSetTx;

typedef SX1280LLDCmdSetTx SX1280LLDCmdSetRx;

typedef struct _SX1280LLDCmdSetPacketType
{
	uint8_t packetType;
} SX1280LLDCmdSetPacketType;

typedef struct _SX1280LLDCmdSetRfFrequency
{
	uint32_t frequency_Hz;
} SX1280LLDCmdSetRfFrequency;

typedef struct _SX1280LLDCmdSetTxParams
{
	uint8_t power;
	uint8_t rampTime;
} SX1280LLDCmdSetTxParams;

typedef struct _SX1280LLDCmdSetBufferBaseAddress
{
	uint8_t txBaseAddress;
	uint8_t rxBaseAddress;
} SX1280LLDCmdSetBufferBaseAddress;

typedef struct _SX1280LLDCmdSetModulationParams
{
	uint8_t params[3]; // Depends on packet type
} SX1280LLDCmdSetModulationParams;

typedef struct _SX1280LLDCmdSetPacketParams
{
	uint8_t params[7]; // Depends on packet type
} SX1280LLDCmdSetPacketParams;

typedef struct _SX1280LLDCmdGetRxBufferStatus
{
	uint8_t payloadLength;
	uint8_t startBufferPointer;
} SX1280LLDCmdGetRxBufferStatus;

typedef struct _SX1280LLDCmdGetPacketStatus
{
	uint8_t packetStatus[5]; // Depends on packet type
} SX1280LLDCmdGetPacketStatus;

typedef struct _SX1280LLDCmdSetDioIrqParams
{
	uint16_t irqMask;
	uint16_t dioMask[3];
} SX1280LLDCmdSetDioIrqParams;

typedef struct _SX1280LLDCmdGetIrqStatus
{
	uint16_t irqStatus;
} SX1280LLDCmdGetIrqStatus;

typedef struct _SX1280LLDCmdClearIrqStatus
{
	uint16_t irqMask;
} SX1280LLDCmdClearIrqStatus;

typedef struct _SX1280LLDCmdSetAutoFs
{
	uint8_t enable;
} SX1280LLDCmdSetAutoFs;

typedef SX1280LLDCmdSetAutoFs SX1280LLDCmdSetLongPreamble;

typedef enum _SX1280LLDCmdState
{
	SX1280LLD_CMD_STATE_DETACHED,
	SX1280LLD_CMD_STATE_QUEUED,
	SX1280LLD_CMD_STATE_ACTIVE,
	SX1280LLD_CMD_STATE_DONE,
} SX1280LLDCmdState;

typedef enum _SX1280LLDCmdResult
{
	SX1280LLD_CMD_RESULT_OK,
	SX1280LLD_CMD_RESULT_FAIL,
	SX1280LLD_CMD_RESULT_TIMEOUT,
} SX1280LLDCmdResult;

typedef enum _SX1280LLDDio
{
	SX1280LLD_DIO_1 = 0x01,
	SX1280LLD_DIO_2 = 0x02,
	SX1280LLD_DIO_3 = 0x04,
	SX1280LLD_DIO_ALL = 0x07,
} SX1280LLDDio;

typedef struct _SX1280LLDCmdWaitDio
{
	SX1280LLDDio mask;
} SX1280LLDCmdWaitDio;

typedef SX1280LLDCmdWaitDio SX1280LLDCmdClearDio;

typedef struct _SX1280LLDPacketStatus
{
	int32_t rssi_mdBm;

	union
	{
		struct
		{
			int32_t snr_mdB;
		} lora;

		struct
		{
			struct
			{
				uint8_t syncError; // only with sync address detection enabled
				uint8_t lengthError; // only with dynamic payload length
				uint8_t crcError; // only with CRC enabled
				uint8_t headerReceived; // received header of dynamic payload length packet
				uint8_t packetReceived; // reception complete
				uint8_t syncCode; // Detected sync address, if enabled
			} rx;

			struct
			{
				uint8_t pktSent;
			} tx;

			uint8_t abortError;
			uint8_t packetCtrlBusy;
		} flrc;

		struct
		{
			struct
			{
				uint8_t syncError; // only with sync address detection enabled
				uint8_t lengthError; // only with dynamic payload length
				uint8_t crcError; // only with CRC enabled
				uint8_t headerReceived; // received header of dynamic payload length packet
				uint8_t packetReceived; // reception complete
				uint8_t syncCode; // Detected sync address, if enabled
			} rx;

			uint8_t abortError;
			uint8_t packetCtrlBusy;
		} gfsk;
	};
} SX1280LLDPacketStatus;

typedef struct _SX1280LLDCmdTrace
{
	uint8_t cmd;
	SX1280LLDCmdResult result;
	uint32_t tQueued_us;
	uint32_t timeQueued_us;
	uint32_t timeTransfer_us;
	uint32_t timeBusy_us;
} SX1280LLDCmdTrace;

typedef struct _SX1280LLDCmd
{
	// User data
	uint8_t cmd;
	uint16_t timeout_us;

	union
	{
		SX1280LLDCmdGetStatus getStatus;
		SX1280LLDCmdWriteRegister writeRegister;
		SX1280LLDCmdReadRegister readRegister;
		SX1280LLDCmdWriteBuffer writeBuffer;
		SX1280LLDCmdReadBuffer readBuffer;
		SX1280LLDCmdSetSleep setSleep;
		SX1280LLDCmdSetStandby setStandby;
		SX1280LLDCmdSetTx setTx;
		SX1280LLDCmdSetRx setRx;
		SX1280LLDCmdSetPacketType setPacketType;
		SX1280LLDCmdSetRfFrequency setRfFrequency;
		SX1280LLDCmdSetTxParams setTxParams;
		SX1280LLDCmdSetBufferBaseAddress setBufferBaseAddress;
		SX1280LLDCmdSetModulationParams setModulationParams;
		SX1280LLDCmdSetPacketParams setPacketParams;
		SX1280LLDCmdGetRxBufferStatus getRxBufferStatus;
		SX1280LLDCmdGetPacketStatus getPacketStatus;
		SX1280LLDCmdSetDioIrqParams setDioIrqParams;
		SX1280LLDCmdGetIrqStatus getIrqStatus;
		SX1280LLDCmdClearIrqStatus clearIrqStatus;
		SX1280LLDCmdSetAutoFs setAutoFs;
		SX1280LLDCmdSetLongPreamble setLongPreamble;
		SX1280LLDCmdClearDio clearDio;
		SX1280LLDCmdWaitDio waitDio;
	};

	// Internal data
	volatile SX1280LLDCmdState state;
	volatile SX1280LLDCmdResult result;

	uint32_t tQueued_us;

	uint32_t timeQueued_us; // Time from QUEUED to ACTIVE
	uint32_t timeTransfer_us; // SPI transfer time
	uint32_t timeBusy_us; // Time from transfer complete to busy pin low or timeout

	SX1280LLDCmdTrace* volatile pTrace;
} SX1280LLDCmd;

typedef struct _SX1280LLD
{
    SPILLD* pSpi;

    uint8_t* pTx;
    uint8_t* pRx;

    GPIOPin csPin;
    uint32_t busyPinMask;
    uint32_t dioPinMasks[3];

    TimerSimpleLLD* pTimer1us;

    IRQn_Type highPrioIRQn;

    uint8_t lastStatus;

	volatile uint32_t tCmdTransferStart_us;
	volatile uint32_t tCmdEnd_us;

    volatile uint8_t cmdTimedOut;
    volatile uint8_t cmdDone;

    SX1280LLDCmd* volatile pActiveCmd;

    SX1280LLDCmdDoneFunc cmdDoneCallback;
    void* pUser;
} SX1280LLD;

typedef struct _SX1280LLDData
{
	SPILLD* pSpi;
	GPIOPin csPin;
	uint32_t prescaler;

	// busy pin EXTI must be configured by caller
	GPIOPin busyPin;

	GPIOPin dioPins[3];

	TimerSimpleLLD* pTimer1us;

	IRQn_Type highPrioIRQn;
} SX1280LLDData;

void SX1280LLDInit(SX1280LLD* pSX, SX1280LLDData* pData);
void SX1280LLDHighPrioIRQ(SX1280LLD* pSX);
void SX1280LLDBusyIRQ(SX1280LLD* pSX);
void SX1280LLDDioIRQ(SX1280LLD* pSX);
void SX1280LLDEnqueueCmd(SX1280LLD* pSX, SX1280LLDCmd* pCmd);
uint8_t SX1280LLDIsCmdDone(SX1280LLDCmd* pCmd);
const char* SX1280LLDGetCmdName(uint8_t cmd);
void SX1280LLDParsePacketStatus(const SX1280LLDCmdGetPacketStatus* pCmd, uint8_t packetType, SX1280LLDPacketStatus* pStatus);
