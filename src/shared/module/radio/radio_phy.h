#pragma once

#include "sx1280_lld.h"
#include "drv/fem_interface.h"

// Maximum packet size over-the-air. This is fixed, do not change.
#define RADIO_PHY_MAX_PACKET_SIZE 128

// Number of trace buffer entries available for low-level driver
#define RADIO_PHY_LLD_TRACE_BUF_SIZE 6

typedef enum RadioPhyState
{
	RADIO_PHY_STATE_START,

	RADIO_PHY_STATE_INIT_TX,
	RADIO_PHY_STATE_WBUF,
	RADIO_PHY_STATE_CIRQ_TX,
	RADIO_PHY_STATE_CDIO_TX,
	RADIO_PHY_STATE_STX,
	RADIO_PHY_STATE_WDIO_TX,

	RADIO_PHY_STATE_CIRQ_RX,
	RADIO_PHY_STATE_CDIO_RX,
	RADIO_PHY_STATE_SRX,
	RADIO_PHY_STATE_WDIO_RX,
	RADIO_PHY_STATE_GPSR,
	RADIO_PHY_STATE_RBUF,

	RADIO_PHY_STATE_RREG,

	RADIO_PHY_STATE_FINAL,
} RadioPhyState;

typedef enum RadioPhyOpType
{
	RADIO_PHY_OP_IDLE,
	RADIO_PHY_OP_RX,
	RADIO_PHY_OP_TX,
	RADIO_PHY_OP_CFG,
	RADIO_PHY_OP_MOD_REG,
} RadioPhyOpType;

typedef struct _RadioPhyOpTrace
{
	uint8_t type;
	uint32_t tStart_us;
	uint32_t tEnd_us;
	uint32_t tReceive_us;

	SX1280LLDCmdTrace cmdTrace[RADIO_PHY_LLD_TRACE_BUF_SIZE];
	volatile uint32_t cmdTraceUsed;
} RadioPhyOpTrace;

typedef struct _RadioPhyOp
{
	// User supplied
	RadioPhyOpType type;

	union
	{
		struct
		{
			uint8_t data[RADIO_PHY_MAX_PACKET_SIZE];
			uint8_t size;
			uint8_t isLongWait;
			uint8_t isPacketReceived;
			SX1280LLDCmdGetPacketStatus rxStatus;
			uint32_t receiveTime_us; // Time when RX DONE IRQ pin changes level
		} rx;

		struct
		{
			uint8_t data[RADIO_PHY_MAX_PACKET_SIZE];
			uint8_t size;
			uint8_t useTxDelay;
		} tx;

		struct
		{
			SX1280LLDCmd cmd;
		} cfg;

		struct
		{
			uint16_t address;
			uint8_t clearMask;
			uint8_t setMask;
		} modReg;
	};

	RadioPhyOpTrace* volatile pTrace;

	// Internal
	uint32_t tStart_us;
} RadioPhyOp;

typedef void(*RadioPhyOpDoneFunc)(const RadioPhyOp*, void*);
typedef void(*RadioPhyOpPrepareNextOpFunc)(RadioPhyOp*, void*);

typedef struct _RadioPhyTimeouts
{
	uint16_t default_us;
	uint16_t bufferIo_us;
	uint16_t modeChange_us;
	uint16_t txDelay_us;
	uint16_t rxWait_us;
	uint16_t rxWaitLong_us;
	uint16_t txWait_us;
} RadioPhyTimeouts;

typedef struct _RadioPhyData
{
	SX1280LLD* pSX;
	FEMInterface* pFem;
	RadioPhyTimeouts timeouts;
} RadioPhyData;

typedef struct _RadioPhy
{
	RadioPhyData data;

	RadioPhyState state;
	SX1280LLDCmd cmd;

	RadioPhyOpDoneFunc opDoneCallback;
	RadioPhyOpPrepareNextOpFunc opPrepareNextCallback;
	void* pUser;

	RadioPhyOp operation;
} RadioPhy;

void RadioPhyInit(RadioPhy* pPhy, RadioPhyData* pInit);
const char* RadioPhyGetOpTypeName(RadioPhyOpType type);
