/*
 * PHY Statemachine Chart
 *
┌──►START
│     │ GetOp():NEXT
│     │
│     ├─────────────────────┬────────────────────┬─────────┬────────┐
│     │ [TX]                │ [RX]               │ [MOD]   │ [CFG]  │ [IDLE]   Guards:
│     │                     │ ->CIRQ             │ ->RREG  │ ->CFG  │ ->IDLE   RX:  Receive Operation
│     ├───────────┐         │                    │         │        │          TX:  Transmit Operation
│     │ [DELAY]   │ [else]  │                  WAIT_RREG   │        │          CFG: Set Configuration
│     │ ->IDLE    │         ▼                    │ ->WREG  │        │          MOD: Modify Register (RMW)
│     ▼           │       WAIT_CIRQRX            │         │        │          TO:  Timeout
│   INIT_TX◄──────┘         │ ->CDIO             │         │        │
│     │ ->WBUF              ▼                    │         │        │          Actions:
│     ▼                   WAIT_CDIORX            │         │        │          ->RREG: Read register
│   WAIT_WBUF               │ ->SRX              │         │        │          ->WREG: Write register
│     │ ->CIRQ              ▼                    │         │        │          ->CFG:  Send custom low-level command
│     ▼                   WAIT_SRX               │         │        │          ->WBUF: Write buffer
│   WAIT_CIRQTX             │ ->WDIO             │         │        │          ->CIRQ: Clear IRQ on SX1280
│     │ ->CDIO              ▼                    │         │        │          ->CDIO: Clear Digital IO (IRQ) on host
│     ▼                   WAIT_WDIORX──┐         │         │        │          ->STX:  Start transmitting
│   WAIT_CDIOTX             │ [TO=0]   │ [TO=1]  │         │        │          ->WDIO: Wait for Digital IO (IRQ)
│     │ ->STX               │ ->GPSR   │ ->SFS   │         │        │          ->IDLE: Do nothing for some time
│     ▼                     ▼          │         │         │        │          ->SRX:  Start receiving
│   WAIT_STX              WAIT_GPSR    │         │         │        │          ->GPSR: Get Packet Status
│     │ ->WDIO              │  ->RBUF  │         │         │        │          ->RBUF: Read buffer
│     ▼                     ▼          │         │         │        │          ->SFS:  Switch to FS Mode (abort receiving)
│   WAIT_WDIOTX           WAIT_RBUF    │         │         │        │
│     │                     │          │         │         │        │
│     │                     │◄─────────┘         │         │        │
│     │                     │                    │         │        │
│     ▼                     ▼                    ▼         ▼        ▼
└───FINAL◄──────────────────────────────────────────────────────────┘
*/

#include "radio_phy.h"
#include "sx1280_def.h"
#include "hal/sys_time.h"
#include <string.h>

static void lldDoneCallback(void* pUser)
{
	RadioPhy* pPhy = (RadioPhy*)pUser;
	uint8_t runAgain = 1;

	while(runAgain)
	{
		runAgain = 0;

		RadioPhyOp* pOp = &pPhy->operation;
		SX1280LLDCmd* pCmd = &pPhy->cmd;
		pCmd->cmd = CMD_NOP;

		switch(pPhy->state)
		{
			case RADIO_PHY_STATE_START:
			{
				pOp->type = RADIO_PHY_OP_IDLE;
				pOp->pTrace = 0;

				if(pPhy->opPrepareNextCallback)
					(*pPhy->opPrepareNextCallback)(pOp, pPhy->pUser);

				pOp->tStart_us = SysTimeUSec();
				if(pOp->pTrace)
					pOp->pTrace->cmdTraceUsed = 0;

				if(pOp->type == RADIO_PHY_OP_RX)
				{
					pCmd->cmd = CMD_CLR_IRQSTATUS;
					pCmd->timeout_us = pPhy->data.timeouts.default_us;
					pCmd->clearDio.mask = SX1280LLD_DIO_ALL;

					pPhy->state = RADIO_PHY_STATE_CIRQ_RX;
				}
				else if(pOp->type == RADIO_PHY_OP_TX)
				{
					if(pOp->tx.useTxDelay)
					{
						pCmd->cmd = CMD_TIMED_IDLE;
						pCmd->timeout_us = pPhy->data.timeouts.txDelay_us;
					}
					else
					{
						runAgain = 1;
					}

					pPhy->state = RADIO_PHY_STATE_INIT_TX;
				}
				else if(pOp->type == RADIO_PHY_OP_CFG)
				{
					pCmd = &pOp->cfg.cmd;

					if(pCmd->cmd == CMD_NOP)
						runAgain = 1;

					pPhy->state = RADIO_PHY_STATE_FINAL;
				}
				else if(pOp->type == RADIO_PHY_OP_MOD_REG)
				{
					pCmd->cmd = CMD_READ_REGISTER;
					pCmd->timeout_us = pPhy->data.timeouts.default_us;
					pCmd->readRegister.address = pOp->modReg.address;
					pCmd->readRegister.size = 1;
					pPhy->state = RADIO_PHY_STATE_RREG;
				}
				else
				{
					pCmd->cmd = CMD_TIMED_IDLE;
					pCmd->timeout_us = pPhy->data.timeouts.default_us;

					pPhy->state = RADIO_PHY_STATE_FINAL;
				}
			}
			break;
			case RADIO_PHY_STATE_RREG:
			{
				if(pCmd->result == SX1280LLD_CMD_RESULT_OK)
				{
					uint8_t data = pCmd->readRegister.data[0];
					data &= ~pOp->modReg.clearMask;
					data |= pOp->modReg.setMask;

					pCmd->cmd = CMD_WRITE_REGISTER;
					pCmd->timeout_us = pPhy->data.timeouts.default_us;
					pCmd->writeRegister.address = pOp->modReg.address;
					pCmd->writeRegister.size = 1;
					pCmd->writeRegister.data[0] = data;
				}
				else
				{
					runAgain = 1;
				}

				pPhy->state = RADIO_PHY_STATE_FINAL;
			}
			break;
			case RADIO_PHY_STATE_INIT_TX:
			{
				pCmd->cmd = CMD_WRITE_BUFFER;
				pCmd->timeout_us = pPhy->data.timeouts.bufferIo_us;
				pCmd->writeBuffer.offset = 128;
				pCmd->writeBuffer.size = pOp->tx.size;
				memcpy(pCmd->writeBuffer.data, pOp->tx.data, pOp->tx.size);

				pPhy->state = RADIO_PHY_STATE_WBUF;
			}
			break;
			case RADIO_PHY_STATE_WBUF:
			{
				pCmd->cmd = CMD_CLR_IRQSTATUS;
				pCmd->timeout_us = pPhy->data.timeouts.default_us;
				pCmd->clearIrqStatus.irqMask = IRQ_RADIO_ALL;

				pPhy->state = RADIO_PHY_STATE_CIRQ_TX;
			}
			break;
			case RADIO_PHY_STATE_CIRQ_TX:
			{
				pCmd->cmd = CMD_CLEAR_PENDING_DIO;
				pCmd->timeout_us = pPhy->data.timeouts.default_us;
				pCmd->clearDio.mask = SX1280LLD_DIO_ALL;

				pPhy->state = RADIO_PHY_STATE_CDIO_TX;
			}
			break;
			case RADIO_PHY_STATE_CDIO_TX:
			{
				if(pPhy->data.pFem)
					(*pPhy->data.pFem->setModeFunc)(pPhy->data.pFem->pInterfaceData, FEM_MODE_TX);

				pCmd->cmd = CMD_SET_TX;
				pCmd->timeout_us = pPhy->data.timeouts.modeChange_us;
				pCmd->setTx.periodBase = PERIOD_TICK_SIZE_1000_US;
				pCmd->setTx.periodCount = 0;

				pPhy->state = RADIO_PHY_STATE_STX;
			}
			break;
			case RADIO_PHY_STATE_STX:
			{
				pCmd->cmd = CMD_WAIT_DIO;
				pCmd->timeout_us = pPhy->data.timeouts.txWait_us;
				pCmd->waitDio.mask = SX1280LLD_DIO_1;

				pPhy->state = RADIO_PHY_STATE_WDIO_TX;
			}
			break;
			case RADIO_PHY_STATE_WDIO_TX:
			{
				if(pPhy->data.pFem)
					(*pPhy->data.pFem->setModeFunc)(pPhy->data.pFem->pInterfaceData, FEM_MODE_OFF);

				pPhy->state = RADIO_PHY_STATE_FINAL;
				runAgain = 1;
			}
			break;
			case RADIO_PHY_STATE_CIRQ_RX:
			{
				pCmd->cmd = CMD_CLEAR_PENDING_DIO;
				pCmd->timeout_us = pPhy->data.timeouts.default_us;
				pCmd->clearDio.mask = SX1280LLD_DIO_ALL;

				pPhy->state = RADIO_PHY_STATE_CDIO_RX;
			}
			break;
			case RADIO_PHY_STATE_CDIO_RX:
			{
				if(pPhy->data.pFem)
					(*pPhy->data.pFem->setModeFunc)(pPhy->data.pFem->pInterfaceData, FEM_MODE_RX);

				pCmd->cmd = CMD_SET_RX;
				pCmd->timeout_us = pPhy->data.timeouts.modeChange_us;
				pCmd->setRx.periodBase = PERIOD_TICK_SIZE_1000_US;
				pCmd->setRx.periodCount = 0;

				pPhy->state = RADIO_PHY_STATE_SRX;
			}
			break;
			case RADIO_PHY_STATE_SRX:
			{
				pCmd->cmd = CMD_WAIT_DIO;
				pCmd->timeout_us = pOp->rx.isLongWait ? pPhy->data.timeouts.rxWaitLong_us : pPhy->data.timeouts.rxWait_us;
				pCmd->waitDio.mask = SX1280LLD_DIO_1;

				pPhy->state = RADIO_PHY_STATE_WDIO_RX;
			}
			break;
			case RADIO_PHY_STATE_WDIO_RX:
			{
				if(pCmd->result == SX1280LLD_CMD_RESULT_OK)
				{
					pOp->rx.isPacketReceived = 1;
					pOp->rx.receiveTime_us = pCmd->tQueued_us + pCmd->timeQueued_us + pCmd->timeTransfer_us + pCmd->timeBusy_us;

					pCmd->cmd = CMD_GET_PACKETSTATUS;
					pCmd->timeout_us = pPhy->data.timeouts.default_us;

					pPhy->state = RADIO_PHY_STATE_GPSR;
				}
				else
				{
					pOp->rx.isPacketReceived = 0;

					pCmd->cmd = CMD_SET_FS;
					pCmd->timeout_us = pPhy->data.timeouts.modeChange_us;

					pPhy->state = RADIO_PHY_STATE_FINAL;
				}

				if(pPhy->data.pFem)
					(*pPhy->data.pFem->setModeFunc)(pPhy->data.pFem->pInterfaceData, FEM_MODE_OFF);
			}
			break;
			case RADIO_PHY_STATE_GPSR:
			{
				memcpy(&pOp->rx.rxStatus, &pCmd->getPacketStatus, sizeof(SX1280LLDCmdGetPacketStatus));

				pCmd->cmd = CMD_READ_BUFFER;
				pCmd->timeout_us = pPhy->data.timeouts.bufferIo_us;
				pCmd->readBuffer.offset = 0;
				pCmd->readBuffer.size = pOp->rx.size;

				pPhy->state = RADIO_PHY_STATE_RBUF;
			}
			break;
			case RADIO_PHY_STATE_RBUF:
			{
				memcpy(pOp->rx.data, pCmd->readBuffer.data, pOp->rx.size);

				pPhy->state = RADIO_PHY_STATE_FINAL;
				runAgain = 1;
			}
			break;
			case RADIO_PHY_STATE_FINAL:
			{
				if(pOp->pTrace)
				{
					RadioPhyOpTrace* pTrace = pOp->pTrace;
					pTrace->type = pOp->type;
					pTrace->tStart_us = pOp->tStart_us;
					pTrace->tEnd_us = SysTimeUSec();
					if(pOp->rx.isPacketReceived && pOp->type == RADIO_PHY_OP_RX)
						pTrace->tReceive_us = pOp->rx.receiveTime_us;
					else
						pTrace->tReceive_us = 0;
				}

				if(pPhy->opDoneCallback)
					(*pPhy->opDoneCallback)(pOp, pPhy->pUser);

				pPhy->state = RADIO_PHY_STATE_START;
				runAgain = 1;
			}
			break;
		}

		if(pCmd->cmd != CMD_NOP)
		{
			if(pOp->pTrace && pOp->pTrace->cmdTraceUsed < RADIO_PHY_LLD_TRACE_BUF_SIZE)
				pCmd->pTrace = &pOp->pTrace->cmdTrace[pOp->pTrace->cmdTraceUsed++];

			SX1280LLDEnqueueCmd(pPhy->data.pSX, pCmd);
		}
	}
}

void RadioPhyInit(RadioPhy* pPhy, RadioPhyData* pInit)
{
	pPhy->data = *pInit;
	pPhy->data.pSX->cmdDoneCallback = &lldDoneCallback;
	pPhy->data.pSX->pUser = pPhy;
	pPhy->state = RADIO_PHY_STATE_START;

	SX1280LLDEnqueueCmd(pPhy->data.pSX, 0); // Triggers start of state machine
}

const char* RadioPhyGetOpTypeName(RadioPhyOpType type)
{
	const char* pName = 0;

	switch(type)
	{
		case RADIO_PHY_OP_IDLE: pName = "IDL"; break;
		case RADIO_PHY_OP_RX: pName = "RX "; break;
		case RADIO_PHY_OP_TX: pName = "TX "; break;
		case RADIO_PHY_OP_CFG: pName = "CFG"; break;
		case RADIO_PHY_OP_MOD_REG: pName = "MOD"; break;
		default: pName = "Unknown"; break;
	}

	return pName;
}
