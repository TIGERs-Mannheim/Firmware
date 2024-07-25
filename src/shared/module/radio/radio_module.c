#include "radio_module.h"
#include "radio_pkt.h"
#include "sx1280_def.h"
#include "ch.h"
#include <string.h>
#include <stdio.h>

static void phyOpPrepareNextSetup(RadioPhyOp* pOp, void* pUser);

void RadioModuleInit(RadioModule* pModule, RadioModuleData* pInit)
{
	pModule->data = *pInit;

	if(pInit->pwrPin.pPort)
	{
		// set up power pin, controls low-noise LDO for radio and PA/LNA chips
		GPIOPinReset(pInit->pwrPin);

		GPIOInitData gpioInit;
		gpioInit.mode = GPIO_MODE_OUTPUT;
		gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
		gpioInit.ospeed = GPIO_OSPEED_2MHZ;
		gpioInit.pupd = GPIO_PUPD_NONE;
		GPIOInit(pInit->pwrPin.pPort, pInit->pwrPin.pin, &gpioInit);

		// turn off and deplete capacitors to force reset on SX1280
		chThdSleepMilliseconds(50);

		GPIOPinSet(pInit->pwrPin);
		chThdSleepMilliseconds(10);
	}

	pModule->data.pPhy->pUser = pModule;
	pModule->data.pPhy->opPrepareNextCallback = &phyOpPrepareNextSetup;

	// Wait until setup sequence is complete
	while(pModule->data.pPhy->opPrepareNextCallback)
	{
		chThdSleepMilliseconds(10);
	}
}

RadioModuleOpTrace* RadioModuleGetNextTrace(RadioModule* pRadio)
{
	RadioModuleOpTrace* pTrace = &pRadio->traces[pRadio->tracePos];
	pRadio->tracePos = (pRadio->tracePos + 1) % RADIO_MODULE_TRACE_BUF_SIZE;

	return pTrace;
}

void RadioModulePrintTrace(RadioModule* pRadio)
{
	static RadioModuleOpTrace traces[RADIO_MODULE_TRACE_BUF_SIZE];

	uint32_t tracePos = pRadio->tracePos;
	memcpy(traces, pRadio->traces, sizeof(traces));

	for(uint32_t i = 0; i < RADIO_MODULE_TRACE_BUF_SIZE; i++)
	{
		RadioModuleOpTrace* pSlot = &traces[(i+tracePos) % RADIO_MODULE_TRACE_BUF_SIZE];
		RadioModuleOpTrace* pPrevSlot = &traces[(i+tracePos+RADIO_MODULE_TRACE_BUF_SIZE-1) % RADIO_MODULE_TRACE_BUF_SIZE];
		RadioPhyOpTrace* pPhyOp = &pSlot->opTrace;

		uint32_t dSlotIdle_us = pSlot->opTrace.tStart_us - pPrevSlot->opTrace.tEnd_us;

		printf("\e[36mIDLE %u\e[0m\r\n", dSlotIdle_us);

		const char* pName = RadioPhyGetOpTypeName(pPhyOp->type);
		printf("+ \e[35m%s 0x%02hX                           % 10u    % 5u  % 10u\e[0m\r\n", pName,
				(uint16_t)(pSlot->id & (RADIO_HEADER_CLIENT_ID_MASK | RADIO_HEADER_BROADCAST)),
				pPhyOp->tStart_us, pPhyOp->tEnd_us - pPhyOp->tStart_us, pPhyOp->tReceive_us);
		printf("|   \e[90mCMD                           RES   tQueued    dExec   dQue  dXfer  dBusy\e[0m\r\n");

		for(uint32_t j = 0; j < pPhyOp->cmdTraceUsed; j++)
		{
			SX1280LLDCmdTrace* pEvt = &pPhyOp->cmdTrace[j];

			uint32_t dExec_us = pEvt->timeQueued_us + pEvt->timeTransfer_us + pEvt->timeBusy_us;
			const char* pName = SX1280LLDGetCmdName(pEvt->cmd);

			printf("|   %s 0x%02hX %1u  % 10u % 8d % 6d % 6d % 6d\r\n", pName, (uint16_t)pEvt->cmd,
					pEvt->result, pEvt->tQueued_us, dExec_us, pEvt->timeQueued_us, pEvt->timeTransfer_us, pEvt->timeBusy_us);
		}

		printf("+---\r\n");

		chThdSleepMilliseconds(10);
	}
}

static void phyOpPrepareNextSetup(RadioPhyOp* pOp, void* pUser)
{
	RadioModule* pRadio = (RadioModule*)pUser;

	pOp->type = RADIO_PHY_OP_CFG;

	RadioModuleOpTrace* pTrace = RadioModuleGetNextTrace(pRadio);
	pTrace->id = 0xFF;

	pOp->pTrace = (RadioPhyOpTrace*)pTrace;

	SX1280LLDCmd* pCmd = &pOp->cfg.cmd;
	pCmd->cmd = CMD_NOP;
	pCmd->timeout_us = 10000;

	switch(pRadio->setupStep)
	{
		case 0:
		{
			pCmd->cmd = CMD_GET_STATUS;
		}
		break;
		case 1:
		{
			pCmd->cmd = CMD_SET_STANDBY;
			pCmd->setStandby.standbyConfig = 0;
		}
		break;
		case 2:
		{
			pCmd->cmd = CMD_SET_STANDBY;
			pCmd->setStandby.standbyConfig = 1;
		}
		break;
		case 3:
		{
			pCmd->cmd = CMD_SET_PACKETTYPE;
			pCmd->setPacketType.packetType = pRadio->data.packetType;
		}
		break;
		case 4:
		{
			pCmd->cmd = CMD_SET_RFFREQUENCY;
			pCmd->setRfFrequency.frequency_Hz = pRadio->data.settingsCommon.frequency;
		}
		break;
		case 5:
		{
			pCmd->cmd = CMD_SET_BUFFERBASEADDRESS;
			pCmd->setBufferBaseAddress.rxBaseAddress = 0;
			pCmd->setBufferBaseAddress.txBaseAddress = 128;
		}
		break;
		case 6:
		{
			pOp->type = RADIO_PHY_OP_MOD_REG;
			pOp->modReg.address = 0x0891;

			if(pRadio->data.settingsCommon.highSensitivityMode)
			{
				pOp->modReg.clearMask = 0;
				pOp->modReg.setMask = 0xC0;
			}
			else
			{
				pOp->modReg.clearMask = 0xC0;
				pOp->modReg.setMask = 0;
			}
		}
		break;
		case 7:
		{
			pCmd->cmd = CMD_SET_MODULATIONPARAMS;

			switch(pRadio->data.packetType)
			{
				case PACKET_TYPE_GFSK:
				{
					pCmd->setModulationParams.params[0] = pRadio->data.settings.gfsk.bitrateAndBandwidth;
					pCmd->setModulationParams.params[1] = pRadio->data.settings.gfsk.modulationIndex;
					pCmd->setModulationParams.params[2] = pRadio->data.settings.gfsk.modulationShaping;
				}
				break;
				case PACKET_TYPE_FLRC:
				{
					pCmd->setModulationParams.params[0] = pRadio->data.settings.flrc.bitrateAndBandwidth;
					pCmd->setModulationParams.params[1] = pRadio->data.settings.flrc.codingRate;
					pCmd->setModulationParams.params[2] = pRadio->data.settings.flrc.modulationShaping;
				}
				break;
				default: break; // Unsupported packet type
			}
		}
		break;
		case 8:
		{
			pCmd->cmd = CMD_SET_PACKETPARAMS;

			switch(pRadio->data.packetType)
			{
				case PACKET_TYPE_GFSK:
				{
					pCmd->setPacketParams.params[0] = pRadio->data.settings.gfsk.preambleLength;
					pCmd->setPacketParams.params[1] = GFSK_SYNC_WORD_LEN_4_B;
					pCmd->setPacketParams.params[2] = RX_MATCH_SYNCWORD_1;
					pCmd->setPacketParams.params[3] = 0; // Fixed length mode
					pCmd->setPacketParams.params[4] = RADIO_AIR_PACKET_SIZE;
					pCmd->setPacketParams.params[5] = pRadio->data.settings.gfsk.crcLength;
					pCmd->setPacketParams.params[6] = pRadio->data.settings.gfsk.enableWhitening;
				}
				break;
				case PACKET_TYPE_FLRC:
				{
					pCmd->setPacketParams.params[0] = pRadio->data.settings.flrc.preambleLength;
					pCmd->setPacketParams.params[1] = FLRC_SYNCWORD_LENGTH_4_BYTE;
					pCmd->setPacketParams.params[2] = RX_MATCH_SYNCWORD_1;
					pCmd->setPacketParams.params[3] = 0; // Fixed length mode
					pCmd->setPacketParams.params[4] = RADIO_AIR_PACKET_SIZE;
					pCmd->setPacketParams.params[5] = pRadio->data.settings.flrc.crcLength;
					pCmd->setPacketParams.params[6] = pRadio->data.settings.flrc.enableWhitening;
				}
				break;
				default: break; // Unsupported packet type
			}
		}
		break;
		case 9:
		{
			uint32_t sync;

			switch(pRadio->data.packetType)
			{
				case PACKET_TYPE_GFSK: sync = __REV(pRadio->data.settings.gfsk.syncWord); break;
				case PACKET_TYPE_FLRC: sync = __REV(pRadio->data.settings.flrc.syncWord); break;
				default: sync = 0; break;
			}

			pCmd->cmd = CMD_WRITE_REGISTER;
			pCmd->writeRegister.address = 0x09CF;
			pCmd->writeRegister.size = 4;
			memcpy(pCmd->writeRegister.data, &sync, 4);
		}
		break;
		case 10:
		{
			pOp->type = RADIO_PHY_OP_MOD_REG;
			pOp->modReg.address = 0x09CD;
			pOp->modReg.clearMask = 0x0F;

			switch(pRadio->data.packetType)
			{
				case PACKET_TYPE_GFSK: pOp->modReg.setMask = pRadio->data.settings.gfsk.syncWordTolerance & 0x0F; break;
				case PACKET_TYPE_FLRC: pOp->modReg.setMask = pRadio->data.settings.flrc.syncWordTolerance & 0x0F; break;
				default: pOp->modReg.setMask = 0; break;
			}
		}
		break;
		case 11:
		{
			pCmd->cmd = CMD_SET_AUTOFS;
			pCmd->setAutoFs.enable = 1;
		}
		break;
		case 12:
		{
			pCmd->cmd = CMD_SET_TXPARAMS;
			pCmd->setTxParams.power = pRadio->data.settingsCommon.txPower;
			pCmd->setTxParams.rampTime = pRadio->data.settingsCommon.paRampTime;
		}
		break;
		case 13:
		{
			pCmd->cmd = CMD_SET_FS;
		}
		break;
		case 14:
		{
			pCmd->cmd = CMD_CLR_IRQSTATUS;
			pCmd->clearIrqStatus.irqMask = IRQ_RADIO_ALL;
		}
		break;
		case 15:
		{
			pCmd->cmd = CMD_SET_DIOIRQPARAMS;
			pCmd->setDioIrqParams.irqMask = IRQ_TX_DONE | IRQ_RX_DONE;
			pCmd->setDioIrqParams.dioMask[0] = IRQ_TX_DONE | IRQ_RX_DONE;
			pCmd->setDioIrqParams.dioMask[1] = 0;
			pCmd->setDioIrqParams.dioMask[2] = 0;
		}
		break;
		default:
		{
			pOp->type = RADIO_PHY_OP_IDLE;
			pRadio->data.pPhy->opPrepareNextCallback = 0;
			pRadio->data.pPhy->pUser = 0;
		}
		return;
	}

	pRadio->setupStep++;
}
