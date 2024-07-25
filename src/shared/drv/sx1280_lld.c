#include "sx1280_lld.h"
#include "sx1280_def.h"
#include "errors.h"
#include "util/log.h"
#include "hal/sys_time.h"
#include <string.h>

static uint32_t prepareTxData(uint8_t* pTx, SX1280LLDCmd* pCmd);
static void processRxData(const uint8_t* pRx, SX1280LLDCmd* pCmd);

static uint32_t getDioPinMask(SX1280LLD* pSX, SX1280LLDDio dioMask)
{
	uint32_t pinMask = 0;

	if(dioMask & SX1280LLD_DIO_1)
		pinMask |= pSX->dioPinMasks[0];

	if(dioMask & SX1280LLD_DIO_2)
		pinMask |= pSX->dioPinMasks[1];

	if(dioMask & SX1280LLD_DIO_3)
		pinMask |= pSX->dioPinMasks[2];

	return pinMask;
}

// This must be the highest IRQ level (preferably 0)
void SX1280LLDHighPrioIRQ(SX1280LLD* pSX)
{
	// Process active command (if any)
	SX1280LLDCmd* pActiveCmd = pSX->pActiveCmd;
	if(pActiveCmd)
	{
		if(pActiveCmd->state == SX1280LLD_CMD_STATE_QUEUED)
		{
			// Start a new command
			uint32_t transferSize = prepareTxData(pSX->pTx, pActiveCmd);

			pSX->tCmdTransferStart_us = SysTimeUSec();
			pSX->cmdTimedOut = 0;
			pSX->cmdDone = 0;
			pActiveCmd->state = SX1280LLD_CMD_STATE_ACTIVE;

			if(pActiveCmd->cmd == CMD_TIMED_IDLE)
			{
				TimerSimpleStartPulse(pSX->pTimer1us, pActiveCmd->timeout_us);
			}
			else if(pActiveCmd->cmd == CMD_CLEAR_PENDING_DIO)
			{
				pSX->tCmdEnd_us = pSX->tCmdTransferStart_us;

				uint32_t pinMask = getDioPinMask(pSX, pActiveCmd->clearDio.mask);

#ifdef STM32H7XX
				EXTI_D1->PR1 = pinMask; // clear pending bits
#else
				EXTI->PR = pinMask; // clear pending bits
#endif

				pActiveCmd->result = SX1280LLD_CMD_RESULT_OK;
				pActiveCmd->state = SX1280LLD_CMD_STATE_DONE;
			}
			else if(pActiveCmd->cmd == CMD_WAIT_DIO)
			{
				uint32_t pinMask = getDioPinMask(pSX, pActiveCmd->waitDio.mask);

#ifdef STM32H7XX
				EXTI_D1->IMR1 |= pinMask; // Enable pin edge IRQs
#else
				EXTI->IMR |= pinMask; // Enable pin edge IRQs
#endif

				TimerSimpleStartPulse(pSX->pTimer1us, pActiveCmd->timeout_us);
			}
			else
			{
				int16_t result = SPILLDStartTransferSelect(pSX->pSpi, pSX->csPin, transferSize);
				if(result)
				{
					LogErrorC("SPILLDStartTransfer failed", result | ((uint32_t)pSX->pTx[0] << 16));
					pSX->tCmdEnd_us = SysTimeUSec();
					pActiveCmd->result = SX1280LLD_CMD_RESULT_FAIL;
					pActiveCmd->state = SX1280LLD_CMD_STATE_DONE;
				}
				else
				{
#ifdef STM32H7XX
					EXTI_D1->PR1 = pSX->busyPinMask; // clear busy pending bit (might be set by spurious falling edge)
					EXTI_D1->IMR1 |= pSX->busyPinMask; // Enable busy pin edge IRQ
#else
					EXTI->PR = pSX->busyPinMask; // clear busy pending bit (might be set by spurious falling edge)
					EXTI->IMR |= pSX->busyPinMask; // Enable busy pin edge IRQ
#endif

					TimerSimpleStartPulse(pSX->pTimer1us, pActiveCmd->timeout_us);

					SPILLDStartTransferExec(pSX->pSpi);
				}
			}
		}

		if(pActiveCmd->state == SX1280LLD_CMD_STATE_ACTIVE)
		{
			if(pSX->cmdDone)
			{
				TimerSimpleStop(pSX->pTimer1us);
				pActiveCmd->result = SX1280LLD_CMD_RESULT_OK;
				pActiveCmd->state = SX1280LLD_CMD_STATE_DONE;
			}
			else if(pSX->cmdTimedOut)
			{
				if(pActiveCmd->cmd == CMD_TIMED_IDLE)
				{
					pActiveCmd->result = SX1280LLD_CMD_RESULT_OK;
				}
				else
				{
					pActiveCmd->result = SX1280LLD_CMD_RESULT_TIMEOUT;
				}

				pActiveCmd->state = SX1280LLD_CMD_STATE_DONE;
			}

			pSX->cmdTimedOut = 0;
			pSX->cmdDone = 0;
		}

		if(pActiveCmd->state == SX1280LLD_CMD_STATE_DONE)
		{
			uint32_t tCmdTransferEnd_us = pSX->pSpi->transferCompletionTime_us;

			if(pActiveCmd->cmd == CMD_TIMED_IDLE || pActiveCmd->cmd == CMD_CLEAR_PENDING_DIO || pActiveCmd->cmd == CMD_WAIT_DIO || pActiveCmd->cmd == CMD_NOP)
				tCmdTransferEnd_us = pSX->tCmdTransferStart_us;
			else
				pSX->lastStatus = pSX->pRx[0];

			pActiveCmd->timeQueued_us = pSX->tCmdTransferStart_us - pActiveCmd->tQueued_us;
			pActiveCmd->timeTransfer_us = tCmdTransferEnd_us - pSX->tCmdTransferStart_us;
			pActiveCmd->timeBusy_us = pSX->tCmdEnd_us - tCmdTransferEnd_us;

			if(pActiveCmd->pTrace)
			{
				SX1280LLDCmdTrace* pTrace = pActiveCmd->pTrace;
				pTrace->cmd = pActiveCmd->cmd;
				pTrace->result = pActiveCmd->result;
				pTrace->tQueued_us = pActiveCmd->tQueued_us;
				pTrace->timeQueued_us = pActiveCmd->timeQueued_us;
				pTrace->timeTransfer_us = pActiveCmd->timeTransfer_us;
				pTrace->timeBusy_us = pActiveCmd->timeBusy_us;
			}

			processRxData(pSX->pRx, pActiveCmd);

			pSX->pActiveCmd = 0;

			if(pSX->cmdDoneCallback)
				(*pSX->cmdDoneCallback)(pSX->pUser);
		}
	}
	else if(pSX->cmdDoneCallback)
	{
		(*pSX->cmdDoneCallback)(pSX->pUser);
	}
}

static void timerTriggeredIRQ(void* pUser)
{
	SX1280LLD* pSX = (SX1280LLD*)pUser;
	pSX->tCmdEnd_us = SysTimeUSec();
	pSX->cmdTimedOut = 1;
	NVIC_SetPendingIRQ(pSX->highPrioIRQn);
}

void SX1280LLDBusyIRQ(SX1280LLD* pSX)
{
	SX1280LLDCmd* pActiveCmd = pSX->pActiveCmd;

	if(!pActiveCmd || pActiveCmd->state != SX1280LLD_CMD_STATE_ACTIVE || pActiveCmd->cmd == CMD_WAIT_DIO || SPILLDIsTransferActive(pSX->pSpi))
	{
		return; // This was a spurious busy IRQ due to RX/TX/FS mode changes
	}

#ifdef STM32H7XX
	EXTI_D1->IMR1 &= ~pSX->busyPinMask; // disable busy pin detection again
#else
	EXTI->IMR &= ~pSX->busyPinMask; // disable busy pin detection again
#endif

	pSX->tCmdEnd_us = SysTimeUSec();
	pSX->cmdDone = 1;
	NVIC_SetPendingIRQ(pSX->highPrioIRQn);
}

void SX1280LLDDioIRQ(SX1280LLD* pSX)
{
	SX1280LLDCmd* pActiveCmd = pSX->pActiveCmd;

	if(!pActiveCmd || pActiveCmd->state != SX1280LLD_CMD_STATE_ACTIVE || pActiveCmd->cmd != CMD_WAIT_DIO)
	{
		LogError("Spurious data IRQ");
		return;
	}

	uint32_t allPinMask = getDioPinMask(pSX, SX1280LLD_DIO_ALL);

#ifdef STM32H7XX
	EXTI_D1->IMR1 &= ~allPinMask; // disable data pin detection again
#else
	EXTI->IMR &= ~allPinMask; // disable data pin detection again
#endif

	pSX->tCmdEnd_us = SysTimeUSec();
	pSX->cmdDone = 1;
	NVIC_SetPendingIRQ(pSX->highPrioIRQn);
}

void SX1280LLDInit(SX1280LLD* pSX, SX1280LLDData* pData)
{
	pSX->pSpi = pData->pSpi;

	pSX->pTx = SPILLDGetTxBuf(pSX->pSpi);
	pSX->pRx = SPILLDGetRxBuf(pSX->pSpi);

	if(pSX->pSpi->data.dmaBufSize < 280)
		while(1); // Not enough memory

	pSX->pTimer1us = pData->pTimer1us;
	pSX->pTimer1us->pCallback = &timerTriggeredIRQ;
	pSX->pTimer1us->pUser = pSX;

	pSX->highPrioIRQn = pData->highPrioIRQn;

	pSX->csPin = pData->csPin;
	pSX->busyPinMask = pData->busyPin.pin;

	// init pins
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_EXTI;
	gpioInit.extiTrigger = GPIO_EXTI_TRIG_FALLING;
	gpioInit.pupd = GPIO_PUPD_UP;
	GPIOInit(pData->busyPin.pPort, pData->busyPin.pin, &gpioInit);

	gpioInit.extiTrigger = GPIO_EXTI_TRIG_RISING;
	gpioInit.pupd = GPIO_PUPD_DOWN;

	for(uint8_t i = 0; i < 3; i++)
	{
		if(pData->dioPins[i].pPort)
		{
			GPIOInit(pData->dioPins[i].pPort, pData->dioPins[i].pin, &gpioInit);
			pSX->dioPinMasks[i] = pData->dioPins[i].pin;

#ifdef STM32H7XX
			EXTI_D1->IMR1 &= ~pSX->dioPinMasks[i]; // mask ISR for now, will be activated on demand
			EXTI_D1->PR1 = pSX->dioPinMasks[i];
#else
			EXTI->IMR &= ~pSX->dioPinMasks[i]; // mask ISR for now, will be activated on demand
			EXTI->PR = pSX->dioPinMasks[i];
#endif
		}
	}


#ifdef STM32H7XX
	EXTI_D1->IMR1 &= ~pSX->busyPinMask; // mask ISR for now, will be activated on demand
	EXTI_D1->PR1 = pSX->busyPinMask;
#else
	EXTI->IMR &= ~pSX->busyPinMask; // mask ISR for now, will be activated on demand
	EXTI->PR = pSX->busyPinMask;
#endif

	SPILLDConfigureCsPin(pData->csPin);
	SPILLDReconfigure(pSX->pSpi, pData->prescaler, 0, 0);
}

void SX1280LLDEnqueueCmd(SX1280LLD* pSX, SX1280LLDCmd* pCmd)
{
	if(pCmd)
	{
		pCmd->state = SX1280LLD_CMD_STATE_QUEUED;
		pCmd->result = SX1280LLD_CMD_RESULT_OK;
		pCmd->tQueued_us = SysTimeUSec();
	}

	if(pSX->pActiveCmd)
	{
		LogErrorC("Failed to enqueue command", pCmd ? pCmd->cmd : 0);
		return;
	}

	pSX->pActiveCmd = pCmd;

	NVIC_SetPendingIRQ(pSX->highPrioIRQn);
}

uint8_t SX1280LLDIsCmdDone(SX1280LLDCmd* pCmd)
{
	return pCmd->state == SX1280LLD_CMD_STATE_DONE;
}

const char* SX1280LLDGetCmdName(uint8_t cmd)
{
	const char* pName = 0;

	switch(cmd)
	{
		case 0xC0: pName = "GetStatus               "; break;
		case 0x18: pName = "WriteRegister           "; break;
		case 0x19: pName = "ReadRegister            "; break;
		case 0x1A: pName = "WriteBuffer             "; break;
		case 0x1B: pName = "ReadBuffer              "; break;
		case 0x84: pName = "SetSleep                "; break;
		case 0x80: pName = "SetStandby              "; break;
		case 0xC1: pName = "SetFs                   "; break;
		case 0x83: pName = "SetTx                   "; break;
		case 0x82: pName = "SetRx                   "; break;
		case 0x94: pName = "SetRxDutyCycle          "; break;
		case 0xC5: pName = "SetCad                  "; break;
		case 0xD1: pName = "SetTxContinuousWave     "; break;
		case 0xD2: pName = "SetTxContinuousPreamble "; break;
		case 0x8A: pName = "SetPacketType           "; break;
		case 0x03: pName = "GetPacketType           "; break;
		case 0x86: pName = "SetRfFrequency          "; break;
		case 0x8E: pName = "SetTxParams             "; break;
		case 0x88: pName = "SetCadParams            "; break;
		case 0x8F: pName = "SetBufferBaseAddress    "; break;
		case 0x8B: pName = "SetModulationParams     "; break;
		case 0x8C: pName = "SetPacketParams         "; break;
		case 0x17: pName = "GetRxBufferStatus       "; break;
		case 0x1D: pName = "GetPacketStatus         "; break;
		case 0x1F: pName = "GetRssiInst             "; break;
		case 0x8D: pName = "SetDioIrqParams         "; break;
		case 0x15: pName = "GetIrqStatus            "; break;
		case 0x97: pName = "ClrIrqStatus            "; break;
		case 0x96: pName = "SetRegulatorMode        "; break;
		case 0xD5: pName = "SetSaveContext          "; break;
		case 0x9E: pName = "SetAutoFS               "; break;
		case 0x98: pName = "SetAutoTx               "; break;
		case 0x9B: pName = "SetLongPreamble         "; break;
		case 0x9D: pName = "SetUartSpeed            "; break;
		case 0xA3: pName = "SetRangingRole          "; break;
		case 0x9A: pName = "SetAdvancedRanging      "; break;
		case 0xF0: pName = "ClearPendingDio         "; break;
		case 0xF1: pName = "WaitDio                 "; break;
		case 0xF2: pName = "TimedIdle               "; break;
		case 0xFF: pName = "NOP                     "; break;
		default:   pName = "Unknown                 "; break;
	}

	return pName;
}

static uint32_t prepareTxData(uint8_t* pTx, SX1280LLDCmd* pCmd)
{
	pTx[0] = pCmd->cmd;

	switch(pCmd->cmd)
	{
		case CMD_GET_STATUS:
			return 1;
		case CMD_READ_REGISTER:
			pTx[1] = (uint8_t)(pCmd->readRegister.address >> 8);
			pTx[2] = (uint8_t)(pCmd->readRegister.address & 0xFF);
			memset(pTx+3, 0, pCmd->readRegister.size+1);
			return pCmd->readRegister.size+4;
		case CMD_WRITE_REGISTER:
			pTx[1] = (uint8_t)(pCmd->writeRegister.address >> 8);
			pTx[2] = (uint8_t)(pCmd->writeRegister.address & 0xFF);
			memcpy(pTx+3, pCmd->writeRegister.data, pCmd->writeRegister.size);
			return pCmd->writeRegister.size+3;
		case CMD_READ_BUFFER:
			pTx[1] = pCmd->readBuffer.offset;
			memset(pTx+2, 0, pCmd->readBuffer.size+1);
			return pCmd->readBuffer.size+3;
		case CMD_WRITE_BUFFER:
			pTx[1] = pCmd->writeBuffer.offset;
			memcpy(pTx+2, pCmd->writeBuffer.data, pCmd->writeBuffer.size);
			return pCmd->writeBuffer.size+2;
		case CMD_SET_SLEEP:
			pTx[1] = pCmd->setSleep.sleepConfig;
			return 2;
		case CMD_SET_STANDBY:
			pTx[1] = pCmd->setStandby.standbyConfig;
			return 2;
		case CMD_SET_FS:
			return 1;
		case CMD_SET_TX:
			pTx[1] = pCmd->setTx.periodBase;
			pTx[2] = (uint8_t)(pCmd->setTx.periodCount >> 8);
			pTx[3] = (uint8_t)(pCmd->setTx.periodCount & 0xFF);
			return 4;
		case CMD_SET_RX:
			pTx[1] = pCmd->setRx.periodBase;
			pTx[2] = (uint8_t)(pCmd->setRx.periodCount >> 8);
			pTx[3] = (uint8_t)(pCmd->setRx.periodCount & 0xFF);
			return 4;
		case CMD_SET_RXDUTYCYCLE:
			return 0; // not implemented
		case CMD_SET_CAD:
			return 0; // not implemented
		case CMD_SET_TXCONTINUOUSWAVE:
			return 0; // not implemented
		case CMD_SET_TXCONTINUOUSPREAMBLE:
			return 0; // not implemented
		case CMD_SET_PACKETTYPE:
			pTx[1] = pCmd->setPacketType.packetType;
			return 2;
		case CMD_GET_PACKETTYPE:
			return 0; // not implemented
		case CMD_SET_RFFREQUENCY:
			uint32_t val = (uint32_t)(((float)pCmd->setRfFrequency.frequency_Hz)*0.005041230769230769f);
			pTx[1] = (uint8_t) ((val >> 16) & 0xFF);
			pTx[2] = (uint8_t) ((val >> 8) & 0xFF);
			pTx[3] = (uint8_t) (val & 0xFF);
			return 4;
		case CMD_SET_TXPARAMS:
			pTx[1] = pCmd->setTxParams.power;
			pTx[2] = pCmd->setTxParams.rampTime;
			return 3;
		case CMD_SET_CADPARAMS:
			return 0; // not implemented
		case CMD_SET_BUFFERBASEADDRESS:
			pTx[1] = pCmd->setBufferBaseAddress.txBaseAddress;
			pTx[2] = pCmd->setBufferBaseAddress.rxBaseAddress;
			return 3;
		case CMD_SET_MODULATIONPARAMS:
			memcpy(pTx+1, pCmd->setModulationParams.params, 3);
			return 4;
		case CMD_SET_PACKETPARAMS:
			memcpy(pTx+1, pCmd->setPacketParams.params, 7);
			return 8;
		case CMD_GET_RXBUFFERSTATUS:
			memset(pTx+1, 0, 3);
			return 4;
		case CMD_GET_PACKETSTATUS:
			memset(pTx+1, 0, 6);
			return 7;
		case CMD_GET_RSSIINST:
			return 0; // not implemented
		case CMD_SET_DIOIRQPARAMS:
			pTx[1] = (uint8_t)(pCmd->setDioIrqParams.irqMask >> 8);
			pTx[2] = (uint8_t)(pCmd->setDioIrqParams.irqMask & 0xFF);
			pTx[3] = (uint8_t)(pCmd->setDioIrqParams.dioMask[0] >> 8);
			pTx[4] = (uint8_t)(pCmd->setDioIrqParams.dioMask[0] & 0xFF);
			pTx[5] = (uint8_t)(pCmd->setDioIrqParams.dioMask[1] >> 8);
			pTx[6] = (uint8_t)(pCmd->setDioIrqParams.dioMask[1] & 0xFF);
			pTx[7] = (uint8_t)(pCmd->setDioIrqParams.dioMask[2] >> 8);
			pTx[8] = (uint8_t)(pCmd->setDioIrqParams.dioMask[2] & 0xFF);
			return 9;
		case CMD_GET_IRQSTATUS:
			memset(pTx+1, 0, 3);
			return 4;
		case CMD_CLR_IRQSTATUS:
			pTx[1] = (uint8_t)(pCmd->clearIrqStatus.irqMask >> 8);
			pTx[2] = (uint8_t)(pCmd->clearIrqStatus.irqMask & 0xFF);
			return 3;
		case CMD_SET_REGULATORMODE:
			return 0; // not implemented
		case CMD_SET_SAVECONTEXT:
			return 0; // not implemented
		case CMD_SET_AUTOTX:
			return 0; // not implemented
		case CMD_SET_AUTOFS:
			pTx[1] = pCmd->setAutoFs.enable;
			return 2;
		case CMD_SET_LONGPREAMBLE:
			pTx[1] = pCmd->setLongPreamble.enable;
			return 2;
		case CMD_SET_UARTSPEED:
			return 0; // not implemented
		case CMD_SET_RANGING_ROLE:
			return 0; // not implemented
		default: return 0;
	}
}

static void processRxData(const uint8_t* pRx, SX1280LLDCmd* pCmd)
{
	switch(pCmd->cmd)
	{
		case CMD_GET_STATUS:
			pCmd->getStatus.status = pRx[0];
			break;
		case CMD_READ_REGISTER:
			memcpy(pCmd->readRegister.data, pRx+4, pCmd->readRegister.size);
			break;
		case CMD_READ_BUFFER:
			memcpy(pCmd->readBuffer.data, pRx+3, pCmd->readBuffer.size);
			break;
		case CMD_GET_RXBUFFERSTATUS:
			pCmd->getRxBufferStatus.payloadLength = pRx[2];
			pCmd->getRxBufferStatus.startBufferPointer = pRx[3];
			break;
		case CMD_GET_PACKETSTATUS:
			memcpy(pCmd->getPacketStatus.packetStatus, pRx+2, 5);
			break;
		case CMD_GET_IRQSTATUS:
			pCmd->getIrqStatus.irqStatus = (((uint16_t)pRx[2]) << 8) | pRx[3];
			break;
		default: return;
	}
}

void SX1280LLDParsePacketStatus(const SX1280LLDCmdGetPacketStatus* pCmd, uint8_t packetType, SX1280LLDPacketStatus* pStatus)
{
	switch(packetType)
	{
		case PACKET_TYPE_GFSK:
		case PACKET_TYPE_BLE:
		{
			pStatus->rssi_mdBm = (int32_t)pCmd->packetStatus[1] * -500;
			pStatus->gfsk.rx.syncError = (pCmd->packetStatus[2] >> 6) & 0x01;
			pStatus->gfsk.rx.lengthError = (pCmd->packetStatus[2] >> 5) & 0x01;
			pStatus->gfsk.rx.crcError = (pCmd->packetStatus[2] >> 4) & 0x01;
			pStatus->gfsk.abortError = (pCmd->packetStatus[2] >> 3) & 0x01;
			pStatus->gfsk.rx.headerReceived = (pCmd->packetStatus[2] >> 2) & 0x01;
			pStatus->gfsk.rx.packetReceived = (pCmd->packetStatus[2] >> 1) & 0x01;
			pStatus->gfsk.packetCtrlBusy = (pCmd->packetStatus[2] >> 0) & 0x01;
			pStatus->gfsk.rx.syncCode = pCmd->packetStatus[4] & 0x07;
		}
		break;
		case PACKET_TYPE_FLRC:
		{
			pStatus->rssi_mdBm = (int32_t)pCmd->packetStatus[1] * -500;
			pStatus->flrc.rx.syncError = (pCmd->packetStatus[2] >> 6) & 0x01;
			pStatus->flrc.rx.lengthError = (pCmd->packetStatus[2] >> 5) & 0x01;
			pStatus->flrc.rx.crcError = (pCmd->packetStatus[2] >> 4) & 0x01;
			pStatus->flrc.abortError = (pCmd->packetStatus[2] >> 3) & 0x01;
			pStatus->flrc.rx.headerReceived = (pCmd->packetStatus[2] >> 2) & 0x01;
			pStatus->flrc.rx.packetReceived = (pCmd->packetStatus[2] >> 1) & 0x01;
			pStatus->flrc.packetCtrlBusy = (pCmd->packetStatus[2] >> 0) & 0x01;
			pStatus->flrc.tx.pktSent = pCmd->packetStatus[3] & 0x01;
			pStatus->flrc.rx.syncCode = pCmd->packetStatus[4] & 0x07;
		}
		break;
		case PACKET_TYPE_LORA:
		case PACKET_TYPE_RANGING:
		{
			pStatus->rssi_mdBm = (int32_t)pCmd->packetStatus[1] * -500;
			int8_t snr = (int8_t)pCmd->packetStatus[1];
			pStatus->lora.snr_mdB = (int32_t)snr * 250;
		}
		break;
		default: break;
	}
}
