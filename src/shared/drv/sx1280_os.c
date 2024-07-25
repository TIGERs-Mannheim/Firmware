#include "sx1280_os.h"
#include "sx1280_def.h"
#include "errors.h"
#include "util/log.h"
#include <string.h>

static void doneCallback(void* pUser)
{
	SX1280OS* pSX = (SX1280OS*)pUser;

	NVIC_SetPendingIRQ(pSX->lowPrioIRQn);
}

static int16_t executeListSync(SX1280OS* pSX, SX1280LLDCmd* pList)
{
	SX1280LLDCmd* pLast = pList;
	while(pLast->pNext)
		pLast = pLast->pNext;

	pSX->pDrv->cmdDoneCallback = &doneCallback;
	pSX->pDrv->pUser = pSX;

	chBSemReset(&pSX->transferDoneSem, TRUE);

	SX1280LLDEnqueueCmdList(pSX->pDrv, pList);

	chBSemWait(&pSX->transferDoneSem);

	if(pLast->result == SX1280LLD_CMD_RESULT_FAIL)
		return ERROR_SX1280_CMD_EXEC;

	if(pLast->result == SX1280LLD_CMD_RESULT_TIMEOUT)
		return ERROR_SX1280_TIMEOUT;

	return 0;
}

static int16_t executeSync(SX1280OS* pSX, SX1280LLDCmd* pCmd)
{
	pSX->pDrv->cmdDoneCallback = &doneCallback;
	pSX->pDrv->pUser = pSX;

	pCmd->timeout_us = 10000;

	chBSemReset(&pSX->transferDoneSem, 1);

	SX1280LLDEnqueueCmd(pSX->pDrv, pCmd);

	chBSemWait(&pSX->transferDoneSem);

	if(pCmd->result == SX1280LLD_CMD_RESULT_FAIL)
		return ERROR_SX1280_CMD_EXEC;

	if(pCmd->result == SX1280LLD_CMD_RESULT_TIMEOUT)
		return ERROR_SX1280_TIMEOUT;

	return 0;
}

static void getPacketStatusFromCmd(SX1280LLDCmd* pCmd, SX1280OSPacketStatus* pStatus)
{
	pStatus->rfu = pCmd->getPacketStatus.packetStatus[0];
	pStatus->rssiSync = pCmd->getPacketStatus.packetStatus[1];
	pStatus->errors = pCmd->getPacketStatus.packetStatus[2];
	pStatus->status = pCmd->getPacketStatus.packetStatus[3];
	pStatus->sync = pCmd->getPacketStatus.packetStatus[4];
	pStatus->snr = 0;
}

static int16_t getStatus(SX1280OS* pSX)
{
	SX1280LLDCmd* pSetup = &pSX->setup;
	pSetup->cmd = CMD_GET_STATUS;
	return executeSync(pSX, pSetup);
}

static int16_t setStandby(SX1280OS* pSX, uint8_t enableExternalOscillator)
{
	SX1280LLDCmd* pSetup = &pSX->setup;
	pSetup->cmd = CMD_SET_STANDBY;
	pSetup->setStandby.standbyConfig = enableExternalOscillator ? 1 : 0;
	return executeSync(pSX, pSetup);
}

static int16_t setSleep(SX1280OS* pSX, uint8_t sleepConfig)
{
	SX1280LLDCmd* pSetup = &pSX->setup;
	pSetup->cmd = CMD_SET_SLEEP;
	pSetup->setSleep.sleepConfig = sleepConfig;
	return executeSync(pSX, pSetup);
}

static int16_t setPacketType(SX1280OS* pSX, uint8_t type)
{
	SX1280LLDCmd* pSetup = &pSX->setup;
	pSetup->cmd = CMD_SET_PACKETTYPE;
	pSetup->setPacketType.packetType = type;
	return executeSync(pSX, pSetup);
}

static int16_t setRfFrequency(SX1280OS* pSX, uint32_t frequency)
{
	SX1280LLDCmd* pSetup = &pSX->setup;
	pSetup->cmd = CMD_SET_RFFREQUENCY;
	pSetup->setRfFrequency.frequency_Hz = frequency;
	return executeSync(pSX, pSetup);
}

static int16_t setBufferBaseAddress(SX1280OS* pSX, uint8_t txBase, uint8_t rxBase)
{
	SX1280LLDCmd* pSetup = &pSX->setup;
	pSetup->cmd = CMD_SET_BUFFERBASEADDRESS;
	pSetup->setBufferBaseAddress.txBaseAddress = txBase;
	pSetup->setBufferBaseAddress.rxBaseAddress = rxBase;
	return executeSync(pSX, pSetup);
}

static int16_t modifyRegister(SX1280OS* pSX, uint16_t address, uint8_t clearMask, uint8_t setMask)
{
	SX1280LLDCmd* pSetup = &pSX->setup;

	pSetup->cmd = CMD_READ_REGISTER;
	pSetup->readRegister.address = address;
	pSetup->readRegister.size = 1;
	int16_t result = executeSync(pSX, pSetup);
	if(result)
		return result;

	uint8_t data = pSetup->readRegister.data[0];

	data &= ~clearMask;
	data |= setMask;

	pSetup->cmd = CMD_WRITE_REGISTER;
	pSetup->writeRegister.address = address;
	pSetup->writeRegister.size = 1;
	pSetup->writeRegister.data[0] = data;
	return executeSync(pSX, pSetup);
}

static int16_t setHighSensitivityMode(SX1280OS* pSX, uint8_t enable)
{
	if(enable)
		return modifyRegister(pSX, 0x0891, 0, 0xC0);
	else
		return modifyRegister(pSX, 0x0891, 0xC0, 0);
}

static int16_t setModulationParams(SX1280OS* pSX, uint8_t param1, uint8_t param2, uint8_t param3)
{
	SX1280LLDCmd* pSetup = &pSX->setup;
	pSetup->cmd = CMD_SET_MODULATIONPARAMS;
	pSetup->setModulationParams.params[0] = param1;
	pSetup->setModulationParams.params[1] = param2;
	pSetup->setModulationParams.params[2] = param3;
	return executeSync(pSX, pSetup);
}

static int16_t setPacketParams(SX1280OS* pSX, uint8_t param1, uint8_t param2, uint8_t param3, uint8_t param4, uint8_t param5, uint8_t param6, uint8_t param7)
{
	SX1280LLDCmd* pSetup = &pSX->setup;
	pSetup->cmd = CMD_SET_PACKETPARAMS;
	pSetup->setPacketParams.params[0] = param1;
	pSetup->setPacketParams.params[1] = param2;
	pSetup->setPacketParams.params[2] = param3;
	pSetup->setPacketParams.params[3] = param4;
	pSetup->setPacketParams.params[4] = param5;
	pSetup->setPacketParams.params[5] = param6;
	pSetup->setPacketParams.params[6] = param7;
	return executeSync(pSX, pSetup);
}

static int16_t setSyncWords(SX1280OS* pSX, uint32_t word1, uint32_t word2, uint32_t word3)
{
	word1 = __REV(word1);
	word2 = __REV(word2);
	word3 = __REV(word3);

	SX1280LLDCmd* pSetup = &pSX->setup;
	pSetup->cmd = CMD_WRITE_REGISTER;
	pSetup->writeRegister.address = 0x09CF;
	pSetup->writeRegister.size = 4;
	memcpy(pSetup->writeRegister.data, &word1, 4);
	int16_t result = executeSync(pSX, pSetup);
	if(result)
		return result;

	pSetup->writeRegister.address = 0x09D4;
	memcpy(pSetup->writeRegister.data, &word2, 4);
	result = executeSync(pSX, pSetup);
	if(result)
		return result;

	pSetup->writeRegister.address = 0x09D9;
	memcpy(pSetup->writeRegister.data, &word3, 4);
	return executeSync(pSX, pSetup);
}

static int16_t setSyncWordTolerance(SX1280OS* pSX, uint8_t tolerance)
{
	return modifyRegister(pSX, 0x09CD, 0x0F, tolerance & 0x0F);
}

static int16_t setAutoFs(SX1280OS* pSX, uint8_t enable)
{
	SX1280LLDCmd* pSetup = &pSX->setup;
	pSetup->cmd = CMD_SET_AUTOFS;
	pSetup->setAutoFs.enable = enable;
	return executeSync(pSX, pSetup);
}

static int16_t setTxParams(SX1280OS* pSX, uint8_t power, uint8_t rampTime)
{
	SX1280LLDCmd* pSetup = &pSX->setup;
	pSetup->cmd = CMD_SET_TXPARAMS;
	pSetup->setTxParams.power = power;
	pSetup->setTxParams.rampTime = rampTime;
	return executeSync(pSX, pSetup);
}

static int16_t setFs(SX1280OS* pSX)
{
	SX1280LLDCmd* pSetup = &pSX->setup;
	pSetup->cmd = CMD_SET_FS;
	return executeSync(pSX, pSetup);
}

static int16_t clearIrqStatus(SX1280OS* pSX, uint16_t irq)
{
	SX1280LLDCmd* pSetup = &pSX->setup;
	pSetup->cmd = CMD_CLR_IRQSTATUS;
	pSetup->clearIrqStatus.irqMask = irq;
	return executeSync(pSX, pSetup);
}

static int16_t writeBuffer(SX1280OS* pSX, uint8_t offset, uint8_t numBytes, const void* pData)
{
	SX1280LLDCmd* pSetup = &pSX->setup;
	pSetup->cmd = CMD_WRITE_BUFFER;
	pSetup->writeBuffer.offset = offset;
	pSetup->writeBuffer.size = numBytes;
	memcpy(pSetup->writeBuffer.data, pData, numBytes);
	return executeSync(pSX, pSetup);
}

static int16_t setDioIrqParams(SX1280OS* pSX, uint16_t irq, uint16_t dio1, uint16_t dio2, uint16_t dio3)
{
	SX1280LLDCmd* pSetup = &pSX->setup;
	pSetup->cmd = CMD_SET_DIOIRQPARAMS;
	pSetup->setDioIrqParams.irqMask = irq;
	pSetup->setDioIrqParams.dioMask[0] = dio1;
	pSetup->setDioIrqParams.dioMask[1] = dio2;
	pSetup->setDioIrqParams.dioMask[2] = dio3;
	return executeSync(pSX, pSetup);
}

static int16_t setRx(SX1280OS* pSX, uint8_t periodBase, uint16_t periodCount)
{
	SX1280LLDCmd* pSetup = &pSX->setup;
	pSetup->cmd = CMD_SET_RX;
	pSetup->setRx.periodBase = periodBase;
	pSetup->setRx.periodCount = periodCount;
	return executeSync(pSX, pSetup);
}

static int16_t getPacketStatus(SX1280OS* pSX, SX1280OSPacketStatus* pStatus)
{
	SX1280LLDCmd* pSetup = &pSX->setup;
	pSetup->cmd = CMD_GET_PACKETSTATUS;
	int16_t result = executeSync(pSX, pSetup);
	if(result)
		return result;

	getPacketStatusFromCmd(pSetup, pStatus);

	return 0;
}

static int16_t setTx(SX1280OS* pSX, uint8_t periodBase, uint16_t periodCount)
{
	SX1280LLDCmd* pSetup = &pSX->setup;
	pSetup->cmd = CMD_SET_TX;
	pSetup->setTx.periodBase = periodBase;
	pSetup->setTx.periodCount = periodCount;
	return executeSync(pSX, pSetup);
}

static int16_t getRxBufferStatus(SX1280OS* pSX, uint8_t* pPayloadLength, uint8_t* pBufferOffset)
{
	SX1280LLDCmd* pSetup = &pSX->setup;
	pSetup->cmd = CMD_GET_RXBUFFERSTATUS;
	int16_t result = executeSync(pSX, pSetup);
	if(result)
		return result;

	*pPayloadLength = pSetup->getRxBufferStatus.payloadLength;
	*pBufferOffset = pSetup->getRxBufferStatus.startBufferPointer;

	return 0;
}

static int16_t readBuffer(SX1280OS* pSX, uint8_t offset, uint8_t numBytes, void* pData)
{
	SX1280LLDCmd* pSetup = &pSX->setup;
	pSetup->cmd = CMD_READ_BUFFER;
	pSetup->readBuffer.offset = offset;
	pSetup->readBuffer.size = numBytes;
	int16_t result = executeSync(pSX, pSetup);
	if(result)
		return result;

	memcpy(pData, pSetup->readBuffer.data, numBytes);

	return 0;
}

void SX1280OSLowPrioIRQ(SX1280OS* pSX)
{
	chSysLockFromISR();
	chBSemSignalI(&pSX->transferDoneSem);
	chSysUnlockFromISR();
}

void SX1280OSInit(SX1280OS* pSX, SX1280OSData* pInit)
{
	pSX->pDrv = pInit->pDrv;
	pSX->pFem = pInit->pFem;
	pSX->lowPrioIRQn = pInit->lowPrioIRQn;

	chBSemObjectInit(&pSX->transferDoneSem, TRUE);
	chMtxObjectInit(&pSX->dataMutex);
}

int16_t SX1280OSSetupFLRC(SX1280OS* pSX, const SX1280OSSettingsCommon* pCommon, const SX1280OSSettingsFLRC* pSettingsFLRC)
{
	int16_t result;

	getStatus(pSX);

	setStandby(pSX, 0);
	getStatus(pSX);
	LogDebugC("setStandby", pSX->lastStatus);

	setSleep(pSX, 0);
	getStatus(pSX);
	LogDebugC("setSleep", pSX->lastStatus);

	setStandby(pSX, 1);
	getStatus(pSX);
	LogDebugC("setStandby", pSX->lastStatus);

	setPacketType(pSX, PACKET_TYPE_FLRC);
	getStatus(pSX);
	LogDebugC("setPacketType", pSX->lastStatus);

	setRfFrequency(pSX, pCommon->frequency);
	getStatus(pSX);
	LogDebugC("setRfFrequency", pSX->lastStatus);

	setBufferBaseAddress(pSX, 128, 0);
	getStatus(pSX);
	LogDebugC("setBufferBaseAddress", pSX->lastStatus);

	setHighSensitivityMode(pSX, pCommon->highSensitivityMode);

	setModulationParams(pSX, pSettingsFLRC->bitrateAndBandwidth, pSettingsFLRC->codingRate, pSettingsFLRC->modulationShaping);
	getStatus(pSX);
	LogDebugC("setModulationParams", pSX->lastStatus);

	setPacketParams(pSX, pSettingsFLRC->preambleLength, pSettingsFLRC->syncWordLength, pSettingsFLRC->syncWordMatch, pCommon->fixedLength, pCommon->payloadLength, pSettingsFLRC->crcLength, pSettingsFLRC->whitening);

	setSyncWords(pSX, pSettingsFLRC->syncWords[0], pSettingsFLRC->syncWords[1], pSettingsFLRC->syncWords[2]);

	setSyncWordTolerance(pSX, pSettingsFLRC->syncWordTolerance);

	setAutoFs(pSX, 1);

	setTxParams(pSX, pCommon->txPower, pCommon->paRampTime);
	getStatus(pSX);
	LogDebugC("setTxParams", pSX->lastStatus);

	result = setFs(pSX);
	if(result)
		return result;

	clearIrqStatus(pSX, IRQ_RADIO_ALL);

	return result;
}

int16_t SX1280OSFastRxTxReceive(SX1280OS* pSX, uint8_t txBytes, const void* pTxData, uint32_t rxTimeout_us)
{
	chMtxLock(&pSX->dataMutex);

	pSX->cmdWriteBuffer.cmd = CMD_WRITE_BUFFER;
	pSX->cmdWriteBuffer.timeout_us = 1000;
	pSX->cmdWriteBuffer.pNext = &pSX->cmdSetDioIrqParams;
	pSX->cmdWriteBuffer.writeBuffer.offset = 128;
	pSX->cmdWriteBuffer.writeBuffer.size = txBytes;
	memcpy(pSX->cmdWriteBuffer.writeBuffer.data, pTxData, txBytes);

	pSX->cmdSetDioIrqParams.cmd = CMD_SET_DIOIRQPARAMS;
	pSX->cmdSetDioIrqParams.timeout_us = 100;
	pSX->cmdSetDioIrqParams.pNext = &pSX->cmdClearDio;
	pSX->cmdSetDioIrqParams.setDioIrqParams.irqMask = IRQ_RX_DONE;
	pSX->cmdSetDioIrqParams.setDioIrqParams.dioMask[0] = IRQ_RX_DONE;

	pSX->cmdClearDio.cmd = CMD_CLEAR_PENDING_DIO;
	pSX->cmdClearDio.timeout_us = 100;
	pSX->cmdClearDio.pNext = &pSX->cmdSetRx;
	pSX->cmdClearDio.clearDio.mask = SX1280LLD_DIO_ALL;

	pSX->cmdSetRx.cmd = CMD_SET_RX;
	pSX->cmdSetRx.timeout_us = 200;
	pSX->cmdSetRx.pNext = &pSX->cmdWaitDio;
	pSX->cmdSetRx.setRx.periodBase = PERIOD_TICK_SIZE_1000_US;
	pSX->cmdSetRx.setRx.periodCount = 0;

	pSX->cmdWaitDio.cmd = CMD_WAIT_DIO;
	pSX->cmdWaitDio.timeout_us = rxTimeout_us;
	pSX->cmdWaitDio.pNext = 0;
	pSX->cmdWaitDio.waitDio.mask = SX1280LLD_DIO_1;

	memset(pSX->cmdGetPacketStatusRx.getPacketStatus.packetStatus, 0, 5);

	int16_t result = executeListSync(pSX, &pSX->cmdWriteBuffer);
	if(result)
	{
		pSX->cmdSetFs.cmd = CMD_SET_FS;
		pSX->cmdSetFs.timeout_us = 200;
		pSX->cmdSetFs.pNext = &pSX->cmdClearIrqStatus;
	}
	else
	{
		pSX->cmdGetPacketStatusRx.cmd = CMD_GET_PACKETSTATUS;
		pSX->cmdGetPacketStatusRx.timeout_us = 100;
		pSX->cmdGetPacketStatusRx.pNext = &pSX->cmdClearIrqStatus;
	}

	pSX->cmdClearIrqStatus.cmd = CMD_CLR_IRQSTATUS;
	pSX->cmdClearIrqStatus.timeout_us = 100;
	pSX->cmdClearIrqStatus.pNext = 0;
	pSX->cmdClearIrqStatus.clearIrqStatus.irqMask = IRQ_RADIO_ALL;

	if(result)
		executeListSync(pSX, &pSX->cmdSetFs);
	else
		executeListSync(pSX, &pSX->cmdGetPacketStatusRx);

	chMtxUnlock(&pSX->dataMutex);

	if(result)
		return ERROR_SX1280_TIMEOUT;

	return 0;
}

int16_t SX1280OSFastRxTxTransmit(SX1280OS* pSX, SX1280OSRxResult* pResult, uint8_t rxBytes, void* pRxData)
{
	chMtxLock(&pSX->dataMutex);

	pSX->cmdSetDioIrqParams.cmd = CMD_SET_DIOIRQPARAMS;
	pSX->cmdSetDioIrqParams.timeout_us = 100;
	pSX->cmdSetDioIrqParams.pNext = &pSX->cmdClearDio;
	pSX->cmdSetDioIrqParams.setDioIrqParams.irqMask = IRQ_TX_DONE;
	pSX->cmdSetDioIrqParams.setDioIrqParams.dioMask[0] = IRQ_TX_DONE;

	pSX->cmdClearDio.cmd = CMD_CLEAR_PENDING_DIO;
	pSX->cmdClearDio.timeout_us = 100;
	pSX->cmdClearDio.pNext = &pSX->cmdSetTx;
	pSX->cmdClearDio.clearDio.mask = SX1280LLD_DIO_ALL;

	pSX->cmdSetTx.cmd = CMD_SET_TX;
	pSX->cmdSetTx.timeout_us = 200;
	pSX->cmdSetTx.pNext = &pSX->cmdWaitDio;
	pSX->cmdSetTx.setTx.periodBase = PERIOD_TICK_SIZE_1000_US;
	pSX->cmdSetTx.setTx.periodCount = 25;

	SX1280OSPacketStatus rxStatus;
	getPacketStatusFromCmd(&pSX->cmdGetPacketStatusRx, &rxStatus);

	pSX->cmdGetRxBufferStatus.getRxBufferStatus.payloadLength = 0;
	pSX->cmdGetRxBufferStatus.getRxBufferStatus.startBufferPointer = 0;

	if((rxStatus.errors & 0x79) == 0)
	{
		pSX->cmdSetTx.pNext = &pSX->cmdGetRxBufferStatus;

		pSX->cmdGetRxBufferStatus.cmd = CMD_GET_RXBUFFERSTATUS;
		pSX->cmdGetRxBufferStatus.timeout_us = 100;
		pSX->cmdGetRxBufferStatus.pNext = &pSX->cmdReadBuffer;

		pSX->cmdReadBuffer.cmd = CMD_READ_BUFFER;
		pSX->cmdReadBuffer.timeout_us = 1000;
		pSX->cmdReadBuffer.readBuffer.offset = 0;
		pSX->cmdReadBuffer.readBuffer.size = rxBytes;
		pSX->cmdReadBuffer.pNext = &pSX->cmdWaitDio;
	}

	pSX->cmdWaitDio.cmd = CMD_WAIT_DIO;
	pSX->cmdWaitDio.timeout_us = 50000;
	pSX->cmdWaitDio.pNext = 0;
	pSX->cmdWaitDio.waitDio.mask = SX1280LLD_DIO_1;

	int16_t result = executeListSync(pSX, &pSX->cmdSetDioIrqParams);
	if(result)
	{
		pSX->cmdSetFs.cmd = CMD_SET_FS;
		pSX->cmdSetFs.timeout_us = 200;
		pSX->cmdSetFs.pNext = &pSX->cmdClearIrqStatus;
	}

	pSX->cmdClearIrqStatus.cmd = CMD_CLR_IRQSTATUS;
	pSX->cmdClearIrqStatus.timeout_us = 100;
	pSX->cmdClearIrqStatus.pNext = 0;
	pSX->cmdClearIrqStatus.clearIrqStatus.irqMask = IRQ_RADIO_ALL;

	if(result)
		executeListSync(pSX, &pSX->cmdSetFs);
	else
		executeListSync(pSX, &pSX->cmdClearIrqStatus);

	if(pSX->cmdGetRxBufferStatus.getRxBufferStatus.payloadLength > rxBytes)
		pSX->cmdGetRxBufferStatus.getRxBufferStatus.payloadLength = rxBytes;

	memcpy(pRxData, pSX->cmdReadBuffer.readBuffer.data, pSX->cmdGetRxBufferStatus.getRxBufferStatus.payloadLength);

	pResult->bytesReceived = pSX->cmdGetRxBufferStatus.getRxBufferStatus.payloadLength;

	chMtxUnlock(&pSX->dataMutex);

	pResult->rssiSync = rxStatus.rssiSync;
	pResult->syncCode = rxStatus.sync & 0x07;

	pResult->syncError = (rxStatus.errors & 0x40) >> 6;
	pResult->lengthError = (rxStatus.errors & 0x20) >> 5;
	pResult->crcError = (rxStatus.errors & 0x10) >> 4;
	pResult->abortError = (rxStatus.errors & 0x08) >> 3;
	pResult->headerReceived = (rxStatus.errors & 0x04) >> 2;
	pResult->packetReceived = (rxStatus.errors & 0x02) >> 1;
	pResult->packetCtrlBusy = (rxStatus.errors & 0x01);

	pResult->rxPid = (rxStatus.status & 0xC0) >> 6;
	pResult->noAck = (rxStatus.status & 0x20) >> 5;
	pResult->rxPidError = (rxStatus.status & 0x10) >> 4;

	if(result)
		return ERROR_SX1280_TIMEOUT;

	return 0;
}

int16_t SX1280OSTransmit(SX1280OS* pSX, uint8_t numBytes, const void* pData)
{
//	chMtxLock(&pSX->dataMutex);
//
//	writeBuffer(pSX, 128, numBytes, pData);
//	setDioIrqParams(pSX, IRQ_TX_DONE, IRQ_TX_DONE, 0, 0);
//
//	chBSemReset(&pSX->dataIrqSem, TRUE);
//
//	setTx(pSX, PERIOD_TICK_SIZE_1000_US, 50);
//
//	msg_t waitResult = chBSemWaitTimeout(&pSX->dataIrqSem, TIME_MS2I(10));
//
//	SX1280OSPacketStatus status;
//	getPacketStatus(pSX, &status);
//
//	clearIrqStatus(pSX, IRQ_RADIO_ALL);
//
//	chMtxUnlock(&pSX->dataMutex);
//
//	if(waitResult != MSG_OK)
//		return ERROR_SX1280_TIMEOUT;
//
//	if((status.status & 0x01) == 0)
//		return ERROR_SX1280_TX_FAIL;

	return 0;
}

int16_t SX1280OSReceive(SX1280OS* pSX, uint8_t maxBytes, void* pData, SX1280OSRxResult* pResult, uint16_t timeoutTicks)
{
//	pResult->bytesReceived = 0;
//
//	chMtxLock(&pSX->dataMutex);
//
//	setDioIrqParams(pSX, IRQ_RX_DONE, IRQ_RX_DONE, 0, 0);
//
//	chBSemReset(&pSX->dataIrqSem, TRUE);
//
//	setRx(pSX, PERIOD_TICK_SIZE_1000_US, 0);
//
//	msg_t waitResult = chBSemWaitTimeout(&pSX->dataIrqSem, timeoutTicks);
//	if(waitResult != MSG_OK)
//	{
//		setFs(pSX);
//	}
//
//	SX1280OSPacketStatus status;
//	getPacketStatus(pSX, &status);
//
//	clearIrqStatus(pSX, IRQ_RADIO_ALL);
//
//	uint8_t payloadLen = 0;
//	if((status.errors & 0x79) == 0)
//	{
//		uint8_t offset;
//		getRxBufferStatus(pSX, &payloadLen, &offset);
//
//		if(payloadLen > maxBytes)
//			payloadLen = maxBytes;
//
//		readBuffer(pSX, 0, payloadLen, pData);
//	}
//
//	chMtxUnlock(&pSX->dataMutex);
//
//	if(waitResult != MSG_OK)
//		return ERROR_SX1280_TIMEOUT;
//
//	pResult->bytesReceived = payloadLen;
//	pResult->rssiSync = status.rssiSync;
//	pResult->syncCode = status.sync & 0x07;
//
//	pResult->syncError = (status.errors & 0x40) >> 6;
//	pResult->lengthError = (status.errors & 0x20) >> 5;
//	pResult->crcError = (status.errors & 0x10) >> 4;
//	pResult->abortError = (status.errors & 0x08) >> 3;
//	pResult->headerReceived = (status.errors & 0x04) >> 2;
//	pResult->packetReceived = (status.errors & 0x02) >> 1;
//	pResult->packetCtrlBusy = (status.errors & 0x01);
//
//	pResult->rxPid = (status.status & 0xC0) >> 6;
//	pResult->noAck = (status.status & 0x20) >> 5;
//	pResult->rxPidError = (status.status & 0x10) >> 4;

	return 0;
}

int16_t SX1280OSSetChannel(SX1280OS* pSX, uint8_t channel)
{
	pSX->channel = channel;

	uint32_t freq = channel;
	freq = (freq + 2300)*1000000UL;

	SX1280LLDCmd* pSetup = &pSX->setup;
	pSetup->cmd = CMD_SET_RFFREQUENCY;
	pSetup->setRfFrequency.frequency_Hz = freq;
	return executeSync(pSX, pSetup);
}

/**
5 bits = 32 code words
6 repetitions = 30 bits
Minimum Hamming Distance: 6

abcde	abcd eabc deab cde1  abcd eabc deab cde0

code	16bit binary		hex
00000	0000 0000 0000 0001	0001 0000
00001	0000 1000 0100 0011	0843 0842
00010	0001 0000 1000 0101	1085 1084
00011	0001 1000 1100 0111	18C7 18C6
00100	0010 0001 0000 1001	2109 2108
00101	0010 1001 0100 1011	294B 294A
00110	0011 0001 1000 1101	318D 318C
00111	0011 1001 1100 1111	39CF 39CE
01000	0100 0010 0001 0001	4211 4210
01001	0100 1010 0101 0011	4A53 4A52
01010	0101 0010 1001 0101	5295 5294
01011	0101 1010 1101 0111	55D7 55D6
01100	0110 0011 0001 1001	6319 6318
01101	0110 1011 0101 1011	6B5B 6B5A
01110	0111 0011 1001 1101	739D 739C
01111	0111 1011 1101 1111	7BDF 7BDE

10000	1000 0100 0010 0001	8421 8420
10001	1000 1100 0110 0011	8C63 8C62
10010	1001 0100 1010 0101	94A5 94A4
10011	1001 1100 1110 0111	9CE7 9CE6
10100	1010 0101 0010 1001	A529 A528
10101	1010 1101 0110 1011	AD6B AD6A
10110	1011 0101 1010 1101	B5AD B5AC
10111	1011 1101 1110 1111	BDEF BDEE
11000	1100 0110 0011 0001	C631 C630
11001	1100 1110 0111 0011	CE73 CE72
11010	1101 0110 1011 0101	D6B5 D6B4
11011	1101 1110 1111 0111	DEF7 DEF6
11100	1110 0111 0011 1001	E739 E738
11101	1110 1111 0111 1011	EF7B EF7A
11110	1111 0111 1011 1101	F7BD F7BC
11111	1111 1111 1111 1111	FFFF FFFE
*/

static const uint32_t hamming6[] = {
	0x08430842,
	0x10851084,
	0x18C718C6,
	0x21092108,
	0x294B294A,
	0x318D318C,
	0x39CF39CE,
	0x42114210,
	0x4A534A52,
	0x52955294,
	0x55D755D6,
	0x63196318,
	0x6B5B6B5A,
	0x739D739C,
	0x7BDF7BDE,
	0x84218420,
	0x8C638C62,
	0x94A594A4,
	0x9CE79CE6,
	0xA529A528,
	0xAD6BAD6A,
	0xB5ADB5AC,
	0xBDEFBDEE,
	0xC631C630,
	0xCE73CE72,
	0xD6B5D6B4,
	0xDEF7DEF6,
	0xE739E738,
	0xEF7BEF7A,
	0xF7BDF7BC,
	0x00010000,
	0xFFFFFFFE,
};

int16_t SX1280OSSetAddress(SX1280OS* pSX, uint8_t address)
{
	address &= 0x1F;

	chMtxLock(&pSX->dataMutex);

	int16_t result = setSyncWords(pSX, hamming6[address], 0, 0);

	chMtxUnlock(&pSX->dataMutex);

	pSX->address = address;

	return result;
}
