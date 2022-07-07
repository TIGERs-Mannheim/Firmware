/*
 * nrf24ex.c
 *
 *  Created on: 29.09.2013
 *      Author: AndreR
 */

#include "nrf24_ex.h"

#if defined(STM32F4XX) || defined(STM32F7XX)

#include "errors.h"

#include "util/log.h"
#include "util/init_hal.h"

#include <string.h>

#define NRF24_SPEED_250K	0x20
#define NRF24_SPEED_1M		0x00
#define NRF24_SPEED_2M		0x08

// commands
#define R_REGISTER			0			// | register address
#define W_REGISTER			0b00100000	// | register address
#define R_RX_PAYLOAD		0b01100001
#define W_TX_PAYLOAD		0b10100000
#define W_TX_PAYLOAD_NO_ACK	0b10110000
#define FLUSH_TX			0b11100001
#define FLUSH_RX			0b11100010
#define REUSE_TX_PL			0b11100011
#define R_RX_PL_WID			0b01100000
#define W_ACK_PAYLOAD		0b10101000	// | PPP = pipe
#define W_TX_PAYLOAD_NO_ACK	0b10110000
#define NOP					0b11111111

// register addresses
#define REG_CONFIG 		0x00
#define REG_EN_AA 		0x01
#define REG_EN_RXADDR	0x02
#define REG_SETUP_AW	0x03
#define REG_SETUP_RETR	0x04
#define REG_RF_CH		0x05
#define REG_RF_SETUP	0x06
#define REG_STATUS		0x07
#define REG_OBSERVE_TX	0x08
#define REG_RPD			0x09
#define REG_ADDR_P0		0x0A
#define REG_ADDR_P1		0x0B
// ...
#define REG_TX_ADDR		0x10
#define REG_RX_PW_P0	0x11
#define REG_RX_PW_P1	0x12
// ...
#define REG_FIFO_STATUS	0x17
#define REG_DYNPD		0x1C
#define REG_FEATURE		0x1D

#define REG_STATUS_RX_DR	0x40
#define REG_STATUS_TX_DS	0x20
#define REG_STATUS_MAX_RT	0x10
#define REG_STATUS_RX_P_NO	0x0E
#define REG_STATUS_TX_FULL	0x01

#define W_CONFIG (W_REGISTER | REG_CONFIG)
#define R_CONFIG (R_REGISTER | REG_CONFIG)
#define W_EN_AA (W_REGISTER | REG_EN_AA)
#define R_EN_AA (R_REGISTER | REG_EN_AA)

// macros
#define CE_LOW(nrf)		GPIOReset(nrf->pCEPort, nrf->cePin)
#define CE_HIGH(nrf)	GPIOSet(nrf->pCEPort, nrf->cePin)

#define NRF_ACK_DELAY_VAL(delayUs) (((delayUs/250)-1) << 4)

static const uint8_t addresses[] = {
		0xF1, 0xF5, 0xE2, 0xEA,
		0xD3, 0xDC, 0xC4, 0xCF,
		0xB5, 0xB1, 0xA6, 0xAE,
		0x97, 0x92, 0x88, 0x8F,
		0x79, 0x71, 0x6A, 0x63,
		0x5B, 0x54, 0x4C, 0x47,
		0x3D, 0x31, 0x2E, 0x29,
		0x1F, 0x13, 0xFF, 0x00};

#ifdef STM32F7XX
static uint8_t tx[36] __attribute__((aligned(16), section(".dtc")));
static uint8_t rx[36] __attribute__((aligned(16), section(".dtc")));
#else
static uint8_t tx[36] __attribute__((aligned(4)));
static uint8_t rx[36] __attribute__((aligned(4)));
#endif

static int16_t downloadOneRxSlot(NRF24Ex* pNRF24, uint8_t rxLen);
static int16_t uploadOneTxSlot(NRF24Ex* pNRF24, uint8_t alwaysUploadSeq);
static void queueFeedRxData(NRF24Queue* pQueue, uint8_t* pData, uint32_t size);
static uint32_t queueFetchTxData(NRF24Queue* pQueue, uint8_t* pDst);

static __inline__ void enable(NRF24Ex* pNRF24)
{
	pNRF24->enabled = 1;
	CE_HIGH(pNRF24);
}

static __inline__ void disable(NRF24Ex* pNRF24)
{
	pNRF24->enabled = 0;
	CE_LOW(pNRF24);
}

static int16_t readRxLength(NRF24Ex* pNRF24, uint8_t* pValue)
{
	tx[0] = R_RX_PL_WID;
	tx[1] = 0;

	int16_t result = SPISyncTransfer(&pNRF24->slave, tx, rx, 2);
	if(result)
		return result;

	pNRF24->lastStatus = rx[0];
	*pValue = rx[1];

	return 0;
}

static int16_t writeSpecialCmd(NRF24Ex* pNRF24, uint8_t cmd)
{
	tx[0] = cmd;

	int16_t result = SPISyncTransfer(&pNRF24->slave, tx, rx, 1);
	if(result)
		return result;

	pNRF24->lastStatus = rx[0];

	return 0;
}

static int16_t writeConfig(NRF24Ex* pNRF24, uint8_t powerUp, uint8_t rx)
{
	return NRF24ExWriteRegister(pNRF24, REG_CONFIG, (powerUp << 1) | rx | 0x0C);	// + enable CRC16
}

static int16_t setId(NRF24Ex* pNRF24, uint8_t id)
{
	if(id > NRF24_MAX_BOT_ID)
		id = NRF24_MAX_BOT_ID+1;

	uint8_t address = addresses[id];

	tx[0] = W_REGISTER | REG_ADDR_P0;

	// address is repeated 3 times
	for(uint8_t i = 0; i < 3; i++)
		tx[i+1] = address;

	// set RX address
	int16_t result = SPISyncTransfer(&pNRF24->slave, tx, rx, 4);
	if(result)
		return result;

	// set TX address
	tx[0] = W_REGISTER | REG_TX_ADDR;
	result = SPISyncTransfer(&pNRF24->slave, tx, rx, 4);
	if(result)
		return result;

	pNRF24->lastStatus = rx[0];

	return 0;
}

int16_t NRF24ExWaitForIRQ(NRF24Ex* pNRF24, uint32_t waitTicks)
{
	if(chBSemWaitTimeout(&pNRF24->irqSem, waitTicks) != MSG_OK)
		return ERROR_NRF24EX_TIMEOUT;

	return 0;
}

int16_t NRF24ExChangeId(NRF24Ex* pNRF24, uint8_t id)
{
	if(pNRF24->pQueue)
		pNRF24->pQueue->id = id;

	return setId(pNRF24, id);
}

int16_t NRF24ExInit(NRF24Ex* pNRF24, NRF24ExData* pData)
{
	pNRF24->cePin = pData->cePin;
	pNRF24->pCEPort = pData->pCEPort;
	pNRF24->enabled = 0;
	pNRF24->rx = pData->rx;
	pNRF24->channel = pData->channel;
	pNRF24->rwDelay.read2M = 300;
	pNRF24->rwDelay.write2M = 300;
	pNRF24->rwDelay.read1M = 500;
	pNRF24->rwDelay.write1M = 300;
	pNRF24->rwDelay.read250k = 1500;
	pNRF24->rwDelay.write250k = 500;

	if(pData->speed == 0)
	{
		pNRF24->speed = NRF24_SPEED_250K;
		pNRF24->ackDelay = NRF_ACK_DELAY_VAL(1500);
	}
	else if(pData->speed == 1)
	{
		pNRF24->speed = NRF24_SPEED_1M;
		pNRF24->ackDelay = NRF_ACK_DELAY_VAL(500);
	}
	else
	{
		pNRF24->speed = NRF24_SPEED_2M;
		pNRF24->ackDelay = NRF_ACK_DELAY_VAL(500);
	}

	chBSemObjectInit(&pNRF24->irqSem, FALSE);

	// init CE pin
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_OUTPUT;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.pupd = GPIO_PUPD_NONE;
	GPIOInit(pData->pCEPort, pData->cePin, &gpioInit);

	CE_LOW(pNRF24);

	// init SPI
	pData->spiData.dmaPrio = 3;	// very high

	SPISyncInit(&pNRF24->spi, &pData->spiData);

	pNRF24->slave.cpha = 0;
	pNRF24->slave.cpol = 0;
	pNRF24->slave.prescaler = SPI_CR1_BRDIV64;
	pNRF24->slave.timeoutTicks = MS2ST(20);
	pNRF24->slave.pCSPort = pData->pCSPort;
	pNRF24->slave.csPin = pData->csPin;

	SPISyncSlaveInit(&pNRF24->spi, &pNRF24->slave);

	return 0;
}

static int16_t configureRegisters(NRF24Ex* pNRF24)
{
	int16_t result;

	result = NRF24ExWriteRegister(pNRF24, REG_FEATURE, 0x07); // enable dynamic packet payload and ack payload and no-ack
	if(result)
		return result;

	result = NRF24ExWriteRegister(pNRF24, REG_EN_AA, 0x01); // enable auto-ack on pipe 0
	if(result)
		return result;

	result = NRF24ExWriteRegister(pNRF24, REG_EN_RXADDR, 0x01); // enable data pipe 0
	if(result)
		return result;

	result = NRF24ExWriteRegister(pNRF24, REG_DYNPD, 0x01);	// enable dynamic payload length on pipe 0
	if(result)
		return result;

	result = NRF24ExWriteRegister(pNRF24, REG_SETUP_AW, 0x01);
	if(result)
		return result;

	result = setId(pNRF24, 0);
	if(result)
		return result;

	result = NRF24ExWriteRegister(pNRF24, REG_RF_SETUP, 0b00000110 | pNRF24->speed);	// speed, 0dBm output power
	if(result)
		return result;

	result = NRF24ExWriteRegister(pNRF24, REG_RF_CH, pNRF24->channel);
	if(result)
		return result;

	result = NRF24ExWriteRegister(pNRF24, REG_SETUP_RETR, pNRF24->ackDelay);

	return result;
}

int16_t NRF24ExSetup(NRF24Ex* pNRF24)
{
	int16_t result;

	// first write is lost on STM32F4, don't know why
	result = NRF24ExWriteRegister(pNRF24, REG_STATUS, 0x70);
	if(result)
		return result;

	result = configureRegisters(pNRF24);
	if(result)
		return result;

	result = writeSpecialCmd(pNRF24, FLUSH_RX);
	if(result)
		return result;

	result = writeSpecialCmd(pNRF24, FLUSH_TX);
	if(result)
		return result;

	result = NRF24ExWriteRegister(pNRF24, REG_STATUS, 0x70);
	if(result)
		return result;

	// finally power up
	writeConfig(pNRF24, 1, pNRF24->rx);

	if(pNRF24->rx)
		enable(pNRF24);

	return 0;
}

int16_t NRF24ExRestart(NRF24Ex* pNRF24)
{
	int16_t result;
	uint8_t wasEnabled = pNRF24->enabled;

	if(pNRF24->enabled)
		disable(pNRF24);

	result = writeConfig(pNRF24, 0, pNRF24->rx);	// power down
	if(result)
		return result;

	chThdSleep(MS2ST(5));	// wait

	result = configureRegisters(pNRF24);
	if(result)
		return result;

	result = writeConfig(pNRF24, 1, pNRF24->rx);	// power up
	if(result)
		return result;

	// power up delay
	chThdSleep(MS2ST(20));

	if(wasEnabled && pNRF24->rx)
		enable(pNRF24);

	return 0;
}

int16_t NRF24ExCheckChannel(NRF24Ex* pNRF24, uint8_t* pRPD)
{
	int16_t result;
	uint8_t wasEnabled = pNRF24->enabled;

	if(pNRF24->enabled)
		disable(pNRF24);

	result = writeConfig(pNRF24, 0, pNRF24->rx);	// power down
	if(result)
		return result;

	chThdSleep(MS2ST(2));

	result = writeConfig(pNRF24, 1, 1);	// power up as rx
	if(result)
		return result;

	uint8_t rpd;
	*pRPD = 0;

	for(uint8_t i = 0; i < 100; i++)
	{
		enable(pNRF24);

		chThdSleep(MS2ST(2));

		disable(pNRF24);

		result = NRF24ExReadRegister(pNRF24, REG_RPD, &rpd);
		if(result)
			return result;

		if(rpd & 0x01)
			(*pRPD)++;
	}

	result = writeConfig(pNRF24, 0, pNRF24->rx);	// power down
	if(result)
		return result;

	// power up
	result = writeConfig(pNRF24, 1, pNRF24->rx);
	if(result)
		return result;

	// power up delay
	chThdSleep(MS2ST(20));

	if(wasEnabled && pNRF24->rx)
		enable(pNRF24);

	return 0;
}

int16_t NRF24ExChangeChannel(NRF24Ex* pNRF24, uint8_t channel)
{
	int16_t result = 0;

	if(channel > 127)
	{
		return ERROR_NRF24EX_INVALID_CHANNEL;
	}

	pNRF24->channel = channel;

	// set channel
	result = NRF24ExWriteRegister(pNRF24, REG_RF_CH, channel);
	if(result)
		return result;

	return NRF24ExRestart(pNRF24);
}

uint8_t NRF24ExGetSpeedId(NRF24Ex* pNRF24)
{
	switch(pNRF24->speed)
	{
		case NRF24_SPEED_250K: return 0;
		case NRF24_SPEED_1M: return 1;
		case NRF24_SPEED_2M: return 2;
		default: return 0xFF;
	}
}

int16_t NRF24ExChangeSpeed(NRF24Ex* pNRF24, uint8_t speed)
{
	int16_t result = 0;

	if(speed == 0)
	{
		pNRF24->speed = NRF24_SPEED_250K;
		pNRF24->ackDelay = NRF_ACK_DELAY_VAL(1500);
	}
	else if(speed == 1)
	{
		pNRF24->speed = NRF24_SPEED_1M;
		pNRF24->ackDelay = NRF_ACK_DELAY_VAL(500);
	}
	else if(speed == 2)
	{
		pNRF24->speed = NRF24_SPEED_2M;
		pNRF24->ackDelay = NRF_ACK_DELAY_VAL(500);
	}
	else
		return ERROR_INVALID_PARAMETER;

	// set speed
	result = NRF24ExWriteRegister(pNRF24, REG_RF_SETUP, 0b00000110 | pNRF24->speed);
	if(result)
		return result;

	// and ACK delay
	result = NRF24ExWriteRegister(pNRF24, REG_SETUP_RETR, pNRF24->ackDelay);

	return result;
}

void NRF24ExGetReadWriteDelay(NRF24Ex* pNRF24, uint32_t* pRead, uint32_t* pWrite)
{
	switch(pNRF24->speed)
	{
		case NRF24_SPEED_2M:
		{
			*pRead = pNRF24->rwDelay.read2M;
			*pWrite = pNRF24->rwDelay.write2M;
		}
		break;
		case NRF24_SPEED_1M:
		{
			*pRead = pNRF24->rwDelay.read1M;
			*pWrite = pNRF24->rwDelay.write1M;
		}
		break;
		default:
		{
			*pRead = pNRF24->rwDelay.read250k;
			*pWrite = pNRF24->rwDelay.write250k;
		}
	}
}

void NRF24ExSetReadWriteDelay(NRF24Ex* pNRF24, const NRF24ReadWriteDelay* pNew)
{
	memcpy(&pNRF24->rwDelay, pNew, sizeof(NRF24ReadWriteDelay));
}

// IRQL: wifi handler
void NRF24ExIRQ(NRF24Ex* pNRF24)
{
	if(!pNRF24->enabled)
		return;

	chSysLockFromISR();
	chBSemSignalI(&pNRF24->irqSem);
	chSysUnlockFromISR();
}

int16_t NRF24ExReadRegister(NRF24Ex* pNRF24, uint8_t reg, uint8_t* pValue)
{
	tx[0] = R_REGISTER | reg;
	tx[1] = 0;

	int16_t result = SPISyncTransfer(&pNRF24->slave, tx, rx, 2);
	if(result)
		return result;

	pNRF24->lastStatus = rx[0];
	*pValue = rx[1];

	return 0;
}

int16_t NRF24ExWriteRegister(NRF24Ex* pNRF24, uint8_t reg, uint8_t value)
{
	tx[0] = W_REGISTER | reg;
	tx[1] = value;

	int16_t result = SPISyncTransfer(&pNRF24->slave, tx, rx, 2);
	if(result)
		return result;

	pNRF24->lastStatus = rx[0];

	return 0;
}

int16_t NRF24ExRxDownload(NRF24Ex* pNRF24)
{
	uint8_t rxLen;
	int16_t result;

	result = NRF24ExWriteRegister(pNRF24, REG_STATUS, 0x70);	// clear IRQ sources
	if(result)
		return result;

	while(1)
	{
		result = readRxLength(pNRF24, &rxLen);
		if(result)
			return result;

		// RX fifo empty?
		if((pNRF24->lastStatus & REG_STATUS_RX_P_NO) == REG_STATUS_RX_P_NO)
			break;

		result = downloadOneRxSlot(pNRF24, rxLen);
		if(result)
			return result;
	}

	return 0;
}

int16_t NRF24ExRxUpload(NRF24Ex* pNRF24)
{
	static uint16_t txFullOnEntry = 0;
	int16_t result = 0;

	if(pNRF24->lastStatus & REG_STATUS_TX_FULL)
	{
		++txFullOnEntry;
	}

	if(txFullOnEntry > 10)
	{
		txFullOnEntry = 0;
		LogError("TX FIFO full");

		result = NRF24ExRestart(pNRF24);
		if(result)
			return result;
	}

	while((pNRF24->lastStatus & REG_STATUS_TX_FULL) == 0)
	{
		result = uploadOneTxSlot(pNRF24, 0);
		if(result == ERROR_NRF24EX_QUEUE_EMPTY)
			break;
		else if(result)
			return result;

		tx[0] = NOP;
		result = SPISyncTransfer(&pNRF24->slave, tx, rx, 1);
		if(result)
			return result;

		pNRF24->lastStatus = rx[0];
	}

	return 0;
}

int16_t NRF24ExTxProcessOne(NRF24Ex* pNRF24)
{
	int16_t retResult = 0;
	uint8_t rxLen;
	int16_t result;

	LogDebugC("uploadOneTxSlot", pNRF24->pQueue->id);
	uploadOneTxSlot(pNRF24, 1);

	LogDebugC("chBSemWaitTimeout", pNRF24->pQueue->id);
	chBSemWaitTimeout(&pNRF24->irqSem, TIME_IMMEDIATE);	// clear semaphore

	LogDebugC("enable", pNRF24->pQueue->id);
	enable(pNRF24);

	LogDebugC("NRF24ExWaitForIRQ", pNRF24->pQueue->id);
	if(NRF24ExWaitForIRQ(pNRF24, MS2ST(20)))
	{
		LogDebugC("disable", pNRF24->pQueue->id);
		disable(pNRF24);
		return ERROR_NRF24EX_IRQ_MALFUNCTION;
	}

	LogDebugC("disable", pNRF24->pQueue->id);
	disable(pNRF24);

	LogDebugC("readRxLength", pNRF24->pQueue->id);
	result = readRxLength(pNRF24, &rxLen);
	if(result)
		return result;

	uint8_t status = pNRF24->lastStatus;

	if (status & REG_STATUS_RX_DR)
	{
		result = downloadOneRxSlot(pNRF24, rxLen);
		if(result)
			return result;
	}

	if(status & REG_STATUS_TX_DS)
	{
		++pNRF24->pQueue->stats.nrfFeedback.txPacketsAcked;
	}

	if(status & REG_STATUS_MAX_RT)
	{
		++pNRF24->pQueue->stats.nrfFeedback.txPacketsMaxRT;

		result = writeSpecialCmd(pNRF24, FLUSH_TX);
		if(result)
			return result;

		retResult = ERROR_NRF24EX_TIMEOUT;
	}

	result = NRF24ExWriteRegister(pNRF24, REG_STATUS, 0x70);// clear IRQ sources
	if (result)
		return result;

	uint16_t pipe = (pNRF24->lastStatus & 0x0E) >> 1;

	if(pipe != 0x07)
		LogWarnC("RxFIFO not empty!", pipe);

	return retResult;
}

void NRF24ExUseQueue(NRF24Ex* pNRF24, NRF24Queue* pQueue)
{
	pNRF24->pQueue = pQueue;

	setId(pNRF24, pQueue->id);
}

void NRF24QueueSetOutgoingCmdCallback(NRF24Queue* pQueue, NRF24OutgoingCmdFunc pFunc)
{
	pQueue->pOutgoindCmdCb = pFunc;
}

static int16_t uploadOneTxSlot(NRF24Ex* pNRF24, uint8_t alwaysUploadSeq)
{
	int16_t result;
	uint32_t size;

	if(alwaysUploadSeq)
		tx[0] = W_TX_PAYLOAD;
	else
		tx[0] = W_ACK_PAYLOAD;

	tx[1] = pNRF24->pQueue->txSeq;
	if(pNRF24->pQueue->txBufUsed != 0)
		tx[1] |= 0x80;	// flag signals data continuation

	size = queueFetchTxData(pNRF24->pQueue, &tx[2]);
	if(size == 0 && alwaysUploadSeq == 0)
		return ERROR_NRF24EX_QUEUE_EMPTY;

	++pNRF24->pQueue->stats.nrfIO.txPackets;
	pNRF24->pQueue->stats.nrfIO.txBytes += size+1;

	++pNRF24->pQueue->txSeq;
	pNRF24->pQueue->txSeq &= 0x7F;

	result = SPISyncTransfer(&pNRF24->slave, tx, 0, 2+size);

	return result;
}

static int16_t downloadOneRxSlot(NRF24Ex* pNRF24, uint8_t rxLen)
{
	int16_t result = 0;

	if (rxLen > 32)
	{
		LogErrorC("rxLen>32 -> transmission error!", rxLen);

		result = writeSpecialCmd(pNRF24, FLUSH_RX);
		if(result)
			return result;
	}
	else if(rxLen > 0)
	{
		tx[0] = R_RX_PAYLOAD;
		result = SPISyncTransfer(&pNRF24->slave, tx, rx, rxLen+1);
		if(result)
			return result;

		++pNRF24->pQueue->stats.nrfIO.rxPackets;
		pNRF24->pQueue->stats.nrfIO.rxBytes += rxLen;

		uint8_t* pRxPayload = rx+2;
		rxLen--;

		// process first byte with sequence and data continuation information
		++pNRF24->pQueue->rxSeq;
		pNRF24->pQueue->rxSeq &= 0x7F;

		uint8_t dataContinuation = rx[1] & 0x80;
		uint8_t recvSeq = rx[1] & 0x7F;

		if(recvSeq == pNRF24->pQueue->rxSeq)
		{
			// everything is fine, sequence as expected, no packet loss
			queueFeedRxData(pNRF24->pQueue, pRxPayload, rxLen);
			return 0;
		}

		// we have a sequence mismatch => packet loss
		LogWarnC("Seq mismatch", pNRF24->pQueue->rxSeq | ((uint32_t)recvSeq) << 16);

		pNRF24->pQueue->stats.nrfFeedback.rxPacketsLost += (recvSeq - pNRF24->pQueue->rxSeq) & 0x7F;

		// packet lost, everything in the rx buffer is invalid
		pNRF24->pQueue->rxBufUsed = 0;

		// adjust to new sequence number
		pNRF24->pQueue->rxSeq = recvSeq;

		if(dataContinuation == 0)
		{
			// with this packet a new command starts, process it as normal
			queueFeedRxData(pNRF24->pQueue, pRxPayload, rxLen);
			return 0;
		}

		// packet lost and continuation from the last packet is signaled, drop until next zero
		uint16_t nullPos;
		for(nullPos = 0; nullPos < rxLen; nullPos++)
		{
			if(pRxPayload[nullPos] == 0)
				break;
		}

		if(nullPos < rxLen-1)
		{
			// there starts another packet, process it
			queueFeedRxData(pNRF24->pQueue, &pRxPayload[nullPos+1], rxLen-nullPos-1);
		}
	}

	return 0;
}

void NRF24QueueInit(NRF24Queue* pQueue, uint8_t* pDataTx, uint16_t txSize, uint8_t* pDataRx, uint16_t rxSize)
{
	FifoLinInit(&pQueue->rxFifo, pDataRx, rxSize);
	FifoLinInit(&pQueue->txFifo, pDataTx, txSize);

	pQueue->rxBufUsed = 0;
	pQueue->txBufUsed = 0;

	memset(&pQueue->stats, 0, sizeof(pQueue->stats));
}

int16_t NRF24QueueGetPacket(NRF24Queue* pQueue, uint8_t* pDst, uint32_t dstSize, uint32_t* pPktSize)
{
	uint8_t* pSrc;
	*pPktSize = 0;

	if(FifoLinGet(&pQueue->rxFifo, &pSrc, pPktSize))
		return ERROR_NRF24EX_QUEUE_EMPTY;

	if(*pPktSize > dstSize)
		return ERROR_NOT_ENOUGH_MEMORY;

	memcpy(pDst, pSrc, *pPktSize);

	FifoLinDelete(&pQueue->rxFifo);

	return 0;
}

/**
 * Put data in an NRF24 queue.
 *
 * @param pQueue the queue
 * @param pData data to send
 * @param dataSize datasize
 * @return 0 on success, errorcode otherwise
 */
int16_t NRF24QueueSend(NRF24Queue* pQueue, uint8_t* pData, uint32_t dataSize)
{
	uint8_t* pDst;
	int16_t result;

	if(dataSize > NRF24_MAX_MSG_SIZE)
		return ERROR_NRF24_INVALID_LENGTH;

	result = FifoLinReserve(&pQueue->txFifo, dataSize, &pDst);
	if(result)
	{
		++pQueue->stats.queueFeedback.txPacketsLost;
		return result;
	}

	memcpy(pDst, pData, dataSize);

	FifoLinCommit(&pQueue->txFifo, dataSize);

	return 0;
}

void NRF24QueueClear(NRF24Queue* pQueue)
{
	FifoLinClear(&pQueue->txFifo);
	FifoLinClear(&pQueue->rxFifo);
}

static uint32_t queueFetchTxData(NRF24Queue* pQueue, uint8_t* pDst)
{
	uint8_t* pData;
	uint32_t pktSize;
	uint32_t bytesWritten;

	while(pQueue->txBufUsed < NRF24_MAX_PKT_LENGTH)
	{
		if(FifoLinGet(&pQueue->txFifo, &pData, &pktSize))
			break;

		if(pktSize > NRF24_MAX_MSG_SIZE)	// assert
		{
			LogErrorC("Oversized HL message", pktSize);
			FifoLinDelete(&pQueue->txFifo);
			continue;
		}

		if(pQueue->pOutgoindCmdCb)
			(*pQueue->pOutgoindCmdCb)(pQueue->id, pData, pktSize);

		COBSEncode(pData, pktSize, &pQueue->txBuf[pQueue->txBufUsed], NRF24_MAX_ENCODED_MSG_SIZE, &bytesWritten);
		FifoLinDelete(&pQueue->txFifo);

		pQueue->txBufUsed += bytesWritten;
		pQueue->txBuf[pQueue->txBufUsed++] = 0;

		++pQueue->stats.queueIO.txPackets;
		pQueue->stats.queueIO.txBytes += pktSize;
	}

	uint32_t bytesToReturn = pQueue->txBufUsed;

	if(bytesToReturn > NRF24_MAX_PKT_LENGTH)
		bytesToReturn = NRF24_MAX_PKT_LENGTH;

	if(bytesToReturn > 0)
	{
		memcpy(pDst, pQueue->txBuf, bytesToReturn);
		pQueue->txBufUsed -= bytesToReturn;
		memmove(pQueue->txBuf, &pQueue->txBuf[bytesToReturn], pQueue->txBufUsed);
	}

	return bytesToReturn;
}

static void shiftOutRxData(NRF24Queue* pQueue, uint32_t nullPos)
{
	if(pQueue->rxBufUsed < nullPos+1)	// assert
	{
		LogErrorC("Invalid RX shift", nullPos);
		pQueue->rxBufUsed = 0;
		return;
	}

	pQueue->rxBufUsed -= nullPos+1;
	memmove(pQueue->rxBuf, &pQueue->rxBuf[nullPos+1], pQueue->rxBufUsed);
}

static void queueFeedRxData(NRF24Queue* pQueue, uint8_t* pData, uint32_t size)
{
	uint32_t nullPos;
	int16_t result;

	if(size == 0)
		return;

	if(size > NRF24_MAX_PKT_LENGTH)	// assert
	{
		LogErrorC("Oversized LL packet", size);
		return;
	}

	// append received data to rxBuf
	memcpy(&pQueue->rxBuf[pQueue->rxBufUsed], pData, size);
	pQueue->rxBufUsed += size;

	while(pQueue->rxBufUsed > 0)
	{
		// search for packet delimiter
		for(nullPos = 0; nullPos < pQueue->rxBufUsed; nullPos++)
		{
			if(pQueue->rxBuf[nullPos] == 0)
				break;
		}

		if(nullPos == pQueue->rxBufUsed)
		{
			if(pQueue->rxBufUsed > NRF24_MAX_ENCODED_MSG_SIZE)
			{
				// we must have missed something, max message size reached and no null found
				pQueue->rxBufUsed = 0;
			}

			break;	// no packet delimiter found
		}

		if(nullPos > NRF24_MAX_ENCODED_MSG_SIZE)
		{
			// we must also have missed something here, that message is too big
			shiftOutRxData(pQueue, nullPos);
			continue;
		}

		uint8_t* pDst;
		uint32_t bytesWritten;
		result = FifoLinReserve(&pQueue->rxFifo, NRF24_MAX_MSG_SIZE, &pDst);
		if(result)
		{
			// no memory to save all eventual data
			shiftOutRxData(pQueue, nullPos);
			++pQueue->stats.queueFeedback.rxPacketsLost;
			continue;
		}

		result = COBSDecode(pQueue->rxBuf, nullPos, pDst, NRF24_MAX_MSG_SIZE, &bytesWritten);
		if(result)
		{
			// looks like we received garbage
			shiftOutRxData(pQueue, nullPos);
			continue;
		}

		if(bytesWritten >= PACKET_HEADER_SIZE)
		{
			FifoLinCommit(&pQueue->rxFifo, bytesWritten);

			++pQueue->stats.queueIO.rxPackets;
			pQueue->stats.queueIO.rxBytes += bytesWritten;
		}

		// shift out processed data
		shiftOutRxData(pQueue, nullPos);
	}
}

#endif
