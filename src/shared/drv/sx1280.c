/*
 * sx1280.c
 *
 *  Created on: 14.09.2017
 *      Author: AndreR
 */

#include "sx1280.h"

#include "hal/init_hal.h"
#include "util/log.h"
#include "sx1280_def.h"
#include "errors.h"
#include <string.h>

#define BUF_SIZE_RXTX 272

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

static int16_t waitUntilReady(SX1280* pSX)
{
	// wait until the busy pin is low (IRQ based)
	chBSemReset(&pSX->busyIrqSem, TRUE); // clear semaphore
	if(pSX->busyPin.pPort->IDR & pSX->busyPin.pin)
	{
		if(chBSemWaitTimeout(&pSX->busyIrqSem, TIME_MS2I(10)) != MSG_OK)
		{
			LogWarn("Timeout waiting for low busy pin");
			return ERROR_SX1280_TIMEOUT;
		}
	}

	return 0;
}

static int16_t doTransfer(SX1280* pSX, uint32_t numBytes)
{
	int16_t waitResult = waitUntilReady(pSX);

	SPITransfer(&pSX->spiSlave, numBytes);

	pSX->lastStatus = pSX->pRx[0];

	return waitResult;
}

static int16_t setSleep(SX1280* pSX, uint8_t sleepConfig)
{
	pSX->pTx[0] = CMD_SET_SLEEP;
	pSX->pTx[1] = sleepConfig;

	return doTransfer(pSX, 2);
}

static int16_t setStandby(SX1280* pSX, uint8_t enableExternalOscillator)
{
	pSX->pTx[0] = CMD_SET_STANDBY;
	pSX->pTx[1] = enableExternalOscillator ? 1 : 0;

	return doTransfer(pSX, 2);
}

static int16_t getStatus(SX1280* pSX)
{
	pSX->pTx[0] = CMD_GET_STATUS;

	return doTransfer(pSX, 1);
}

int16_t SX1280GetStatus(SX1280* pSX, uint16_t* pStatus)
{
	chMtxLock(&pSX->dataMutex);

	int16_t result = getStatus(pSX);
	*pStatus = pSX->lastStatus & 0x01;
	*pStatus |= (pSX->lastStatus & 0x001C) << 2;
	*pStatus |= (pSX->lastStatus & 0x00E0) << 3;

	chMtxUnlock(&pSX->dataMutex);

	return result;
}

static int16_t setPacketType(SX1280* pSX, uint8_t type)
{
	pSX->pTx[0] = CMD_SET_PACKETTYPE;
	pSX->pTx[1] = type;

	return doTransfer(pSX, 2);
}

// frequency in [Hz]
static int16_t setRfFrequency(SX1280* pSX, uint32_t frequency)
{
	uint32_t val = (uint32_t)(((float)frequency)*0.005041230769230769f);

	pSX->pTx[0] = CMD_SET_RFFREQUENCY;
	pSX->pTx[1] = (uint8_t) ((val >> 16) & 0xFF);
	pSX->pTx[2] = (uint8_t) ((val >> 8) & 0xFF);
	pSX->pTx[3] = (uint8_t) (val & 0xFF);

	return doTransfer(pSX, 4);
}

static int16_t setBufferBaseAddress(SX1280* pSX, uint8_t txBase, uint8_t rxBase)
{
	pSX->pTx[0] = CMD_SET_BUFFERBASEADDRESS;
	pSX->pTx[1] = txBase;
	pSX->pTx[2] = rxBase;

	return doTransfer(pSX, 3);
}

static int16_t setModulationParams(SX1280* pSX, uint8_t param1, uint8_t param2, uint8_t param3)
{
	pSX->pTx[0] = CMD_SET_MODULATIONPARAMS;
	pSX->pTx[1] = param1;
	pSX->pTx[2] = param2;
	pSX->pTx[3] = param3;

	return doTransfer(pSX, 4);
}

static int16_t setPacketParams(SX1280* pSX, uint8_t preambleLen, uint8_t syncWordEnable, uint8_t syncWordMatch, uint8_t variableLength, uint8_t payloadLen,
		uint8_t crcLen)
{
	pSX->pTx[0] = CMD_SET_PACKETPARAMS;
	pSX->pTx[1] = preambleLen;
	pSX->pTx[2] = syncWordEnable ? FLRC_SYNCWORD_LENGTH_4_BYTE : FLRC_NO_SYNCWORD;
	pSX->pTx[3] = syncWordMatch;
	pSX->pTx[4] = variableLength ? PACKET_LENGTH_VARIABLE : PACKET_LENGTH_FIXED;
	pSX->pTx[5] = payloadLen;
	pSX->pTx[6] = crcLen;
	pSX->pTx[7] = 0x08; // whitening must be off

	return doTransfer(pSX, 8);
}

static int16_t setFs(SX1280* pSX)
{
	pSX->pTx[0] = CMD_SET_FS;

	return doTransfer(pSX, 1);
}

static int16_t setTx(SX1280* pSX, uint8_t periodBase, uint16_t periodCount)
{
	pSX->pTx[0] = CMD_SET_TX;
	pSX->pTx[1] = periodBase;
	pSX->pTx[2] = (uint8_t)(periodCount >> 8);
	pSX->pTx[3] = (uint8_t)(periodCount & 0xFF);

	return doTransfer(pSX, 4);
}

static int16_t setRx(SX1280* pSX, uint8_t periodBase, uint16_t periodCount)
{
	pSX->pTx[0] = CMD_SET_RX;
	pSX->pTx[1] = periodBase;
	pSX->pTx[2] = (uint8_t)(periodCount >> 8);
	pSX->pTx[3] = (uint8_t)(periodCount & 0xFF);

	return doTransfer(pSX, 4);
}

__attribute__((unused)) static int16_t setAutoTx(SX1280* pSX, uint16_t timeUs)
{
	pSX->pTx[0] = CMD_SET_AUTOTX;
	pSX->pTx[1] = (uint8_t)(timeUs >> 8);
	pSX->pTx[2] = (uint8_t)(timeUs & 0xFF);

	return doTransfer(pSX, 3);
}

static int16_t setAutoFs(SX1280* pSX, uint8_t enable)
{
	pSX->pTx[0] = CMD_SET_AUTOFS;
	pSX->pTx[1] = enable;

	return doTransfer(pSX, 2);
}

static int16_t setTxParams(SX1280* pSX, uint8_t power, uint8_t rampTime)
{
	pSX->pTx[0] = CMD_SET_TXPARAMS;
	pSX->pTx[1] = power;
	pSX->pTx[2] = rampTime;

	return doTransfer(pSX, 3);
}

static int16_t getRxBufferStatus(SX1280* pSX, uint8_t* pPayloadLength, uint8_t* pBufferOffset)
{
	pSX->pTx[0] = CMD_GET_RXBUFFERSTATUS;
	pSX->pTx[1] = 0;
	pSX->pTx[2] = 0;
	pSX->pTx[3] = 0;

	int16_t result = doTransfer(pSX, 4);
	if(result)
		return result;

	*pPayloadLength = pSX->pRx[2];
	*pBufferOffset = pSX->pRx[3];

	return 0;
}

static int16_t getPacketStatus(SX1280* pSX, SX1280PacketStatus* pStatus)
{
	pSX->pTx[0] = CMD_GET_PACKETSTATUS;
	pSX->pTx[1] = 0;
	pSX->pTx[2] = 0;
	pSX->pTx[3] = 0;
	pSX->pTx[4] = 0;
	pSX->pTx[5] = 0;
	pSX->pTx[6] = 0;

	int16_t result = doTransfer(pSX, 7);
	if(result)
		return result;

	pStatus->rfu = pSX->pRx[2];
	pStatus->rssiSync = pSX->pRx[3];
	pStatus->errors = pSX->pRx[4];
	pStatus->status = pSX->pRx[5];
	pStatus->sync = pSX->pRx[6];

	return 0;
}

__attribute__((unused)) static int16_t getRssiInst(SX1280* pSX, uint8_t* pRssi)
{
	pSX->pTx[0] = CMD_GET_RSSIINST;
	pSX->pTx[1] = 0;
	pSX->pTx[2] = 0;

	int16_t result = doTransfer(pSX, 3);
	if(result)
		return result;

	*pRssi = pSX->pRx[2];

	return 0;
}

static int16_t setDioIrqParams(SX1280* pSX, uint16_t irq, uint16_t dio1, uint16_t dio2, uint16_t dio3)
{
	pSX->pTx[0] = CMD_SET_DIOIRQPARAMS;
	pSX->pTx[1] = (uint8_t)(irq >> 8);
	pSX->pTx[2] = (uint8_t)(irq & 0xFF);
	pSX->pTx[3] = (uint8_t)(dio1 >> 8);
	pSX->pTx[4] = (uint8_t)(dio1 & 0xFF);
	pSX->pTx[5] = (uint8_t)(dio2 >> 8);
	pSX->pTx[6] = (uint8_t)(dio2 & 0xFF);
	pSX->pTx[7] = (uint8_t)(dio3 >> 8);
	pSX->pTx[8] = (uint8_t)(dio3 & 0xFF);

	return doTransfer(pSX, 9);
}

__attribute__((unused)) static int16_t getIrqStatus(SX1280* pSX, uint16_t* pIrqStatus)
{
	pSX->pTx[0] = CMD_GET_IRQSTATUS;
	pSX->pTx[1] = 0;
	pSX->pTx[2] = 0;
	pSX->pTx[3] = 0;

	int16_t result = doTransfer(pSX, 4);
	if(result)
		return result;

	*pIrqStatus = (((uint16_t)pSX->pRx[2]) << 8) | pSX->pRx[3];

	return 0;
}

static int16_t clearIrqStatus(SX1280* pSX, uint16_t irq)
{
	pSX->pTx[0] = CMD_CLR_IRQSTATUS;
	pSX->pTx[1] = (uint8_t)(irq >> 8);
	pSX->pTx[2] = (uint8_t)(irq & 0xFF);

	return doTransfer(pSX, 3);
}

static int16_t writeBuffer(SX1280* pSX, uint8_t offset, uint8_t numBytes, const void* pData)
{
	if(numBytes+2 > BUF_SIZE_RXTX)
		return ERROR_NOT_ENOUGH_MEMORY;

	pSX->pTx[0] = CMD_WRITE_BUFFER;
	pSX->pTx[1] = offset;

	memcpy(pSX->pTx+2, pData, numBytes);

	return doTransfer(pSX, numBytes+2);
}

static int16_t readBuffer(SX1280* pSX, uint8_t offset, uint8_t numBytes, void* pData)
{
	if(numBytes+3 > BUF_SIZE_RXTX)
		return ERROR_NOT_ENOUGH_MEMORY;

	pSX->pTx[0] = CMD_READ_BUFFER;
	pSX->pTx[1] = offset;
	memset(pSX->pTx+2, 0, numBytes+1);

	int16_t result = doTransfer(pSX, numBytes+3);
	if(result)
		return result;

	memcpy(pData, pSX->pRx+3, numBytes);

	return 0;
}

static int16_t writeRegisters(SX1280* pSX, uint16_t address, uint8_t numBytes, const void* pData)
{
	if(numBytes+3 > BUF_SIZE_RXTX)
		return ERROR_NOT_ENOUGH_MEMORY;

	pSX->pTx[0] = CMD_WRITE_REGISTER;
	pSX->pTx[1] = (uint8_t)(address >> 8);
	pSX->pTx[2] = (uint8_t)(address & 0xFF);

	memcpy(pSX->pTx+3, pData, numBytes);

	return doTransfer(pSX, numBytes+3);
}

static int16_t writeRegister(SX1280* pSX, uint16_t address, uint8_t data)
{
	pSX->pTx[0] = CMD_WRITE_REGISTER;
	pSX->pTx[1] = (uint8_t)(address >> 8);
	pSX->pTx[2] = (uint8_t)(address & 0xFF);
	pSX->pTx[3] = data;

	return doTransfer(pSX, 4);
}

__attribute__((unused)) static int16_t readRegisters(SX1280* pSX, uint16_t address, uint8_t numBytes, void* pData)
{
	if(numBytes+4 > BUF_SIZE_RXTX)
		return ERROR_NOT_ENOUGH_MEMORY;

	pSX->pTx[0] = CMD_READ_REGISTER;
	pSX->pTx[1] = (uint8_t)(address >> 8);
	pSX->pTx[2] = (uint8_t)(address & 0xFF);
	memset(pSX->pTx+3, 0, numBytes+1);

	int16_t result = doTransfer(pSX, numBytes+4);
	if(result)
		return result;

	memcpy(pData, pSX->pRx+4, numBytes);

	return 0;
}

static int16_t readRegister(SX1280* pSX, uint16_t address, uint8_t* pData)
{
	pSX->pTx[0] = CMD_READ_REGISTER;
	pSX->pTx[1] = (uint8_t)(address >> 8);
	pSX->pTx[2] = (uint8_t)(address & 0xFF);
	pSX->pTx[3] = 0;
	pSX->pTx[4] = 0;

	int16_t result = doTransfer(pSX, 5);
	if(result)
		return result;

	*pData = pSX->pRx[4];

	return 0;
}

static int16_t modifyRegister(SX1280* pSX, uint16_t address, uint8_t clearMask, uint8_t setMask)
{
	uint8_t data;

	int16_t result = readRegister(pSX, address, &data);
	if(result)
		return result;

	data &= ~clearMask;
	data |= setMask;

	return writeRegister(pSX, address, data);
}

// Setting range: 1 (lowest) - 13 (max gain)
__attribute__((unused)) static int16_t setManualGain(SX1280* pSX, uint8_t setting)
{
	int16_t result = modifyRegister(pSX, 0x089F, 0, 0x80);
	if(result)
		return result;

	result = modifyRegister(pSX, 0x895, 0x01, 0);
	if(result)
		return result;

	return modifyRegister(pSX, 0x089E, 0x0F, setting & 0x0F);
}

static int16_t setHighSensitivityMode(SX1280* pSX, uint8_t enable)
{
	if(enable)
		return modifyRegister(pSX, 0x0891, 0, 0xC0);
	else
		return modifyRegister(pSX, 0x0891, 0xC0, 0);
}

int16_t SX1280GetFirmwareVersion(SX1280* pSX, uint16_t* pVersion)
{
	*pVersion = 0;
	uint8_t* pVer = (uint8_t*)pVersion;

	chMtxLock(&pSX->dataMutex);

	int16_t result = readRegister(pSX, 0x0154, pVer);
	if(result)
	{
		chMtxUnlock(&pSX->dataMutex);
		return result;
	}

	result = readRegister(pSX, 0x0153, pVer+1);

	chMtxUnlock(&pSX->dataMutex);

	return result;
}

static int16_t setSyncWords(SX1280* pSX, uint32_t word1, uint32_t word2, uint32_t word3)
{
	word1 = __REV(word1);
	word2 = __REV(word2);
	word3 = __REV(word3);

	int16_t result = writeRegisters(pSX, 0x09CF, 4, &word1);
	if(result)
		return result;

	result = writeRegisters(pSX, 0x09D4, 4, &word2);
	if(result)
		return result;

	return writeRegisters(pSX, 0x09D9, 4, &word3);
}

static int16_t setSyncWord1(SX1280* pSX, uint32_t word1)
{
	word1 = __REV(word1);

	return writeRegisters(pSX, 0x09CF, 4, &word1);
}

__attribute__((unused)) static int16_t getSyncWordTolerance(SX1280* pSX, uint8_t* pTolerance)
{
	int16_t result = readRegister(pSX, 0x09CD, pTolerance);
	*pTolerance &= 0x0F;
	return result;
}

static int16_t setSyncWordTolerance(SX1280* pSX, uint8_t tolerance)
{
	return modifyRegister(pSX, 0x09CD, 0x0F, tolerance & 0x0F);
}

void SX1280Init(SX1280* pSX, SX1280Data* pData)
{
	pSX->busyPin = pData->busyPin;
	pSX->irqPin = pData->irqPin;

	pSX->address = 0xFF;

	chBSemObjectInit(&pSX->busyIrqSem, FALSE);
	chBSemObjectInit(&pSX->dataIrqSem, FALSE);
	chMtxObjectInit(&pSX->dataMutex);

	// init busy pin
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_EXTI;
	gpioInit.extiTrigger = GPIO_EXTI_TRIG_FALLING;
	gpioInit.pupd = GPIO_PUPD_UP;
	GPIOInit(pData->busyPin.pPort, pData->busyPin.pin, &gpioInit);

	gpioInit.extiTrigger = GPIO_EXTI_TRIG_RISING;
	gpioInit.pupd = GPIO_PUPD_DOWN;
	GPIOInit(pData->irqPin.pPort, pData->irqPin.pin, &gpioInit);

	// init SPI
	SPISlave* pSlave = &pSX->spiSlave;
	pSlave->cpol = 0;
	pSlave->cpha = 0;
	pSlave->csPin = pData->csPin;
	pSlave->prescaler = pData->prescaler;
	pSlave->timeoutTicks = TIME_MS2I(100);

	SPISlaveInit(pData->pSPI, &pSX->spiSlave);

	if(!SPIHasEnoughMemory(pSlave, SX1280_BUF_SIZE_RXTX))
	{
		chSysHalt("Not enough SPI memory for SX1280 driver");
	}

	// This driver permanently and exclusively acquires the given SPI bus
	SPIAcquire(pSlave, &pSX->pTx, &pSX->pRx);

	// get dummy status to generate traffic on SPI Interface => SX1280 will enable SPI mode
	getStatus(pSX);
}

int16_t SX1280Setup(SX1280* pSX, const SX1280SettingsFLRC* pSettings)
{
	int16_t result;

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

	setRfFrequency(pSX, pSettings->frequency);
	getStatus(pSX);
	LogDebugC("setRfFrequency", pSX->lastStatus);

	setBufferBaseAddress(pSX, 128, 0);
	getStatus(pSX);
	LogDebugC("setBufferBaseAddress", pSX->lastStatus);

	setHighSensitivityMode(pSX, pSettings->highSensitivityMode);

	setModulationParams(pSX, pSettings->bitrate, pSettings->coderate, MOD_SHAPING_BT_1_0);
	getStatus(pSX);
	LogDebugC("setModulationParams", pSX->lastStatus);

	setPacketParams(pSX, PREAMBLE_LENGTH_16_BITS, pSettings->syncWordEnable, pSettings->syncWordEnable ? RX_MATCH_SYNCWORD_1 : RX_MATCH_SYNCWORD_OFF,
			pSettings->variableLength, pSettings->payloadLength, pSettings->crcSize);

	setSyncWords(pSX, pSettings->syncWord, 0, 0);

	setSyncWordTolerance(pSX, 2);

	setAutoFs(pSX, 1);

	setTxParams(pSX, pSettings->txPower, PA_RAMP_04_US);
	getStatus(pSX);
	LogDebugC("setTxParams", pSX->lastStatus);

	result = setFs(pSX);
	if(result)
		return result;

	clearIrqStatus(pSX, IRQ_RADIO_ALL);

	return result;
}

int16_t SX1280SetRfFrequency(SX1280* pSX, uint32_t frequency)
{
	chMtxLock(&pSX->dataMutex);

	int16_t result = setRfFrequency(pSX, frequency);

	chMtxUnlock(&pSX->dataMutex);

	return result;
}

int16_t SX1280SetChannel(SX1280* pSX, uint8_t channel)
{
	pSX->channel = channel;

	uint32_t freq = channel;
	freq = (freq + 2300)*1000000UL;

	return SX1280SetRfFrequency(pSX, freq);
}

int16_t SX1280SetAddress(SX1280* pSX, uint8_t address)
{
	address &= 0x1F;

	chMtxLock(&pSX->dataMutex);

	int16_t result = setSyncWord1(pSX, hamming6[address]);

	chMtxUnlock(&pSX->dataMutex);

	pSX->address = address;

	return result;
}

int16_t SX1280FastRxTxReceive(SX1280* pSX, uint8_t txBytes, const void* pTxData, uint32_t rxTimeoutTicks)
{
	chMtxLock(&pSX->dataMutex);

	writeBuffer(pSX, 128, txBytes, pTxData);

	setDioIrqParams(pSX, IRQ_RX_DONE, IRQ_RX_DONE, 0, 0);

	chBSemReset(&pSX->dataIrqSem, TRUE);

	setRx(pSX, PERIOD_TICK_SIZE_1000_US, 0);

	msg_t waitResult = chBSemWaitTimeout(&pSX->dataIrqSem, rxTimeoutTicks);
	if(waitResult != MSG_OK)
	{
		setFs(pSX);
	}

	getPacketStatus(pSX, &pSX->rxStatus);

	clearIrqStatus(pSX, IRQ_RADIO_ALL);

	chMtxUnlock(&pSX->dataMutex);

	if(waitResult != MSG_OK)
	{
		return ERROR_SX1280_TIMEOUT;
	}

	return 0;
}

int16_t SX1280FastRxTxTransmit(SX1280* pSX, SX1280RxResult* pResult, uint8_t rxBytes, void* pRxData)
{
	chMtxLock(&pSX->dataMutex);

	setDioIrqParams(pSX, IRQ_TX_DONE, IRQ_TX_DONE, 0, 0);

	chBSemReset(&pSX->dataIrqSem, TRUE);

	setTx(pSX, PERIOD_TICK_SIZE_1000_US, 50);

	msg_t waitResult = chBSemWaitTimeout(&pSX->dataIrqSem, TIME_MS2I(10));

	SX1280PacketStatus txStatus;
	getPacketStatus(pSX, &txStatus);

	clearIrqStatus(pSX, IRQ_RADIO_ALL);

	uint8_t payloadLen = 0;
	if((pSX->rxStatus.errors & 0x79) == 0)
	{
		uint8_t offset;
		getRxBufferStatus(pSX, &payloadLen, &offset);

		if(payloadLen > rxBytes)
			payloadLen = rxBytes;

		readBuffer(pSX, 0, payloadLen, pRxData);
	}

	chMtxUnlock(&pSX->dataMutex);

	pResult->bytesReceived = payloadLen;
	pResult->rssiSync = pSX->rxStatus.rssiSync;
	pResult->syncCode = pSX->rxStatus.sync & 0x07;

	pResult->syncError = (pSX->rxStatus.errors & 0x40) >> 6;
	pResult->lengthError = (pSX->rxStatus.errors & 0x20) >> 5;
	pResult->crcError = (pSX->rxStatus.errors & 0x10) >> 4;
	pResult->abortError = (pSX->rxStatus.errors & 0x08) >> 3;
	pResult->headerReceived = (pSX->rxStatus.errors & 0x04) >> 2;
	pResult->packetReceived = (pSX->rxStatus.errors & 0x02) >> 1;
	pResult->packetCtrlBusy = (pSX->rxStatus.errors & 0x01);

	pResult->rxPid = (pSX->rxStatus.status & 0xC0) >> 6;
	pResult->noAck = (pSX->rxStatus.status & 0x20) >> 5;
	pResult->rxPidError = (pSX->rxStatus.status & 0x10) >> 4;

	if(waitResult != MSG_OK)
		return ERROR_SX1280_TIMEOUT;

	if((txStatus.status & 0x01) == 0)
		return ERROR_SX1280_TX_FAIL;

	return 0;
}

int16_t SX1280Transmit(SX1280* pSX, uint8_t numBytes, const void* pData)
{
	chMtxLock(&pSX->dataMutex);

	writeBuffer(pSX, 128, numBytes, pData);
	setDioIrqParams(pSX, IRQ_TX_DONE, IRQ_TX_DONE, 0, 0);

	chBSemReset(&pSX->dataIrqSem, TRUE);

	setTx(pSX, PERIOD_TICK_SIZE_1000_US, 50);

	msg_t waitResult = chBSemWaitTimeout(&pSX->dataIrqSem, TIME_MS2I(10));

	SX1280PacketStatus status;
	getPacketStatus(pSX, &status);

	clearIrqStatus(pSX, IRQ_RADIO_ALL);

	chMtxUnlock(&pSX->dataMutex);

	if(waitResult != MSG_OK)
		return ERROR_SX1280_TIMEOUT;

	if((status.status & 0x01) == 0)
		return ERROR_SX1280_TX_FAIL;

	return 0;
}

int16_t SX1280Receive(SX1280* pSX, uint8_t maxBytes, void* pData, SX1280RxResult* pResult, uint16_t timeoutTicks)
{
	pResult->bytesReceived = 0;

	chMtxLock(&pSX->dataMutex);

	setDioIrqParams(pSX, IRQ_RX_DONE, IRQ_RX_DONE, 0, 0);

	chBSemReset(&pSX->dataIrqSem, TRUE);

	setRx(pSX, PERIOD_TICK_SIZE_1000_US, 0);

	msg_t waitResult = chBSemWaitTimeout(&pSX->dataIrqSem, timeoutTicks);
	if(waitResult != MSG_OK)
	{
		setFs(pSX);
	}

	SX1280PacketStatus status;
	getPacketStatus(pSX, &status);

	clearIrqStatus(pSX, IRQ_RADIO_ALL);

	uint8_t payloadLen = 0;
	if((status.errors & 0x79) == 0)
	{
		uint8_t offset;
		getRxBufferStatus(pSX, &payloadLen, &offset);

		if(payloadLen > maxBytes)
			payloadLen = maxBytes;

		readBuffer(pSX, 0, payloadLen, pData);
	}

	chMtxUnlock(&pSX->dataMutex);

	if(waitResult != MSG_OK)
		return ERROR_SX1280_TIMEOUT;

	pResult->bytesReceived = payloadLen;
	pResult->rssiSync = status.rssiSync;
	pResult->syncCode = status.sync & 0x07;

	pResult->syncError = (status.errors & 0x40) >> 6;
	pResult->lengthError = (status.errors & 0x20) >> 5;
	pResult->crcError = (status.errors & 0x10) >> 4;
	pResult->abortError = (status.errors & 0x08) >> 3;
	pResult->headerReceived = (status.errors & 0x04) >> 2;
	pResult->packetReceived = (status.errors & 0x02) >> 1;
	pResult->packetCtrlBusy = (status.errors & 0x01);

	pResult->rxPid = (status.status & 0xC0) >> 6;
	pResult->noAck = (status.status & 0x20) >> 5;
	pResult->rxPidError = (status.status & 0x10) >> 4;

	return 0;
}

// Called at IRQ level
void SX1280BusyIRQ(SX1280* pSX)
{
	chSysLockFromISR();
	chBSemSignalI(&pSX->busyIrqSem);
	chSysUnlockFromISR();
}

// Called at IRQ level
void SX1280DataIRQ(SX1280* pSX)
{
	chSysLockFromISR();
	chBSemSignalI(&pSX->dataIrqSem);
	chSysUnlockFromISR();
}
