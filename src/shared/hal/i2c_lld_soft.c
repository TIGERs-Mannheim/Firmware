#include "i2c_lld_soft.h"
#include "errors.h"

#define SDA_HIGH() GPIOPinSet(pI2C->data.sdaPin)
#define SCL_HIGH() GPIOPinSet(pI2C->data.sclPin)
#define SDA_LOW() GPIOPinReset(pI2C->data.sdaPin)
#define SCL_LOW() GPIOPinReset(pI2C->data.sclPin)

#define HL_STATE_DONE		0
#define HL_STATE_IDLE		1
#define HL_STATE_START		2
#define HL_STATE_ADDR		3
#define HL_STATE_STOP		4
#define HL_STATE_WRITE_BYTE	5
#define HL_STATE_READ_BYTE	6

static void* i2cSoftGetTxBuf(I2CLLD* pBase);
static void* i2cSoftGetRxBuf(I2CLLD* pBase);
static uint32_t i2cSoftGetBufSize(I2CLLD* pBase);
static int16_t i2cSoftTransfer(I2CLLD* pBase, uint8_t addr, uint8_t txLength, uint8_t rxLength);
static void i2cSoftAbort(I2CLLD* pBase);
static uint8_t i2cSoftIsActive(I2CLLD* pBase);

static const I2CLLDVTable i2cSoftVTable = {
	.getTxBuf = &i2cSoftGetTxBuf,
	.getRxBuf = &i2cSoftGetRxBuf,
	.getBufSize = &i2cSoftGetBufSize,
	.transfer = &i2cSoftTransfer,
	.abort = &i2cSoftAbort,
	.isActive = &i2cSoftIsActive,
};

static void dummyDoneCallback(int16_t result, void* pData)
{
	(void)result;
	(void)pData;
}

static inline uint8_t writeByte(I2CLLDSoft* pI2C, uint8_t data, uint8_t* pAck)
{
	switch(pI2C->opState)
	{
		case 0:
		{
			pI2C->bitCounter = 7;
			++pI2C->opState;
		}
		// fall through
		// no break
		case 1:
		{
			if(data & (1 << pI2C->bitCounter))
				SDA_HIGH();
			else
				SDA_LOW();

			++pI2C->opState;
		}
		break;
		case 2:
		{
			SCL_HIGH();
			++pI2C->opState;
		}
		break;
		case 3:
		{
			SCL_LOW();
			--pI2C->bitCounter;

			if(pI2C->bitCounter >= 0)
				pI2C->opState = 1;
			else
				++pI2C->opState;
		}
		break;
		case 4:
		{
			SDA_HIGH();
			++pI2C->opState;
		}
		break;
		case 5:
		{
			SCL_HIGH();
			++pI2C->opState;
		}
		break;
		case 6:
		{
			*pAck = (pI2C->data.sdaPin.pPort->IDR & pI2C->data.sdaPin.pin) ? 0 : 1;
			SCL_LOW();

			return 1;
		}
		break;
	}

	return 0;
}

static inline uint8_t readByte(I2CLLDSoft* pI2C, volatile uint8_t* pData, uint8_t ack)
{
	switch(pI2C->opState)
	{
		case 0:
		{
			*pData = 0;
			pI2C->bitCounter = 7;
			++pI2C->opState;
		}
		// fall through
		// no break
		case 1:
		{
			SDA_HIGH();
			++pI2C->opState;
		}
		break;
		case 2:
		{
			SCL_HIGH();
			++pI2C->opState;
		}
		break;
		case 3:
		{
			uint8_t read = (pI2C->data.sdaPin.pPort->IDR & pI2C->data.sdaPin.pin) ? 1 : 0;
			*pData |= (read << pI2C->bitCounter);
			SCL_LOW();

			--pI2C->bitCounter;

			if(pI2C->bitCounter >= 0)
				pI2C->opState = 1;
			else
				++pI2C->opState;
		}
		break;
		case 4:
		{
			if(ack)
				SDA_LOW();
			else
				SDA_HIGH();

			++pI2C->opState;
		}
		break;
		case 5:
		{
			SCL_HIGH();
			++pI2C->opState;
		}
		break;
		case 6:
		{
			SCL_LOW();

			return 1;
		}
		break;
	}

	return 0;
}

void I2CLLDSoftTick(I2CLLDSoft* pI2C)
{
	I2CLLD* pBase = &pI2C->base;

	pI2C->data.pTimer->SR = 0;

	if(pI2C->abortPending)
	{
		pI2C->abortPending = 0;
		if(pI2C->hlState != HL_STATE_DONE)
			pI2C->hlState = HL_STATE_STOP;
	}

	switch(pI2C->hlState)
	{
		case HL_STATE_IDLE:
		{
			pI2C->opState = 0;
			pI2C->hlState = HL_STATE_START;
			pI2C->transferResult = I2C_RESULT_TRANSFER_COMPLETE;
		}
		break;
		case HL_STATE_START:
		{
			switch(pI2C->opState)
			{
				case 0:
				{
					SCL_HIGH();
					SDA_HIGH();
					++pI2C->opState;
				}
				break;
				case 1:
				{
					// If SDA and SCL are not high some slave is broken, we lost arbitration
					uint8_t sdaState = (pI2C->data.sdaPin.pPort->IDR & pI2C->data.sdaPin.pin) ? 1 : 0;
					uint8_t sclState = (pI2C->data.sclPin.pPort->IDR & pI2C->data.sclPin.pin) ? 1 : 0;
					if(!sdaState || !sclState)
					{
						pI2C->transferResult = I2C_RESULT_ERROR;

						pI2C->opState = 0;
						pI2C->hlState = HL_STATE_STOP;
					}
					else
					{
						SDA_LOW();
						++pI2C->opState;
					}
				}
				break;
				case 2:
				{
					SCL_LOW();

					pI2C->opState = 0;
					pI2C->hlState = HL_STATE_ADDR;
				}
				break;
			}
		}
		break;
		case HL_STATE_ADDR:
		{
			uint8_t ack = 0;
			uint8_t readOnly = pI2C->pTxData == 0 ? 1 : 0;

			if(writeByte(pI2C, (pI2C->addr << 1) | readOnly, &ack))
			{
				if(ack)
				{
					if(readOnly)
					{
						if(pI2C->pRxData)
						{
							pI2C->opState = 0;
							pI2C->hlState = HL_STATE_READ_BYTE;
						}
						else
						{
							pI2C->opState = 0;
							pI2C->hlState = HL_STATE_STOP;
						}
					}
					else
					{
						pI2C->opState = 0;
						pI2C->hlState = HL_STATE_WRITE_BYTE;
					}
				}
				else
				{
					pI2C->transferResult = I2C_RESULT_NACK;

					pI2C->opState = 0;
					pI2C->hlState = HL_STATE_STOP;
				}
			}
		}
		break;
		case HL_STATE_STOP:
		{
			switch(pI2C->opState)
			{
				case 0:
				{
					SCL_LOW();
					SDA_LOW();
					++pI2C->opState;
				}
				break;
				case 1:
				{
					SCL_HIGH();
					++pI2C->opState;
				}
				break;
				case 2:
				{
					SDA_HIGH();

					pI2C->hlState = HL_STATE_DONE;

					(*pBase->doneCallback)(pI2C->transferResult, pBase->pCallbackData);
				}
				break;
			}
		}
		break;
		case HL_STATE_WRITE_BYTE:
		{
			uint8_t ack = 0;

			if(writeByte(pI2C, *pI2C->pTxData, &ack))
			{
				++pI2C->pTxData;
				pI2C->opState = 0;

				if(pI2C->pTxData == pI2C->pTxDataEnd)
				{
					pI2C->pTxData = 0;

					if(pI2C->pRxData)
					{
						pI2C->hlState = HL_STATE_START;
					}
					else
					{
						pI2C->hlState = HL_STATE_STOP;
					}
				}
			}
		}
		break;
		case HL_STATE_READ_BYTE:
		{
			uint8_t ack = pI2C->pRxData != pI2C->pRxDataEnd-1;

			if(readByte(pI2C, pI2C->pRxData, ack))
			{
				++pI2C->pRxData;
				pI2C->opState = 0;

				if(pI2C->pRxData == pI2C->pRxDataEnd)
				{
					pI2C->hlState = HL_STATE_STOP;
				}
			}
		}
		break;
		default:
			break;
	}

	if(pI2C->hlState != HL_STATE_DONE)
	{
		pI2C->data.pTimer->CR1 |= TIM_CR1_CEN;
	}
}

void I2CLLDSoftInit(I2CLLDSoft* pI2C, I2CLLDSoftData* pInit)
{
	pI2C->base.pVTable = &i2cSoftVTable;
	pI2C->base.doneCallback = &dummyDoneCallback;
	pI2C->base.pCallbackData = 0;

	pI2C->data = *pInit;

	SDA_HIGH();
	SCL_HIGH();

	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_OUTPUT;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.otype = GPIO_OTYPE_OPEN_DRAIN;
	gpioInit.pupd = GPIO_PUPD_NONE;

	GPIOInit(pInit->sclPin.pPort, pInit->sclPin.pin, &gpioInit);
	GPIOInit(pInit->sdaPin.pPort, pInit->sdaPin.pin, &gpioInit);
}

static void* i2cSoftGetTxBuf(I2CLLD* pBase)
{
	I2CLLDSoft* pI2C = (I2CLLDSoft*)pBase;

	return pI2C->data.pBufTx;
}

static void* i2cSoftGetRxBuf(I2CLLD* pBase)
{
	I2CLLDSoft* pI2C = (I2CLLDSoft*)pBase;

	return pI2C->data.pBufRx;
}

static uint32_t i2cSoftGetBufSize(I2CLLD* pBase)
{
	I2CLLDSoft* pI2C = (I2CLLDSoft*)pBase;

	return pI2C->data.bufSize;
}

static int16_t i2cSoftTransfer(I2CLLD* pBase, uint8_t addr, uint8_t txLength, uint8_t rxLength)
{
	I2CLLDSoft* pI2C = (I2CLLDSoft*)pBase;

	if(txLength + rxLength > 255)
		return ERROR_INVALID_PARAMETER;

	if(pI2C->hlState != HL_STATE_DONE)
		return ERROR_DEVICE_BUSY;

	pI2C->abortPending = 0;
	pI2C->addr = addr;

	if(txLength)
		pI2C->pTxData = pI2C->data.pBufTx;
	else
		pI2C->pTxData = 0;

	pI2C->pTxDataEnd = pI2C->pTxData + txLength;

	if(rxLength)
		pI2C->pRxData = pI2C->data.pBufRx;
	else
		pI2C->pRxData = 0;

	pI2C->pRxDataEnd = pI2C->pRxData + rxLength;

	pI2C->hlState = HL_STATE_IDLE;

	pI2C->data.pTimer->CR1 |= TIM_CR1_CEN;

	return 0;
}

static void i2cSoftAbort(I2CLLD* pBase)
{
	I2CLLDSoft* pI2C = (I2CLLDSoft*)pBase;

	pI2C->abortPending = 1;
}

static uint8_t i2cSoftIsActive(I2CLLD* pBase)
{
	I2CLLDSoft* pI2C = (I2CLLDSoft*)pBase;

	return pI2C->hlState != HL_STATE_DONE;
}
