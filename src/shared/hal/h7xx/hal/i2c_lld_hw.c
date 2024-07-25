#include "i2c_lld_hw.h"
#include "errors.h"

static void* i2cHwGetTxBuf(I2CLLD* pBase);
static void* i2cHwGetRxBuf(I2CLLD* pBase);
static uint32_t i2cHwGetBufSize(I2CLLD* pBase);
static int16_t i2cHwTransfer(I2CLLD* pBase, uint8_t addr, uint8_t txLength, uint8_t rxLength);
static void i2cHwAbort(I2CLLD* pBase);
static uint8_t i2cHwIsActive(I2CLLD* pBase);

static const I2CLLDVTable i2cHwVTable = {
	.getTxBuf = &i2cHwGetTxBuf,
	.getRxBuf = &i2cHwGetRxBuf,
	.getBufSize = &i2cHwGetBufSize,
	.transfer = &i2cHwTransfer,
	.abort = &i2cHwAbort,
	.isActive = &i2cHwIsActive,
};

static void dummyDoneCallback(int16_t result, void* pData)
{
	(void)result;
	(void)pData;
}

void I2CLLDHWEventIRQ(I2CLLDHW* pI2C)
{
	// handles TC/TCR, TXIS, RXNE, STOPF, ADDR, NACKF

	I2CLLD* pBase = &pI2C->base;

	uint32_t isr = pI2C->data.pRegister->ISR;

	if((isr & I2C_ISR_TXE) && pI2C->pTxData)
	{
		pI2C->data.pRegister->TXDR = *pI2C->pTxData++;
	}

	if((isr & I2C_ISR_RXNE) && pI2C->pRxData)
	{
		*pI2C->pRxData = pI2C->data.pRegister->RXDR;
		pI2C->pRxData++;
	}

	if(isr & I2C_ISR_TC)
	{
		if(pI2C->pTxData && pI2C->pRxData)
		{
			// TX + RX transfer, configure to read data
			pI2C->pTxData = 0;

			pI2C->data.pRegister->CR1 &= ~(I2C_CR1_TXIE);
			pI2C->data.pRegister->CR1 |= I2C_CR1_RXIE;

			// repeated start
			uint32_t addr = pI2C->data.pRegister->CR2 & I2C_CR2_SADD_Msk;
			pI2C->data.pRegister->CR2 = addr | (pI2C->rxDataLength << I2C_CR2_NBYTES_Pos) | I2C_CR2_RD_WRN | I2C_CR2_START;
		}
		else
		{
			pI2C->data.pRegister->CR2 |= I2C_CR2_STOP;
		}
	}

	if(isr & I2C_ISR_STOPF)
	{
		pI2C->data.pRegister->CR1 = 0;
		pI2C->isActive = 0;

		(*pBase->doneCallback)(I2C_RESULT_TRANSFER_COMPLETE, pBase->pCallbackData);
	}

	if(isr & I2C_ISR_NACKF)
	{
		pI2C->data.pRegister->CR1 = 0;
		pI2C->isActive = 0;

		(*pBase->doneCallback)(I2C_RESULT_NACK, pBase->pCallbackData);
	}
}

void I2CLLDHWErrorIRQ(I2CLLDHW* pI2C)
{
	// handles BERR, OVR, ARLO, TIMEOUT, ALERT, PECERR
	// required: ARLO, BERR

	I2CLLD* pBase = &pI2C->base;

	pI2C->data.pRegister->CR1 = 0;

	uint8_t wasActive = pI2C->isActive;
	pI2C->isActive = 0;

	// Need to check if I2C was active, this may have been triggered by an abort call while not active.
	if(wasActive)
		(*pBase->doneCallback)(I2C_RESULT_ERROR, pBase->pCallbackData);
}

void I2CLLDHWInit(I2CLLDHW* pI2C, I2CLLDHWData* pInit)
{
	pI2C->base.pVTable = &i2cHwVTable;
	pI2C->base.doneCallback = &dummyDoneCallback;
	pI2C->base.pCallbackData = 0;

	pI2C->data = *pInit;
	pI2C->isActive = 0;

	// Recommended settings for 200kHz Fast-Mode I2C
	pI2C->data.pRegister->TIMINGR = (12 << I2C_TIMINGR_PRESC_Pos)
			| (19 << I2C_TIMINGR_SCLL_Pos) | (15 << I2C_TIMINGR_SCLH_Pos)
			| (2 << I2C_TIMINGR_SDADEL_Pos)	| (4 << I2C_TIMINGR_SCLDEL_Pos);

	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.otype = GPIO_OTYPE_OPEN_DRAIN;
	gpioInit.pupd = GPIO_PUPD_NONE;

	gpioInit.alternate = pInit->sclPin.alternate;
	GPIOInit(pInit->sclPin.pPort, pInit->sclPin.pin, &gpioInit);

	gpioInit.alternate = pInit->sdaPin.alternate;
	GPIOInit(pInit->sdaPin.pPort, pInit->sdaPin.pin, &gpioInit);
}

static void* i2cHwGetTxBuf(I2CLLD* pBase)
{
	I2CLLDHW* pI2C = (I2CLLDHW*)pBase;

	return pI2C->data.pBufTx;
}

static void* i2cHwGetRxBuf(I2CLLD* pBase)
{
	I2CLLDHW* pI2C = (I2CLLDHW*)pBase;

	return pI2C->data.pBufRx;
}

static uint32_t i2cHwGetBufSize(I2CLLD* pBase)
{
	I2CLLDHW* pI2C = (I2CLLDHW*)pBase;

	return pI2C->data.bufSize;
}

static int16_t i2cHwTransfer(I2CLLD* pBase, uint8_t addr, uint8_t txLength, uint8_t rxLength)
{
	I2CLLDHW* pI2C = (I2CLLDHW*)pBase;

	if(txLength + rxLength > 255)
		return ERROR_INVALID_PARAMETER;

	if(pI2C->isActive)
		return ERROR_DEVICE_BUSY;

	pI2C->isActive = 1;

	if(txLength)
		pI2C->pTxData = pI2C->data.pBufTx;
	else
		pI2C->pTxData = 0;

	if(rxLength)
		pI2C->pRxData = pI2C->data.pBufRx;
	else
		pI2C->pRxData = 0;

	pI2C->rxDataLength = rxLength;

	pI2C->data.pRegister->CR1 |= I2C_CR1_PE;

	// start transfer
	if(txLength)
	{
		pI2C->data.pRegister->CR1 |= I2C_CR1_ERRIE | I2C_CR1_NACKIE | I2C_CR1_TXIE | I2C_CR1_STOPIE | I2C_CR1_TCIE;
		pI2C->data.pRegister->CR2 = (addr << 1) | (txLength << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	}
	else
	{
		pI2C->data.pRegister->CR1 |= I2C_CR1_ERRIE | I2C_CR1_NACKIE | I2C_CR1_RXIE | I2C_CR1_STOPIE | I2C_CR1_TCIE;
		pI2C->data.pRegister->CR2 = (addr << 1) | (rxLength << I2C_CR2_NBYTES_Pos) | I2C_CR2_RD_WRN | I2C_CR2_START;
	}

	return 0;
}

static void i2cHwAbort(I2CLLD* pBase)
{
	I2CLLDHW* pI2C = (I2CLLDHW*)pBase;

	IRQn_Type errIrq;

	switch((uint32_t)pI2C->data.pRegister)
	{
		case I2C1_BASE: errIrq = I2C1_ER_IRQn; break;
		case I2C2_BASE: errIrq = I2C2_ER_IRQn; break;
		case I2C3_BASE: errIrq = I2C3_ER_IRQn; break;
		case I2C4_BASE: errIrq = I2C4_ER_IRQn; break;
		default: return;
	}

	// Error IRQ will clear isActive and execute callback
	NVIC_SetPendingIRQ(errIrq);
}

static uint8_t i2cHwIsActive(I2CLLD* pBase)
{
	I2CLLDHW* pI2C = (I2CLLDHW*)pBase;

	return pI2C->isActive;
}
