/*
 * i2c.c
 *
 *  Created on: 06.01.2019
 *      Author: AndreR
 */

#include "ch.h"
#include "i2c.h"

#define EVENT_TRANSFER_COMPLETE	0
#define EVENT_NACK 				1
#define EVENT_ERROR				2

void I2CEventIRQ(I2C* pI2C)
{
	CH_IRQ_PROLOGUE();

	// handles TC/TCR, TXIS, RXNE, STOPF, ADDR, NACKF

	uint32_t isr = pI2C->pRegister->ISR;

	if(isr & I2C_ISR_TXE)
	{
		pI2C->pRegister->TXDR = *pI2C->pTxData++;
	}

	if(isr & I2C_ISR_RXNE)
	{
		*pI2C->pRxData = pI2C->pRegister->RXDR;
		pI2C->pRxData++;
	}

	if(isr & I2C_ISR_TC)
	{
		pI2C->pRegister->CR2 |= I2C_CR2_STOP;
	}

	if(isr & I2C_ISR_STOPF)
	{
		pI2C->pRegister->CR1 = 0;

		chSysLockFromISR();
		chMBPostI(&pI2C->eventQueue, EVENT_TRANSFER_COMPLETE);
		chSysUnlockFromISR();
	}

	if(isr & I2C_ISR_NACKF)
	{
		pI2C->pRegister->CR1 = 0;

		chSysLockFromISR();
		chMBPostI(&pI2C->eventQueue, EVENT_NACK);
		chSysUnlockFromISR();
	}

	CH_IRQ_EPILOGUE();
}

void I2CErrorIRQ(I2C* pI2C)
{
	CH_IRQ_PROLOGUE();

	// handles BERR, OVR, ARLO, TIMEOUT, ALERT, PECERR
	// required: ARLO, BERR

	pI2C->pRegister->CR1 = 0;

	chSysLockFromISR();
	chMBPostI(&pI2C->eventQueue, EVENT_ERROR);
	chSysUnlockFromISR();

	CH_IRQ_EPILOGUE();
}

void I2CInit(I2C* pI2C, I2C_TypeDef* pI2CRegister)
{
	pI2C->pRegister = pI2CRegister;

	chMtxObjectInit(&pI2C->mtxInUse);
	chMBObjectInit(&pI2C->eventQueue, pI2C->eventQueueData, 10);

	// Recommended settings for 200kHz Fast-Mode I2C
	pI2C->pRegister->TIMINGR = (12 << I2C_TIMINGR_PRESC_Pos)
			| (19 << I2C_TIMINGR_SCLL_Pos) | (15 << I2C_TIMINGR_SCLH_Pos)
			| (2 << I2C_TIMINGR_SDADEL_Pos)	| (4 << I2C_TIMINGR_SCLDEL_Pos);
}


int16_t I2CWrite(I2C* pI2C, uint8_t addr, uint8_t length, uint8_t* pData)
{
	int16_t result = 0;

	chMtxLock(&pI2C->mtxInUse);

	chMBReset(&pI2C->eventQueue);

	pI2C->pTxData = pData;

	pI2C->pRegister->CR1 |= I2C_CR1_PE;
	pI2C->pRegister->CR1 |= I2C_CR1_ERRIE | I2C_CR1_NACKIE | I2C_CR1_TXIE | I2C_CR1_STOPIE | I2C_CR1_TCIE;

	// start transfer
	pI2C->pRegister->CR2 = (addr << 1) | (length << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;

	msg_t event;
	msg_t chResult = chMBFetch(&pI2C->eventQueue, &event, MS2ST(100));
	if(chResult != MSG_OK)
	{
		pI2C->pRegister->CR1 = 0;
		result = -1;
	}

	result = event;

	chMtxUnlock(&pI2C->mtxInUse);

	return result;
}

int16_t I2CRead(I2C* pI2C, uint8_t addr, uint8_t length, uint8_t* pData)
{
	int16_t result = 0;

	chMtxLock(&pI2C->mtxInUse);

	chMBReset(&pI2C->eventQueue);

	pI2C->pRxData = pData;

	pI2C->pRegister->CR1 |= I2C_CR1_PE;
	pI2C->pRegister->CR1 |= I2C_CR1_ERRIE | I2C_CR1_NACKIE | I2C_CR1_RXIE | I2C_CR1_STOPIE | I2C_CR1_TCIE;

	// start transfer
	pI2C->pRegister->CR2 = (addr << 1) | (length << I2C_CR2_NBYTES_Pos) | I2C_CR2_RD_WRN | I2C_CR2_START;

	msg_t event;
	msg_t chResult = chMBFetch(&pI2C->eventQueue, &event, MS2ST(100));
	if(chResult != MSG_OK)
	{
		pI2C->pRegister->CR1 = 0;
		result = -1;
	}

	result = event;

	chMtxUnlock(&pI2C->mtxInUse);

	return result;
}
