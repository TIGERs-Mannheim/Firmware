/*
 * usart1.c
 *
 *  Created on: 15.01.2019
 *      Author: AndreR
 */

#include "main.h"
#include "system_init.h"
#include "adc.h"
#include <string.h>

#define USART1_RX_SIZE 64

typedef struct _RxEntry
{
	uint8_t data[USART1_RX_SIZE];
	volatile uint16_t length;
} RxEntry;

static RxEntry rxSlots[4];
static uint8_t activeSlot;
static volatile uint8_t txBusy = 0;

void USART1_IRQHandler()
{
	uint32_t sr = USART1->ISR;
	if(sr & USART_ISR_IDLE)
	{
		DMA1_Channel3->CCR &= ~DMA_CCR_EN;
		USART1->ICR = USART_ICR_IDLECF;

		rxSlots[activeSlot].length = USART1_RX_SIZE - DMA1_Channel3->CNDTR;
		++activeSlot;
		activeSlot &= 0x03;

		rxSlots[activeSlot].length = 0;

		DMA1_Channel3->CMAR = (uint32_t)rxSlots[activeSlot].data;
		DMA1_Channel3->CNDTR = USART1_RX_SIZE;
		DMA1_Channel3->CCR |= DMA_CCR_EN;
	}

	if(sr & USART_ISR_TC)
	{
		USART1->CR1 &= ~USART_CR1_TCIE;
		DMA1_Channel2->CCR &= ~DMA_CCR_EN;
		txBusy = 0;
	}
}

void USART1Init()
{
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.alternate = 1;
	gpioInit.ospeed = GPIO_OSPEED_10MHZ;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.pupd = GPIO_PUPD_UP;
	GPIOInit(GPIOA, GPIO_PIN_14 | GPIO_PIN_15, &gpioInit);

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	// 8,E,1 configuration
	USART1->CR1 = USART_CR1_M | USART_CR1_PCE | USART_CR1_TE | USART_CR1_RE;
	USART1->CR2 = 0;
	USART1->CR3 = USART_CR3_DMAT | USART_CR3_DMAR;
	USART1->BRR = 24; // => 2Mbaud/s, approx. 181kB/s with 8,E,1
	USART1->CR1 |= USART_CR1_UE;

	// USART1_TX, medium priority, memory increment, memory to peripheral
	DMA1_Channel2->CCR = DMA_CCR_PL_0 | DMA_CCR_MINC | DMA_CCR_DIR;
	DMA1_Channel2->CMAR = 0;
	DMA1_Channel2->CPAR = (uint32_t)&USART1->TDR;
	DMA1_Channel2->CNDTR = 0;

	// USART1_RX, low priority, memory increment, peripheral to memory
	DMA1_Channel3->CCR = DMA_CCR_MINC;
	DMA1_Channel3->CMAR = (uint32_t)rxSlots[0].data;
	DMA1_Channel3->CPAR = (uint32_t)&USART1->RDR;
	DMA1_Channel3->CNDTR = USART1_RX_SIZE;

	DMA1->IFCR = DMA_IFCR_CGIF2 | DMA_IFCR_CGIF3;

	USART1->ICR = USART_ICR_IDLECF;
	USART1->CR1 |= USART_CR1_IDLEIE;
	DMA1_Channel3->CCR |= DMA_CCR_EN;

	irqTable[USART1_IRQn] = &USART1_IRQHandler;
	NVIC_SetPriority(USART1_IRQn, 10);
	NVIC_EnableIRQ(USART1_IRQn);
}

void USART1Transmit(const uint8_t* pData, uint16_t length)
{
	if(txBusy)
		return;

	txBusy = 1;

	DMA1->IFCR = DMA_IFCR_CGIF2;
	DMA1_Channel2->CMAR = (uint32_t)pData;
	DMA1_Channel2->CNDTR = length;

	USART1->ICR = USART_ICR_TCCF;
	USART1->CR1 |= USART_CR1_TCIE;

	DMA1_Channel2->CCR |= DMA_CCR_EN;
}

uint8_t* USART1Read(uint16_t* pLength)
{
	for(uint8_t i = 0; i < 4; ++i)
	{
		if(rxSlots[i].length != 0)
		{
			*pLength = rxSlots[i].length;
			rxSlots[i].length = 0;
			return rxSlots[i].data;
		}
	}

	return 0;
}

void USART1SendFeedback()
{
	static MotorExchangeMISO miso;
	static uint8_t debugDataDowncounter = 0;

	miso.hallPos = data.sensors.hall.position;
	miso.hallComTime = data.sensors.hall.commutationTime;
	miso.hallInvalidTransitions = data.sensors.hall.invalidTransitions;

	miso.encPos = data.sensors.encoder.position;
	miso.encDelta = data.sensors.encoder.delta;
	miso.encTicks = TIM3->ARR;
	miso.encPrescaler = TIM3->PSC;

	if(data.sensors.encoder.correction > INT16_MAX)
		miso.encCorrection = INT16_MAX;
	else if(data.sensors.encoder.correction < INT16_MIN)
		miso.encCorrection = INT16_MIN;
	else
		miso.encCorrection = data.sensors.encoder.correction;

	miso.idleTicksNoLoad = data.performance.idleIncrements;
	miso.idleTicksActive = data.performance.idleCounter;

	miso.avgSupplyVoltage = adc.avgVoltage_Q16_0;
	miso.avgTemperature = adc.avgTemperature_S15_0;

	miso.avgCurrentUVW[0] = adc.avgCurrentUVW_S14_0[0];
	miso.avgCurrentUVW[1] = adc.avgCurrentUVW_S14_0[1];
	miso.avgCurrentUVW[2] = adc.avgCurrentUVW_S14_0[2];

	miso.avgCurrentDQ[0] = adc.avgCurrentDQ_S14_0[0];
	miso.avgCurrentDQ[1] = adc.avgCurrentDQ_S14_0[1];

	miso.avgVoltageDQ[0] = data.ctrl.avgVoltageDQ[0];
	miso.avgVoltageDQ[1] = data.ctrl.avgVoltageDQ[1];

	miso.velCtrlOutputCurrent = (data.ctrl.speed.output*10800) >> 12;

	miso.flags = 0;

	if(data.sensors.encoder.installed)
		miso.flags |= MOTOR_EXCHANGE_MISO_FLAG_ENCODER_INSTALLED;

	if(data.motor.overcurrentDetected)
	{
		miso.flags |= MOTOR_EXCHANGE_MISO_FLAG_BREAK_ACTIVE;
		data.motor.overcurrentDetected = 0;
	}

	if(data.ctrl.currentD.overload)
		miso.flags |= MOTOR_EXCHANGE_MISO_FLAG_D_CUR_OVERLOAD;

	if(data.ctrl.currentQ.overload)
		miso.flags |= MOTOR_EXCHANGE_MISO_FLAG_Q_CUR_OVERLOAD;

	if(data.ctrl.speed.overload)
		miso.flags |= MOTOR_EXCHANCE_MISO_FLAG_VEL_OVERLOAD;

	if(data.comm.timeout)
		miso.flags |= MOTOR_EXCHANGE_MISO_FLAG_RX_TIMEOUT;

	if(data.command.recordMode)
	{
		if(debugDataDowncounter)
		{
			debugDataDowncounter--;
		}
		else
		{
			miso.hasDebugData = 1;
		}

		ADCSample* pADCSamples = adc.pFinishedSampleBuf;
		for(uint8_t i = 0; i < MOTOR_EXCHANGE_MISO_MAX_MEAS; ++i)
		{
			miso.currentMeas[i][0] = pADCSamples[i].iMotor;
			miso.currentMeas[i][1] = pADCSamples[i].iMotor2;
		}

		USART1Transmit((uint8_t*)&miso, sizeof(MotorExchangeMISO));
	}
	else
	{
		miso.hasDebugData = 0;
		debugDataDowncounter = 1;

		USART1Transmit((uint8_t*)&miso, sizeof(MotorExchangeMISO) - sizeof(miso.currentMeas));
	}
}

void USART1ReceiveCommand()
{
	uint16_t rxLength;
	uint8_t* pRxData;

	while((pRxData = USART1Read(&rxLength)) != 0)
	{
		if(rxLength < sizeof(MotorExchangeMOSI))
			continue;

		memcpy(&data.command, pRxData, sizeof(MotorExchangeMOSI));

		data.comm.lastCommandTimeMs = data.timeMs;
	}
}
