/*
 * dribbler_motor.c
 *
 *  Created on: 21.08.2020
 *      Author: AndreR
 */

#include "dribbler_motor.h"
#include "main.h"
#include "system_init.h"

/**
 * Motor notes:
 * - CCR4 used for ADC trigger => possible to start before center of PWM
 * - Phase off: CCxE=1, CCxNE=0, force inactive
 * - Phase low: CCxE=0, CCxNE=1, force active
 * - Phase PWM: CCxE=1, CCxNE=1, PWM mode 1
 */

#define CCMR1_PHASE1_OFF  (TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_2)
#define  CCER_PHASE1_OFF  (TIM_CCER_CC1E)
#define CCMR1_PHASE1_LOW  (TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_2)
#define  CCER_PHASE1_LOW  (TIM_CCER_CC1NE)
#define CCMR1_PHASE1_HIGH (TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_2)
#define  CCER_PHASE1_HIGH (TIM_CCER_CC1E)
#define CCMR1_PHASE1_PWM  (TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1CE)
#define  CCER_PHASE1_PWM  (TIM_CCER_CC1E | TIM_CCER_CC1NE)

#define CCMR1_PHASE2_OFF  (TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_2)
#define  CCER_PHASE2_OFF  (TIM_CCER_CC2E)
#define CCMR1_PHASE2_LOW  (TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_2)
#define  CCER_PHASE2_LOW  (TIM_CCER_CC2NE)
#define CCMR1_PHASE2_HIGH (TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_2)
#define  CCER_PHASE2_HIGH (TIM_CCER_CC2E)
#define CCMR1_PHASE2_PWM  (TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2CE)
#define  CCER_PHASE2_PWM  (TIM_CCER_CC2E | TIM_CCER_CC2NE)

#define CCMR2_PHASE3_OFF  (TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_2)
#define  CCER_PHASE3_OFF  (TIM_CCER_CC3E)
#define CCMR2_PHASE3_LOW  (TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_0 | TIM_CCMR2_OC3M_2)
#define  CCER_PHASE3_LOW  (TIM_CCER_CC3NE)
#define CCMR2_PHASE3_HIGH (TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_0 | TIM_CCMR2_OC3M_2)
#define  CCER_PHASE3_HIGH (TIM_CCER_CC3E)
#define CCMR2_PHASE3_PWM  (TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3CE)
#define  CCER_PHASE3_PWM  (TIM_CCER_CC3E | TIM_CCER_CC3NE)

// Channel 4 is used to trigger ADC
#define CCMR2_PHASE4_PWM (TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2)
#define  CCER_PHASE4_PWM (TIM_CCER_CC4E)

typedef struct _ComEntry
{
	uint16_t ccmr1;
	uint16_t ccmr2;
	uint16_t ccer;
} ComEntry;

static const ComEntry comTable[] = {
	{ // invalid hall position => all phases off
		CCMR1_PHASE1_OFF | CCMR1_PHASE2_OFF, CCMR2_PHASE3_OFF | CCMR2_PHASE4_PWM,
		 CCER_PHASE1_OFF |  CCER_PHASE2_OFF | CCER_PHASE3_OFF |  CCER_PHASE4_PWM
	},
	{ // hall pos 1 => 1 low, 2 off, 3 PWM
		CCMR1_PHASE1_LOW | CCMR1_PHASE2_OFF, CCMR2_PHASE3_PWM | CCMR2_PHASE4_PWM,
		 CCER_PHASE1_LOW |  CCER_PHASE2_OFF | CCER_PHASE3_PWM |  CCER_PHASE4_PWM
	},
	{ // hall pos 2 => 1 PWM, 2 low, 3 off
		CCMR1_PHASE1_PWM | CCMR1_PHASE2_LOW, CCMR2_PHASE3_OFF | CCMR2_PHASE4_PWM,
		 CCER_PHASE1_PWM |  CCER_PHASE2_LOW | CCER_PHASE3_OFF |  CCER_PHASE4_PWM
	},
	{ // hall pos 3 => 1 off, 2 low, 3 PWM
		CCMR1_PHASE1_OFF | CCMR1_PHASE2_LOW, CCMR2_PHASE3_PWM | CCMR2_PHASE4_PWM,
		 CCER_PHASE1_OFF |  CCER_PHASE2_LOW | CCER_PHASE3_PWM |  CCER_PHASE4_PWM
	},
	{ // hall pos 4 => 1 off, 2 PWM, 3 low
		CCMR1_PHASE1_OFF | CCMR1_PHASE2_PWM, CCMR2_PHASE3_LOW | CCMR2_PHASE4_PWM,
		 CCER_PHASE1_OFF |  CCER_PHASE2_PWM | CCER_PHASE3_LOW |  CCER_PHASE4_PWM
	},
	{ // hall pos 5 => 1 low, 2 PWM, 3 off
		CCMR1_PHASE1_LOW | CCMR1_PHASE2_PWM, CCMR2_PHASE3_OFF | CCMR2_PHASE4_PWM,
		 CCER_PHASE1_LOW |  CCER_PHASE2_PWM | CCER_PHASE3_OFF |  CCER_PHASE4_PWM
	},
	{ // hall pos 6 => 1 PWM, 2 off, 3 low
		CCMR1_PHASE1_PWM | CCMR1_PHASE2_OFF, CCMR2_PHASE3_LOW | CCMR2_PHASE4_PWM,
		 CCER_PHASE1_PWM |  CCER_PHASE2_OFF | CCER_PHASE3_LOW |  CCER_PHASE4_PWM
	},
	{ // invalid hall position => all low sides on
		CCMR1_PHASE1_LOW | CCMR1_PHASE2_LOW, CCMR2_PHASE3_LOW | CCMR2_PHASE4_PWM,
		 CCER_PHASE1_LOW |  CCER_PHASE2_LOW | CCER_PHASE3_LOW |  CCER_PHASE4_PWM
	}
};

DribblerMotorData dribblerMotor;

void Dribbler_TIM2_IRQHandler()
{
	// hall commutation IRQ
	TIM2->SR &= ~TIM_SR_UIF;

	uint8_t hallPos;

	if(dribblerMotor.enabled)
	{
		hallPos = HALL_GET_POS();
		if(dribblerMotor.direction > 0)
			hallPos ^= 0x07;
	}
	else
	{
		hallPos = 0;
	}

	const ComEntry* pCom = &comTable[hallPos];

	// registers are pre-loaded, update with COM event
	TIM1->CCMR1 = pCom->ccmr1;
	TIM1->CCMR2 = pCom->ccmr2;
	TIM1->CCER = pCom->ccer;
//	TIM1->EGR = TIM_EGR_COMG;
}

void DribblerMotorInit()
{
	dribblerMotor.direction = 1;

	// ### Motor outputs (TIM1) ###
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.alternate = 2;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.pupd = GPIO_PUPD_DOWN;
	GPIOInit(GPIOA, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10, &gpioInit); // HS1, HS2, HS3
	GPIOInit(GPIOA, GPIO_PIN_12, &gpioInit); // OC_COMP => TIM1_ETR
	GPIOInit(GPIOB, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, &gpioInit); // LS1, LS2, LS3

	gpioInit.mode = GPIO_MODE_OUTPUT;
	GPIOInit(GPIOA, GPIO_PIN_11, &gpioInit); // OC_SEL
	GPIOInit(GPIOF, GPIO_PIN_6 | GPIO_PIN_7, &gpioInit); // OC_TH_STBY2, OC_TH_STBY1

	// OC_SEL = 0 => OC comparator output signal is visible only to MCU (default)
	GPIOReset(GPIOA, GPIO_PIN_11);

	// Set OC threshold to 500mV => 10.82A
	GPIOSet(GPIOF, GPIO_PIN_6 | GPIO_PIN_7);

	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	TIM1->PSC = 0; // => 48MHz
	TIM1->ARR = 599; // 40kHz in Center-Aligned Mode
	TIM1->RCR = 0;
	// ARR is preloaded, Center-Aligned Mode 1, compare flag for ADC trigger only when downcounting
	TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_CMS_0;
	// CCxE, CCxNE, OCxM are preloaded, update with COM event
	TIM1->CR2 = TIM_CR2_CCPC;
	// OCREF_CLR_INT connected to ETRF, external trigger filter = 1us
	// ITR1 (TIM2) on TRGI, slave mode disabled
	TIM1->SMCR = TIM_SMCR_OCCS | TIM_SMCR_ETF_3 | TIM_SMCR_TS_0;
	TIM1->DIER = 0;
	TIM1->SR = 0;
	// CCR registers preloaded, all half-bridges off, OCxRef cleared by ETRF
	TIM1->CCMR1 = CCMR1_PHASE1_OFF | CCMR1_PHASE2_OFF;
	TIM1->CCMR2 = CCMR2_PHASE3_OFF | CCMR2_PHASE4_PWM;
	TIM1->CCER = CCER_PHASE1_OFF | CCER_PHASE2_OFF | CCER_PHASE3_OFF | CCER_PHASE4_PWM;
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR4 = 20; // ADC trigger point
	TIM1->EGR = TIM_EGR_UG | TIM_EGR_COMG;
	// Outputs forced to inactive level in idle (MOE=0), 250ns dead time, active high break input
	TIM1->BDTR = TIM_BDTR_OSSR | TIM_BDTR_OSSI | (12 << TIM_BDTR_DTG_Pos) | TIM_BDTR_BKE | TIM_BDTR_BKP;
	// DMA not used
	TIM1->DCR = 0;

	// Enable SRAM parity checks and link it with hardfault locks to TIM1 break input
	SYSCFG->CFGR2 |= SYSCFG_CFGR2_SRAM_PARITY_LOCK | SYSCFG_CFGR2_LOCKUP_LOCK;

	// DMA1_CH4 used to capture encoder position at ADC trigger events
	// TIM1_CH4, very high priority, memory increment, peripheral to memory, 16bit data size, circular mode
//	DMA1_Channel4->CCR = DMA_CCR_MINC | DMA_CCR_PL | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_CIRC;
//	DMA1_Channel4->CMAR = (uint32_t)motor.encDmaData;
//	DMA1_Channel4->CPAR = (uint32_t)&TIM3->CNT;
//	DMA1_Channel4->CNDTR = 8;
//
//	DMA1->IFCR = DMA_IFCR_CGIF4;
//	DMA1_Channel4->CCR |= DMA_CCR_EN;

//	irqTable[TIM1_BRK_UP_TRG_COM_IRQn] = &Dribbler_TIM1_BRK_UP_TRG_COM_IRQHandler;
//	NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 2);
//	NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);

	TIM1->SR = 0;
	TIM1->BDTR |= TIM_BDTR_MOE; // enable output
	TIM1->CR1 |= TIM_CR1_CEN; // enable & start timer
//	TIM1->DIER |= TIM_DIER_COMIE;

//	TIM1->DIER |= TIM_DIER_UIE | TIM_DIER_UDE | TIM_DIER_CC4DE; // enable DMA transfers

//	TIM1->RCR = 1; // now set RCR, update events from now on only at underflow
//	TIM1->EGR = TIM_EGR_UG; // and update timer once to load first DMA set and take RCR in account



	// ### Hall sensor input (TIM2) ###
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.alternate = 2;
	gpioInit.pupd = GPIO_PUPD_DOWN;
	gpioInit.otype = GPIO_OTYPE_OPEN_DRAIN;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	GPIOInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2, &gpioInit);

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	TIM2->CR1 = 0;
	// XOR TI1 to TI3, reset signal used as TRGO
	TIM2->CR2 = TIM_CR2_TI1S;
	// TI1 (filtered) edge detector as TRGI, slave mode: reset
	TIM2->SMCR = TIM_SMCR_TS_2 | TIM_SMCR_SMS_2;
	TIM2->SR = 0;
	TIM2->DIER = TIM_DIER_UIE;
	// IC1 is mapped to TI1 filtered (XOR output)
	TIM2->CCMR1 = TIM_CCMR1_CC1S | (7 << TIM_CCMR1_IC1F_Pos);
	TIM2->CCMR2 = 0;
	// CCR1 in input mode, capture all flanks of TI1F
	TIM2->CCER = TIM_CCER_CC1E;
	TIM2->CNT = 0;
	TIM2->PSC = 0; // => 48MHz
	TIM2->ARR = UINT32_MAX;
	TIM2->EGR = TIM_EGR_UG;

	irqTable[TIM2_IRQn] = &Dribbler_TIM2_IRQHandler;
	NVIC_SetPriority(TIM2_IRQn, 0);
	NVIC_EnableIRQ(TIM2_IRQn);

	TIM2->CR1 |= TIM_CR1_CEN;
}

static inline int16_t voltageToPwm(int32_t voltage)
{
	int32_t pwm; // Q12.0

	if(voltage >= data.sensors.adc.vSupply_Q16_0)
		pwm = 4095;
	else if(voltage <= -data.sensors.adc.vSupply_Q16_0)
		pwm = -4096;
	else
		pwm = ((voltage * data.sensors.adc.vSupplyReciprocal_Q0_31) >> 19);

	return pwm;
}

void DribblerMotorSetOff()
{
	dribblerMotor.enabled = 0;

	const ComEntry* pCom = &comTable[0];
	TIM1->CCMR1 = pCom->ccmr1;
	TIM1->CCMR2 = pCom->ccmr2;
	TIM1->CCER = pCom->ccer;
	TIM1->EGR = TIM_EGR_COMG; // force immediate update
}

// voltage: +-65536 [mV]
void DribblerMotorSetVoltage(int32_t voltage)
{
	if(voltage < 0)
	{
		voltage *= -1;
		dribblerMotor.direction = -1;
	}
	else
	{
		dribblerMotor.direction = 1;
	}

	int32_t pwm_S12_0 = voltageToPwm(voltage);

	// convert 0-4096 to 0-600 (CCR range)
	int32_t ccr = (pwm_S12_0 * 600) >> 12;

	TIM1->CCR1 = ccr;
	TIM1->CCR2 = ccr;
	TIM1->CCR3 = ccr;

	if(dribblerMotor.enabled == 0)
	{
		dribblerMotor.enabled = 1;
		TIM2->EGR = TIM_EGR_UG;
	}
}
