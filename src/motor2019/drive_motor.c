/*
 * drive_motor.c
 *
 *  Created on: 25.01.2019
 *      Author: AndreR
 */

/**
 * Hall notes:
 * TIM2:
 * - Full 48MHz
 * - XOR (TI1S in CR2)
 * - Trigger selection TI1F_ED
 * - Slave in Reset Mode
 * - Timer resets with hall sensor change
 * - CCR1 in input capture on TRC => time between commutations
 */

#include "main.h"
#include "hal/system_init.h"
#include "drive_motor.h"

#define CCMR1_PHASE1_OFF (TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_2)
#define  CCER_PHASE1_OFF (TIM_CCER_CC1E)
#define CCMR1_PHASE1_PWM2 (TIM_CCMR1_OC1PE | TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1CE)
#define  CCER_PHASE1_PWM2 (TIM_CCER_CC1E | TIM_CCER_CC1NE)

#define CCMR1_PHASE2_OFF (TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_2)
#define  CCER_PHASE2_OFF (TIM_CCER_CC2E)
#define CCMR1_PHASE2_PWM2 (TIM_CCMR1_OC2PE | TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2CE)
#define  CCER_PHASE2_PWM2 (TIM_CCER_CC2E | TIM_CCER_CC2NE)

#define CCMR2_PHASE3_OFF (TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_2)
#define  CCER_PHASE3_OFF (TIM_CCER_CC3E)
#define CCMR2_PHASE3_PWM2 (TIM_CCMR2_OC3PE | TIM_CCMR2_OC3M_0 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3CE)
#define  CCER_PHASE3_PWM2 (TIM_CCER_CC3E | TIM_CCER_CC3NE)

// Channel 4 is used to trigger ADC
#define CCMR2_PHASE4_PWM2 (TIM_CCMR2_OC4M_0 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE)
#define  CCER_PHASE4_PWM2 (TIM_CCER_CC4E)

static void encInit();
static void hallInit();

DriveMotorData driveMotor = {
	.dmaData = {
	// [ CCR1 CCR2 CCR3 ]
		    0,  40,  40, //  ON OFF OFF
		   40,   0,  40, // OFF  ON OFF
		   40,  40,   0, // OFF OFF  ON
		   40,  40,   0,
		   40,   0,  40,
		    0,  40,  40,
		   40,  40,  40, // OFF OFF OFF
		 2000,2000,2000, // long PWM cycle, updated by control
	}
};

void TIM1_BRK_UP_TRG_COM_IRQHandler()
{
	static uint32_t cnt = 0;

	TIM1->SR &= ~TIM_SR_UIF;

	if(cnt == 7)
	{
		TIM1->ARR = 1004;
	}
	else
	{
		TIM1->ARR = 28;
	}

	++cnt;
	cnt &= 0x07;
}

void DriveMotorInit()
{
	driveMotor.currentMode = MOTOR_MODE_OFF;

	hallInit();
	encInit();

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
	TIM1->ARR = 1004;
	TIM1->RCR = 0;
	// ARR is preloaded, Center-Aligned Mode 1, compare flag for encoder trigger only when downcounting, t_DTS = 2*t_CKINT
	TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_CMS_0 | TIM_CR1_CKD_0;
	// CCxE, CCxNE, OCxM are preloaded, update with COM
	TIM1->CR2 = TIM_CR2_CCPC;
	// OCREF_CLR_INT connected to ETRF, external trigger filter = 10.67us
	// ETRF on TRGI, Slave mode controller in trigger mode (clear events now visible on TIF bit in SR)
	TIM1->SMCR = TIM_SMCR_OCCS | TIM_SMCR_ETF | TIM_SMCR_TS | TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1;
	TIM1->DIER = 0;
	TIM1->SR = 0;
	// CCR registers preloaded, all half-bridges off, OCxRef cleared by ETRF
	TIM1->CCMR1 = CCMR1_PHASE1_OFF | CCMR1_PHASE2_OFF;
	TIM1->CCMR2 = CCMR2_PHASE3_OFF | CCMR2_PHASE4_PWM2;
	TIM1->CCER = CCER_PHASE1_OFF | CCER_PHASE2_OFF | CCER_PHASE3_OFF | CCER_PHASE4_PWM2;
	TIM1->CCR1 = 1004;
	TIM1->CCR2 = 1004;
	TIM1->CCR3 = 1004;
	TIM1->CCR4 = 20;
	TIM1->EGR = TIM_EGR_UG | TIM_EGR_COMG;
	// Outputs forced to inactive level in idle (MOE=0), 250ns dead time, active high break input
	TIM1->BDTR = TIM_BDTR_OSSR | TIM_BDTR_OSSI | (6 << TIM_BDTR_DTG_Pos) | TIM_BDTR_BKE | TIM_BDTR_BKP;
	// Configure DMA: target = CCR1, 3 transfers => CCR1 to CCR3
	TIM1->DCR = (2 << TIM_DCR_DBL_Pos) | (13 << TIM_DCR_DBA_Pos);

	// Enable SRAM parity checks and link it with hardfault locks to TIM1 break input
	SYSCFG->CFGR2 |= SYSCFG_CFGR2_SRAM_PARITY_LOCK | SYSCFG_CFGR2_LOCKUP_LOCK;

	// DMA1_CH5 used to write new timer data to ARR/CCMR registers at update event
	// TIM1_UP, medium priority, memory increment, memory to peripheral, 16bit data size, circular mode
	DMA1_Channel5->CCR = DMA_CCR_MINC | DMA_CCR_PL_0 | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_CIRC | DMA_CCR_DIR;
	DMA1_Channel5->CMAR = (uint32_t)driveMotor.dmaData;
	DMA1_Channel5->CPAR = (uint32_t)&TIM1->DMAR;
	DMA1_Channel5->CNDTR = 24;

	DMA1->IFCR = DMA_IFCR_CGIF5;
	DMA1_Channel5->CCR |= DMA_CCR_EN;

	// DMA1_CH4 used to capture encoder position at ADC trigger events
	// TIM1_CH4, very high priority, memory increment, peripheral to memory, 16bit data size, circular mode
	DMA1_Channel4->CCR = DMA_CCR_MINC | DMA_CCR_PL | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_CIRC;
	DMA1_Channel4->CMAR = (uint32_t)driveMotor.encDmaData;
	DMA1_Channel4->CPAR = (uint32_t)&TIM3->CNT;
	DMA1_Channel4->CNDTR = 8;

	DMA1->IFCR = DMA_IFCR_CGIF4;
	DMA1_Channel4->CCR |= DMA_CCR_EN;

	irqTable[TIM1_BRK_UP_TRG_COM_IRQn] = &TIM1_BRK_UP_TRG_COM_IRQHandler;
	NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 0);
	NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);

	TIM1->SR = 0;
	TIM1->BDTR |= TIM_BDTR_MOE; // enable output
	TIM1->CR1 |= TIM_CR1_CEN; // enable timer
	TIM1->EGR = TIM_EGR_TG; // and start it
	TIM1->SR = 0; // clear TIF that was just provoked by EGR_TG

	TIM1->DIER |= TIM_DIER_UIE | TIM_DIER_UDE | TIM_DIER_CC4DE; // enable DMA transfers

	TIM1->RCR = 1; // now set RCR, update events from now on only at underflow
	TIM1->EGR = TIM_EGR_UG; // and update timer once to load first DMA set and take RCR in account
}

#define GPIO_PIN_ENCODER (GPIO_PIN_6 | GPIO_PIN_7)

static void hallInit()
{
	GPIOInitData gpioInit;
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
	TIM2->DIER = 0;
	// IC1 is mapped to TI1 filtered (XOR output)
	TIM2->CCMR1 = TIM_CCMR1_CC1S | (7 << TIM_CCMR1_IC1F_Pos);
	TIM2->CCMR2 = 0;
	// CCR1 in input mode, capture all flanks of TI1F
	TIM2->CCER = TIM_CCER_CC1E;
	TIM2->CNT = 0;
	TIM2->PSC = 0; // => 48MHz
	TIM2->ARR = UINT32_MAX;
	TIM2->EGR = TIM_EGR_UG;

	TIM2->CR1 |= TIM_CR1_CEN;
}

static void encInit()
{
	uint32_t timSMCR = TIM_SMCR_TS_0;

	GPIOInitData gpioInit;
	// TODO: disabled until proven to work or removed
/*	gpioInit.mode = GPIO_MODE_INPUT;
	gpioInit.pupd = GPIO_PUPD_DOWN;
	GPIOInit(GPIOA, GPIO_PIN_ENCODER, &gpioInit);

	SystemWaitMs(10);

	uint16_t pullDownState = GPIOA->IDR & GPIO_PIN_ENCODER;

	gpioInit.pupd = GPIO_PUPD_UP;
	GPIOInit(GPIOA, GPIO_PIN_ENCODER, &gpioInit);

	SystemWaitMs(10);

	uint16_t pullUpState = GPIOA->IDR & GPIO_PIN_ENCODER;

	if(pullDownState == 0 && pullUpState == GPIO_PIN_ENCODER)
		data.sensors.encoder.installed = 0;
	else
		data.sensors.encoder.installed = 1;
*/
	data.sensors.encoder.installed = 1;

	if(data.sensors.encoder.installed)
	{
		timSMCR |= (3 << TIM_SMCR_SMS_Pos);
	}

	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.alternate = 1;
	gpioInit.pupd = GPIO_PUPD_NONE;
	GPIOInit(GPIOA, GPIO_PIN_ENCODER, &gpioInit);

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	// Encoder max 23040cpr, 6080 RPM, 0.43us edge separation => 2.3MHz
	TIM3->CR1 = TIM_CR1_ARPE;
	TIM3->CR2 = 0;
	TIM3->PSC = data.sensors.encoder.installed ? 0 : 0xFFFF;
	TIM3->ARR = 2879;
	TIM3->DIER = 0;
	TIM3->EGR = TIM_EGR_UG;
	TIM3->SR = 0;
	// if installed: encoder mode 3, count all flanks
	// always: Internal Trigger 1 (TIM2) on TRC
	TIM3->SMCR = timSMCR;
	// IC1 mapped to TI1, IC2 mapped to TI2, filter = 0x02 on both (4 samples at 48MHz), no prescaler
	TIM3->CCMR1 = TIM_CCMR1_CC1S_0 | (2 << TIM_CCMR1_IC1F_Pos) | TIM_CCMR1_CC2S_0 | (2 << TIM_CCMR1_IC2F_Pos);
	// IC3 mapped to TRC (TIM2 TRGO) => capture encoder position at hall change
	TIM3->CCMR2 = TIM_CCMR2_CC3S;
	TIM3->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;	// enable input capture, rising polarity

	TIM3->CR1 |= TIM_CR1_CEN;
}

static inline int32_t voltageToPwm(int32_t voltage)
{
	int32_t pwm_S12_4;

	if(voltage >= (data.sensors.adc.vSupply_Q16_0 >> 1))
		pwm_S12_4 = 4095 << 4;
	else if(voltage <= -(data.sensors.adc.vSupply_Q16_0 >> 1))
		pwm_S12_4 = -(4096 << 4);
	else
		pwm_S12_4 = ((voltage * data.sensors.adc.vSupplyReciprocal_Q0_31) >> 14);

	return pwm_S12_4;
}

static inline void getPhaseVoltages(int32_t alpha_S12_15, int32_t beta_S12_15, int32_t* pVp)
{
	// Va = Valpha;
	// Vb = -1/2 * Valpha + sqrt(3)/2*Vbeta;
	// Vc = -1/2 * Valpha - sqrt(3)/2*Vbeta;

	int32_t alphaHalfNeg_S12_15 = -(alpha_S12_15 >> 1);
	int32_t betaSqrt3Half_S12_15 = ((beta_S12_15 >> 6)*887) >> 4;

	pVp[0] = alpha_S12_15;
	pVp[1] = alphaHalfNeg_S12_15 + betaSqrt3Half_S12_15;
	pVp[2] = alphaHalfNeg_S12_15 - betaSqrt3Half_S12_15;

	// power-invariant Clarke transform
//	int32_t alphaOverSqrt6_S12_15 = ((alpha_S12_15 >> 8)*1672) >> 4;
//	int32_t betaOverSqrt2_S12_15 = ((beta_S12_15 >> 8)*2896) >> 4;
//
//	pVp[0] = alphaOverSqrt6_S12_15*2;
//	pVp[1] = -alphaOverSqrt6_S12_15 + betaOverSqrt2_S12_15;
//	pVp[2] = -alphaOverSqrt6_S12_15 - betaOverSqrt2_S12_15;
}

static inline void getSortIndices(const int32_t* pIn, uint8_t* pSortIndices)
{
	// compare and LUT to find min, mid, max values
	// Sector:                              0  1  2  3  4  5
	uint8_t aLargerB = pIn[0] >= pIn[1]; // 0  1  1  1  0  0
	uint8_t bLargerC = pIn[1] >= pIn[2]; // 1  1  0  0  0  1
	uint8_t cLargerA = pIn[2] > pIn[0];  // 0  0  0  1  1  1
	uint8_t sectorGray = aLargerB | bLargerC << 1 | cLargerA << 2;

	static const uint8_t vLowIndex[] = { 0, 1, 2, 2, 0, 1, 0, 0 }; // -i
	static const uint8_t vMidIndex[] = { 0, 2, 0, 1, 1, 0, 2, 0 };
	static const uint8_t vHigIndex[] = { 0, 0, 1, 0, 2, 2, 1, 0 }; // +i

	pSortIndices[0] = vLowIndex[sectorGray];
	pSortIndices[1] = vMidIndex[sectorGray];
	pSortIndices[2] = vHigIndex[sectorGray];
}

// alpha, beta: +-4096 PWM value
static void setAlphaBeta7Segment(int32_t alpha_S12_15, int32_t beta_S12_15)
{
	int32_t Vp_S12_15[3];

	getPhaseVoltages(alpha_S12_15, beta_S12_15, Vp_S12_15);

	uint8_t sortIndices[3];
	getSortIndices(Vp_S12_15, sortIndices);

	int32_t VLow_S12_15 = Vp_S12_15[sortIndices[0]];
	int32_t VHigh_S12_15 = Vp_S12_15[sortIndices[2]];

	int32_t Vcom = (VLow_S12_15 + VHigh_S12_15) >> 1;
	const int32_t offset = 1 << 27;

	int32_t Vp_U13_15[3];
	Vp_U13_15[0] = ((Vp_S12_15[0] - Vcom) >> 7)*153 + offset;
	Vp_U13_15[1] = ((Vp_S12_15[1] - Vcom) >> 7)*153 + offset;
	Vp_U13_15[2] = ((Vp_S12_15[2] - Vcom) >> 7)*153 + offset;

	if(Vp_U13_15[0] > (8192 << 15))
		Vp_U13_15[0] = (8192 << 15);

	if(Vp_U13_15[1] > (8192 << 15))
		Vp_U13_15[1] = (8192 << 15);

	if(Vp_U13_15[2] > (8192 << 15))
		Vp_U13_15[2] = (8192 << 15);

	// bring 0-8192 to 0-1004 range
	// U13.15 >> 10 = U13.5 * U11.0 = U24.5 >> 18 = U11.0
	driveMotor.dmaData[21] = 1004L - (((Vp_U13_15[0] >> 10) * 1004) >> 18);
	driveMotor.dmaData[22] = 1004L - (((Vp_U13_15[1] >> 10) * 1004) >> 18);
	driveMotor.dmaData[23] = 1004L - (((Vp_U13_15[2] >> 10) * 1004) >> 18);
}

// alpha, beta: +-65536 [mV]
void DriveMotorSetAlphaBeta(int32_t alpha_S16_0, int32_t beta_S16_0)
{
	int32_t alpha_S12_15 = voltageToPwm(alpha_S16_0) << 11;
	int32_t beta_S12_15 = voltageToPwm(beta_S16_0) << 11;

	setAlphaBeta7Segment(alpha_S12_15, beta_S12_15);
}

void DriveMotorUpdate()
{
	if(driveMotor.currentMode == data.motor.mode)
		return;

	if(data.motor.mode == MOTOR_MODE_OFF)
	{
		TIM1->CCMR1 = CCMR1_PHASE1_OFF | CCMR1_PHASE2_OFF;
		TIM1->CCMR2 = CCMR2_PHASE3_OFF | CCMR2_PHASE4_PWM2;
		TIM1->CCER = CCER_PHASE1_OFF | CCER_PHASE2_OFF | CCER_PHASE3_OFF | CCER_PHASE4_PWM2;
		TIM1->EGR = TIM_EGR_COMG;
	}
	else
	{
		TIM1->CCMR1 = CCMR1_PHASE1_PWM2 | CCMR1_PHASE2_PWM2;
		TIM1->CCMR2 = CCMR2_PHASE3_PWM2 | CCMR2_PHASE4_PWM2;
		TIM1->CCER = CCER_PHASE1_PWM2 | CCER_PHASE2_PWM2 | CCER_PHASE3_PWM2 |  CCER_PHASE4_PWM2;
		TIM1->EGR = TIM_EGR_COMG;
	}

	driveMotor.currentMode = data.motor.mode;
}
