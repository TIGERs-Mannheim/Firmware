/**
 * Motor current calculations to ADC:
Vdd : 3.3 $
Rs : 0.025 $
Rb : 16.2e3 $
Rlp : 1.33e3 $
U_signal : I*Rs*Rb/(Rb+Rlp);
U_bias : Vdd*Rlp/(Rb+Rlp);
U : U_signal + U_bias;
A : (7.32+1.33)/1.33;
U_adc = expand(A*U*2^12/3.3);
s1 : solve(U_adc = expand(A*U*2^12/3.3), I);
float(expand(s1[1]));

I [mA] = 5.361871705068514*ADC_VAL - 10837.03703703705
I = m*ADC + b
ADC: Q12.0 => 0 - 4096
m: Q3.15 => 5.36187744140625 (error -0.000005736337735984254) (5 << 15 | 11858)
b: Q14.15 => 10837.03704833984375 (error -0.00001130279375000098) (10837 << 15 | 1214)
m*ADC => Q15.15 + b => Q16.15

 * Voltage measurement:
R1 : 243e3 $
R2 : 24.1e3 $
(R1+R2)/R2;
ADC = U_in * R2/(R1+R2)*2^12/3.3;
s1 : solve(ADC = U_in * R2/(R1+R2)*2^12/3.3, U_in);
float(s1[1])*1000;

U [mV] = 8.929164775674275*ADC_VAL
U = m*ADC
ADC: Q12.0 => 0 - 4096
m: Q4.15 => 8.929168701171875 (error -0.000003925497599976957) (8 << 15 | 30447)
m*ADC => Q16.15

 * Temperature measurement:
ad(t) := (1.43 + (t-30)*4.3e-3)*4096/3.3;
ts1 : ad(30);
ts2 : ad(110);
tsd : ad(40);
(110-30)/(ts2-ts1)*(tsd-ts1) + 30;
factor : floor(1/(ts2-ts1) * (2^29-1) * 800 * 1/2^12);
floor(f*4096);
floor(%) * 1/2^17.0 + 300;
floor(1/2^7 * (2^29-1) * 800);

 * ADC calculations:
t_adc : 12.5 $
t_smp : 1.5 $
t_latency : 2.625 $
f_adc : 12e6 $
t_conv : (t_adc+t_smp+t_latency)/f_adc;
t_single : (t_adc+t_smp)/f_adc;
T_6s : 6*t_single;
d_6s : 1/2*t_single;
T_5s : 4.5*t_single;
d_5s : 1/4*t_single;
T_4s : 3.6*t_single;
d_4s : 1/10*t_single;
T_4so : 4*t_single;
d_4so : 1/6*t_single;
T_4so/6;
T_4s/6;
*/

#include "adc.h"
#include "main.h"
#include "hal/system_init.h"

ADCData adc;

static void doControl();
static void voltageMeasurementDone();
static void tempMeasurementDone();

void DMA1_Channel1_IRQHandler()
{
	// clear flags
	DMA1->IFCR = DMA_IFCR_CGIF1;
	ADC1->ISR = ADC_ISR_EOSMP | ADC_ISR_EOC | ADC_ISR_EOSEQ | ADC_ISR_OVR;
	
	// configure for voltage measurement
	ADC1->CHSELR = ADC_CHSELR_CHSEL3; // bus voltage
	ADC1->CFGR1 = ADC_CFGR1_OVRMOD;
	ADC1->IER = ADC_IER_EOSEQIE;
	ADC1->CR |= ADC_CR_ADSTART;

	adc.pNextAdcFunc = &voltageMeasurementDone;

	doControl();
}

void ADC1_IRQHandler()
{
	// clear flags
	ADC1->ISR = ADC_ISR_EOSMP | ADC_ISR_EOC | ADC_ISR_EOSEQ | ADC_ISR_OVR;

	(*adc.pNextAdcFunc)();
}

static void voltageMeasurementDone()
{
	adc.volAdc_U12_0 = ADC1->DR;

	ADC1->CHSELR = ADC_CHSELR_CHSEL16; // temperature sensor
	ADC1->SMPR = ADC_SMPR_SMP_2 | ADC_SMPR_SMP_1; // 71.5 clock cycles sampling time
	ADC1->CFGR1 = ADC_CFGR1_OVRMOD;
	ADC1->CR |= ADC_CR_ADSTART;

	adc.pNextAdcFunc = &tempMeasurementDone;
}

static void tempMeasurementDone()
{
	adc.tempAdc_S12_0 = ADC1->DR;

	ADC1->CHSELR = ADC_CHSELR_CHSEL4; // channel 4 (motor current)
	ADC1->SMPR = 0; // 1.5 ADC clock cycles sampling time
	ADC1->CFGR1 = ADC_CFGR1_OVRMOD | ADC_CFGR1_DMAEN | ADC_CFGR1_EXTSEL_0 | ADC_CFGR1_EXTEN_0 | ADC_CFGR1_CONT;
	ADC1->IER = 0;
	ADC1->CR |= ADC_CR_ADSTART;
}

static inline void doControl()
{
	const uint32_t volMult_Q4_15 = 292591UL; // 8.929164775674275

	// resulting voltage is in [mV] and Q16.15 fixed-point representation
	uint32_t voltage_Q16_15 = volMult_Q4_15 * adc.volAdc_U12_0;

	// resulting temperature is in deci-degree celsius [d�C]
	int32_t temperature_Q15_0 = (((adc.tempAdc_S12_0 - adc.temperatureOffset) * adc.temperatureFactor) >> 17) + 300;

	(*adc.controlCallback)();

	++adc.logCounter;
	if(adc.logCounter >= adc.logPrescaler)
	{
		adc.logCounter = 0;

		adc.currentUVWSum[0] += data.sensors.adc.currentUVW_S15_15[0] >> 15;
		adc.currentUVWSum[1] += data.sensors.adc.currentUVW_S15_15[1] >> 15;
		adc.currentUVWSum[2] += data.sensors.adc.currentUVW_S15_15[2] >> 15;
		adc.currentDQSum[0] += data.sensors.adc.currentDQ_S16_0[0];
		adc.currentDQSum[1] += data.sensors.adc.currentDQ_S16_0[1];
		adc.voltageSum += voltage_Q16_15 >> 15;
		adc.tempSum += temperature_Q15_0;

		++adc.pSampleCurrent;

		if(adc.pSampleCurrent == adc.pSampleBufEnd)
		{
			// all samples in a 1ms interval taken, swap active buffers
			// and trigger low-priority task (main)
			if(adc.pSampleBufEnd == &adc.sampleBuf[0][ADC_NUM_SAMPLES_PER_MS])
			{
				adc.pSampleBufEnd = &adc.sampleBuf[1][ADC_NUM_SAMPLES_PER_MS];
				adc.pSampleCurrent = adc.sampleBuf[1];
				adc.pFinishedSampleBuf = adc.sampleBuf[0];
			}
			else
			{
				adc.pSampleBufEnd = &adc.sampleBuf[0][ADC_NUM_SAMPLES_PER_MS];
				adc.pSampleCurrent = adc.sampleBuf[0];
				adc.pFinishedSampleBuf = adc.sampleBuf[1];
			}

			adc.avgVoltage_Q16_0 = (adc.voltageSum*205) >> 12; // => *1/20
			adc.avgTemperature_S15_0 = (adc.tempSum*205) >> 12;
			adc.avgCurrentUVW_S14_0[0] = (adc.currentUVWSum[0]*205) >> 12;
			adc.avgCurrentUVW_S14_0[1] = (adc.currentUVWSum[1]*205) >> 12;
			adc.avgCurrentUVW_S14_0[2] = (adc.currentUVWSum[2]*205) >> 12;
			adc.avgCurrentDQ_S14_0[0] = (adc.currentDQSum[0]*205) >> 12;
			adc.avgCurrentDQ_S14_0[1] = (adc.currentDQSum[1]*205) >> 12;

			adc.voltageSum = 0;
			adc.tempSum = 0;
			adc.currentUVWSum[0] = 0;
			adc.currentUVWSum[1] = 0;
			adc.currentUVWSum[2] = 0;
			adc.currentDQSum[0] = 0;
			adc.currentDQSum[1] = 0;

			adc.trigger1ms = 1;
		}
	}
}

void ADCInit(ADCControlCallback controlFunc, uint32_t numCurSamples, uint32_t logPrescaler)
{
	adc.controlCallback = controlFunc;
	adc.logPrescaler = logPrescaler;

	adc.avgVoltage_Q16_0 = 22200;

	adc.pSampleCurrent = adc.sampleBuf[0];
	adc.pSampleBufEnd = &adc.sampleBuf[0][ADC_NUM_SAMPLES_PER_MS];
	adc.pFinishedSampleBuf = adc.sampleBuf[1];

	// internal temperature sensor calibration values at 30degC and 110degC
	int32_t tsCal30Deg = *((volatile uint16_t*)0x1FFFF7B8);
	int32_t tsCal110Deg = *((volatile uint16_t*)0x1FFFF7C2);

	// ((2^29-1) / (ts2-ts1)) * 800 * 1/2^12
	adc.temperatureFactor = (0x1FFFFFFF/(tsCal110Deg - tsCal30Deg) * 800L) >> 12;
	adc.temperatureOffset = tsCal30Deg;

	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

	// select 12MHz PCLK as source
	ADC1->CFGR2 = ADC_CFGR2_JITOFFDIV4;

	// do ADC calibration
	ADC1->CR = ADC_CR_ADCAL;
	while(ADC1->CR & ADC_CR_ADCAL);

	// configure analog input pins
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_ANALOG;
	gpioInit.pupd = GPIO_PUPD_NONE;
	GPIOInit(GPIOA, GPIO_PIN_3 | GPIO_PIN_4, &gpioInit);

	ADC1->ISR = ADC_ISR_ADRDY;
	ADC1->CR = ADC_CR_ADEN;
	while((ADC1->ISR & ADC_ISR_ADRDY) == 0);

	// enable temperature sensor
	ADC->CCR |= ADC_CCR_TSEN;

	ADC1->SMPR = 0; // 1.5 ADC clock cycles sampling time
	ADC1->CHSELR = ADC_CHSELR_CHSEL4; // channel 4 (motor current)

	// clear all flags
	ADC1->ISR = 0xFF;
	ADC1->IER = 0;

	// DMA enabled (continuous mode upon trigger), external trigger TRG1 (TIM1_CC4)
	// rising edge, overwrite old data
	ADC1->CFGR1 = ADC_CFGR1_OVRMOD | ADC_CFGR1_DMAEN | ADC_CFGR1_EXTSEL_0 | ADC_CFGR1_EXTEN_0 | ADC_CFGR1_CONT;

	// ADC, high priority, memory increment, peripheral to memory, 16bit data size, circular mode
	DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_PL_1 | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_CIRC;
	DMA1_Channel1->CMAR = (uint32_t)adc.dmaData;
	DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
	DMA1_Channel1->CNDTR = numCurSamples;

	DMA1->IFCR = DMA_IFCR_CGIF1;

	DMA1_Channel1->CCR |= DMA_CCR_TCIE | DMA_CCR_TEIE | DMA_CCR_EN;

	irqTable[DMA1_Channel1_IRQn] = &DMA1_Channel1_IRQHandler;
	NVIC_SetPriority(DMA1_Channel1_IRQn, 2);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	irqTable[ADC1_IRQn] = &ADC1_IRQHandler;
	NVIC_SetPriority(ADC1_IRQn, 1);
	NVIC_EnableIRQ(ADC1_IRQn);

	ADC1->CR |= ADC_CR_ADSTART;
}

void __attribute__((optimize("O0"))) ADCWaitFor1MsTrigger()
{
	register uint32_t counter = 0;

	while(adc.trigger1ms == 0)
		++counter;

	adc.trigger1ms = 0;

	data.performance.idleCounter = counter;
	if(data.performance.idleCounter > UINT16_MAX)
		data.performance.idleCounter = UINT16_MAX;

	++data.timeMs;
}

void ADCUpdate()
{
	data.sensors.adc.vSupply_Q16_0 = adc.avgVoltage_Q16_0;
	data.sensors.adc.vSupplyReciprocal_Q0_31 = (1 << 31)/adc.avgVoltage_Q16_0;
}
