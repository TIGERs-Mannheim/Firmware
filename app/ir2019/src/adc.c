/*
 * adc.c
 *
 *  Created on: 17.01.2019
 *      Author: AndreR
 */

/**

ADC calculations:
t_adc : 12.5 $
t_smp : 7.5 $
f_adc : 12e6 $
t_conv : (t_adc+t_smp)/f_adc;
t_total : 7*4*t_conv;

Peripheral interconnections:

      TIM1_UP
  +-----------------> DMA1_CH5 +-------> GPIOB_BSRR
  |
  |
  +   TIM1_CC4
TIM1 +--------------> ADC1 +--------> DMA1_CH1 +---------> ADC Samples
                                            +
                                            |
                                            |   DMA1_CH1_TC
                                            +-----------------> IRQ
ADC Config:
- 12MHz clock
- 12Bit resolution => 12.5 cycles
- 7.5 cycles sampling time
- 7 channels, 4 samples each => total time: 47us

ADC calculations:
t_adc : 12.5 $
t_smp : 7.5 $
f_adc : 12e6 $
t_conv : (t_adc+t_smp)/f_adc;
t_total : 7*4*t_conv;

IR signal settling time: ~50us

Sequence in 1ms (blocks):
Off, Barrier
IR1, Barrier
IR2, Barrier
IR3, Barrier
IR4, Barrier

=> 10 configs per 1ms
=> 100us per config
=> 10*7 = 70 values => 140 Bytes/ms, 181kB/s max

TIM1 Config:
- 80 ADC conversion sequences per ms => 80kHz
- 12.5us sample interval

DMA Config:
- IRQ triggered when two blocks are complete
- A block always includes one barrier measurement and one other config
- Which block was converted is read from DMA1_CH5 CNDTR register
- All buffers run in circular mode to ensure timing are met

*/

#include "adc.h"
#include "usart1.h"
#include "system_init.h"
#include <string.h>

#define NUM_BLOCKS 2
#define NUM_SAMPLES 8
#define NUM_CHANNELS 7
#define NUM_DATAPOINTS (NUM_BLOCKS*NUM_SAMPLES*NUM_CHANNELS)

#define IR_OUT_BARRIER GPIO_PIN_7
#define IR_OUT_1 GPIO_PIN_5
#define IR_OUT_2 GPIO_PIN_6
#define IR_OUT_3 GPIO_PIN_3
#define IR_OUT_4 GPIO_PIN_4
#define IR_OUT_PIN_ALL (GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7)
#define IR_OUT_PIN_OFF (IR_OUT_PIN_ALL << 16)

static uint16_t dmaData[NUM_DATAPOINTS];

// use double buffering for transmission, otherwise transmission and new data writes collide
IrExchangeMISO miso[2];
static IrExchangeMISO* pTxActiveMiso = &miso[0];

static const uint32_t txPinConfigs[] = {
	IR_OUT_PIN_OFF | IR_OUT_BARRIER,

	// Block 2 / config 1
//	IR_OUT_PIN_OFF | IR_OUT_1 | IR_OUT_2 | IR_OUT_3 | IR_OUT_4,
	IR_OUT_PIN_OFF | IR_OUT_1,
	IR_OUT_PIN_OFF | IR_OUT_BARRIER,

	// Block 4 / config 2
	IR_OUT_PIN_OFF | IR_OUT_2,
	IR_OUT_PIN_OFF | IR_OUT_BARRIER,

	// Block 6 / config 3
	IR_OUT_PIN_OFF | IR_OUT_3,
	IR_OUT_PIN_OFF | IR_OUT_BARRIER,

	// Block 8 / config 4
	IR_OUT_PIN_OFF | IR_OUT_4,
	IR_OUT_PIN_OFF | IR_OUT_BARRIER,

	// Block 0 / config 0
	IR_OUT_PIN_OFF,
};

#define NUM_TX_CONFIGS (sizeof(txPinConfigs)/sizeof(txPinConfigs[0]))

void DMA1_Channel1_IRQHandler()
{
	DMA1->IFCR = DMA_IFCR_CGIF1;

	// determine which config was converted
	uint32_t currentTxConfig = NUM_TX_CONFIGS - DMA1_Channel5->CNDTR;

	uint32_t previousTxConfig;
	if(currentTxConfig <= 1)
		previousTxConfig = NUM_TX_CONFIGS-2+currentTxConfig;
	else
		previousTxConfig = currentTxConfig-2;

	pTxActiveMiso->config = previousTxConfig >> 1;

	// sum up measured data
	// from samples 0..7 only samples 0..6 are used. Other samples are taken during signal settling time.
	for(uint16_t block = 0; block < NUM_BLOCKS; ++block)
	{
		for(uint16_t channel = 0; channel < NUM_CHANNELS; ++channel)
		{
			pTxActiveMiso->channels[block*NUM_CHANNELS + channel] = 0;
		}

		for(uint16_t sample = 0; sample < NUM_SAMPLES-1; ++sample)
		{
			for(uint16_t channel = 0; channel < NUM_CHANNELS; ++channel)
			{
				pTxActiveMiso->channels[block*NUM_CHANNELS + channel] += dmaData[block*NUM_SAMPLES*NUM_CHANNELS + sample*NUM_CHANNELS + channel];
			}
		}
	}

	// calculate checksum
	uint8_t* pIrData = (uint8_t*)pTxActiveMiso->channels;
	uint8_t xor = 0;
	for(uint16_t i = 0; i < sizeof(pTxActiveMiso->channels)+sizeof(pTxActiveMiso->config); ++i)
		xor ^= pIrData[i];

	pTxActiveMiso->xorChecksum = xor;

	// transmit data and swap double buffer
	USART1Transmit(pTxActiveMiso, sizeof(IrExchangeMISO));

	if(pTxActiveMiso == &miso[0])
		pTxActiveMiso = &miso[1];
	else
		pTxActiveMiso = &miso[0];
}

void ADCInit()
{
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
	GPIOInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6, &gpioInit);

	GPIOB->BSRR = IR_OUT_PIN_OFF;

	gpioInit.mode = GPIO_MODE_OUTPUT;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.pupd = GPIO_PUPD_NONE;
	GPIOInit(GPIOB, IR_OUT_PIN_ALL, &gpioInit);

	ADC1->ISR = ADC_ISR_ADRDY;
	ADC1->CR = ADC_CR_ADEN;
	while((ADC1->ISR & ADC_ISR_ADRDY) == 0);

	ADC1->SMPR = ADC_SMPR_SMP_0; // 7.5 ADC clock cycles sampling time

	ADC1->CHSELR = 0x7F; // channel 0 - 6

	// single (sequence) conversion mode, DMA enabled, DMA circular mode, external trigger TRG1 (TIM1_CC4), rising edge
	ADC1->CFGR1 = ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG | ADC_CFGR1_EXTSEL_0 | ADC_CFGR1_EXTEN_0;

	// ADC, very high priority, memory increment, peripheral to memory, 16bit data size, circular mode
	DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_PL | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_CIRC;
	DMA1_Channel1->CMAR = (uint32_t)dmaData;
	DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
	DMA1_Channel1->CNDTR = NUM_DATAPOINTS;

	DMA1->IFCR = DMA_IFCR_CGIF1;

	DMA1_Channel1->CCR |= DMA_CCR_TCIE | DMA_CCR_TEIE | DMA_CCR_EN;

	NVIC_SetPriority(DMA1_Channel1_IRQn, 5);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	// TIM1_UP, high priority, memory increment, memory to peripheral, 32bit data size, circular mode
	DMA1_Channel5->CCR = DMA_CCR_PL_1 | DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1 | DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_DIR;
	DMA1_Channel5->CMAR = (uint32_t)txPinConfigs;
	DMA1_Channel5->CPAR = (uint32_t)&GPIOB->BSRR;
	DMA1_Channel5->CNDTR = NUM_TX_CONFIGS;

	DMA1->IFCR = DMA_IFCR_CGIF5;

	DMA1_Channel5->CCR |= DMA_CCR_EN;

	// Setup TIM1 as trigger source for ADC and GPIO pin config DMA transfer
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	TIM1->CR1 = 0;
	TIM1->CR2 = 0;
	TIM1->SMCR = 0;
	TIM1->CNT = 0;
	TIM1->PSC = 5; // => 8MHz
	TIM1->ARR = 99; // => 80kHz
	TIM1->RCR = 7; // update event every 8th overflow
	TIM1->CCMR1 = 0;
	TIM1->CCMR2 = TIM_CCMR2_OC4M | TIM_CCMR2_OC4PE; // PWM mode 2 on CC4 (used as ADC trigger)
	TIM1->CCER = 0;
	TIM1->BDTR = TIM_BDTR_MOE;
	TIM1->CCR4 = 90; // trigger ADC quite late
	TIM1->CCER = TIM_CCER_CC4E;
	TIM1->EGR = TIM_EGR_UG; // force update of registers
	TIM1->SR = 0;
	TIM1->DIER = TIM_DIER_UDE;

	ADC1->CR |= ADC_CR_ADSTART;

	TIM1->CR1 |= TIM_CR1_CEN;
}
