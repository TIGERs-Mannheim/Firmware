/*
 * main.c
 *
 *  Created on: 14.01.2019
 *      Author: AndreR
 */

/**
 * ### Pinout ###
 *
 *  PA0: T-ADC (Dribbler temperature)
 *  PA1: IR-ADC0 (Barrier)
 *  PA2: IR-ADC5 (rightmost LED)
 *  PA3: IR-ADC4
 *  PA4: IR-ADC1 (leftmost LED)
 *  PA5: IR-ADC2
 *  PA6: IR-ADC3
 *  PA7:
 *  PA8:
 *  PA9: USART1-TX
 * PA10: USART1-RX
 * PA11:
 * PA12: IR-LED
 * PA13: SWDIO
 * PA14: SWCLK
 * PA15:
 *
 *  PB0:
 *  PB1:
 *  PB3: IR3
 *  PB4: IR4
 *  PB5: IR1 (leftmost LED)
 *  PB6: IR2
 *  PB7: IR0 (Barrier)
 *
 *  PF0: OSC
 *  PF1:
 *
 * ### DMA Config ###
 *
 * DMA1_CH1: ADC (very high prio)
 * DMA1_CH2: USART1_TX (med prio)
 * DMA1_CH3: USART1_RX (low prio)
 * DMA1_CH4: -
 * DMA1_CH5: TIM1_UP (high prio)
 *
 * ### Timers ###
 *
 * TIM1: ADC and GPIO trigger
 * TIM2: -
 * TIM3: -
 * TIM14: -
 * TIM16: -
 * TIM17: -
 */

#include "hal/system_init.h"
#include "usart1.h"
#include "adc.h"

#define SYSTEM_CLOCK_HZ 48000000UL
#define SYSTEM_TICK_FREQ 1000

#define TOGGLE_PRIMARY_LED() GPIOA->ODR ^= GPIO_ODR_12

int main()
{
	uint32_t idleCounter;
	uint32_t timeMs = 0;

	// Set AHB and APB1/APB2 clocks to 48MHz
	SystemInit(SYSTEM_CLOCK_HZ, SYSTEM_TICK_FREQ);

	uint32_t idleIncrements = SystemMeasureIdleIncrements();
	if(idleIncrements > UINT16_MAX)
		idleIncrements = UINT16_MAX;

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_DMAEN;

	// HAL
	USART1Init();
	ADCInit();

	// main LED
	GPIOA->MODER |= GPIO_MODER_MODER12_0;
	GPIOA->BSRR |= GPIO_BSRR_BR_12;

	miso[0].idleTicksNoLoad = idleIncrements;
	miso[1].idleTicksNoLoad = idleIncrements;

	while(1)
	{
		idleCounter = SystemWaitUntilSystick();
		if(idleCounter > UINT16_MAX)
			idleCounter = UINT16_MAX;

		miso[0].idleTicksActive = idleCounter;
		miso[1].idleTicksActive = idleCounter;

		++timeMs;

		if(timeMs % 512 == 0)
			TOGGLE_PRIMARY_LED();
	}

	return 0;
}
