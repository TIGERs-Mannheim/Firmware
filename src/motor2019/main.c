/*
 * main.c
 *
 *  Created on: 14.01.2019
 *      Author: AndreR
 */

/**
 * ### Pinout ###
 *
 *  PA0: M-H1 (TIM2_CH1_ETR, AF2)
 *  PA1: M-H2 (TIM2_CH2, AF2)
 *  PA2: M-H3 (TIM2_CH3, AF2)
 *  PA3: M-VOL
 *  PA4: M-CUR
 *  PA5: -
 *  PA6: ENCA (TIM3_CH1, AF1)
 *  PA7: ENCB (TIM3_CH2, AF1)
 *  PA8: [HS1] (TIM1_CH1, AF2)
 *  PA9: [HS2] (TIM1_CH2, AF2)
 * PA10: [HS3] (TIM1_CH3, AF2)
 * PA11: [OC_SEL]
 * PA12: [OC_COMP_INT2] (TIM1_ETR, AF2)
 * PA13: SWDIO
 * PA14: SWCLK / USART1-TX
 * PA15: USART1-RX
 *
 *  PB0: -
 *  PB1: -
 *  PB3: -
 *  PB4: -
 *  PB5: -
 *  PB6: -
 *  PB7: LED
 *  PB8: -
 *  PB9: -
 * PB10: -
 * PB11: -
 * PB12: [OC_COMP_INT] (TIM1_BKIN, AF2)
 * PB13: [LS1] (TIM1_CH1N, AF2)
 * PB14: [LS2] (TIM1_CH2N, AF2)
 * PB15: [LS3] (TIM1_CH3N, AF2)
 *
 * PC13: -
 * PC14: -
 * PC15: -
 *
 *  PF0: OSC
 *  PF1: -
 *  PF6: [OC_TH_STBY2]
 *  PF7: [OC_TH_STBY1]
 *
 * ### DMA Config ###
 *
 * DMA1_CH1: ADC (high prio)
 * DMA1_CH2: USART1_TX (medium prio)
 * DMA1_CH3: USART1_RX (low prio)
 * DMA1_CH4: TIM1_CH4 - ADC trigger & encoder capture (very high prio)
 * DMA1_CH5: TIM1_UP - motor PWM data load (medium prio)
 *
 * ### Timers ###
 *
 * TIM1: Motor control and ADC trigger
 * TIM2: Hall sensors
 * TIM3: Encoder
 * TIM14: -
 * TIM16: -
 * TIM17: -
 */

#include "main.h"
#include "hal/system_init.h"
#include "dribbler.h"
#include "drive.h"

#define SYSTEM_CLOCK_HZ 48000000UL
#define SYSTEM_TICK_FREQ 1000

Data data;

volatile uint32_t* pMotorId = (volatile uint32_t*)0x08007FF8; // programmed by bootloader

int main()
{
	SystemInit(SYSTEM_CLOCK_HZ, SYSTEM_TICK_FREQ);

	// map embedded SRAM to 0x00000000, vector table from SRAM is now used
	SYSCFG->CFGR1 |= SYSCFG_CFGR1_MEM_MODE;

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOFEN | RCC_AHBENR_DMAEN;

	// primary LED
	GPIOB->MODER |= GPIO_MODER_MODER7_0;
	GPIOB->BSRR |= GPIO_BSRR_BR_7;

	data.performance.idleIncrements = SystemMeasureIdleIncrements();
	if(data.performance.idleIncrements > UINT16_MAX)
		data.performance.idleIncrements = UINT16_MAX;

	if(*pMotorId < 4)
		DriveMain();
	else
		DribblerMain();

	return 0;
}

static void defaultIrqHandler()
{
	while(1)
	{
		SystemWaitMs(50);
		TOGGLE_PRIMARY_LED();
	}
}

extern void Reset_Handler();

void (*irqTableCore[])() = {
	0,
	&Reset_Handler,     // Reset_Handler
	&defaultIrqHandler, // NMI_Handler
	&defaultIrqHandler, // HardFault_Handler
	&defaultIrqHandler,
	&defaultIrqHandler,
	&defaultIrqHandler,
	&defaultIrqHandler,
	&defaultIrqHandler,
	&defaultIrqHandler,
	&defaultIrqHandler,
	&defaultIrqHandler, // SVC_Handler
	&defaultIrqHandler,
	&defaultIrqHandler,
	&defaultIrqHandler, // PendSV_Handler
	&defaultIrqHandler, // SysTick_Handler
};

void (*irqTable[])() = {
	&defaultIrqHandler, // WWDG_IRQHandler
	&defaultIrqHandler, // PVD_IRQHandler
	&defaultIrqHandler, // RTC_IRQHandler
	&defaultIrqHandler, // FLASH_IRQHandler
	&defaultIrqHandler, // RCC_IRQHandler
	&defaultIrqHandler, // EXTI0_1_IRQHandler
	&defaultIrqHandler, // EXTI2_3_IRQHandler
	&defaultIrqHandler, // EXTI4_15_IRQHandler
	&defaultIrqHandler,
	&defaultIrqHandler, // DMA1_Channel1_IRQHandler
	&defaultIrqHandler, // DMA1_Channel2_3_IRQHandler
	&defaultIrqHandler, // DMA1_Channel4_5_IRQHandler
	&defaultIrqHandler, // ADC1_IRQHandler
	&defaultIrqHandler, // TIM1_BRK_UP_TRG_COM_IRQHandler
	&defaultIrqHandler, // TIM1_CC_IRQHandler
	&defaultIrqHandler, // TIM2_IRQHandler
	&defaultIrqHandler, // TIM3_IRQHandler
	&defaultIrqHandler,
	&defaultIrqHandler,
	&defaultIrqHandler, // TIM14_IRQHandler
	&defaultIrqHandler,
	&defaultIrqHandler, // TIM16_IRQHandler
	&defaultIrqHandler, // TIM17_IRQHandler
	&defaultIrqHandler, // I2C1_IRQHandler
	&defaultIrqHandler,
	&defaultIrqHandler, // SPI1_IRQHandler
	&defaultIrqHandler,
	&defaultIrqHandler, // USART1_IRQHandler
	&defaultIrqHandler,
	&defaultIrqHandler,
	&defaultIrqHandler,
	&defaultIrqHandler,
};
