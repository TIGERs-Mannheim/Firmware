/*
 * system_init.c
 *
 *  Created on: 14.01.2019
 *      Author: AndreR
 */

#include "system_init.h"

void SystemInit(uint32_t systemClock, uint32_t sysTickFreq)
{
	// reset all clocks and select HSI
	RCC->CR |= RCC_CR_HSION;
	RCC->CFGR = 0x00000000;
	RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_CSSON | RCC_CR_HSEBYP | RCC_CR_HSEON);
	RCC->CFGR2 = 0;
	RCC->CFGR3 = 0;
	RCC->CIR = 0;

	// startup HSE clock
	RCC->CR |= RCC_CR_HSEBYP;
	RCC->CR |= RCC_CR_HSEON;

	while((RCC->CR & RCC_CR_HSERDY) == 0)
		asm volatile("nop");

	// Configure PLL
	RCC->CFGR |= RCC_CFGR_PLLMUL12 | RCC_CFGR_PLLSRC_HSE_PREDIV;

	// start PLL
	RCC->CR |= RCC_CR_PLLON;
	while((RCC->CR & RCC_CR_PLLRDY) == 0)
		asm volatile("nop");

	// enable pre-fetch buffer and increase latency to 1 as SYSCLK will be over 24MHz
	FLASH->ACR = FLASH_ACR_LATENCY | FLASH_ACR_PRFTBE;

	// select PLL as system clock
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	while((RCC->CFGR & RCC_CFGR_SWS_PLL) != RCC_CFGR_SWS_PLL)
		asm volatile("nop");

	SysTick->LOAD = (systemClock / sysTickFreq) - 1;
	SysTick->VAL = (uint32_t)0;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}

void GPIOInit(GPIO_TypeDef* pGPIO, uint16_t pins, GPIOInitData* pInit)
{
	if(pInit->mode != GPIO_MODE_AF && pInit->mode != GPIO_MODE_EXTI_AF)
		pInit->alternate = 0;

	if(	pInit->mode == GPIO_MODE_INPUT || pInit->mode == GPIO_MODE_ANALOG ||
		pInit->mode == GPIO_MODE_EXTI || pInit->mode == GPIO_MODE_EXTI_AF)
	{
		pInit->otype = GPIO_OTYPE_PUSH_PULL;
		pInit->ospeed = 0;
	}

	for(uint16_t pin = 0; pin < 16; pin++)
	{
		if((pins & (1 << pin)) == 0)
			continue;

		pGPIO->MODER &= ~(GPIO_MODER_MODER0 << (pin*2));
		if(pInit->mode == GPIO_MODE_EXTI || pInit->mode == GPIO_MODE_EXTI_AF)
			pGPIO->MODER |= (GPIO_MODE_INPUT << (pin*2));
		else
			pGPIO->MODER |= (pInit->mode << (pin*2));

		pGPIO->OTYPER &= ~(GPIO_OTYPER_OT_0 << pin);
		pGPIO->OTYPER |= (pInit->otype << pin);

		pGPIO->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (pin*2));
		pGPIO->OSPEEDR |= (pInit->ospeed << (pin*2));

		pGPIO->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (pin*2));
		pGPIO->PUPDR |= (pInit->pupd << (pin*2));

		if(pin < 8)
		{
			pGPIO->AFR[0] &= ~(0x0F << (pin*4));
			pGPIO->AFR[0] |= (pInit->alternate << (pin*4));
		}
		else
		{
			pGPIO->AFR[1] &= ~(0x0F << ((pin-8)*4));
			pGPIO->AFR[1] |= (pInit->alternate << ((pin-8)*4));
		}

		if(pInit->mode == GPIO_MODE_EXTI || pInit->mode == GPIO_MODE_EXTI_AF)
		{
			uint32_t port = (uint32_t)pGPIO;
			port -= GPIOA_BASE;
			port >>= 10;

			SYSCFG->EXTICR[pin >> 2] &= ~(0x0F << (4*(pin & 0x03)));
			SYSCFG->EXTICR[pin >> 2] |= (port << (4*(pin & 0x03)));

			EXTI->IMR |= (1 << pin);
			if(pInit->extiTrigger & GPIO_EXTI_TRIG_RISING)
				EXTI->RTSR |= (1 << pin);
			if(pInit->extiTrigger & GPIO_EXTI_TRIG_FALLING)
				EXTI->FTSR |= (1 << pin);
		}
	}
}

void SystemWaitMs(uint32_t timeMs)
{
	for(uint32_t i = 0; i < timeMs+1; ++i)
	{
		while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);
	}
}

uint32_t __attribute__((optimize("O0"))) SystemMeasureIdleIncrements()
{
	register uint32_t counter = 0;

	// wait until the systick triggers
	while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);

	// and now count how many increments we can do in one systick period
	while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0)
		++counter;

	return counter;
}

uint32_t __attribute__((optimize("O0"))) SystemWaitUntilSystick()
{
	register uint32_t counter = 0;

	while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0)
		++counter;

	return counter;
}
