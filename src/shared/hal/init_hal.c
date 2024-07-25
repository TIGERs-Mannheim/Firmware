/*
 * init_hal.c
 *
 *  Created on: 30.07.2014
 *      Author: AndreR
 */

#include "init_hal.h"
#include "ch.h"

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

		pGPIO->MODER &= ~(GPIO_MODER_MODE0 << (pin*2));
		if(pInit->mode == GPIO_MODE_EXTI || pInit->mode == GPIO_MODE_EXTI_AF)
			pGPIO->MODER |= (GPIO_MODE_INPUT << (pin*2));
		else
			pGPIO->MODER |= (pInit->mode << (pin*2));

		pGPIO->OTYPER &= ~(GPIO_OTYPER_OT0 << pin);
		pGPIO->OTYPER |= (pInit->otype << pin);

		pGPIO->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED0 << (pin*2));
		pGPIO->OSPEEDR |= (pInit->ospeed << (pin*2));

		pGPIO->PUPDR &= ~(GPIO_PUPDR_PUPD0 << (pin*2));
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

#ifdef STM32H7XX
			EXTI_D1->IMR1 |= (1 << pin);
			if(pInit->extiTrigger & GPIO_EXTI_TRIG_RISING)
				EXTI->RTSR1 |= (1 << pin);
			if(pInit->extiTrigger & GPIO_EXTI_TRIG_FALLING)
				EXTI->FTSR1 |= (1 << pin);
#else
			EXTI->IMR |= (1 << pin);
			if(pInit->extiTrigger & GPIO_EXTI_TRIG_RISING)
				EXTI->RTSR |= (1 << pin);
			if(pInit->extiTrigger & GPIO_EXTI_TRIG_FALLING)
				EXTI->FTSR |= (1 << pin);
#endif
		}
	}
}

void GPIODeInit(GPIO_TypeDef* pGPIO, uint16_t pins)
{
	for(uint16_t pin = 0; pin < 16; pin++)
	{
		if((pins & (1 << pin)) == 0)
			continue;

		pGPIO->MODER &= ~(GPIO_MODER_MODE0 << (pin*2));
		pGPIO->OTYPER &= ~(GPIO_OTYPER_OT0 << pin);
		pGPIO->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED0 << (pin*2));
		pGPIO->PUPDR &= ~(GPIO_PUPDR_PUPD0 << (pin*2));

		if(pin < 8)
		{
			pGPIO->AFR[0] &= ~(0x0F << (pin*4));
		}
		else
		{
			pGPIO->AFR[1] &= ~(0x0F << ((pin-8)*4));
		}

		uint32_t port = (uint32_t)pGPIO;
		port -= GPIOA_BASE;
		port >>= 10;

		SYSCFG->EXTICR[pin >> 2] &= ~(0x0F << (4*(pin & 0x03)));

#ifdef STM32H7XX
		EXTI_D1->IMR1 &= ~(1 << pin);
		EXTI->RTSR1 &= ~(1 << pin);
		EXTI->FTSR1 &= ~(1 << pin);
#else
		EXTI->IMR &= ~(1 << pin);
		EXTI->RTSR &= ~(1 << pin);
		EXTI->FTSR &= ~(1 << pin);
#endif
	}
}

#if defined(STM32F4XX) || defined(STM32F7XX) || defined(STM32H7XX)
/**
 * pDMA: DMA1 or DMA2
 * stream: 0 - 7
 */
DMA_Stream_TypeDef* DMAGenerateStreamTypeDef(DMA_TypeDef* pDMA, uint32_t stream)
{
	return (DMA_Stream_TypeDef*)( ((uint32_t*)pDMA) +  6*stream + 4);
}

volatile uint32_t* DMAGenerateIFCR(DMA_TypeDef* pDMA, uint32_t stream)
{
	if(stream < 4)
		return &pDMA->LIFCR;
	else
		return &pDMA->HIFCR;
}

volatile uint32_t* DMAGenerateISR(DMA_TypeDef* pDMA, uint32_t stream)
{
	if(stream < 4)
		return &pDMA->LISR;
	else
		return &pDMA->HISR;
}

static uint8_t ifcrLUT[8] = {0, 6, 16, 22, 0, 6, 16, 22};

uint32_t DMAGenerateMask(uint32_t flags, uint32_t stream)
{
	return flags << ifcrLUT[stream];
}
#endif

void NVICEnableIRQ(uint32_t irqn, uint32_t irql)
{
	NVIC_SetPriority(irqn, irql);
	NVIC_EnableIRQ(irqn);
}

uint16_t USARTCalcBRR(uint32_t baudRate, uint32_t peripheralClock, uint8_t over8)
{
	uint32_t tmpreg;
	uint32_t integerDiv;
	uint32_t fractionalDiv;

	// Determine the integer part
	if(over8)
	{
		// Integer part computing in case Oversampling mode is 8 Samples
		integerDiv = ((25 * peripheralClock) / (2 * baudRate));
	}
	else
	{
		// Integer part computing in case Oversampling mode is 16 Samples
		integerDiv = ((25 * peripheralClock) / (4 * baudRate));
	}
	tmpreg = (integerDiv / 100) << 4;

	// Determine the fractional part
	fractionalDiv = integerDiv - (100 * (tmpreg >> 4));

	// Implement the fractional part in the register
	if(over8)
	{
		tmpreg |= ((((fractionalDiv * 8) + 50) / 100)) & ((uint8_t) 0x07);
	}
	else
	{
		tmpreg |= ((((fractionalDiv * 16) + 50) / 100)) & ((uint8_t) 0x0F);
	}

	return (uint16_t)tmpreg;
}

SystemClockInfo systemClockInfo;

#ifndef STM32H7XX
void SystemClockInit(SystemClockInitData* pInit, uint32_t sysClk)
{
	// fill system clock info
	systemClockInfo.SYSClk = sysClk;
	systemClockInfo.APB1PeriphClk = sysClk / pInit->APB1Div;
	systemClockInfo.APB2PeriphClk = sysClk / pInit->APB2Div;
	if(pInit->APB1Div == 1)
		systemClockInfo.APB1TimerClk = systemClockInfo.APB1PeriphClk;
	else
		systemClockInfo.APB1TimerClk = systemClockInfo.APB1PeriphClk*2;

	if(pInit->APB2Div == 1)
		systemClockInfo.APB2TimerClk = systemClockInfo.APB2PeriphClk;
	else
		systemClockInfo.APB2TimerClk = systemClockInfo.APB2PeriphClk*2;

	// reset clocks to default reset state
	// Set HSION bit
	RCC->CR |= (uint32_t)0x00000001;

	// Reset CFGR register
	RCC->CFGR = 0x00000000;

	// Reset HSEON, CSSON and PLLON bits
	RCC->CR &= (uint32_t)0xFEF6FFFF;

#if defined(STM32F4XX) || defined(STM32F7XX)
	// Reset PLLCFGR register
	RCC->PLLCFGR = 0x24003010;
#endif

	// Reset HSEBYP bit
	RCC->CR &= (uint32_t)0xFFFBFFFF;

	// Disable all interrupts
	RCC->CIR = 0x00000000;

	// configure clocks
	if(pInit->HSEBypass)
		RCC->CR = RCC_CR_HSEBYP;

	RCC->CR |= RCC_CR_HSEON;

	while((RCC->CR & RCC_CR_HSERDY) == 0)
		asm volatile("nop");

	switch(pInit->APB1Div)
	{
		case  2: pInit->APB1Div = RCC_CFGR_PPRE1_DIV2; break;
		case  4: pInit->APB1Div = RCC_CFGR_PPRE1_DIV4; break;
		case  8: pInit->APB1Div = RCC_CFGR_PPRE1_DIV8; break;
		case 16: pInit->APB1Div = RCC_CFGR_PPRE1_DIV16; break;
		default: pInit->APB1Div = RCC_CFGR_PPRE1_DIV1; break;
	}

	switch(pInit->APB2Div)
	{
		case  2: pInit->APB2Div = RCC_CFGR_PPRE2_DIV2; break;
		case  4: pInit->APB2Div = RCC_CFGR_PPRE2_DIV4; break;
		case  8: pInit->APB2Div = RCC_CFGR_PPRE2_DIV8; break;
		case 16: pInit->APB2Div = RCC_CFGR_PPRE2_DIV16; break;
		default: pInit->APB2Div = RCC_CFGR_PPRE2_DIV1; break;
	}

#if defined(STM32F4XX) || defined(STM32F7XX)
	RCC->PLLCFGR = (2 << 28) | (pInit->pll.Q << 24) | (((pInit->pll.P/2)-1) << 16) | (pInit->pll.N << 6) | pInit->pll.M | RCC_PLLCFGR_PLLSRC_HSE;
	RCC->CFGR = (pInit->RTCDiv << 16) | pInit->APB1Div | pInit->APB2Div;
#else
	RCC->CFGR = ((pInit->pllMul-2) << 18) | RCC_CFGR_PLLSRC_PREDIV1 | pInit->APB1Div | pInit->APB2Div;
#endif

	RCC->CR |= RCC_CR_PLLON;

#ifdef STM32F7XX
	// Configure Over-Drive mode for highest frequencies
	PWR->CR1 |= PWR_CR1_ODEN;

	while((PWR->CSR1 & PWR_CSR1_ODRDY) == 0)
		asm volatile("nop");

	PWR->CR1 |= PWR_CR1_ODSWEN;

	while((PWR->CSR1 & PWR_CSR1_ODSWRDY) == 0)
		asm volatile("nop");
#endif


	while((RCC->CR & RCC_CR_PLLRDY) == 0)
		asm volatile("nop");

	// increase flash latency and enable data/instruction cache + prefetch buffer
#ifdef STM32F7XX
	FLASH->ACR = pInit->flashLatency | FLASH_ACR_PRFTEN | FLASH_ACR_ARTEN;
#endif
#ifdef STM32F4XX
	FLASH->ACR = pInit->flashLatency | FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_PRFTEN;
#endif
#ifdef STM32F30X
	FLASH->ACR = pInit->flashLatency | FLASH_ACR_PRFTBE;
#endif

	RCC->CFGR |= RCC_CFGR_SW_PLL;

	while((RCC->CFGR & RCC_CFGR_SWS_PLL) == 0)
		asm volatile("nop");

	SysTick->LOAD = (sysClk / pInit->sysTickFreq) - (systime_t)1;
	SysTick->VAL = (uint32_t)0;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;

	NVIC_SetPriority(SysTick_IRQn, CORTEX_PRIORITY_SVCALL+1);
}
#endif

#ifdef STM32F30X
void ADCRegularChannelConfig(ADC_TypeDef* ADCx, uint8_t ADC_Channel, uint8_t Rank, uint8_t ADC_SampleTime)
{
	uint32_t tmpreg1 = 0;
	uint32_t tmpreg2 = 0;

	/* Regular sequence configuration */
	/* For Rank 1 to 4 */
	if (Rank < 5)
	{
		/* Get the old register value */
		tmpreg1 = ADCx->SQR1;
		/* Calculate the mask to clear */
		tmpreg2 = 0x1F << (6 * (Rank));
		/* Clear the old SQx bits for the selected rank */
		tmpreg1 &= ~tmpreg2;
		/* Calculate the mask to set */
		tmpreg2 = (uint32_t) (ADC_Channel) << (6 * (Rank));
		/* Set the SQx bits for the selected rank */
		tmpreg1 |= tmpreg2;
		/* Store the new register value */
		ADCx->SQR1 = tmpreg1;
	}
	/* For Rank 5 to 9 */
	else if (Rank < 10)
	{
		/* Get the old register value */
		tmpreg1 = ADCx->SQR2;
		/* Calculate the mask to clear */
		tmpreg2 = ADC_SQR2_SQ5 << (6 * (Rank - 5));
		/* Clear the old SQx bits for the selected rank */
		tmpreg1 &= ~tmpreg2;
		/* Calculate the mask to set */
		tmpreg2 = (uint32_t) (ADC_Channel) << (6 * (Rank - 5));
		/* Set the SQx bits for the selected rank */
		tmpreg1 |= tmpreg2;
		/* Store the new register value */
		ADCx->SQR2 = tmpreg1;
	}
	/* For Rank 10 to 14 */
	else if (Rank < 15)
	{
		/* Get the old register value */
		tmpreg1 = ADCx->SQR3;
		/* Calculate the mask to clear */
		tmpreg2 = ADC_SQR3_SQ10 << (6 * (Rank - 10));
		/* Clear the old SQx bits for the selected rank */
		tmpreg1 &= ~tmpreg2;
		/* Calculate the mask to set */
		tmpreg2 = (uint32_t) (ADC_Channel) << (6 * (Rank - 10));
		/* Set the SQx bits for the selected rank */
		tmpreg1 |= tmpreg2;
		/* Store the new register value */
		ADCx->SQR3 = tmpreg1;
	}
	else
	{
		/* Get the old register value */
		tmpreg1 = ADCx->SQR4;
		/* Calculate the mask to clear */
		tmpreg2 = ADC_SQR3_SQ15 << (6 * (Rank - 15));
		/* Clear the old SQx bits for the selected rank */
		tmpreg1 &= ~tmpreg2;
		/* Calculate the mask to set */
		tmpreg2 = (uint32_t) (ADC_Channel) << (6 * (Rank - 15));
		/* Set the SQx bits for the selected rank */
		tmpreg1 |= tmpreg2;
		/* Store the new register value */
		ADCx->SQR4 = tmpreg1;
	}

	/* Channel sampling configuration */
	/* if ADC_Channel_10 ... ADC_Channel_18 is selected */
	if (ADC_Channel > 9)
	{
		/* Get the old register value */
		tmpreg1 = ADCx->SMPR2;
		/* Calculate the mask to clear */
		tmpreg2 = ADC_SMPR2_SMP10 << (3 * (ADC_Channel - 10));
		/* Clear the old channel sample time */
		ADCx->SMPR2 &= ~tmpreg2;
		/* Calculate the mask to set */
		ADCx->SMPR2 |= (uint32_t) ADC_SampleTime << (3 * (ADC_Channel - 10));

	}
	else /* ADC_Channel include in ADC_Channel_[0..9] */
	{
		/* Get the old register value */
		tmpreg1 = ADCx->SMPR1;
		/* Calculate the mask to clear */
		tmpreg2 = ADC_SMPR1_SMP1 << (3 * (ADC_Channel - 1));
		/* Clear the old channel sample time */
		ADCx->SMPR1 &= ~tmpreg2;
		/* Calculate the mask to set */
		ADCx->SMPR1 |= (uint32_t) ADC_SampleTime << (3 * (ADC_Channel));
	}
}
#endif


extern uint32_t __main_stack_base__;
extern uint32_t __main_stack_end__;
extern uint32_t __process_stack_base__;
extern uint32_t __process_stack_end__;

uint32_t chThdGetFreeStack(void* pThread)
{
	uint8_t* pEnd = 0;
	uint8_t* pStart = 0;

	if(&ch0.mainthread == pThread)
	{
		pEnd = (uint8_t*)&__process_stack_end__;
		pStart = (uint8_t*)&__process_stack_base__;
	}
	else if(pThread == 0)
	{
		pEnd = (uint8_t*)&__main_stack_end__;
		pStart = (uint8_t*)&__main_stack_base__;
	}
	else
	{
		thread_t* pThd = (thread_t*)pThread;
		pEnd = (uint8_t*)(pThd);
		pStart = (uint8_t*)pThd->wabase;
	}

	uint8_t* pStack = pStart;

	while(pStack < pEnd && *pStack == CH_DBG_STACK_FILL_VALUE)
		pStack++;

	uint32_t stackFree = (uint32_t)pStack - (uint32_t)pStart;

	return stackFree;
}
