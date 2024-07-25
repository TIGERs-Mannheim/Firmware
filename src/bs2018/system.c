#include "system.h"
#include "ch.h"
#include "hal/init_hal.h"

static void resetClocks();
static void configureClocks();
static void configureSysTick();
static void mpuInit();

CH_IRQ_HANDLER(SysTick_Handler)
{
	CH_IRQ_PROLOGUE();

	chSysLockFromISR();
	chSysTimerHandlerI();
	chSysUnlockFromISR();

	CH_IRQ_EPILOGUE();
}

void SystemInit()
{
	mpuInit();
	resetClocks();
	configureClocks();
	configureSysTick();
}

void SystemDeinit()
{
	SysTick->CTRL = 0;
	NVIC_DisableIRQ(SysTick_IRQn);

	MPU->CTRL = 0;
    __DSB();
    __ISB();

	resetClocks();
}

static void resetClocks()
{
	__disable_irq();

	// reset clocks to default reset state
	// Set HSION bit
	RCC->CR |= RCC_CR_HSION;

	// Reset CFGR register
	RCC->CFGR = 0x00000000;

	// Reset HSEON, CSSON and PLLON bits
	RCC->CR &= (uint32_t)0xFEF6FFFF;

	// Reset PLLCFGR register
	RCC->PLLCFGR = 0x24003010;

	// Reset HSEBYP bit
	RCC->CR &= (uint32_t)0xFFFBFFFF;

	// Disable all interrupts
	RCC->CIR = 0x00000000;

    // Clear Interrupt Enable Register & Interrupt Pending Register
    for(uint32_t i = 0; i < 5; i++)
    {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    __enable_irq();
}

static void configureClocks()
{
	// results in 216MHz main clock and 48MHz USB clock

	// enable HSE clock
	RCC->CR |= RCC_CR_HSEBYP;
	RCC->CR |= RCC_CR_HSEON;

	while((RCC->CR & RCC_CR_HSERDY) == 0)
		asm volatile("nop");

	RCC->PLLCFGR = (2 << RCC_PLLCFGR_PLLR_Pos) | (9 << RCC_PLLCFGR_PLLQ_Pos) | (0 << RCC_PLLCFGR_PLLP_Pos) | (432 << RCC_PLLCFGR_PLLN_Pos) | (25 << RCC_PLLCFGR_PLLM_Pos) | RCC_PLLCFGR_PLLSRC_HSE;
	RCC->CFGR = (12 << RCC_CFGR_RTCPRE_Pos) | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV4;

	RCC->CR |= RCC_CR_PLLON;

	// Configure Over-Drive mode for highest frequencies
	PWR->CR1 |= PWR_CR1_ODEN;

	while((PWR->CSR1 & PWR_CSR1_ODRDY) == 0)
		asm volatile("nop");

	PWR->CR1 |= PWR_CR1_ODSWEN;

	while((PWR->CSR1 & PWR_CSR1_ODSWRDY) == 0)
		asm volatile("nop");

	while((RCC->CR & RCC_CR_PLLRDY) == 0)
		asm volatile("nop");

	// increase flash latency and enable data/instruction cache + prefetch buffer
	FLASH->ACR = (7 << FLASH_ACR_LATENCY_Pos) | FLASH_ACR_PRFTEN | FLASH_ACR_ARTEN;

	RCC->CFGR |= RCC_CFGR_SW_PLL;

	while((RCC->CFGR & RCC_CFGR_SWS_PLL) == 0)
		asm volatile("nop");

	// boost timer clock to full 216MHz
	RCC->DCKCFGR1 |= RCC_DCKCFGR1_TIMPRE;

	systemClockInfo.SYSClk = 216000000UL;
	systemClockInfo.APB1PeriphClk = 54000000UL;
	systemClockInfo.APB1TimerClk = 216000000UL;
	systemClockInfo.APB2PeriphClk = 54000000UL;
	systemClockInfo.APB2TimerClk = 216000000UL;
}

static void configureSysTick()
{
	uint32_t sysClk = 216000000UL;
	uint32_t sysTickFreq = CH_CFG_ST_FREQUENCY;

	SysTick->LOAD = (sysClk / sysTickFreq) - (systime_t)1;
	SysTick->VAL = (uint32_t)0;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;

	NVIC_SetPriority(SysTick_IRQn, CORTEX_PRIORITY_SVCALL+1);
}

static void mpuInit()
{
	// Enable MPU with default regions as background
	MPU->CTRL = MPU_CTRL_ENABLE_Msk | MPU_CTRL_PRIVDEFENA_Msk;

	// Set up MPU to disable caching on the TFT external RAM address
	// select region 0
	MPU->RNR = 0;
	// set base address
	MPU->RBAR = 0x60000000;
	// 512MB size, execute never, full access, non-shared device (disables caching)
	MPU->RASR = MPU_RASR_XN_Msk | (0b010 << 19)| (3 << 24) | (28 << 1) | MPU_RASR_ENABLE_Msk;

	// read only from 0x00 (128MB)
	MPU->RNR = 1;
	MPU->RBAR = 0x00000000;
	MPU->RASR = (0b101 << 24) | MPU_RASR_B_Msk | MPU_RASR_C_Msk | MPU_RASR_S_Msk | (0b001 << 19) | (26 << 1) | MPU_RASR_ENABLE_Msk;

	// 256MB, no access, except for first 32MB
	MPU->RNR = 2;
	MPU->RBAR = 0x50000000;
	MPU->RASR = MPU_RASR_XN_Msk | (0b010 << MPU_RASR_TEX_Pos) |
			(0x01 << MPU_RASR_SRD_Pos) | (27 << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk;

	MPU->RNR = 3;
	MPU->RBAR = 0;
	MPU->RASR = 0;

	__DSB();
	__ISB();
}
