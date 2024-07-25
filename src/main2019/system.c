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

	// Reset HSEON, CSSON , CSION,RC48ON, CSIKERON PLL1ON, PLL2ON and PLL3ON bits
	RCC->CR &= (uint32_t)0xEAF6ED7F;

	RCC->D1CFGR = 0x00000000;
	RCC->D2CFGR = 0x00000000;
	RCC->D3CFGR = 0x00000000;
	RCC->PLLCKSELR = 0x00000000;
	RCC->PLLCFGR = 0x00000000;
	RCC->PLL1DIVR = 0x00000000;
	RCC->PLL1FRACR = 0x00000000;
	RCC->PLL2DIVR = 0x00000000;
	RCC->PLL2FRACR = 0x00000000;
	RCC->PLL3DIVR = 0x00000000;
	RCC->PLL3FRACR = 0x00000000;

	// Reset HSEBYP bit
	RCC->CR &= (uint32_t)0xFFFBFFFF;

	// Disable all interrupts
	RCC->CIER = 0x00000000;

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
	// set voltage scaling to highest performance level (VOS1)
	// required for more than 300MHz
	PWR->CR3 |= PWR_CR3_SCUEN | PWR_CR3_USB33DEN;
	PWR->D3CR |= PWR_D3CR_VOS;

	while((PWR->D3CR & PWR_D3CR_VOSRDY) == 0)
		asm volatile("nop");

	// enable HSE clock
	RCC->CR |= RCC_CR_HSEBYP;
	RCC->CR |= RCC_CR_HSEON;

	while((RCC->CR & RCC_CR_HSERDY) == 0)
		asm volatile("nop");

	// Set PLL source clock to HSE
	RCC->PLLCKSELR = RCC_PLLCKSELR_PLLSRC_HSE;

	// Set PER source clock to HSE
	RCC->D1CCIPR |= RCC_D1CCIPR_CKPERSEL_1;

	/** PLL dividers and input frequencies:
	 * PLL1
	 * PLL 1  DIVM  DIVN  DIVP  DIVQ  DIVR
	 * DIV       1   200     2     8     8
	 * MHz       4   800   400   100   100
	 * State                ON    ON   OFF
	 * PLL 2
	 * DIV 	     1    72     4     4     4
	 * MHz       4   288    72    72    72
	 * State                ON   OFF   OFF
	 * PLL 3
	 * DIV       1   192    25    16     8
	 * MHz       4   768 30.72    48    96
	 * State                ON    ON   OFF
	 */
	// Set DIVM1, DIVM2, DIVM3
	RCC->PLLCKSELR |= (1 << RCC_PLLCKSELR_DIVM1_Pos) | (1 << RCC_PLLCKSELR_DIVM2_Pos) | (1 << RCC_PLLCKSELR_DIVM3_Pos);

	// Set PLL input frequency range to 2-4MHz
	RCC->PLLCFGR |= (1 << RCC_PLLCFGR_PLL1RGE_Pos) | (1 << RCC_PLLCFGR_PLL2RGE_Pos) | (1 << RCC_PLLCFGR_PLL3RGE_Pos);

	// PLL 1 configuration
	RCC->PLL1DIVR = (7 << RCC_PLL1DIVR_R1_Pos) | (7 << RCC_PLL1DIVR_Q1_Pos) | (1 << RCC_PLL1DIVR_P1_Pos) | (199 << RCC_PLL1DIVR_N1_Pos);

	// PLL 2 configuration
	RCC->PLL2DIVR = (3 << RCC_PLL2DIVR_R2_Pos) | (3 << RCC_PLL2DIVR_Q2_Pos) | (3 << RCC_PLL2DIVR_P2_Pos) | (71 << RCC_PLL2DIVR_N2_Pos);

	// PLL 3 configuration
	RCC->PLL3DIVR = (7 << RCC_PLL3DIVR_R3_Pos) | (15 << RCC_PLL3DIVR_Q3_Pos) | (24 << RCC_PLL3DIVR_P3_Pos) | (191 << RCC_PLL3DIVR_N3_Pos);

	// PLL 2 VCO uses medium range (150-420MHz)
	RCC->PLLCFGR |= RCC_PLLCFGR_PLL2VCOSEL;

	// Enable DIV1P, DIV1Q, DIV2P, DIV3P, and DIV3Q outputs
	RCC->PLLCFGR |= RCC_PLLCFGR_DIVP1EN | RCC_PLLCFGR_DIVQ1EN | RCC_PLLCFGR_DIVP2EN | RCC_PLLCFGR_DIVP3EN | RCC_PLLCFGR_DIVQ3EN;

	// Select PLL2P as SPI123 clock
	RCC->D2CCIP1R |= RCC_D2CCIP1R_SPI123SEL_0;

	// Select PLL3P as DFSDM audio clock
	RCC->D2CCIP1R |= RCC_D2CCIP1R_SAI1SEL_1;

	// Select PLL3Q as USB input
	RCC->D2CCIP2R |= RCC_D2CCIP2R_USBSEL_1;

	// start PLL1
	RCC->CR |= RCC_CR_PLL1ON;

	while((RCC->CR & RCC_CR_PLL1RDY) == 0)
		asm volatile("nop");

	// start PLL2
	RCC->CR |= RCC_CR_PLL2ON;

	while((RCC->CR & RCC_CR_PLL2RDY) == 0)
		asm volatile("nop");

	// start PLL3
	RCC->CR |= RCC_CR_PLL3ON;

	while((RCC->CR & RCC_CR_PLL3RDY) == 0)
		asm volatile("nop");

	// HPRE set to 4 => all peripherals and timers running with 100Mhz
	RCC->D1CFGR |= RCC_D1CFGR_HPRE_DIV4;

	// RTC HSE prescaler = 40 => 100kHz RTC clock
	RCC->CFGR |= (40 << 8);
	// TODO: RTC in backup domain must be configured to use HSE as RTC input

	// select PLL1P as system clock input => 400MHz
	RCC->CFGR |= RCC_CFGR_SW_PLL1;

	while((RCC->CFGR & RCC_CFGR_SWS_PLL1) != RCC_CFGR_SWS_PLL1)
		asm volatile("nop");

	// reduce flash latency from default high values to acceptable minimum
	// Programming delay 0b10, 4 Wait States
	FLASH->ACR = 0x24;

	systemClockInfo.SYSClk = 400000000UL;
	systemClockInfo.APB1PeriphClk = 100000000UL;
	systemClockInfo.APB1TimerClk = 100000000UL;
	systemClockInfo.APB2PeriphClk = 100000000UL;
	systemClockInfo.APB2TimerClk = 100000000UL;
	systemClockInfo.APB3PeriphClk = 100000000UL;
	systemClockInfo.H3Clk = 100000000UL;
	systemClockInfo.PLL1QClk = 100000000UL;
}

static void configureSysTick()
{
	uint32_t sysClk = 400000000UL;
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
	// 512MB size, execute never, full access, non-shared device (disables caching)
	MPU->RNR = 0;
	MPU->RBAR = 0x60000000;
	MPU->RASR = MPU_RASR_XN_Msk | (0b010 << MPU_RASR_TEX_Pos)| (3 << MPU_RASR_AP_Pos) |
			(28 << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk;

	// read only from 0x00 (128MB), no more writing to ITCM with zero pointers after MPU init
	MPU->RNR = 1;
	MPU->RBAR = 0x00000000;
	MPU->RASR = (0b101 << MPU_RASR_AP_Pos) | MPU_RASR_B_Msk | MPU_RASR_C_Msk | MPU_RASR_S_Msk |
			(0b001 << MPU_RASR_TEX_Pos) | (26 << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk;

	// 64MB, no access, used to catch memory fill value of 0x55555555
	MPU->RNR = 2;
	MPU->RBAR = 0x54000000;
	MPU->RASR = MPU_RASR_XN_Msk | (0b010 << MPU_RASR_TEX_Pos) | (25 << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk;

	// 256MB, disable caching on SRAM1, SRAM2, SRAM3, SRAM4, Backup SRAM (only AXI SRAM is cachable)
	// other SRAMs are mostly used for DMA
	MPU->RNR = 3;
	MPU->RBAR = 0x30000000;
	MPU->RASR = MPU_RASR_XN_Msk | (0b001 << MPU_RASR_TEX_Pos) | (3 << MPU_RASR_AP_Pos) |
			(27 << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk;

	// 1MB, execute never, full access, except for first 128k (bootloader)
	// prevent execution of motor/IR controller code and new main program code
	MPU->RNR = 4;
	MPU->RBAR = 0x08000000;
	MPU->RASR = MPU_RASR_XN_Msk | (0b001 << MPU_RASR_AP_Pos) | MPU_RASR_C_Msk |
			(19 << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk | (0x01 << MPU_RASR_SRD_Pos);

	MPU->RNR = 5;
	MPU->RBAR = 0;
	MPU->RASR = 0;

	__DSB();
	__ISB();
}

