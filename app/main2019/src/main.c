/*
 * main.c
 *
 *  Created on: 27.12.2018
 *      Author: AndreR
 */

/*
 * Memory map:
 * 0x00000000 - 0x00010000 ( 64k) ITC RAM
 *
 * 0x08000000 - 0x08100000 (  1M) Flash Bank 1 (D1)
 * 0x08100000 - 0x08200000 (  1M) Flash Bank 2 (D1)
 *
 * 0x20000000 - 0x20020000 (128k) DTC RAM
 *
 * 0x24000000 - 0x24080000 (512k) AXI RAM (D1)
 *
 * 0x30000000 - 0x30020000 (128k) SRAM1 (D2)
 * 0x30020000 - 0x30040000 (128k) SRAM2 (D2)
 * 0x30040000 - 0x30048000 ( 32k) SRAM3 (D2)
 *
 * 0x38000000 - 0x38010000 ( 64k) SRAM4 (D3)
 *
 * 0x38800000 - 0x38801000 (  4k) Backup RAM (D3)
 *
 * 0x40000000 - 0x4000D400 ( 53k) APB1 (D2)
 * 0x40010000 - 0x40017800 ( 30k) APB2 (D2)
 * 0x40020000 - 0x400C0000 (640k) AHB1 (D2)
 * 0x48020000 - 0x48022800 ( 10k) Misc Peripherals (see datasheet)
 * 0x48022800 - 0x48022C00 (  1k) AHB2 (D2)
 *
 * 0x50000000 - 0x50004000 ( 16k) APB3 (D1)
 *
 * 0x51000000 - 0x52009000 ( 16M) AHB3 (D1)
 *
 * 0x58000000 - 0x58006c00 ( 27k) APB4 (D3)
 *
 * 0x58020000 - 0x58026800 ( 26k) AHB4 (D3)
 *
 * 0x60000000 - 0x60000004 (  4B) TFT CMD
 *
 * 0x60000100 - 0x60000104 (  4B) TFT Data
 *
 * 0xE0000000 - 0xE00FFFFF (  1M) Cortex-M7 Internal Peripherals
 *
 * Flash Organization:
 * Bank 1:
 * - S1 (128k) Bootloader [0x08000000]
 * - S2 (128k) Flash Filesystem [0x08020000]
 * - S3 (128k) New Motor Program load [0x08040000]
 * - S4 (128k) New IR Controller Program load [0x08060000]
 * - S5 - S8 (512k) New Main Program load [0x08080000 - 0x080FFFFF]
 *
 * Bank 2:
 * - S1 - S4 (512k) Main Program [0x08100000 - 0x0817FFFF]
 * - S5 (128k) Motor Program Code [0x08180000]
 * - S6 (128k) IR Controller Program Code [0x081A0000]
 *
 * Timer Allocation:
 * - TIM1  Buzzer
 * - TIM3  Front LEDs
 * - TIM4  Front LEDs
 * - TIM12 Pattern Ident
 * - TIM13 Kicker
 * - TIM14 Kicker
 */

#include <math.h>

#include "ch.h"

#include "main/network.h"
#include "main/robot.h"
#include "main/ctrl.h"
#include "main/ctrl_panthera.h"

#include "util/init_hal.h"
#include "util/terminal.h"
#include "util/boot.h"
#include "util/sys_time.h"
#include "util/flash_fs.h"
#include "util/console.h"
#include "util/log_file.h"
#include "util/network_print.h"
#include "util/fw_updater.h"
#include "util/fw_loader_wifi.h"
#include "util/fw_loader_fatfs.h"

#include "hal/led.h"
#include "hal/usart2.h"
#include "hal/buzzer.h"
#include "hal/port_ex.h"
#include "hal/i2c2.h"
#include "hal/tft.h"
#include "hal/usb_fs.h"

#include "usb/usb.h"
#include "usb/usb_hcd.h"
#include "kicker.h"
#include "cpu_load.h"
#include "power.h"
#include "cli.h"
#include "motors.h"
#include "spi4.h"
#include "presenter.h"
#include "sdcard.h"
#include "ir.h"
#include "spi2.h"
#include "test.h"
#include "ext.h"
#include "pattern_ident.h"
#include "microphone.h"
#include "robot_pi.h"
#include "inventory.h"

static THD_WORKING_AREA(waTerminal, 4096);
static THD_WORKING_AREA(waCLI, 8192);
static THD_WORKING_AREA(waPresenter, 4096);

#ifndef ENV_BOOT
static THD_WORKING_AREA(waPower, 1024);
static THD_WORKING_AREA(waSPI2, 4096);
static THD_WORKING_AREA(waSPI4, 4096);
static THD_WORKING_AREA(waKicker, 1024);
static THD_WORKING_AREA(waIr, 4096);
static THD_WORKING_AREA(waMotors, 1024);
static THD_WORKING_AREA(waConfig, 1024);
static THD_WORKING_AREA(waLogFile, 4096);
static THD_WORKING_AREA(waUSB, 4096);
static THD_WORKING_AREA(waUSBHCD, 4096);
static THD_WORKING_AREA(waSDCard, 4096);
static THD_WORKING_AREA(waNetwork, 4096);
static THD_WORKING_AREA(waNetworkRel, 1024);
static THD_WORKING_AREA(waRobot, 8192);
static THD_WORKING_AREA(waTest, 8192);
static THD_WORKING_AREA(waExt, 2048);
static THD_WORKING_AREA(waPatternIdent, 2048);
static THD_WORKING_AREA(waMicrophone, 4096);
static THD_WORKING_AREA(waRobotPi, 4096);
#endif

#define LOG_FILE_BUFFER_SIZE	32768
#ifndef ENV_BOOT
static uint8_t logFileWriteBuf[LOG_FILE_BUFFER_SIZE] __attribute__((aligned(16)));
#endif

USART_TypeDef* pFaultUart = USART2;

static void resetClocks();
static void configureClocks();
static void configureSysTick();
static void mpuInit();

int main()
{
	// There is a bug on Revision Y devices (errata 2.2.9) leading to data corruption from AXI SRAM
	// => change the switch matrix read issuing capability to 1 for the AXI SRAM target (Target 7)
	if((DBGMCU->IDCODE >> 16) == 0x1003)
		*((__IO uint32_t*)0x51008108) = 0x00000001;

	mpuInit();

	// enable backup RAM clock (used for bootloader magic code)
	RCC->AHB4ENR |= RCC_AHB4ENR_BKPRAMEN;
	// disable backup RAM write protection
	PWR->CR1 |= PWR_CR1_DBP;

#ifdef ENV_BOOT
	if(!BootIsBootloaderSelected())
		BootJumpToApplication();

	BootSetBootloaderSelected(0);
#endif

	// enable GPIO clocks
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOCEN |
			RCC_AHB4ENR_GPIODEN | RCC_AHB4ENR_GPIOEEN | RCC_AHB4ENR_GPIOFEN | RCC_AHB4ENR_GPIOGEN;

	RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;
	RCC->AHB2ENR |= RCC_AHB2ENR_D2SRAM1EN | RCC_AHB2ENR_D2SRAM2EN | RCC_AHB2ENR_D2SRAM3EN;
	RCC->AHB4ENR |= RCC_AHB4ENR_BDMAEN;

	// set vector table offset
#ifdef ENV_BOOT
	SCB->VTOR = 0x08000000;
#else
	SCB->VTOR = 0x08000000 + BOOT_APP_OFFSET;
#endif

	resetClocks();
	configureClocks();
	configureSysTick();

	// Enable compensation cell to reduce noise on power lines with 50MHz/100MHz I/Os
	SYSCFG->CCCSR = SYSCFG_CCCSR_EN;

	// Enable usage fault, bus fault, and mem fault
	SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk;

	chSysInit();

	// --- HAL ---
	BootGetApplicationCRC32(); // preload checksum
	LEDInit();
	BuzzerInit();
	InventoryInit();
	FlashFSInit(0x08020000, 0x20000);
	ConfigInit(&ConfigNetworkOutput);
	USART2Init();
	FwUpdaterInit();
	FwUpdaterAddProgram((FwUpdaterProgram){FW_ID_MAIN2019, 0x08100000, 0x08080000, 512*1024});
	FwUpdaterAddProgram((FwUpdaterProgram){FW_ID_MOT2019,  0x08180000, 0x08040000,  32*1024});
	FwUpdaterAddProgram((FwUpdaterProgram){FW_ID_IR2019,   0x081A0000, 0x08060000,  32*1024});
	TFTInit();
#ifndef ENV_BOOT
	FwUpdaterAddSource((FwUpdaterSource){FW_UPDATER_SOURCE_WIFI, &FwLoaderWifi});
	FwUpdaterAddSource((FwUpdaterSource){FW_UPDATER_SOURCE_USB, &FwLoaderFatFsUSB});
	FwUpdaterAddSource((FwUpdaterSource){FW_UPDATER_SOURCE_SDCARD, &FwLoaderFatFsSDCard});
	I2C2Init();
	PortExInit(&i2c2);
	PowerInit(&i2c2);
	MotorsInit();
	SPI4Init();
	SPI2Init();
	TestInit();
	KickerInit();
	IrInit();
	ExtInit(2500000);
	PatternIdentInit();
	RobotPiInit();
	LogFileInit(logFileWriteBuf, LOG_FILE_BUFFER_SIZE);
	USBFsInit();
	SDCardInit();
	NetworkInit();
	NetworkPrintInit(&NetworkSendPacketRaw);
	MicrophoneInit();
	RobotInit();
	CtrlSetController(&ctrlPantheraInstance);
#endif

	// --- High-Level Systems ---
	TerminalInit(&usart2.inQueue, &usart2.outQueue);
	CLIInit();
	PresenterInit();

	// --- Tasks ---
	chThdCreateStatic(waTerminal, sizeof(waTerminal), NORMALPRIO-5, TerminalTask, 0);
	chThdCreateStatic(waCLI, sizeof(waCLI), NORMALPRIO-10, CLITask, 0);
	chThdCreateStatic(waPresenter, sizeof(waPresenter), NORMALPRIO-40, PresenterTask, 0);
#ifndef ENV_BOOT
	chThdCreateStatic(waNetwork, sizeof(waNetwork), NORMALPRIO+50, NetworkTask, 0);
	chThdCreateStatic(waKicker, sizeof(waKicker), NORMALPRIO+40, KickerTask, 0);
	chThdCreateStatic(waSPI2, sizeof(waSPI2), NORMALPRIO+30, SPI2Task, 0);
	chThdCreateStatic(waSPI4, sizeof(waSPI4), NORMALPRIO+20, SPI4Task, 0);
	chThdCreateStatic(waMotors, sizeof(waMotors), NORMALPRIO+15, MotorsTask, 0);
	chThdCreateStatic(waIr, sizeof(waIr), NORMALPRIO+10, IrTask, 0);
	chThdCreateStatic(waRobot, sizeof(waRobot), NORMALPRIO+5, RobotTask, 0);
	chThdCreateStatic(waPower, sizeof(waPower), NORMALPRIO-1, PowerTask, 0);
	chThdCreateStatic(waExt, sizeof(waExt), NORMALPRIO-3, ExtTask, 0);
	chThdCreateStatic(waNetworkRel, sizeof(waNetworkRel), NORMALPRIO-4, NetworkReliableTask, 0);
	chThdCreateStatic(waRobotPi, sizeof(waRobotPi), NORMALPRIO-5, RobotPiTask, 0);
	chThdCreateStatic(waConfig, sizeof(waConfig), NORMALPRIO-7, ConfigTask, 0);
	chThdCreateStatic(waUSB, sizeof(waUSB), NORMALPRIO-8, USBTask, 0);
	chThdCreateStatic(waUSBHCD, sizeof(waUSBHCD), NORMALPRIO-9, USBHCDTask, 0);
	chThdCreateStatic(waSDCard, sizeof(waSDCard), NORMALPRIO-10, SDCardTask, 0);
	chThdCreateStatic(waPatternIdent, sizeof(waPatternIdent), NORMALPRIO-15, PatternIdentTask, 0);
	chThdCreateStatic(waLogFile, sizeof(waLogFile), NORMALPRIO-20, LogFileTask, 0);
	chThdCreateStatic(waMicrophone, sizeof(waMicrophone), NORMALPRIO-30, MicrophoneTask, 0);
	chThdCreateStatic(waTest, sizeof(waTest), NORMALPRIO-50, TestTask, 0);
#endif

#ifdef ENV_BOOT
	ConsolePrint("\f--- Main v2019 (Bootloader) ---\r\n");
#else
	ConsolePrint("\f--- Main v2019 ---\r\n");
#endif

	LEDLeftSet(0.0f, 0.0f, 0.01f, 0.0f);
	LEDRightSet(0.0f, 0.01f, 0.0f, 0.0f);

#ifndef ENV_RUN
	FwUpdaterUpdate();
#endif

	chThdSetPriority(HIGHPRIO-1);

	while(1)
	{
#ifdef ENV_BOOT
		LEDToggle(LED_MAIN_RED);
		chThdSleepMilliseconds(100);
#else
		LEDToggle(LED_MAIN_GREEN);
		chThdSleepMilliseconds(500);
#endif

		CPULoadUpdateUsage();
	}

	return 0;
}

static void resetClocks()
{
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
	 * PLL 2  (Off)
	 * DIV 	     2   100     2     2     2
	 * MHz       2   200   100   100   100
	 * State               OFF   OFF   OFF
	 * PLL 3
	 * DIV       1   192    25    16     8
	 * MHz       4   768 30.72    48    96
	 * State                ON    ON   OFF
	 */
	// Set DIVM1, DIVM2, DIVM3
	RCC->PLLCKSELR |= (1 << RCC_PLLCKSELR_DIVM1_Pos) | (0 << RCC_PLLCKSELR_DIVM2_Pos) | (1 << RCC_PLLCKSELR_DIVM3_Pos);

	// Set PLL input frequency range to 2-4MHz
	RCC->PLLCFGR |= (1 << RCC_PLLCFGR_PLL1RGE_Pos) | (1 << RCC_PLLCFGR_PLL2RGE_Pos) | (1 << RCC_PLLCFGR_PLL3RGE_Pos);

	// PLL 1 configuration
	RCC->PLL1DIVR = (7 << RCC_PLL1DIVR_R1_Pos) | (7 << RCC_PLL1DIVR_Q1_Pos) | (1 << RCC_PLL1DIVR_P1_Pos) | (199 << RCC_PLL1DIVR_N1_Pos);

	// PLL 2 configuration
	RCC->PLL2DIVR = (1 << RCC_PLL2DIVR_R2_Pos) | (1 << RCC_PLL2DIVR_Q2_Pos) | (1 << RCC_PLL2DIVR_P2_Pos) | (99 << RCC_PLL2DIVR_N2_Pos);

	// PLL 3 configuration
	RCC->PLL3DIVR = (7 << RCC_PLL3DIVR_R3_Pos) | (15 << RCC_PLL3DIVR_Q3_Pos) | (24 << RCC_PLL3DIVR_P3_Pos) | (191 << RCC_PLL3DIVR_N3_Pos);

	// PLL 2 VCO uses medium range (150-420MHz)
	RCC->PLLCFGR |= RCC_PLLCFGR_PLL2VCOSEL;

	// Enable DIV1P, DIV1Q, DIV3P, and DIV3Q outputs
	RCC->PLLCFGR |= RCC_PLLCFGR_DIVP1EN | RCC_PLLCFGR_DIVQ1EN | RCC_PLLCFGR_DIVP3EN | RCC_PLLCFGR_DIVQ3EN;

	// Select PLL3P as DFSDM audio clock
	RCC->D2CCIP1R |= RCC_D2CCIP1R_SAI1SEL_1;

	// Select PLL3Q as USB input
	RCC->D2CCIP2R |= RCC_D2CCIP2R_USBSEL_1;

	// start PLL1
	RCC->CR |= RCC_CR_PLL1ON;

	while((RCC->CR & RCC_CR_PLL1RDY) == 0)
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

	// 256MB, disable caching on SRAM1, SRAM2, SRAM3, SRAM4, Backup SRAM (only 256k in AXI SRAM are cachable)
	// other SRAMS are mostly used for DMA
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

	// 128kB, disable caching on last quarter of AXI SRAM
	MPU->RNR = 5;
	MPU->RBAR = 0x24060000;
	MPU->RASR = MPU_RASR_XN_Msk | (0b001 << MPU_RASR_TEX_Pos) | (3 << MPU_RASR_AP_Pos) |
			(16 << MPU_RASR_SIZE_Pos) | MPU_RASR_ENABLE_Msk;

	MPU->RNR = 6;
	MPU->RBAR = 0;
	MPU->RASR = 0;

	__DSB();
	__ISB();
}
