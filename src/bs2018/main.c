/*
 * main.c
 *
 *  Created on: 19.10.2017
 *      Author: AndreR
 */

/*
 * Memory map:
 * 0x00000000 - 0x00004000 (  16k) ITC RAM
 *
 * 0x00200000 - 0x00210000 (  64k) Flash via ITC - Bootloader
 * 0x00220000 - 0x00400000 (1920k) Flash via ITC - Run Code
 *
 * 0x08000000 - 0x08010000 (  64k) Flash via AXIM - Bootloader
 * 0x08010000 - 0x08020000 (  64k) Flash via AXIM - FlashFS
 * 0x08020000 - 0x08200000 (1920k) Flash via AXIM - Run Code
 *
 * 0x20000000 - 0x20020000 ( 128k) DTC RAM
 * 0x20020000 - 0x2007C000 ( 368k) RAM1
 * 0x2007C000 - 0x20080000 (  16k) RAM2
 *
 * 0x40000000 - 0x40007FFF (  32k) APB1
 *
 * 0x40010000 - 0x40017BFF (  31k) APB2
 *
 * 0x40020000 - 0x4007FFFF ( 384k) AHB1
 *
 * 0x50000000 - 0x50060BFF ( 387k) AHB2
 *
 * 0x60000000 - 0x60000004 (   4B) TFT CMD
 *
 * 0x60100000 - 0x60100004 (   4B) TFT Data
 *
 * 0xA0000000 - 0xA0001000 (   4k) FMC Ctrl / AHB3
 * 0xA0001000 - 0xA0002000 (   4k) QSPI Ctrl / AHB3
 *
 * 0xE0000000 - 0xE00FFFFF (   1M) Cortex-M7 Internal Peripherals
 */

#include "ch.h"

#include "util/init_hal.h"
#include "util/boot.h"
#include "util/terminal.h"
#include "util/console.h"
#include "util/sys_time.h"
#include "util/flash_fs.h"

#include "hal/led.h"
#include "hal/usart1.h"
#include "hal/eth.h"
#include "hal/tft.h"
#include "hal/rtc.h"
#include "hal/touch.h"

#include "cpu_load.h"
#include "cli.h"
#include "vision.h"
#include "network.h"
#include "wifi.h"
#include "presenter.h"
#include "hub.h"

static THD_WORKING_AREA(waTerminal, 4096);
static THD_WORKING_AREA(waCLI, 8192);
#ifndef ENV_BOOT
static THD_WORKING_AREA(waNetwork, 4096);
static THD_WORKING_AREA(waWifi, 4096);
static THD_WORKING_AREA(waPresenter, 4096);
static THD_WORKING_AREA(waTouch, 512);
#endif

USART_TypeDef* pFaultUart = USART1;

static void mpuInit();

int main()
{
#ifdef DEBUG
	SCB_DisableDCache();
#endif

#ifdef ENV_BOOT
	if(!BootIsBootloaderSelected())
		BootJumpToApplication();

	BootSetBootloaderSelected(0);
#endif

	uint32_t resetReason = (RCC->CSR & 0xFE000000) >> 24;
	RCC->CSR |= RCC_CSR_RMVF;

	// Enable important clocks
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN |
			RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_CRCEN | RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// results in 216MHz main clock and 48MHz USB clock
	SystemClockInitData clkInit;
	clkInit.pll.M = 25;
	clkInit.pll.N = 432;
	clkInit.pll.P = 2;
	clkInit.pll.Q = 9;
	clkInit.HSEBypass = 1;
	clkInit.APB1Div = 4;	// = 54MHz
	clkInit.APB2Div = 4;	// = 54MHz
	clkInit.RTCDiv = 12;	// = 1MHz
	clkInit.flashLatency = 7;
	clkInit.sysTickFreq = CH_CFG_ST_FREQUENCY;
	SystemClockInit(&clkInit, 216000000UL);

	// boost timer clock to full 216MHz
	RCC->DCKCFGR1 |= RCC_DCKCFGR1_TIMPRE;

	// Enable compensation cell to reduce noise on power lines with 50MHz/100MHz I/Os
	SYSCFG->CMPCR = SYSCFG_CMPCR_CMP_PD;

	// Enable usage fault, bus fault, and mem fault
	SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk;

	// set vector table offset
#ifdef ENV_BOOT
	SCB->VTOR = 0x00200000;
#else
	SCB->VTOR = 0x00200000 + BOOT_APP_OFFSET;
#endif

	mpuInit();

	chSysInit();

	// --- HAL ---
	LEDInit();
	RTCInit();
	FlashFSInit(0x08010000, 0x8000);
	USART1Init();
#ifndef ENV_BOOT
	TFTInit();
	TouchInit();
#endif

	// --- High-Level Systems ---
	TerminalInit(&usart1.inQueue, &usart1.outQueue);
	CLIInit();
#ifndef ENV_BOOT
	VisionInit();
	EthInit();
	NetworkInit();
	WifiInit();
	HubInit();
	PresenterInit();
#endif

	chThdSetPriority(HIGHPRIO-1);

	// --- Tasks ---
#ifndef ENV_BOOT
	chThdCreateStatic(waWifi, sizeof(waWifi), NORMALPRIO+10, WifiTask, 0);
	chThdCreateStatic(waNetwork, sizeof(waNetwork), NORMALPRIO+1, NetworkTask, 0);
	chThdCreateStatic(waTouch, sizeof(waTouch), NORMALPRIO-35, TouchTask, 0);
	chThdCreateStatic(waPresenter, sizeof(waPresenter), NORMALPRIO-40, PresenterTask, 0);
#endif
	chThdCreateStatic(waTerminal, sizeof(waTerminal), NORMALPRIO-5, TerminalTask, 0);
	chThdCreateStatic(waCLI, sizeof(waCLI), NORMALPRIO-10, CLITask, 0);

#ifdef ENV_BOOT
	ConsolePrint("\f--- Base Station v3b (Bootloader, 0x%02X) ---\r\n", resetReason);
#else
	ConsolePrint("\f--- Base Station v3b (0x%02X) ---\r\n", resetReason);
#endif

	while(1)
	{
#ifdef ENV_BOOT
		LEDToggle(LED_RED);
		chThdSleepMilliseconds(100);
#else
		LEDToggle(LED_GREEN);
		chThdSleepMilliseconds(500);
#endif

		CPULoadUpdateUsage();
	}

	return 0;
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
