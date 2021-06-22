/*
 * main.c
 *
 *  Created on: 20.10.2015
 *      Author: AndreR
 */

/*
 * Memory map:
 * 0x00000000 - 0x00004000 ( 16k) ITC RAM
 *
 * 0x00200000 - 0x00210000 ( 64k) Flash via ITC - Bootloader
 * 0x00220000 - 0x00300000 (896k) Flash via ITC - Run Code
 *
 * 0x08000000 - 0x08010000 ( 64k) Flash via AXIM - Bootloader
 * 0x08010000 - 0x08020000 ( 64k) Flash via AXIM - FlashFS
 * 0x08020000 - 0x08100000 (896k) Flash via AXIM - Run Code
 *
 * 0x20000000 - 0x2000FC00 ( 63k) DTC RAM
 * 0x2000FC00 - 0x20010000 (  1k) Boot Code
 * 0x20010000 - 0x2004C000 (240k) RAM1
 * 0x2004C000 - 0x20050000 ( 16k) RAM2
 *
 * 0x40000000 - 0x40007FFF ( 32k) APB1
 *
 * 0x40010000 - 0x40016BFF ( 27k) APB2
 *
 * 0x40020000 - 0x4007FFFF (384k) AHB1
 *
 * 0x50000000 - 0x50060BFF (387k) AHB2
 *
 * 0x60000000 - 0x60000004 (  4B) TFT CMD
 *
 * 0x60000100 - 0x60000104 (  4B) TFT Data
 *
 * 0xA0000000 - 0xA0001000 (  4k) FMC Ctrl
 * 0xA0001000 - 0xA0002000 (  4k) QSPI Ctrl
 *
 * 0xE0000000 - 0xE00FFFFF (  1M) Cortex-M7 Internal Peripherals
 */

#include <main/ctrl.h>
#include <main/ctrl_motor.h>
#include <main/ctrl_panthera.h>
#include <main/skills.h>
#include "main/robot.h"
#include "main/network.h"
#include "ch.h"

#include "util/init_hal.h"
#include "util/boot.h"
#include "util/terminal.h"
#include "util/console.h"
#include "util/sys_time.h"
#include "util/flash_fs.h"
#include "util/network_print.h"
#include "util/fifo_block.h"
#include "util/log_file.h"
#include "util/fw_updater.h"
#include "util/fw_loader_wifi.h"
#include "util/fw_loader_fatfs.h"

#include "hal/led.h"
#include "hal/usart3.h"
#include "hal/uart8.h"
#include "hal/buzzer.h"
#include "hal/tft.h"
#include "hal/motors.h"
#include "hal/ext_flash.h"
#include "hal/kicker.h"

#include "usb_fs.h"
#include "usb/usb_hcd.h"
#include "usb/usb_msc.h"
#include "cli.h"
#include "power.h"
#include "spi4.h"
#include "spi6.h"
#include "presenter.h"
#include "sdcard.h"
#include "cpu_load.h"
#include "test.h"
#include "ext.h"

static THD_WORKING_AREA(waTerminal, 4096);
static THD_WORKING_AREA(waCLI, 8192);
#ifndef ENV_BOOT
//#ifndef DEBUG
static THD_WORKING_AREA(waMotors, 512);
static THD_WORKING_AREA(waRobot, 8192);
//#endif
static THD_WORKING_AREA(waPower, 512);
static THD_WORKING_AREA(waSPI4, 4096);
static THD_WORKING_AREA(waSPI6, 512);
static THD_WORKING_AREA(waPresenter, 4096);
static THD_WORKING_AREA(waSDCard, 4096);
static THD_WORKING_AREA(waUSB, 4096);
static THD_WORKING_AREA(waUSBHCD, 4096);
static THD_WORKING_AREA(waKicker, 512);
static THD_WORKING_AREA(waNetwork, 4096);
static THD_WORKING_AREA(waNetworkRel, 512);
static THD_WORKING_AREA(waConfig, 512);
static THD_WORKING_AREA(waLogFile, 4096);
static THD_WORKING_AREA(waTest, 4096);
static THD_WORKING_AREA(waUART8, 512);
#endif

#define LOG_FILE_BUFFER_SIZE	32768
#ifndef ENV_BOOT
static uint8_t logFileWriteBuf[LOG_FILE_BUFFER_SIZE] __attribute__((aligned(16)));
#endif

static void mpuInit();

USART_TypeDef* pFaultUart = USART3;

int main()
{
#ifdef ENV_BOOT
	if(!BootIsBootloaderSelected())
		BootJumpToApplication();

	BootSetBootloaderSelected(0);
#endif

	uint32_t resetReason = (RCC->CSR & 0xFE000000) >> 24;
	RCC->CSR |= RCC_CSR_RMVF;

	// Enable important clocks
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN |
			RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIOFEN | RCC_AHB1ENR_GPIOGEN | RCC_AHB1ENR_CRCEN |
			RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// results in 216MHz main clock and 48MHz USB clock
	SystemClockInitData clkInit;
	clkInit.pll.M = 6;
	clkInit.pll.N = 216;
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
	BootGetApplicationCRC32(); // preload checksum
	LEDInit();
	BuzzerInit();
	FlashFSInit(0x08010000, 0x8000);
	ConfigInit(&ConfigNetworkOutput);
	USART3Init();
	UART8Init();
	FwUpdaterInit();
	FwUpdaterAddProgram((FwUpdaterProgram){FW_ID_MAIN2016, 0x08020000, 0x08080000, 384*1024});
#ifndef ENV_BOOT
	FwUpdaterAddSource((FwUpdaterSource){FW_UPDATER_SOURCE_WIFI, &FwLoaderWifi});
	FwUpdaterAddSource((FwUpdaterSource){FW_UPDATER_SOURCE_USB, &FwLoaderFatFsUSB});
	FwUpdaterAddSource((FwUpdaterSource){FW_UPDATER_SOURCE_SDCARD, &FwLoaderFatFsSDCard});
	PowerInit();
	SPI4Init();
	SPI6Init();
	MotorsInit();
	KickerInit();
	TFTInit();
#endif
	ExtFlashInit();

	// --- High-Level Systems ---
	TerminalInit(&usart3.inQueue, &usart3.outQueue);
	CLIInit();
#ifndef ENV_BOOT
	TestInit();
	PresenterInit();
	SDCardInit();
	LogFileInit(logFileWriteBuf, LOG_FILE_BUFFER_SIZE);
	USBFsInit();
	NetworkInit();
	NetworkPrintInit(&NetworkSendPacketRaw);
	RobotInit();
	CtrlSetController(&ctrlPantheraInstance);
	ExtInit();
#endif

	// --- Tasks ---
	chThdCreateStatic(waTerminal, sizeof(waTerminal), NORMALPRIO-5, TerminalTask, 0);
	chThdCreateStatic(waCLI, sizeof(waCLI), NORMALPRIO-10, CLITask, 0);
#ifndef ENV_BOOT
//#ifndef DEBUG
	chThdCreateStatic(waMotors, sizeof(waMotors), NORMALPRIO+30, MotorsTask, 0);
	chThdCreateStatic(waRobot, sizeof(waRobot), NORMALPRIO+5, RobotTask, 0);
//#endif
	chThdCreateStatic(waKicker, sizeof(waKicker), NORMALPRIO+40, KickerTask, 0);
	chThdCreateStatic(waSPI4, sizeof(waSPI4), NORMALPRIO+20, SPI4Task, 0);
	chThdCreateStatic(waSPI6, sizeof(waSPI6), NORMALPRIO+10, SPI6Task, 0);
	chThdCreateStatic(waUART8, sizeof(waUART8), NORMALPRIO+15, USARTTask, &uart8.uart);
	chThdCreateStatic(waNetwork, sizeof(waNetwork), NORMALPRIO+1, NetworkTask, 0);
	chThdCreateStatic(waPower, sizeof(waPower), NORMALPRIO-1, PowerTask, 0);
	chThdCreateStatic(waNetworkRel, sizeof(waNetworkRel), NORMALPRIO-4, NetworkReliableTask, 0);
	chThdCreateStatic(waConfig, sizeof(waConfig), NORMALPRIO-7, ConfigTask, 0);
	chThdCreateStatic(waUSB, sizeof(waUSB), NORMALPRIO-8, USBTask, 0);
	chThdCreateStatic(waUSBHCD, sizeof(waUSBHCD), NORMALPRIO-9, USBHCDTask, 0);
	chThdCreateStatic(waSDCard, sizeof(waSDCard), NORMALPRIO-10, SDCardTask, 0);
	chThdCreateStatic(waLogFile, sizeof(waLogFile), NORMALPRIO-20, LogFileTask, 0);
	chThdCreateStatic(waPresenter, sizeof(waPresenter), NORMALPRIO-40, PresenterTask, 0);
	chThdCreateStatic(waTest, sizeof(waTest), NORMALPRIO-50, TestTask, 0);

	botParams.extInstalled = uart8.extInstalled;
#endif

#ifdef ENV_BOOT
	ConsolePrint("\f--- Main v2016 (Bootloader, 0x%02X) ---\r\n", resetReason);
	LEDSet(LED_LEFT_RED);
#else
	ConsolePrint("\f--- Main v2016 (0x%02X) ---\r\n", resetReason);
	LEDSet(LED_LEFT_GREEN | LED_RIGHT_GREEN);
//	BuzzerPlay(&buzzSeqTada);
#endif


#ifndef ENV_RUN
	FwUpdaterUpdate();
#endif

	chThdSetPriority(HIGHPRIO-1);

	while(1)
	{
#ifdef ENV_BOOT
		LEDToggle(LED_LEFT_RED | LED_RIGHT_RED);
		chThdSleepMilliseconds(100);
#else
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
