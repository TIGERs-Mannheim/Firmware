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
 *
 * Timer Allocation:
 * - TIM1  TFT Brightness
 * - TIM2  Microsecond system time
 * - TIM5  Wifi Timeout Trigger
 * - TIM7  (Old) Wifi 1ms trigger
 */

#include "ch.h"

#include "hal/init_hal.h"
#include "hal/sys_time.h"
#include "hal/rng.h"
#include "util/flash_fs.h"
#include "util/cpu_load.h"
#include "util/fault_handler.h"

#include "sys/eth0.h"
#include "sys/rtc.h"
#include "sys/spi3.h"
#include "sys/tim2.h"
#include "sys/tim5.h"
#include "sys/usart1.h"

#include "dev/led.h"
#include "dev/shell.h"
#include "dev/sky_fem.h"
#include "dev/sx1280.h"
#include "dev/tft.h"
#include "dev/touch.h"

#include "misc/inventory.h"

#include "base_station.h"
#include "router.h"
#include "presenter.h"
#include "system.h"
#include "syscalls.h"
#include "shell_commands.h"
#include "constants.h"

#define EVENT_MASK_CLI_UART			EVENT_MASK(0)

static int syscallWrite(const char* pData, int length)
{
	return UartDmaWrite(&usart1, pData, length);
}

int main()
{
	FaultHandlerInit(USART1, 0);

#ifdef DEBUG
	SCB_DisableDCache();
#endif

	uint32_t resetReason = (RCC->CSR & 0xFE000000) >> 24;
	RCC->CSR |= RCC_CSR_RMVF;

	// Enable important clocks
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN |
			RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_CRCEN | RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// set vector table offset
	SCB->VTOR = 0x00220000;

	SystemInit();
	TIM2Init();
	SysTimeInit(&tim2);

	// Enable compensation cell to reduce noise on power lines with 50MHz/100MHz I/Os
	SYSCFG->CMPCR = SYSCFG_CMPCR_CMP_PD;

	// Enable usage fault, bus fault, and mem fault
	SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk;

	chSysInit();

	// --- Software-Only Components ---
	FlashFSInit(0x08010000, 32*1024);
	InventoryInit();

	// --- SYS ---
	USART1Init(IRQL_USART1, TASK_PRIO_CLI_SERIAL);
	SPI3Init(IRQL_WIFI_SPI_DONE);
	TIM5Init(IRQL_WIFI_TIMEOUT);
	RNGInit();
	RTCInit();
	Eth0Init(TASK_PRIO_ETH, IRQL_ETH);

	syscallWriteFunc = &syscallWrite;
	setvbuf(stdout, NULL, _IOLBF, 0); // set stdio (used by printf) to be line-buffered

	// --- DEV ---
	LEDInit();
	DevShellInit(&usart1, TASK_PRIO_SHELL);
	TFTInit();
	TouchInit();
	DevSkyFemInit();
	DevSX1280Init(&spi3, &tim5, IRQL_WIFI_HIGH_PRIO, IRQL_WIFI_PIN);

	// --- High-Level Systems ---
	BaseStationInit();
	RouterInit();
	ShellCommandsInit();
	PresenterInit();

	printf("\f--- Base Station v3b (0x%02X) ---\r\n", resetReason);

	chThdSetPriority(HIGHPRIO-1);

	event_listener_t cliUartListener;

	chEvtRegisterMaskWithFlags(&usart1.eventSource, &cliUartListener, EVENT_MASK_CLI_UART, UART_DMA_EVENT_BREAK_DETECTED);

	while(1)
	{
		LEDToggle(LED_GREEN);
		chThdSleepMilliseconds(500);

		eventmask_t events = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_IMMEDIATE);
		if(events & EVENT_MASK_CLI_UART)
		{
			NVIC_SystemReset();
		}

		CPULoadUpdateUsage();
	}

	return 0;
}
