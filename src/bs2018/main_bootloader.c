#include "ch.h"

#include "util/cpu_load.h"
#include "util/fault_handler.h"

#include "hal/sys_time.h"
#include "sys/tim2.h"
#include "sys/usart1.h"
#include "dev/led.h"

#include "system.h"
#include "constants.h"

#include "module/bootloader.h"

Bootloader bootloader;
BootloaderInterface bootIoUsart1;

static uint8_t uartReadByte(void* pUser, uint8_t* pData)
{
	UartDma* pUart = (UartDma*)pUser;

	return UartDmaRead(pUart, pData, 1);
}

static void uartWriteData(void* pUser, const void* const pData, uint32_t dataLength)
{
	UartDma* pUart = (UartDma*)pUser;

	UartDmaWrite(pUart, pData, dataLength);
}

int main()
{
	FaultHandlerInit(USART1, 0);

#ifdef DEBUG
	SCB_DisableDCache();
#endif

	RCC->CSR |= RCC_CSR_RMVF;

	// Enable important clocks
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIODEN |
			RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_CRCEN | RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// set vector table offset
	SCB->VTOR = 0x00200000;

	SystemInit();
	TIM2Init();
	SysTimeInit(&tim2);

	// Enable compensation cell to reduce noise on power lines with 50MHz/100MHz I/Os
	SYSCFG->CMPCR = SYSCFG_CMPCR_CMP_PD;

	// Enable usage fault, bus fault, and mem fault
	SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk;

	chSysInit();

	// --- SYS ---
	USART1Init(IRQL_USART1, TASK_PRIO_CLI_SERIAL);

	// --- DEV ---
	LEDInit();

	// Bootloader setup
	bootIoUsart1.pReadByte = &uartReadByte;
	bootIoUsart1.pWriteData = &uartWriteData;
	bootIoUsart1.pUser = &usart1;

	BootloaderAddInterface(&bootloader, &bootIoUsart1);

	BootloaderData boot;
	boot.bootloaderProgramAddr = 0x08000000;
	boot.bootloaderMaxProgramSize = 64*1024;
	boot.flashAddr = 0x08000000;
	boot.flashSize = 2*1024*1024;
	boot.activeProgramAddr = 0x08020000;
	boot.newProgramAddr = 0x08080000;
	boot.maxProgramSize = 384*1024;
	boot.flashWriteGranularity = 4;
	boot.flashWriteTime_us = 100;
	boot.flashEraseTime_us = 4000000;
	boot.bootloaderTimeout_ms = 2500;
	boot.pDeviceName = "BSV2018";
	boot.pSystemDeinit = &SystemDeinit;

	BootloaderInit(&bootloader, &boot, NORMALPRIO);

	chThdSetPriority(HIGHPRIO-1);

	while(1)
	{
		LEDToggle(LED_RED);
		chThdSleepMilliseconds(100);

		CPULoadUpdateUsage();
	}

	return 0;
}
