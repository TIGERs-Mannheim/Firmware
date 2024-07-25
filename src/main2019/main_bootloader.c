#include "ch.h"
#include "system.h"
#include "constants.h"
#include "hal/sys_time.h"

#include "util/fault_handler.h"
#include "util/cpu_load.h"

#include "sys/usart2.h"
#include "sys/tim2.h"
#include "dev/leds_main.h"
#include "dev/leds_front.h"

#include "module/bootloader.h"
#include <math.h>

Bootloader bootloader;
BootloaderInterface bootIoUsart2;

static void updateFrontLeds();

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
	FaultHandlerInit(USART2, 0);

	// There is a bug on Revision Y devices (errata 2.2.9) leading to data corruption from AXI SRAM
	// => change the switch matrix read issuing capability to 1 for the AXI SRAM target (Target 7)
	if((DBGMCU->IDCODE >> 16) == 0x1003)
		*((__IO uint32_t*)0x51008108) = 0x00000001;

	// enable backup RAM clock
	RCC->AHB4ENR |= RCC_AHB4ENR_BKPRAMEN;
	// disable backup RAM write protection
	PWR->CR1 |= PWR_CR1_DBP;

	// enable GPIO clocks
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOCEN |
			RCC_AHB4ENR_GPIODEN | RCC_AHB4ENR_GPIOEEN | RCC_AHB4ENR_GPIOFEN | RCC_AHB4ENR_GPIOGEN;

	RCC->APB4ENR |= RCC_APB4ENR_SYSCFGEN;
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_DMA2EN;
	RCC->AHB2ENR |= RCC_AHB2ENR_D2SRAM1EN | RCC_AHB2ENR_D2SRAM2EN | RCC_AHB2ENR_D2SRAM3EN;
	RCC->AHB4ENR |= RCC_AHB4ENR_BDMAEN;

	// set vector table offset
	SCB->VTOR = 0x08000000;

	SystemInit();
	TIM2Init();
	SysTimeInit(&tim2);

	// Enable compensation cell to reduce noise on power lines with 50MHz/100MHz I/Os
	SYSCFG->CCCSR = SYSCFG_CCCSR_EN;

	// Enable usage fault, bus fault, and mem fault
	SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk;

	chSysInit();

	// --- SYS ---
	USART2Init(IRQL_SHELL_SERIAL, TASK_PRIO_CLI_SERIAL);

	// --- DEV ---
	DevLedsMainInit();
	DevLedsFrontInit();

	// Configure pin to check if USB powered only
	GPIOInitData gpioInit;
	gpioInit.alternate = 0;
	gpioInit.pupd = GPIO_PUPD_UP;
	gpioInit.mode = GPIO_MODE_INPUT;
	GPIOInit(GPIOG, GPIO_PIN_15, &gpioInit);

	chThdSleepMilliseconds(10);

	// Bootloader setup
	bootIoUsart2.pReadByte = &uartReadByte;
	bootIoUsart2.pWriteData = &uartWriteData;
	bootIoUsart2.pUser = &usart2;

	BootloaderAddInterface(&bootloader, &bootIoUsart2);

	BootloaderData boot;
	boot.bootloaderProgramAddr = 0x08000000;
	boot.bootloaderMaxProgramSize = 128*1024;
	boot.flashAddr = 0x08000000;
	boot.flashSize = 2*1024*1024;
	boot.activeProgramAddr = 0x08100000;
	boot.newProgramAddr = 0x08040000;
	boot.maxProgramSize = 768*1024;
	boot.flashWriteGranularity = 32;
	boot.flashWriteTime_us = 100;
	boot.flashEraseTime_us = 15000000;
	boot.bootloaderTimeout_ms = 500;
	boot.pDeviceName = "TigerBotV2020";
	boot.pSystemDeinit = &SystemDeinit;

	if(GPIOG->IDR & GPIO_PIN_15)
		boot.bootloaderTimeout_ms = 2500; // Extend timeout if USB powered

	BootloaderInit(&bootloader, &boot, NORMALPRIO);

	// Main Task Loop
	chThdSetPriority(HIGHPRIO-1);

	systime_t tLedToggle = chVTGetSystemTimeX();

	while(1)
	{
		if(chVTTimeElapsedSinceX(tLedToggle) > TIME_MS2I(100))
		{
			tLedToggle = chVTGetSystemTimeX();
			LEDMonoToggle(&devLedMainRed);
		}

		chThdSleepMilliseconds(1);

		updateFrontLeds();

		CPULoadUpdateUsage();
	}

	return 0;
}

static void updateFrontLeds()
{
	BootloaderOperation* pOp = &bootloader.currentOperation;

	float time_s = SysTime();

	float intensity = (sinf(time_s * (M_PI*5.0f)) + 1.0f) * 0.5f;
	float invIntensity = 1.0f - intensity;

	if(pOp->state != BOOTLOADER_OPERATION_STATE_ACTIVE)
		pOp = &bootloader.lastOperation;

	switch(pOp->type)
	{
		case BOOTLOADER_MSG_TYPE_CHECK_CRC:
		{
			LEDRGBWSet(&devLedFrontLeft, 0.0f, 0.0f, 0.1f, 0.0f);
			LEDRGBWSet(&devLedFrontRight, 0.0f, 0.0f, 0.1f, 0.0f);
		}
		break;
		case BOOTLOADER_MSG_TYPE_ERASE:
		{
			LEDRGBWSet(&devLedFrontLeft, 0.0f, 0.0f, 0.1f*intensity, 0.0f);
			LEDRGBWSet(&devLedFrontRight, 0.0f, 0.0f, 0.1f*invIntensity, 0.0f);
		}
		break;
		case BOOTLOADER_MSG_TYPE_WRITE:
		{
			float offset = pOp->addr - bootloader.data.activeProgramAddr;
			float frac = offset / (float)bootloader.data.maxProgramSize;

			LEDRGBWSet(&devLedFrontLeft, 0.0f, 0.2f * frac, 0.0f, 0.0f);
			LEDRGBWSet(&devLedFrontRight, 0.0f, 0.1f*intensity, 0.0f, 0.0f);
		}
		break;
		default:
		{
			LEDRGBWSet(&devLedFrontLeft, 0.0f, 0.0f, 0.01f*intensity, 0.01f*intensity);
			LEDRGBWSet(&devLedFrontRight, 0.0f, 0.0f, 0.01f*invIntensity, 0.01f*invIntensity);
		}
		break;
	}
}
