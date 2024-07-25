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
 * - S3 - S8 (768k) New Main Program load [0x08040000 - 0x080FFFFF]
 *
 * Bank 2:
 * - S1 - S6 (768k) Main Program [0x08100000 - 0x081BFFFF]
 * - S7 - S8 (256k) unused
 *
 * Timer Allocation:
 * - TIM1  Buzzer
 * - TIM2  Microsecond system time
 * - TIM3  Front LEDs
 * - TIM4  Front LEDs
 * - TIM5  Wifi Timeout Trigger
 * - TIM12 Pattern Ident
 * - TIM13 Kicker
 * - TIM14 Kicker
 *
 * DMA Usage:
 * - DMA1-CH0 SPI4_RX (IMU, Mag, Touch)
 * - DMA1-CH1 SPI4_TX
 * - DMA1-CH2 SPI2_RX (Kicker)
 * - DMA1-CH3 SPI2_TX
 * - ...
 * - DMA1-CH6 USART6_RX (RPi)
 * - DMA1-CH7 USART6_TX
 * - DMA2-CH0 SPI1_RX (Wifi)
 * - DMA2-CH1 SPI1_TX
 * - DMA2-CH2 USART2_RX (Shell)
 * - DMA2-CH3 USART2_TX
 * - ...
 */

#include "ch.h"
#include "syscalls.h"
#include <stdio.h>

#include "robot/network.h"
#include "robot/robot.h"

#include "util/flash_fs.h"
#include "util/log_file.h"
#include "util/network_print.h"
#include "util/cpu_load.h"
#include "util/fault_handler.h"

#include "sys/i2c2.h"
#include "sys/i2c5.h"
#include "sys/sdcard.h"
#include "sys/spi1.h"
#include "sys/spi2.h"
#include "sys/spi4.h"
#include "sys/tim2.h"
#include "sys/tim5.h"
#include "sys/tim13.h"
#include "sys/tim14.h"
#include "sys/usart1.h"
#include "sys/usart2.h"
#include "sys/usart3.h"
#include "sys/uart4.h"
#include "sys/uart5.h"
#include "sys/usart6.h"
#include "sys/uart7.h"
#include "sys/uart8.h"
#include "sys/usb_fs.h"

#include "dev/adc_kicker.h"
#include "dev/buzzer.h"
#include "dev/drib_ir.h"
#include "dev/imu.h"
#include "dev/leds_front.h"
#include "dev/leds_main.h"
#include "dev/mag.h"
#include "dev/motors.h"
#include "dev/port_exp.h"
#include "dev/power_mon.h"
#include "dev/raspberry_pi.h"
#include "dev/shell.h"
#include "dev/sky_fem.h"
#include "dev/sx1280.h"
#include "dev/tft.h"
#include "dev/touch.h"

#include "hal/usb/usb_hcd.h"
#include "hal/sys_time.h"

#include "misc/inventory.h"
#include "misc/variant.h"

#include "test/tests.h"

#include "system.h"
#include "presenter.h"
#include "tiger_bot.h"
#include "constants.h"
#include "shell_commands.h"

static THD_WORKING_AREA(waPresenter, 4096);
static THD_WORKING_AREA(waConfig, 1024);
static THD_WORKING_AREA(waLogFile, 4096);
static THD_WORKING_AREA(waUSB, 4096);
static THD_WORKING_AREA(waUSBHCD, 4096);
static THD_WORKING_AREA(waSDCard, 4096);
static THD_WORKING_AREA(waNetwork, 4096);
static THD_WORKING_AREA(waNetworkRel, 1024);
static THD_WORKING_AREA(waRobot, 8192);

#define LOG_FILE_BUFFER_SIZE	(128*1024)
static uint8_t logFileWriteBuf[LOG_FILE_BUFFER_SIZE] __attribute__((aligned(32)));

static void emergencyStop()
{
	GPIOReset(GPIOG, GPIO_PIN_11); // Disable kicker CHG signal
	TIM1->CR1 = 0; // Disable buzzer timer
	// Motors will turn off automatically if not receiving any further messages.
}

static int syscallWrite(const char* pData, int length)
{
	return UartDmaWrite(&usart2, pData, length);
}

int main()
{
	FaultHandlerInit(USART2, &emergencyStop);

	// There is a bug on Revision Y devices (errata 2.2.9) leading to data corruption from AXI SRAM
	// => change the switch matrix read issuing capability to 1 for the AXI SRAM target (Target 7)
	if((DBGMCU->IDCODE >> 16) == 0x1003)
		*((__IO uint32_t*)0x51008108) = 0x00000001;

	// enable backup RAM clock (used for bootloader magic code)
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
	SCB->VTOR = 0x08100000;

	SystemInit();
	TIM2Init();
	SysTimeInit(&tim2);

	// Enable compensation cell to reduce noise on power lines with 50MHz/100MHz I/Os
	SYSCFG->CCCSR = SYSCFG_CCCSR_EN;

	// Enable usage fault, bus fault, and mem fault
	SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk;

	chSysInit();

	// --- Software-Only Components ---
	InventoryInit();
	FlashFSInit(0x08020000, 128*1024);
	ConfigInit(&ConfigNetworkOutput);
	VariantInit();

	// --- SYS ---
	I2C2Init(IRQL_I2C2);
	I2C5Init(IRQL_I2C_SOFT);
	SPI1Init(IRQL_WIFI_SPI_DONE);
	SPI2Init(IRQL_SPI_KICKER);
	SPI4Init(IRQL_SPI4);
	TIM5Init(IRQL_WIFI_TIMEOUT);
	TIM13Init(IRQL_KICK_CTRL);
	TIM14Init(IRQL_KICK_CTRL);
	USART1Init(IRQL_MOTOR_COMMS);
	USART2Init(IRQL_SHELL_SERIAL, TASK_PRIO_CLI_SERIAL);
	USART3Init(IRQL_MOTOR_COMMS);
	UART4Init(IRQL_MOTOR_COMMS);
	UART5Init(IRQL_MOTOR_COMMS);
	USART6Init(IRQL_RPI_SERIAL, TASK_PRIO_RPI_SERIAL);
	UART7Init(IRQL_MOTOR_COMMS);
	UART8Init(IRQL_IR_COMM);

	syscallWriteFunc = &syscallWrite;
	setvbuf(stdout, NULL, _IOLBF, 0); // set stdio (used by printf) to be line-buffered

	// --- DEV ---
	DevLedsMainInit();
	DevLedsFrontInit();
	DevBuzzerInit();
	DevShellInit(&usart2, TASK_PRIO_SHELL);
	DevTFTInit();
	DevSkyFemInit();
	DevSX1280Init(&spi1, &tim5, IRQL_WIFI_HIGH_PRIO, IRQL_WIFI_PIN);
	DevAdcKickerInit(&spi2, TASK_PRIO_ADC_KICKER);
	DevImuInit(&spi4, TASK_PRIO_IMU);
	DevMagInit(&spi4, TASK_PRIO_MAG);
	DevTouchInit(&spi4, TASK_PRIO_TOUCH);
	DevPortExpInit(&i2c2, TASK_PRIO_PORT_EX);
	DevPowerMonInit(&i2c2, TASK_PRIO_PWR_INA226);
	DevDribIrInit(&uart8, &devPortExp.pins[0].pin, &devPortExp.pins[1].pin, TASK_PRIO_MCU_DRIBBLER);
	DevMotorsCfgInit(variant.config.hasPlanetaryGear);
	DevMotorInit(0, &usart1, &devPortExp.pins[12].pin, &devPortExp.pins[13].pin, TASK_PRIO_MCU_MOTOR);
	DevMotorInit(1,  &uart7, &devPortExp.pins[ 4].pin, &devPortExp.pins[ 5].pin, TASK_PRIO_MCU_MOTOR);
	DevMotorInit(2,  &uart4, &devPortExp.pins[ 8].pin, &devPortExp.pins[ 9].pin, TASK_PRIO_MCU_MOTOR);
	DevMotorInit(3, &usart3, &devPortExp.pins[10].pin, &devPortExp.pins[11].pin, TASK_PRIO_MCU_MOTOR);
	DevMotorInit(4,  &uart5, &devPortExp.pins[ 2].pin, &devPortExp.pins[ 3].pin, TASK_PRIO_MCU_MOTOR);
	DevRaspberryPiInit(&usart6, TASK_PRIO_MPU_EXT);

	// --- High-Level Systems ---
	RobotInit();
	TigerBotInit();
	NetworkInit();
	NetworkPrintInit(&NetworkSendPacketRaw);
	LogFileInit(logFileWriteBuf, LOG_FILE_BUFFER_SIZE);
	TestsInit(TASK_PRIO_TESTS);
	USBFsInit();
	SDCardInit();
	PresenterInit();
	ShellCommandsInit();

	DevMotorRegisterShellCommands(&devShell.shell.cmdHandler);

	// --- Tasks ---
	chThdCreateStatic(waPresenter, sizeof(waPresenter), NORMALPRIO-40, PresenterTask, 0);
	chThdCreateStatic(waNetwork, sizeof(waNetwork), NORMALPRIO+50, NetworkTask, 0);
	chThdCreateStatic(waRobot, sizeof(waRobot), NORMALPRIO+5, RobotTask, 0);
	chThdCreateStatic(waNetworkRel, sizeof(waNetworkRel), NORMALPRIO-4, NetworkReliableTask, 0);
	chThdCreateStatic(waConfig, sizeof(waConfig), NORMALPRIO-7, ConfigTask, 0);
	chThdCreateStatic(waUSB, sizeof(waUSB), NORMALPRIO-8, USBTask, 0);
	chThdCreateStatic(waUSBHCD, sizeof(waUSBHCD), NORMALPRIO-9, USBHCDTask, 0);
	chThdCreateStatic(waSDCard, sizeof(waSDCard), NORMALPRIO-10, SDCardTask, 0);
	chThdCreateStatic(waLogFile, sizeof(waLogFile), NORMALPRIO-20, LogFileTask, 0);

	LEDRGBWSet(&devLedFrontLeft, 0.0f, 0.0f, 0.01f, 0.0f);
	LEDRGBWSet(&devLedFrontRight, 0.0f, 0.01f, 0.0f, 0.0f);

	chThdSetPriority(HIGHPRIO-1);

	while(1)
	{
		LEDMonoToggle(&devLedMainGreen);
		chThdSleepMilliseconds(500);

		CPULoadUpdateUsage();
	}

	return 0;
}
