/*
 * power.c
 *
 *  Created on: 01.01.2019
 *      Author: AndreR
 */

#include "power.h"
#include "ext.h"

#include "robot/ctrl.h"

#include "hal/buzzer.h"
#include "kicker.h"
#include "util/init_hal.h"
#include "constants.h"
#include "util/log_file.h"
#include "robot/network.h"
#include "robot/robot.h"
#include "util/console.h"

Power power;

#define SET_KILL()		(GPIOD->BSRR = GPIO_PIN_6)
#define CLEAR_KILL()	(GPIOD->BSRR = GPIO_PIN_6 << 16)
#define GET_PDN()		(GPIOD->IDR & GPIO_PIN_3)
#define GET_USB_PWR()	(GPIOG->IDR & GPIO_PIN_15)

#define EVENT_PDN 0
#define EVENT_BTN_PRESSED 1
#define EVENT_USB_ATTACHED 2
#define EVENT_USB_DETACHED 3

#define I2C_ADDR_INA226	0x40

void EXTI3_IRQHandler()
{
	CH_IRQ_PROLOGUE();

	EXTI_D1->PR1 |= EXTI_PR1_PR3;

	chSysLockFromISR();
	chMBPostI(&power.eventQueue, EVENT_BTN_PRESSED);
	chSysUnlockFromISR();

	CH_IRQ_EPILOGUE();
}

void EXTI15_10_IRQHandler()
{
	CH_IRQ_PROLOGUE();

	EXTI_D1->PR1 |= EXTI_PR1_PR15;

	uint32_t event;
	if(GET_USB_PWR())
		event = EVENT_USB_ATTACHED;
	else
		event = EVENT_USB_DETACHED;

	chSysLockFromISR();
	chMBPostI(&power.eventQueue, event);
	chSysUnlockFromISR();

	CH_IRQ_EPILOGUE();
}

static void ina226Update()
{
	uint8_t selectShuntReg = 0x01;
	uint8_t selectBusReg = 0x02;
	int16_t shunt;
	uint16_t bus;

	I2CWrite(power.pI2C, I2C_ADDR_INA226, 1, &selectShuntReg);
	I2CRead(power.pI2C, I2C_ADDR_INA226, 2, (uint8_t*)&shunt);
	I2CWrite(power.pI2C, I2C_ADDR_INA226, 1, &selectBusReg);
	I2CRead(power.pI2C, I2C_ADDR_INA226, 2, (uint8_t*)&bus);

	bus = __REV16(bus);
	shunt = __REV16(shunt);

	power.iCur = shunt * (2.5e-6f/0.002f);
	power.vBat = bus * 1.25e-3f;

	float cellMinWithLoad = 3.65f - power.iCur * 0.03f;
	float cellEnergeticWithLoad = 3.70f - power.iCur * 0.02f;

	power.cellMin = fmaxf(cellMinWithLoad, 3.35f);
	power.cellEnergetic = fmaxf(cellEnergeticWithLoad, 3.5f);
}

void PowerInit(I2C* pI2C)
{
	power.pI2C = pI2C;
	power.batCells = 6;
	power.cellMin = 3.70f;
	power.cellEnergetic = 3.80f;
	power.cellMax = 4.20f;

	chMBObjectInit(&power.eventQueue, power.eventQueueData, POWER_EVENT_QUEUE_SIZE);

	GPIOInitData gpioInit;
	gpioInit.alternate = 0;

	gpioInit.mode = GPIO_MODE_EXTI;
	gpioInit.pupd = GPIO_PUPD_UP;
	gpioInit.extiTrigger = GPIO_EXTI_TRIG_RISING_FALLING;
	GPIOInit(GPIOG, GPIO_PIN_15, &gpioInit); // USB powered

	gpioInit.extiTrigger = GPIO_EXTI_TRIG_FALLING;
	GPIOInit(GPIOD, GPIO_PIN_3, &gpioInit); // power down request line

	SET_KILL();

	gpioInit.mode = GPIO_MODE_OUTPUT;
	gpioInit.otype = GPIO_OTYPE_OPEN_DRAIN;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	GPIOInit(GPIOD, GPIO_PIN_6, &gpioInit); // primary power kill switch

	NVICEnableIRQ(EXTI3_IRQn, IRQL_EXTI3);
	NVICEnableIRQ(EXTI15_10_IRQn, IRQL_EXTI10_15);
}

void PowerShutdown()
{
	chMBPost(&power.eventQueue, EVENT_PDN, TIME_INFINITE);
}

void PowerTask(void* params)
{
	(void)params;

	chThdSleepMilliseconds(400);

	// sample time 588us and 4x averaging on bus and shunt => 4704us per conversion
	uint8_t ina226Cfg[3] = { 0x00, 0b01000010, 0b11011111 };
	I2CWrite(power.pI2C, I2C_ADDR_INA226, 3, ina226Cfg);

	chThdSleepMilliseconds(10);

	ina226Update();

	msg_t event;
	uint32_t lastPowerGoodTime = chVTGetSystemTimeX();
	uint32_t lastEnergeticTime = chVTGetSystemTimeX();

	power.usbPowered = GET_USB_PWR() ? 1 : 0;

	chRegSetThreadName("Power");

	while(1)
	{
		ina226Update();

		if(power.vBat < 15.0f)
			power.batCells = 4;
		else
			power.batCells = 6;

		// event processing
		if(chMBFetch(&power.eventQueue, &event, MS2ST(5)) == MSG_OK)
		{
			switch(event)
			{
				case EVENT_BTN_PRESSED:
				{
					uint8_t abort = 0;

					for(uint8_t i = 0; i < 5; i++)
					{
						chThdSleepMilliseconds(10);

						if(GET_PDN() != 0)
						{
							abort = 1;
							break;
						}
					}

					if(abort)
						continue;

					chMBPost(&power.eventQueue, EVENT_PDN, TIME_IMMEDIATE);
				}
				break;
				case EVENT_PDN:
				{
					power.batEmpty = 1;
					chThdSleepMilliseconds(10);

					systime_t extShutdownStart = 0;

					if(ext.installed)
					{
						ExtShutdown shutdown;
						shutdown.key = EXT_SHUTDOWN_KEY;

						PacketHeader header;
						header.section = SECTION_EXT;
						header.cmd = CMD_EXT_SHUTDOWN;

						for(uint8_t i = 0; i < 10; i++)
						{
							ExtSendPacket(&header, &shutdown, sizeof(ExtShutdown));
							chThdSleepMilliseconds(10);
						}

						extShutdownStart = chVTGetSystemTimeX();
					}

					LogFileClose();

					if(kicker.installed)
					{
						KickerAutoDischarge();
						chThdSleepMilliseconds(10);

						while(kicker.autoDischarge)
							chThdSleepMilliseconds(10);
					}

					if(extShutdownStart)
					{
						RobotImplBuzzerPlay(BUZZ_DOUBLE_SLOW);

						chThdSleepUntil(extShutdownStart + S2ST(2));

						RobotImplBuzzerPlay(BUZZ_TADA);
					}

					chThdSleepMilliseconds(100);

					CLEAR_KILL();

					chThdSleepMilliseconds(500);
					BuzzerTone(0);
					power.batEmpty = 0;
					power.exhausted = 0;

					SET_KILL();
				}
				break;
				case EVENT_USB_ATTACHED:
				{
					power.usbPowered = 1;

				}
				break;
				case EVENT_USB_DETACHED:
				{
					power.usbPowered = 0;
				}
				break;
			}
		}

		if(power.vBat > power.cellEnergetic*power.batCells || power.usbPowered)
			lastEnergeticTime = chVTGetSystemTimeX();

		if(power.vBat > power.cellMin*power.batCells || power.usbPowered)
			lastPowerGoodTime = chVTGetSystemTimeX();

		if(power.batEmpty && power.vBat < 3.3f*power.batCells)
			PowerShutdown();

		if(chVTTimeElapsedSinceX(lastPowerGoodTime) > S2ST(10) && power.batEmpty == 0)
		{
			power.batEmpty = 1;
		}

		if(chVTTimeElapsedSinceX(lastEnergeticTime) > S2ST(10) && power.exhausted == 0)
		{
			power.exhausted = 1;
		}
	}
}
