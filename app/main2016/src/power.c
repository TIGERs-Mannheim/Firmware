/*
 * power.c
 *
 *  Created on: 24.10.2015
 *      Author: AndreR
 */

#include "power.h"

#include <main/ctrl.h>

#include "hal/buzzer.h"
#include "hal/kicker.h"
#include "util/init_hal.h"
#include "constants.h"
#include "hal/kicker.h"
#include "util/log_file.h"
#include "main/network.h"
#include "main/robot.h"

Power power;

#define SET_KILL()		(GPIOC->BSRR = GPIO_PIN_1)
#define CLEAR_KILL()	(GPIOC->BSRR = GPIO_PIN_1 << 16)
#define GET_PDN()		(GPIOC->IDR & GPIO_PIN_0)

#define EVENT_PDN 0
#define EVENT_BTN_PRESSED 1

void EXTI0_IRQHandler()
{
	CH_IRQ_PROLOGUE();

	EXTI->PR |= EXTI_PR_PR0;

	chSysLockFromISR();
	chMBPostI(&power.eventQueue, EVENT_BTN_PRESSED);
	chSysUnlockFromISR();

	CH_IRQ_EPILOGUE();
}

void PowerInit()
{
	power.batCells = 3;

	chMBObjectInit(&power.eventQueue, power.eventQueueData, POWER_EVENT_QUEUE_SIZE);

	GPIOInitData gpioInit;
	gpioInit.alternate = 0;
	gpioInit.mode = GPIO_MODE_EXTI;
	gpioInit.extiTrigger = GPIO_EXTI_TRIG_FALLING;
	gpioInit.pupd = GPIO_PUPD_UP;
	GPIOInit(GPIOC, GPIO_PIN_0, &gpioInit);

	SET_KILL();

	gpioInit.mode = GPIO_MODE_OUTPUT;
	gpioInit.otype = GPIO_OTYPE_OPEN_DRAIN;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	GPIOInit(GPIOC, GPIO_PIN_1, &gpioInit);

	NVICEnableIRQ(EXTI0_IRQn, IRQL_EXTI0);
}

void PowerShutdown()
{
	chMBPost(&power.eventQueue, EVENT_PDN, TIME_INFINITE);
}

void PowerTask(void* params)
{
	(void)params;

	chThdSleepMilliseconds(400);

	float avgBatt = 0;
	for(uint8_t i = 0; i < 50; i++)
	{
		avgBatt += power.vBat;
		chThdSleepMilliseconds(1);
	}

	avgBatt /= 50.0f;

	if(avgBatt < 12.8f)
		power.batCells = 3;
	else
		power.batCells = 4;

	msg_t event;
	uint32_t lastPowerGoodTime = chVTGetSystemTimeX();
	uint32_t lastEnergeticTime = chVTGetSystemTimeX();

	chRegSetThreadName("Power");

	while(1)
	{
		if(chMBFetch(&power.eventQueue, &event, MS2ST(100)) == MSG_OK)
		{
			if(event == EVENT_BTN_PRESSED)
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

				event = EVENT_PDN;
			}

			if(event == EVENT_PDN)
			{
				power.batEmpty = 1;
				chThdSleepMilliseconds(10);

				LogFileClose();

				KickerAutoDischarge();
				chThdSleepMilliseconds(10);

				while(kicker.autoDischarge)
					chThdSleepMilliseconds(10);

				BuzzerTone(1);

				chThdSleepMilliseconds(100);

				CLEAR_KILL();
			}
		}

		if(power.vBat > 3.5f*power.batCells)
			lastEnergeticTime = chVTGetSystemTimeX();

		if(power.vBat > 3.4f*power.batCells)
			lastPowerGoodTime = chVTGetSystemTimeX();

		if(power.batEmpty && power.vBat < 3.25f*power.batCells)
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
