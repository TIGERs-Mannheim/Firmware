/*
 * kicker.c
 *
 *  Created on: 17.11.2015
 *      Author: AndreR
 */

#include "kicker.h"

#include "../constants.h"
#include "util/init_hal.h"
#include "util/log.h"
#include "struct_ids.h"
#include "util/boot.h"
#include "buzzer.h"
#include <math.h>

#define STATE_IDLE		0
#define STATE_CHG		1
#define STATE_HYST		2
#define STATE_FIRE		3
#define STATE_COOLDOWN	4

#define EVENT_CHG_STOP				0
#define EVENT_CHG_START				1
#define EVENT_VMAX_REACHED			3
#define EVENT_BELOW_HYST			4
#define EVENT_FIRE_DONE				5
#define EVENT_KICK_ERR 				6
#define EVENT_FIRE_START1			7
#define EVENT_FIRE_START2			8
#define EVENT_FIRE_SPEED_START1		9
#define EVENT_FIRE_SPEED_START2		10
#define EVENT_BARRIER_INTERRUPTED	11
#define EVENT_COOLDOWN_ELAPSED		12

#define DIS1 (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_0)
#define DIS2 (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_0)
#define PWM1 (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1)
#define PWM2 (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1)

Kicker kicker = {
		.config = {
				// TODO: take current values from robots
			.maxVoltage = 220,
			.chgHysteresis = 5.0f,
			.cooldown = 80,
			.kickVoltageLimit = 0.9f,
			.kickOffset = 0.05f,
			.kickFactor = 0.68f,
			.kickMax = 10.0f,
			.chipOffset = 1.2f,
			.chipFactor = 0.7f,
			.chipMax = 6.0f,
			.irInterruptedLevel = 50,
			.irInterruptedDelay  = 1,
		} };

static const ConfigFileDesc kickFileDesc =
	{ SID_CFG_KICK, 7, "kick/cfg", 12, (ElementDesc[]) {
		{ FLOAT, "max_voltage", "V", "Max. Voltage" },
		{ FLOAT, "chg_hyst", "V", "Charge Hysteresis" },
		{ UINT32, "cooldown", "ms", "Cooldown" },
		{ FLOAT, "ir_ignore", "", "Ignore Kick Threshold" },
		{ FLOAT, "kick_offset", "", "Kick Conv. Offset" },
		{ FLOAT, "kick_factor", "", "Kick Conv. Factor" },
		{ FLOAT, "kick_max", "", "Kick Speed Max" },
		{ FLOAT, "chip_offset", "", "Chip Conv. Offset" },
		{ FLOAT, "chip_factor", "", "Chip Conv. Factor" },
		{ FLOAT, "chip_max", "", "Chip Speed Max" },
		{ UINT8, "ir_threshold", "%", "IR Interrupted Threshold" },
		{ UINT8, "ir_delay", "ms", "IR Interrupted Delay" },
	} };

void TIM8_BRK_TIM12_IRQHandler()
{
	CH_IRQ_PROLOGUE();

	TIM12->SR = 0;

	chSysLockFromISR();
	chMBPostI(&kicker.eventQueue, EVENT_FIRE_DONE);
	chSysUnlockFromISR();

	CH_IRQ_EPILOGUE();
}

static uint16_t getChgVal()
{
	float chg;

	float arr = TIM14->ARR;
	if(kicker.vCap < 30.0f)
	{
		chg = arr*0.10f;
//		chg = arr*0.05f;
	}
	else if(kicker.vCap < 100.0f)
	{
//		chg = arr*0.70f;	// 0.85 @ 50kHz
		chg = arr*0.7f;
	}
	else
	{
		chg = arr*0.8f;
	}

	return (uint16_t)chg;
}

static void enableChg(uint8_t enable)
{
	if(kicker.v2017)
	{
		if(enable && kicker.chargeOverheated == 0)
			GPIOSet(GPIOA, GPIO_PIN_7);
		else
			GPIOReset(GPIOA, GPIO_PIN_7);
	}
	else
	{
		if(enable && kicker.chargeOverheated == 0)
			TIM14->CCR1 = getChgVal();
		else
			TIM14->CCR1 = 0;
	}
}

static void engagePhaser(uint32_t eventIn)
{
	uint32_t event = (eventIn & 0xFF);

	if(chVTTimeElapsedSinceX(kicker.lastKickDone) < MS2ST(kicker.config.cooldown))
		return; 	// prevent double kick

	kicker.lastKickDone = chVTGetSystemTimeX();

	switch(event)
	{
		case EVENT_FIRE_START1:
		{
			enableChg(0);
			kicker.state = STATE_FIRE;

			uint32_t durUs = (eventIn & 0xFFFF0000) >> 16;
			kicker.lastKickDevice = KICKER_DEVICE_STRAIGHT;
			kicker.vCapBeforeKick = kicker.vCap;
			kicker.lastKickDurationUs = durUs;

			TIM12->CCMR1 = DIS2 | PWM1;	// configure outputs
			TIM12->ARR = durUs;	// set auto-reload value
			TIM12->CR1 |= TIM_CR1_CEN;
		}
		break;
		case EVENT_FIRE_START2:
		{
			enableChg(0);
			kicker.state = STATE_FIRE;

			uint32_t durUs = (eventIn & 0xFFFF0000) >> 16;
			kicker.lastKickDevice = KICKER_DEVICE_CHIP;
			kicker.vCapBeforeKick = kicker.vCap;
			kicker.lastKickDurationUs = durUs;

			TIM12->CCMR1 = DIS1 | PWM2;
			TIM12->ARR = durUs;	// set auto-reload value
			TIM12->CR1 |= TIM_CR1_CEN;
		}
		break;
		case EVENT_FIRE_SPEED_START1:
		{
			enableChg(0);
			kicker.state = STATE_FIRE;

			float speed = (eventIn & 0xFFFF0000) >> 16;
			speed *= 1e-3f;

			if(speed > kicker.config.kickMax)
				speed = kicker.config.kickMax;

			float durMs = kicker.config.kickOffset + kicker.config.kickFactor*speed;
			if(durMs < 0)
				return;

			if(durMs > 14)
				durMs = 14.0f;

			kicker.lastKickDevice = KICKER_DEVICE_STRAIGHT;
			kicker.vCapBeforeKick = kicker.vCap;
			kicker.lastKickDurationUs = durMs*1e3f;

			TIM12->CCMR1 = DIS2 | PWM1;	// configure outputs
			TIM12->ARR = (uint32_t)(durMs*1e3f);	// set auto-reload value
			TIM12->CR1 |= TIM_CR1_CEN;
		}
		break;
		case EVENT_FIRE_SPEED_START2:
		{
			enableChg(0);
			kicker.state = STATE_FIRE;

			float speed = (eventIn & 0xFFFF0000) >> 16;
			speed *= 1e-3f;

			if(speed > kicker.config.chipMax)
				speed = kicker.config.chipMax;

			float durMs = kicker.config.chipOffset + kicker.config.chipFactor*speed;
			if(durMs < 0)
				return;

			if(durMs > 14)
				durMs = 14.0f;

			kicker.lastKickDevice = KICKER_DEVICE_CHIP;
			kicker.vCapBeforeKick = kicker.vCap;
			kicker.lastKickDurationUs = durMs*1e3f;

			TIM12->CCMR1 = DIS1 | PWM2;
			TIM12->ARR = (uint32_t)(durMs*1e3f);	// set auto-reload value
			TIM12->CR1 |= TIM_CR1_CEN;
		}
		break;
		default: break;
	}

	++kicker.kickCounter;
}

static void processEvent(uint32_t eventIn)
{
	uint32_t event = (eventIn & 0xFF);

	switch(kicker.state)
	{
		case STATE_IDLE:
		{
			switch(event)
			{
				case EVENT_CHG_START:
				{
					enableChg(1);
					kicker.state = STATE_CHG;
				}
				break;
				case EVENT_FIRE_SPEED_START1:
				case EVENT_FIRE_SPEED_START2:
				case EVENT_FIRE_START1:
				case EVENT_FIRE_START2:
					engagePhaser(eventIn);
				break;
				default: break;
			}
		}
		break;
		case STATE_CHG:
		{
			switch(event)
			{
				case EVENT_CHG_STOP:
				{
					enableChg(0);
					kicker.state = STATE_IDLE;
				}
				break;
				case EVENT_FIRE_SPEED_START1:
				case EVENT_FIRE_SPEED_START2:
				case EVENT_FIRE_START1:
				case EVENT_FIRE_START2:
					engagePhaser(eventIn);
				break;
				case EVENT_VMAX_REACHED:
				{
					enableChg(0);
					kicker.state = STATE_HYST;
				}
				break;
				default: break;
			}
		}
		break;
		case STATE_HYST:
		{
			switch(event)
			{
				case EVENT_CHG_STOP:
				{
					kicker.state = STATE_IDLE;
				}
				break;
				case EVENT_BELOW_HYST:
				{
					enableChg(1);
					kicker.state = STATE_CHG;
				}
				break;
				case EVENT_FIRE_SPEED_START1:
				case EVENT_FIRE_SPEED_START2:
				case EVENT_FIRE_START1:
				case EVENT_FIRE_START2:
					engagePhaser(eventIn);
				break;
				default: break;
			}
		}
		break;
		case STATE_FIRE:
		{
			switch(event)
			{
				case EVENT_FIRE_DONE:
				{
					kicker.state = STATE_COOLDOWN;
				}
				break;
				default: break;
			}
		}
		break;
		case STATE_COOLDOWN:
		{
			if(event == EVENT_COOLDOWN_ELAPSED)
			{
				kicker.lastCapUsage = kicker.vCapBeforeKick - kicker.vCap;

				if(kicker.lastKickDurationUs >= 3500 && kicker.vCapBeforeKick > 20.0f)
				{
					uint16_t* pDmg = &kicker.straightDamaged;
					if(kicker.lastKickDevice == KICKER_DEVICE_CHIP)
						pDmg = &kicker.chipDamaged;

					if(kicker.vCapBeforeKick - kicker.vCap > 5.0f)
						*pDmg = 0;
					else
						*pDmg = 1;
				}

				if(kicker.autoCharge)
				{
					enableChg(1);
					kicker.state = STATE_CHG;
				}
				else
					kicker.state = STATE_IDLE;
			}
		}
		break;
		default: break;
	}
}

static void identifyKickerBoard()
{
	// check if kicker board is attached
	// - old kickerboards have a pull-up on CHG
	// - new v2017 kickerboards have a pull-down on CHG
	// - if the pin follows all times our pull-up/down there is no kicker attached

	GPIOInitData gpioInit;
	uint8_t hasPullUp = 0;
	uint8_t hasPullDown = 0;

	// enable weak pull-down
	gpioInit.mode = GPIO_MODE_INPUT;
	gpioInit.pupd = GPIO_PUPD_DOWN;
	GPIOInit(GPIOA, GPIO_PIN_7, &gpioInit);

	chThdSleepMilliseconds(10);
	if(GPIOA->IDR & GPIO_PIN_7)	// is the input high?
	{
		hasPullUp = 1;
	}

	// enable weak pull-up
	gpioInit.mode = GPIO_MODE_INPUT;
	gpioInit.pupd = GPIO_PUPD_UP;
	GPIOInit(GPIOA, GPIO_PIN_7, &gpioInit);

	chThdSleepMilliseconds(10);
	if((GPIOA->IDR & GPIO_PIN_7) == 0)	// is the input low?
	{
		hasPullDown = 1;
	}

	if(hasPullUp && !hasPullDown)
	{
		kicker.installed = 1;
		kicker.v2017 = 0;
	}
	else if(!hasPullUp && hasPullDown)
	{
		kicker.installed = 1;
		kicker.v2017 = 1;
	}
	else
	{
		kicker.installed = 0;
		kicker.straightDamaged = 1;
		kicker.chipDamaged = 1;

	}
}

void KickerInit()
{
	GPIOInitData gpioInit;

	// ### Init variables
	kicker.state = STATE_IDLE;
	kicker.autoCharge = 0;
	kicker.autoDischarge = 0;

	chMBObjectInit(&kicker.eventQueue, kicker.eventQueueData, KICKER_EVENT_QUEUE_SIZE);

	EMAFilterInit(&kicker.chgOffsetFilter, 0.95f);
	EMAFilterInit(&kicker.capAvg, 0.98f);

	EMAFilterInit(&kicker.emaIrOn, 0.999f);
	EMAFilterInit(&kicker.emaIrOff, 0.999f);
	kicker.emaIrOn.value = 0.1f;
	kicker.irAutoThreshold = 0.1f;

	kicker.pConfigFile = ConfigOpenOrCreate(&kickFileDesc, &kicker.config, sizeof(KickerInternalConfig), 0, 0);

	// check if and which version is attached
	identifyKickerBoard();

	// KICK-ERR
	gpioInit.mode = GPIO_MODE_INPUT;
	gpioInit.pupd = GPIO_PUPD_UP;
	GPIOInit(GPIOF, GPIO_PIN_0, &gpioInit);

	// TIM12, discharge outputs, straight/chip
	RCC->APB1ENR |= RCC_APB1ENR_TIM12EN;

	TIM12->CR1 = TIM_CR1_OPM;	// one pulse mode
	TIM12->CR2 = 0;
	TIM12->PSC = 215;	// max pulse width 65536us
	TIM12->ARR = 0xFFFF;
	TIM12->CCMR1 = DIS1 | DIS2;
	TIM12->CCR1 = 1;
	TIM12->CCR2 = 1;
	TIM12->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E;
	TIM12->EGR = TIM_EGR_UG;
	TIM12->SR = 0;
	TIM12->DIER = TIM_DIER_UIE;

	NVICEnableIRQ(TIM8_BRK_TIM12_IRQn, IRQL_KICK_CTRL);

	if(kicker.v2017)
	{
		GPIOReset(GPIOA, GPIO_PIN_7);

		gpioInit.mode = GPIO_MODE_OUTPUT;
		gpioInit.pupd = GPIO_PUPD_NONE;
		gpioInit.ospeed = GPIO_OSPEED_25MHZ;
		gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
		GPIOInit(GPIOA, GPIO_PIN_7, &gpioInit);
	}
	else
	{
		// timer setup
		// TIM14, CHG
		RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;

		TIM14->CR1 = 0;
		TIM14->CR2 = TIM_CR2_OIS1;	// output idle state high
		TIM14->PSC = 0;
	//	TIM14->ARR = 10799;	// = 20kHz
	//	TIM14->ARR = 5399;	// = 40kHz
	//	TIM14->ARR = 4319;	// = 50kHz
	//	TIM14->ARR = 2879;	// = 75kHz
	//	TIM14->ARR = 2159;	// = 100kHz
		TIM14->ARR = 1440;	// = 150kHz
	//	TIM14->ARR = 1233;	// = 175kHz
	//	TIM14->ARR = 1079;	// = 200kHz
	//	TIM14->ARR = 719;	// = 300kHz
	//	TIM14->ARR = 431;	// = 500kHz
		TIM14->CCMR1 = 0x70 | TIM_CCMR1_OC1PE;	// PWM mode 2, CCR1 preload enable
		TIM14->CCER = TIM_CCER_CC1E;	// OC1 output enable
		TIM14->DIER = 0;
		TIM14->CCR1 = 0;
		TIM14->CR1 |= TIM_CR1_CEN;	// enable timer

		// CHG output, TIM14_CH1
		gpioInit.mode = GPIO_MODE_AF;
		gpioInit.pupd = GPIO_PUPD_UP;
		gpioInit.ospeed = GPIO_OSPEED_25MHZ;
		gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
		gpioInit.alternate = 9;
		GPIOInit(GPIOA, GPIO_PIN_7, &gpioInit);
	}

	// Straight, Chip (TIM12_CH1, TIM12_CH2)
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.pupd = GPIO_PUPD_UP;
	gpioInit.ospeed = GPIO_OSPEED_25MHZ;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.alternate = 9;
	GPIOInit(GPIOB, GPIO_PIN_14 | GPIO_PIN_15, &gpioInit);
}

void KickerADCUpdate(float ch1, float ch2)
{
	// ch1 and ch2 are scaled from 0.0 - 1.0, use reference voltage to scale

	// capacitor voltage reading
	float vCap;
	if(kicker.v2017)
		vCap = 290.2565217391304f*ch1;
	else
		vCap = 263.8695652173913f*ch1;

	const float alpha = 0.8f;
	kicker.vCap = alpha*kicker.vCap + (1.0f-alpha)*vCap;

	EMAFilterUpdate(&kicker.capAvg, vCap);

	if(kicker.capAvg.value > 260.0f)
	{
		enableChg(0);
		BuzzerTone(3200);
		chThdSleepMilliseconds(500);
		BootReset();
	}

	// channel 2 has either a charge current measurement (old boards) or temperature sensor (v2017)
	if(kicker.v2017)
	{
		kicker.iChg = 0;

		kicker.tEnv = (6600.0f*ch2-800.0f)*0.02564102564102564f;
	}
	else
	{
		kicker.tEnv = 0.0f;

		float iChg = 30.0f*ch2;

		if(kicker.state != STATE_CHG)
		{
			kicker.chgOffset = EMAFilterUpdate(&kicker.chgOffsetFilter, iChg);
		}

		kicker.iChg = iChg - kicker.chgOffset;
		if(kicker.iChg < 0)
			kicker.iChg = 0;
	}

	kicker.lastCapChgUpdate = chVTGetSystemTimeX();

	switch(kicker.state)
	{
		case STATE_CHG:
		{
			if(kicker.vCap > kicker.config.maxVoltage)
			{
				chMBPost(&kicker.eventQueue, EVENT_VMAX_REACHED, TIME_IMMEDIATE);
			}
			else
			{
				enableChg(1);
			}
		}
		break;
		case STATE_HYST:
		{
			if(kicker.vCap < kicker.config.maxVoltage-kicker.config.chgHysteresis)
			{
				chMBPost(&kicker.eventQueue, EVENT_BELOW_HYST, TIME_IMMEDIATE);
			}
		}
		break;
	}
}

void KickerIrUpdate(float vIrOn, float vIrOff)
{
	kicker.vIrOn = vIrOn;
	kicker.vIrOff = vIrOff;

	float irDiff = vIrOn - vIrOff;

	if(irDiff < kicker.irAutoThreshold)
	{
		if(chVTTimeElapsedSinceX(kicker.lastBarrierClearTime) > MS2ST(kicker.config.irInterruptedDelay))
		{
			kicker.interrupted = 1;
			chMBPost(&kicker.eventQueue, EVENT_BARRIER_INTERRUPTED, TIME_IMMEDIATE);

			if(chVTTimeElapsedSinceX(kicker.lastBarrierClearTime) > S2ST(10))
			{
				kicker.barrierDamaged = 1;
			}
		}
	}
	else
	{
		kicker.interrupted = 0;
		kicker.lastBarrierClearTime = chVTGetSystemTimeX();
		kicker.barrierDamaged = 0;

		EMAFilterUpdate(&kicker.emaIrOn, vIrOn);
		EMAFilterUpdate(&kicker.emaIrOff, vIrOff);

		float emaDiff = kicker.emaIrOn.value - kicker.emaIrOff.value;
		if(emaDiff > 0.05f)
		{
			float factor = kicker.config.irInterruptedLevel*0.01f;
			kicker.irAutoThreshold = emaDiff*factor;
		}
		else
		{
			kicker.barrierDamaged = 1;
		}
	}
}

void KickerArmSpeed(float speed, uint8_t device)
{
	kicker.armed.isActive = 1;
	kicker.armed.device = device;
	kicker.armed.speed = speed;
}

void KickerArm(float duration, uint8_t device)
{
	kicker.armed.isActive = 2;
	kicker.armed.device = device;
	kicker.armed.speed = duration;
}

void KickerDisarm()
{
	kicker.armed.isActive = 0;
}

uint8_t KickerIsReadyToShoot()
{
	if(kicker.vCap > kicker.config.maxVoltage-kicker.config.chgHysteresis)
		return 1;

	return 0;
}

void KickerFireSpeed(float speed, uint8_t device)
{
	kicker.armed.isActive = 0;

	speed *= 1e3f;	// convert to mm/s
	if(speed > 65535.0f)
		speed = 65535.0f;

	if(speed < 10.0f)
		return;

	uint32_t event = speed;
	event <<= 16;

	if(device == KICKER_DEVICE_STRAIGHT)
		event |= EVENT_FIRE_SPEED_START1;
	else
		event |= EVENT_FIRE_SPEED_START2;

	chMBPost(&kicker.eventQueue, event, MS2ST(10));
}

void KickerFire(float duration, uint8_t device)
{
	kicker.armed.isActive = 0;

	duration *= 1e6f;	// convert to us
	if(duration > 65535.0f)
		duration = 65535.0f;

	if(duration < 10.0f)
		return;

	uint32_t event = duration;
	event <<= 16;

	if(device == KICKER_DEVICE_STRAIGHT)
		event |= EVENT_FIRE_START1;
	else
		event |= EVENT_FIRE_START2;

	chMBPost(&kicker.eventQueue, event, MS2ST(10));
}

void KickerEnableAutoRecharge(uint8_t enable)
{
	kicker.autoDischarge = 0;
	kicker.autoCharge = enable;
}

void KickerEnableCharge(uint8_t enable)
{
	if(enable)
		chMBPost(&kicker.eventQueue, EVENT_CHG_START, MS2ST(10));
	else
		chMBPost(&kicker.eventQueue, EVENT_CHG_STOP, MS2ST(20));
}

void KickerAutoDischarge()
{
	KickerEnableCharge(0);
	KickerEnableAutoRecharge(0);
	kicker.autoDischarge = 1;
}

void KickerSetMaxVoltage(float vMax)
{
	if(vMax > 230.0f)
		vMax = 230.0f;

	kicker.config.maxVoltage = vMax;

	ConfigNotifyUpdate(kicker.pConfigFile);
}

void KickerSetCooldown(uint32_t cooldown)
{
	kicker.config.cooldown = cooldown;

	ConfigNotifyUpdate(kicker.pConfigFile);
}

void KickerTask(void* params)
{
	(void)params;
	msg_t event;

	chRegSetThreadName("Kicker");

	uint32_t last50Run = chVTGetSystemTimeX();
	uint32_t cooldownBackup = 0;
	uint16_t chgCounter = 0;
	float noChargeLevel = 0;

	while(1)
	{
		if(chMBFetch(&kicker.eventQueue, &event, MS2ST(20)) == MSG_OK)
		{
			processEvent(event);

			if(event == EVENT_BARRIER_INTERRUPTED && kicker.armed.isActive)
			{
				if(kicker.vCap > kicker.config.kickVoltageLimit*kicker.config.maxVoltage)
				{
					// barrier interrupted, kicker armed, voltage good => we can kick

					if(kicker.armed.isActive == 2)
						KickerFire(kicker.armed.speed, kicker.armed.device);

					if(kicker.armed.isActive == 1)
						KickerFireSpeed(kicker.armed.speed, kicker.armed.device);

					kicker.armed.isActive = 0;	// remove this line if kicker should stay armed if voltage is too low
				}
			}

			if(event == EVENT_KICK_ERR)
			{
		//		ConsolePrint("Kicker error!\r\n");
			}
		}

		if(kicker.state == STATE_COOLDOWN && chVTTimeElapsedSinceX(kicker.lastKickDone) > MS2ST(50))
			chMBPost(&kicker.eventQueue, EVENT_COOLDOWN_ELAPSED, TIME_IMMEDIATE);

		if(kicker.stress != 0.0f && kicker.state == STATE_HYST)
		{
			KickerFire(kicker.stress, KICKER_DEVICE_STRAIGHT);
		}

		if(chVTTimeElapsedSinceX(kicker.lastCapChgUpdate) > MS2ST(10))
		{
			// no update from ADC task, stop charging!
			kicker.lastCapChgUpdate = chVTGetSystemTimeX();

			KickerEnableCharge(0);
			KickerEnableAutoRecharge(0);

			LogError("CapChg timeout!");
		}

		if(kicker.chargeOverheated && chVTTimeElapsedSinceX(kicker.overheatStartTime) > S2ST(30))
		{
			kicker.chargeOverheated = 0;
		}

		if(chVTTimeElapsedSinceX(last50Run) > MS2ST(50))
		{
			last50Run = chVTGetSystemTimeX();

			// Voltage watchdog during charging
			if(kicker.state == STATE_CHG && kicker.chargeOverheated == 0)
			{
				chgCounter++;

				if(chgCounter > 60) // 3 seconds
				{
					if(kicker.vCap < noChargeLevel+5.0f)
					{
						// kicker did not charge at least 5V in the last 3 seconds!
						kicker.chargeOverheated = 1;
						kicker.overheatStartTime = chVTGetSystemTimeX();
						enableChg(0);
					}

					chgCounter = 0;
					noChargeLevel = kicker.vCap;
				}
			}
			else
			{
				chgCounter = 0;
				noChargeLevel = kicker.vCap;
			}

			// temperature watchdog during charging
			if(kicker.state == STATE_CHG && kicker.tEnv > 60.0f)
			{
				kicker.chargeOverheated = 1;
				kicker.overheatStartTime = chVTGetSystemTimeX();
				enableChg(0);
			}

			// automatic discharging
			if(kicker.autoDischarge)
			{
				if(cooldownBackup == 0)
				{
					cooldownBackup = kicker.config.cooldown;
					kicker.config.cooldown = 80;
				}

				if(kicker.vCap < 2.0f)
				{
					kicker.autoDischarge = 0;
				}
				else
				{
					if(kicker.straightDamaged && kicker.chipDamaged)
						kicker.autoDischarge = 0;
					else if(kicker.straightDamaged)
						KickerFire(0.1f/(kicker.vCap+20.0f), KICKER_DEVICE_CHIP);
					else
						KickerFire(0.1f/(kicker.vCap+20.0f), KICKER_DEVICE_STRAIGHT);
				}
			}
			else
			{
				if(cooldownBackup != 0)
				{
					kicker.config.cooldown = cooldownBackup;
					cooldownBackup = 0;
				}
			}
		}
	}
}
