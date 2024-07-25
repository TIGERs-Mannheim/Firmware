#include "kicker.h"
#include "errors.h"
#include "util/log.h"

#define EVENT_MASK_TIMER_PULSE_DONE	EVENT_MASK(0)
#define EVENT_MASK_ADC_KICKER		EVENT_MASK(1)
#define EVENT_MASK_MCU_DRIBBLER		EVENT_MASK(2)
#define EVENT_MASK_COOLDOWN_DONE	EVENT_MASK(3)
#define EVENT_MASK_KICK_REQUESTED	EVENT_MASK(4)

#define KICK_STATE_IDLE			0
#define KICK_STATE_ARMED		1
#define KICK_STATE_FORCE_KICK	2
#define KICK_STATE_KICKING		3
#define KICK_STATE_COOLDOWN		4

static void kickerTask(void* pParam);
static void identifyKickerBoard(Kicker* pKicker);
static void adcKickerUpdate(Kicker* pKicker);
static void mcuDribblerUpdate(Kicker* pKicker);
static int16_t kickerControl(Kicker* pKicker, float duration_s, uint8_t device, uint8_t forceKick, uint32_t cooldown_ms);
static void checkKickTriggers(Kicker* pKicker);
static void kickVoltageDropCheck(Kicker* pKicker);
static void cooldownTimerElapsed(virtual_timer_t* pTimer, void* pParam);
static void chargeControl(Kicker* pKicker);
static void dischargeControl(Kicker* pKicker);
static void registerShellCommands(ShellCmdHandler* pHandler);

const char* kickerErrorStrings[] = {
	"KICK_NOT_INSTALLED",
	"STRAIGHT_DMG",
	"CHIP_DMG",
	"CHG_OVERHEATED",
	"CHG_DMG",
	"IR_TX_DMG",
	"IR_RX_DMG"
};

static const ElementDesc kickCfgElements[] =
{
	{ FLOAT, "max_voltage", "V", "Max. Voltage" },
	{ FLOAT, "chg_hyst", "V", "Charge Hysteresis" },
	{ UINT32, "cooldown", "ms", "Cooldown" },
	{ FLOAT, "ir_ignore", "0.0-1.0", "Ignore Kick Threshold" },
	{ FLOAT, "kick_offset", "", "Kick Conv. Offset" },
	{ FLOAT, "kick_slope", "", "Kick Conv. Slope" },
	{ FLOAT, "kick_quad", "", "Kick Conv. Quadratic" },
	{ FLOAT, "kick_t_min", "ms", "Kick Time Min" },
	{ FLOAT, "chip_offset", "", "Chip Conv. Offset" },
	{ FLOAT, "chip_slope", "", "Chip Conv. Slope" },
	{ FLOAT, "chip_quad", "", "Chip Conv. Quadratic" },
	{ FLOAT, "chip_t_min", "ms", "Chip Time Min" },
	{ FLOAT, "dribble_offset", "m/s", "Dribble Force->Speed Offset" },
	{ FLOAT, "dribble_slope", "m/(s*N)", "Dribble Force->Speed Slope" },
	{ FLOAT, "dribble_chip_offset", "m/s", "Dribble Force->Speed Offset, Chip" },
	{ FLOAT, "dribble_chip_slope", "m/(s*N)", "Dribble Force->Speed Slope, Chip" },
};

void KickerInit(Kicker* pKicker, KickerData* pInit, tprio_t prio)
{
	chVTObjectInit(&pKicker->cooldownTimer);
	chMtxObjectInit(&pKicker->kick.writeMutex);

	pKicker->pAdcKicker = pInit->pAdcKicker;
	pKicker->pMcuDribbler = pInit->pMcuDribbler;
	pKicker->pTimerStraight = pInit->pTimerStraight;
	pKicker->pTimerChip = pInit->pTimerChip;
	pKicker->chargePin = pInit->chargePin;

	pKicker->pConfig = pInit->pConfig;
	pKicker->cfgDesc.cfgId = pInit->configId;
	pKicker->cfgDesc.version = pInit->configVersion;
	pKicker->cfgDesc.pName = pInit->pConfigName;
	pKicker->cfgDesc.numElements = sizeof(kickCfgElements) / sizeof(kickCfgElements[0]);
	pKicker->cfgDesc.elements = kickCfgElements;

	pKicker->pConfigFile = ConfigOpenOrCreate(&pKicker->cfgDesc, pKicker->pConfig, sizeof(KickerConfig), 0, 0);

	// Kick setup
	EMAFilterInit(&pKicker->charger.capAvgFast, 0.8f);
	EMAFilterInit(&pKicker->charger.capAvgSlow, 0.98f);

	// Barrier setup
	EMAFilterInit(&pKicker->barrier.emaAutoIrOn, 0.999f);
	EMAFilterInit(&pKicker->barrier.emaAutoIrOff, 0.999f);
	pKicker->barrier.emaAutoIrOn.value = 0.1f;
	pKicker->barrier.irAutoThreshold = 0.1f;

	EMAFilterInit(&pKicker->barrier.emaIrOn, 0.9995f);
	EMAFilterInit(&pKicker->barrier.emaIrOff, 0.9995f);
	pKicker->barrier.emaIrOn.value = 0.1f;

	ShellCmdHandlerInit(&pKicker->cmdHandler, pKicker);
	registerShellCommands(&pKicker->cmdHandler);

	pKicker->pTask = chThdCreateStatic(pKicker->waTask, sizeof(pKicker->waTask), prio, &kickerTask, pKicker);
}

int16_t KickerArmDuration(Kicker* pKicker, float duration_s, uint8_t device, uint8_t forceKick)
{
	int16_t result = kickerControl(pKicker, duration_s, device, forceKick, pKicker->pConfig->cooldown_ms);
	if(!result)
		chEvtSignal(pKicker->pTask, EVENT_MASK_KICK_REQUESTED);

	return result;
}

int16_t KickerArmSpeed(Kicker* pKicker, float speed_mDs, uint8_t device, uint8_t forceKick, float dribbleForce_N)
{
	float duration_ms;

	if(device == KICKER_DEVICE_STRAIGHT)
	{
		if(dribbleForce_N > 0.0f)
		{
			float dribbleCompensation_mDs = pKicker->pConfig->dribbleStraightCoeffs[0] + pKicker->pConfig->dribbleStraightCoeffs[1]*dribbleForce_N;
			speed_mDs += dribbleCompensation_mDs;
		}

		duration_ms = pKicker->pConfig->straightCoeffs[0] + pKicker->pConfig->straightCoeffs[1]*speed_mDs + pKicker->pConfig->straightCoeffs[2]*speed_mDs*speed_mDs;
		if(duration_ms < pKicker->pConfig->straightMinTime_ms)
			duration_ms = pKicker->pConfig->straightMinTime_ms;

		if(duration_ms > 14.0f)
			duration_ms = 14.0f;
	}
	else
	{
		if(dribbleForce_N > 0.0f)
		{
			float dribbleCompensation_mDs = pKicker->pConfig->dribbleChipCoeffs[0] + pKicker->pConfig->dribbleChipCoeffs[1]*dribbleForce_N;
			speed_mDs += dribbleCompensation_mDs;
		}

		duration_ms = pKicker->pConfig->chipCoeffs[0] + pKicker->pConfig->chipCoeffs[1]*speed_mDs + pKicker->pConfig->chipCoeffs[2]*speed_mDs*speed_mDs;
		if(duration_ms < pKicker->pConfig->chipMinTime_ms)
			duration_ms = pKicker->pConfig->chipMinTime_ms;

		if(duration_ms > 14.0f)
			duration_ms = 14.0f;
	}

	return KickerArmDuration(pKicker, duration_ms*1e-3f, device, forceKick);
}

int16_t KickerDisarm(Kicker* pKicker)
{
	int16_t result = 0;

	chMtxLock(&pKicker->kick.writeMutex);

	if(pKicker->kick.state == KICK_STATE_IDLE || pKicker->kick.state == KICK_STATE_ARMED || pKicker->kick.state == KICK_STATE_FORCE_KICK)
		pKicker->kick.state = KICK_STATE_IDLE;
	else
		result = ERROR_DEVICE_BUSY;

	chMtxUnlock(&pKicker->kick.writeMutex);

	return result;
}

void KickerSetChargeMode(Kicker* pKicker, uint16_t chargeMode)
{
	if(chargeMode > KICKER_CHG_MODE_AUTO_DISCHARGE)
	{
		pKicker->charger.mode = KICKER_CHG_MODE_IDLE;
		return;
	}

	pKicker->charger.mode = chargeMode;
}

void KickerSetMaxVoltage(Kicker* pKicker, float vMax_V)
{
	if(vMax_V > 240.0f)
		vMax_V = 240.0f;

	pKicker->pConfig->maxVoltage_V = vMax_V;

	ConfigNotifyUpdate(pKicker->pConfigFile);
}

void KickerSetCooldown(Kicker* pKicker, uint32_t cooldown_ms)
{
	pKicker->pConfig->cooldown_ms = cooldown_ms;

	ConfigNotifyUpdate(pKicker->pConfigFile);
}

uint8_t KickerIsCharged(Kicker* pKicker)
{
	return pKicker->charger.capAvgFast.value >= pKicker->pConfig->maxVoltage_V - pKicker->pConfig->chgHysteresis_V;
}

static void kickerTask(void* pParam)
{
	Kicker* pKicker = (Kicker*)pParam;

	chRegSetThreadName("KICKER");

	event_listener_t eventListenerStraight;
	event_listener_t eventListenerChip;
	event_listener_t eventListenerAdcKicker;
	event_listener_t eventListenerMcuDribbler;

	chEvtRegisterMask(&pKicker->pTimerStraight->eventSource, &eventListenerStraight, EVENT_MASK_TIMER_PULSE_DONE);
	chEvtRegisterMask(&pKicker->pTimerChip->eventSource, &eventListenerChip, EVENT_MASK_TIMER_PULSE_DONE);
	chEvtRegisterMask(&pKicker->pAdcKicker->eventSource, &eventListenerAdcKicker, EVENT_MASK_ADC_KICKER);
	chEvtRegisterMask(&pKicker->pMcuDribbler->eventSource, &eventListenerMcuDribbler, EVENT_MASK_MCU_DRIBBLER);

	identifyKickerBoard(pKicker);

	if(pKicker->errorFlags & KICKER_ERROR_KICK_NOT_INSTALLED)
	{
		pKicker->errorFlags |= KICKER_ERROR_STRAIGHT_DMG | KICKER_ERROR_CHIP_DMG | KICKER_ERROR_CHG_DMG;
	}
	else
	{
		// Configure charge pin as output when board is installed
		GPIOReset(pKicker->chargePin.pPort, pKicker->chargePin.pin);

		GPIOInitData gpioInit;
		gpioInit.mode = GPIO_MODE_OUTPUT;
		gpioInit.pupd = GPIO_PUPD_NONE;
		gpioInit.ospeed = GPIO_OSPEED_25MHZ;
		gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
		GPIOInit(pKicker->chargePin.pPort, pKicker->chargePin.pin, &gpioInit);
	}

	while(1)
	{
		eventmask_t events = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(1));

		if(events & EVENT_MASK_TIMER_PULSE_DONE)
		{
			pKicker->kick.state = KICK_STATE_COOLDOWN;
			++pKicker->kick.numKicks;
			chVTSet(&pKicker->cooldownTimer, TIME_MS2I(pKicker->kick.cooldown_ms), &cooldownTimerElapsed, pKicker);
		}

		if(events & EVENT_MASK_COOLDOWN_DONE)
		{
			pKicker->kick.lastCapVoltageUsed_V = pKicker->kick.capVoltageBeforeKick_V - pKicker->charger.capAvgFast.value;
			kickVoltageDropCheck(pKicker);
			pKicker->kick.state = KICK_STATE_IDLE;
		}

		if(events & EVENT_MASK_ADC_KICKER)
		{
			adcKickerUpdate(pKicker);
		}

		if(events & EVENT_MASK_MCU_DRIBBLER)
		{
			mcuDribblerUpdate(pKicker);
		}

		checkKickTriggers(pKicker);

		dischargeControl(pKicker);

		chargeControl(pKicker);
	}
}

static int16_t kickerControl(Kicker* pKicker, float duration_s, uint8_t device, uint8_t forceKick, uint32_t cooldown_ms)
{
	if(device > 1)
		return ERROR_INVALID_PARAMETER;

	if(duration_s > 50e-3f)
		duration_s = 50e-3f;

	int16_t result = 0;

	chMtxLock(&pKicker->kick.writeMutex);

	if(pKicker->kick.state == KICK_STATE_IDLE || pKicker->kick.state == KICK_STATE_ARMED || pKicker->kick.state == KICK_STATE_FORCE_KICK)
	{
		pKicker->kick.device = device;
		pKicker->kick.duration_s = duration_s;
		pKicker->kick.cooldown_ms = cooldown_ms;

		if(forceKick)
			pKicker->kick.state = KICK_STATE_FORCE_KICK;
		else
			pKicker->kick.state = KICK_STATE_ARMED;
	}
	else
	{
		result = ERROR_DEVICE_BUSY;
	}

	chMtxUnlock(&pKicker->kick.writeMutex);

	return result;
}

static void dischargeControl(Kicker* pKicker)
{
	if(pKicker->charger.mode != KICKER_CHG_MODE_AUTO_DISCHARGE)
		return;

	if(pKicker->charger.capAvgFast.value < 2.0f)
	{
		pKicker->charger.mode = KICKER_CHG_MODE_IDLE;
		return;
	}

	float kickDuration_s = 0.1f/(pKicker->charger.capAvgFast.value+20.0f);

	uint16_t bothDamagedMask = KICKER_ERROR_STRAIGHT_DMG | KICKER_ERROR_CHIP_DMG;

	if((pKicker->errorFlags & bothDamagedMask) == bothDamagedMask)
		pKicker->charger.mode = KICKER_CHG_MODE_IDLE;
	else if(pKicker->errorFlags & KICKER_ERROR_STRAIGHT_DMG)
		kickerControl(pKicker, kickDuration_s, KICKER_DEVICE_CHIP, 1, 80);
	else
		kickerControl(pKicker, kickDuration_s, KICKER_DEVICE_STRAIGHT, 1, 80);
}

static void chargeControl(Kicker* pKicker)
{
	uint8_t enableCharge = 1;
	uint8_t isCharging = GPIOPinIsSet(pKicker->chargePin);

	// Do not charge during a kick
	uint8_t isKicking = pKicker->kick.state == KICK_STATE_KICKING || pKicker->kick.state == KICK_STATE_COOLDOWN;
	if(isKicking)
		enableCharge = 0;

	// Check if ADC measurements for capacitor voltage timed out
	sysinterval_t adcTimeout = TIME_MS2I(10);
	uint8_t isAdcTimedOut = chVTTimeElapsedSinceX(pKicker->charger.tLastUpdate) > adcTimeout;
	if(isAdcTimedOut)
	{
		pKicker->charger.tLastUpdate = chVTGetSystemTimeX() - adcTimeout;

		// no update from ADC task, stop charging!
		enableCharge = 0;
		LogError("CapChg timeout!");
	}

	// Charger overheat protection
	if(pKicker->charger.boardTemp_degC > 60.0f)
		pKicker->errorFlags |= KICKER_ERROR_CHG_OVERHEATED;

	if(pKicker->charger.boardTemp_degC < 55.0f)
		pKicker->errorFlags &= ~KICKER_ERROR_CHG_OVERHEATED;

	if(pKicker->errorFlags & KICKER_ERROR_CHG_OVERHEATED)
		enableCharge = 0;

	// Critical over-voltage check
	if(pKicker->charger.capAvgSlow.value > 260.0f)
	{
		enableCharge = 0;
		LogError("Critical capacitor overcharge detected!");
	}

	// Capacitor voltage watchdog during charging
	if(!isCharging)
	{
		pKicker->charger.tChargeCheckStart = chVTGetSystemTimeX();
		pKicker->charger.capVoltageAtChargeCheckStart_V = pKicker->charger.capAvgFast.value;
	}

	if(pKicker->charger.capAvgFast.value < pKicker->charger.capVoltageAtChargeCheckStart_V + 5.0f)
	{
		if(chVTTimeElapsedSinceX(pKicker->charger.tChargeCheckStart) > TIME_S2I(3))
		{
			pKicker->errorFlags |= KICKER_ERROR_CHG_DMG;
			pKicker->charger.tInhibitChargeStart = chVTGetSystemTimeX();
		}
	}
	else
	{
		pKicker->errorFlags &= ~KICKER_ERROR_CHG_DMG;
	}

	// Charge failures will inhibit charging for a fixed time, afterwards a retry is allowed
	if(pKicker->charger.tInhibitChargeStart)
	{
		enableCharge = 0;

		if(chVTTimeElapsedSinceX(pKicker->charger.tInhibitChargeStart) > TIME_S2I(30))
			pKicker->charger.tInhibitChargeStart = 0;
	}

	// Auto charge logic with hysteresis
	uint8_t isMaxVoltageReached = pKicker->charger.capAvgFast.value > pKicker->pConfig->maxVoltage_V;

	if(isMaxVoltageReached)
		enableCharge = 0;

	if(pKicker->charger.mode == KICKER_CHG_MODE_AUTO_CHARGE)
	{
		if(pKicker->charger.chargeOn && isMaxVoltageReached)
			pKicker->charger.chargeOn = 0;

		if(!pKicker->charger.chargeOn && pKicker->charger.capAvgFast.value < (pKicker->pConfig->maxVoltage_V - pKicker->pConfig->chgHysteresis_V))
			pKicker->charger.chargeOn = 1;

		if(!pKicker->charger.chargeOn)
			enableCharge = 0;
	}
	else
	{
		enableCharge = 0;
	}

	if(enableCharge)
		GPIOPinSet(pKicker->chargePin);
	else
		GPIOPinReset(pKicker->chargePin);
}

static void cooldownTimerElapsed(virtual_timer_t* pTimer, void* pParam)
{
	(void)pTimer;
	Kicker* pKicker = (Kicker*)pParam;

	chSysLockFromISR();
	chEvtSignalI(pKicker->pTask, EVENT_MASK_COOLDOWN_DONE);
	chSysUnlockFromISR();
}

static void kickVoltageDropCheck(Kicker* pKicker)
{
	if(pKicker->kick.duration_s >= 3.5e-3f && pKicker->kick.capVoltageBeforeKick_V > 20.0f)
	{
		uint16_t errorMask = KICKER_ERROR_STRAIGHT_DMG;

		if(pKicker->kick.device == KICKER_DEVICE_CHIP)
			errorMask = KICKER_ERROR_CHIP_DMG;

		if(pKicker->kick.lastCapVoltageUsed_V > 5.0f)
			pKicker->errorFlags &= ~errorMask;
		else
			pKicker->errorFlags |= errorMask;
	}
}

static void startKick(Kicker* pKicker)
{
	GPIOPinReset(pKicker->chargePin);

	pKicker->kick.capVoltageBeforeKick_V = pKicker->charger.capAvgFast.value;

	if(pKicker->kick.device == KICKER_DEVICE_STRAIGHT)
		TimerOpmPulse(pKicker->pTimerStraight, pKicker->kick.duration_s*1e6f);
	else
		TimerOpmPulse(pKicker->pTimerChip, pKicker->kick.duration_s*1e6f);

	pKicker->kick.state = KICK_STATE_KICKING;
}

static void checkKickTriggers(Kicker* pKicker)
{
	chMtxLock(&pKicker->kick.writeMutex);

	if(pKicker->kick.state == KICK_STATE_FORCE_KICK)
	{
		startKick(pKicker);
	}
	else if(pKicker->kick.state == KICK_STATE_ARMED && pKicker->barrier.isInterrupted)
	{
		if(pKicker->charger.capAvgFast.value < pKicker->pConfig->kickVoltageLimit*pKicker->pConfig->maxVoltage_V)
			pKicker->kick.state = KICK_STATE_IDLE;
		else
			startKick(pKicker);
	}

	chMtxUnlock(&pKicker->kick.writeMutex);
}

static void identifyKickerBoard(Kicker* pKicker)
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
	GPIOInit(pKicker->chargePin.pPort, pKicker->chargePin.pin, &gpioInit);

	chThdSleepMilliseconds(10);
	if(pKicker->chargePin.pPort->IDR & pKicker->chargePin.pin)	// is the input high?
		hasPullUp = 1;

	// enable weak pull-up
	gpioInit.mode = GPIO_MODE_INPUT;
	gpioInit.pupd = GPIO_PUPD_UP;
	GPIOInit(pKicker->chargePin.pPort, pKicker->chargePin.pin, &gpioInit);

	chThdSleepMilliseconds(10);
	if((pKicker->chargePin.pPort->IDR & pKicker->chargePin.pin) == 0)	// is the input low?
		hasPullDown = 1;

	if(hasPullUp || !hasPullDown)
		pKicker->errorFlags |= KICKER_ERROR_KICK_NOT_INSTALLED;
}

static void adcKickerUpdate(Kicker* pKicker)
{
	ADCKickerMeasurement meas;
	ADCKickerGet(pKicker->pAdcKicker, &meas);

	if(pKicker->charger.timestamp_us != meas.timestamp_us)
	{
		pKicker->charger.timestamp_us = meas.timestamp_us;
		pKicker->charger.tLastUpdate = chVTGetSystemTimeX();

		EMAFilterUpdate(&pKicker->charger.capAvgFast, meas.capLevel_V);
		EMAFilterUpdate(&pKicker->charger.capAvgSlow, meas.capLevel_V);
		pKicker->charger.boardTemp_degC = meas.boardTemp_degC;
	}
}

static void mcuDribblerUpdate(Kicker* pKicker)
{
	McuDribblerMeasurement meas;
	McuDribblerGet(pKicker->pMcuDribbler, &meas);

	if(pKicker->barrier.timestamp_us != meas.timestamp_us)
	{
		pKicker->barrier.timestamp_us = meas.timestamp_us;

		float irDiff = meas.barrierOn_V - meas.barrierOff_V;

		if(irDiff < pKicker->barrier.irAutoThreshold)
		{
			pKicker->barrier.isInterrupted = 1;
		}
		else
		{
			pKicker->barrier.isInterrupted = 0;

			EMAFilterUpdate(&pKicker->barrier.emaAutoIrOn, meas.barrierOn_V);
			EMAFilterUpdate(&pKicker->barrier.emaAutoIrOff, meas.barrierOff_V);

			float emaDiff = pKicker->barrier.emaAutoIrOn.value - pKicker->barrier.emaAutoIrOff.value;
			if(emaDiff > 0.05f)
			{
				pKicker->barrier.irAutoThreshold = emaDiff*0.5f;
			}
		}

		// fault detection
		EMAFilterUpdate(&pKicker->barrier.emaIrOn, meas.barrierOn_V);
		EMAFilterUpdate(&pKicker->barrier.emaIrOff, meas.barrierOff_V);

		float emaOn = pKicker->barrier.emaIrOn.value;
		float emaOff = pKicker->barrier.emaIrOff.value;
		float onOffDiff = fabsf(emaOn-emaOff);

		if(emaOn < 0.1f && emaOff < 0.1f && onOffDiff <= 0.001f)
			pKicker->errorFlags |= KICKER_ERROR_IR_TX_DMG;

		if(emaOn > 0.3f && emaOff > 0.3f && onOffDiff <= 0.01f)
			pKicker->errorFlags |= KICKER_ERROR_IR_RX_DMG;

		if(fabsf(emaOn-emaOff) > 0.1f)
			pKicker->errorFlags &= ~(KICKER_ERROR_IR_TX_DMG | KICKER_ERROR_IR_RX_DMG);
	}
}

SHELL_CMD(status, "Show kicker status and configuration");

SHELL_CMD_IMPL(status)
{
	(void)argc; (void)argv;
	Kicker* pKicker = (Kicker*)pUser;

	const McuDribblerMeasurement* pIr = &pKicker->pMcuDribbler->meas;

	printf("Errors: ");
	if(pKicker->errorFlags)
	{
		for(uint16_t i = 0; i < 7; i++)
		{
			if(pKicker->errorFlags & (1 << i))
			{
				printf(kickerErrorStrings[i]);
				printf("  ");
			}
		}
	}
	else
	{
		printf("None");
	}
	printf("\r\n");

	printf("Cap:  %.3fV (%.1fV)\r\n", pKicker->charger.capAvgFast.value, pKicker->pConfig->maxVoltage_V);
	printf("Temp: %.2fC\r\n", pKicker->charger.boardTemp_degC);
	printf("IR: %.3f / %.3f (%.3f / %.3f)\r\n", pIr->barrierOff_V, pIr->barrierOn_V, pKicker->barrier.emaIrOff.value, pKicker->barrier.emaIrOn.value);
	printf("Kick State: %hu, Charge Mode: %hu, Cnt: %hu\r\n", (uint16_t)pKicker->kick.state, pKicker->charger.mode, (uint16_t)pKicker->kick.numKicks);

	printf("Last kick usage: %.3fV\r\n", pKicker->kick.lastCapVoltageUsed_V);
	printf("tSample: %u\r\n", pKicker->pAdcKicker->profiler.ioDuration_us);
}

SHELL_CMD(vmax, "Set kicker voltage limit",
	SHELL_ARG(vmax, "Max. voltage [V] (20-240V)")
);

SHELL_CMD_IMPL(vmax)
{
	(void)argc;
	Kicker* pKicker = (Kicker*)pUser;

	float vol = atof(argv[1]);

	if(vol < 20.0f || vol > 240.0f)
	{
		fprintf(stderr, "Invalid voltage limit: %.2f\r\n", vol);
		return;
	}

	KickerSetMaxVoltage(pKicker, vol);

	printf("vMax: %.1fV\r\n", pKicker->pConfig->maxVoltage_V);
}

SHELL_CMD(cooldown, "Set kicker cooldown time (max. kick frequency limit)",
	SHELL_ARG(time, "Cooldown time [ms]")
);

SHELL_CMD_IMPL(cooldown)
{
	(void)argc;
	Kicker* pKicker = (Kicker*)pUser;

	int time = atoi(argv[1]);

	if(time < 1)
	{
		fprintf(stderr, "Invalid argument\r\n");
		return;
	}

	KickerSetCooldown(pKicker, time);

	printf("Kicker cooldown set to %ums\r\n", pKicker->pConfig->cooldown_ms);
}

SHELL_CMD(chg, "Enable automatic capacitor charging",
	SHELL_ARG(enable, "0=don't charge, 1=autocharge")
);

SHELL_CMD_IMPL(chg)
{
	(void)argc;
	Kicker* pKicker = (Kicker*)pUser;

	int enable = atoi(argv[1]);

	if(enable)
	{
		printf("Enable charging\r\n");
		KickerSetChargeMode(pKicker, KICKER_CHG_MODE_AUTO_CHARGE);
	}
	else
	{
		printf("Disable charging\r\n");
		KickerSetChargeMode(pKicker, KICKER_CHG_MODE_IDLE);
	}
}

SHELL_CMD(autodischg, "Auto-Discharge capacitors short kicks");

SHELL_CMD_IMPL(autodischg)
{
	(void)argc; (void)argv;
	Kicker* pKicker = (Kicker*)pUser;

	printf("Enabling auto discharging\r\n");
	KickerSetChargeMode(pKicker, KICKER_CHG_MODE_AUTO_DISCHARGE);
}

SHELL_CMD(dis, "Discharge capacitors for a specific time on a given device (kick)",
	SHELL_ARG(device, "0=straight, 1=chip"),
	SHELL_ARG(duration, "Duration [ms]")
);

SHELL_CMD_IMPL(dis)
{
	(void)argc;
	Kicker* pKicker = (Kicker*)pUser;

	int device = atoi(argv[1]);
	float duration = atof(argv[2]);

	if(device == KICKER_DEVICE_STRAIGHT)
	{
		printf("Straight kick: %.2fms\r\n", duration);
		KickerArmDuration(pKicker, duration * 0.001f, KICKER_DEVICE_STRAIGHT, 1);
	}
	else if(device == KICKER_DEVICE_CHIP)
	{
		printf("Chip kick: %.2fms\r\n", duration);
		KickerArmDuration(pKicker, duration * 0.001f, KICKER_DEVICE_CHIP, 1);
	}
	else
	{
		fprintf(stderr, "Unknown device\r\n");
	}
}

SHELL_CMD(diss, "Discharge capacitors resulting in a specific kick speed on a given device (kick)",
	SHELL_ARG(device, "0=straight, 1=chip"),
	SHELL_ARG(speed, "Speed [m/s]")
);

SHELL_CMD_IMPL(diss)
{
	(void)argc;
	Kicker* pKicker = (Kicker*)pUser;

	int device = atoi(argv[1]);
	float speed = atof(argv[2]);

	if(device == KICKER_DEVICE_STRAIGHT)
	{
		printf("Straight kick: %.2fm/s\r\n", speed);
		KickerArmSpeed(pKicker, speed, KICKER_DEVICE_STRAIGHT, 1, 0);
	}
	else if(device == KICKER_DEVICE_CHIP)
	{
		printf("Chip kick: %.2fm/s\r\n", speed);
		KickerArmSpeed(pKicker, speed, KICKER_DEVICE_CHIP, 1, 0);
	}
	else
	{
		fprintf(stderr, "Unknown device\r\n");
	}
}

SHELL_CMD(arm, "Arm discharging capacitors for a specific time on a given device (kick)",
	SHELL_ARG(device, "0=straight, 1=chip"),
	SHELL_ARG(duration, "Duration [ms]")
);

SHELL_CMD_IMPL(arm)
{
	(void)argc;
	Kicker* pKicker = (Kicker*)pUser;

	int device = atoi(argv[1]);
	float duration = atof(argv[2]);

	if(device == KICKER_DEVICE_STRAIGHT)
	{
		printf("Arm straight kick: %.2fms\r\n", duration);
		KickerArmDuration(pKicker, duration * 0.001f, KICKER_DEVICE_STRAIGHT, 0);
	}
	else if(device == KICKER_DEVICE_CHIP)
	{
		printf("Arm chip kick: %.2fms\r\n", duration);
		KickerArmDuration(pKicker, duration * 0.001f, KICKER_DEVICE_CHIP, 0);
	}
	else
	{
		fprintf(stderr, "Unknown device\r\n");
	}
}

static void registerShellCommands(ShellCmdHandler* pHandler)
{
	ShellCmdAdd(pHandler, status_command);
	ShellCmdAdd(pHandler, vmax_command);
	ShellCmdAdd(pHandler, cooldown_command);
	ShellCmdAdd(pHandler, chg_command);
	ShellCmdAdd(pHandler, autodischg_command);
	ShellCmdAdd(pHandler, dis_command);
	ShellCmdAdd(pHandler, diss_command);
	ShellCmdAdd(pHandler, arm_command);
}
