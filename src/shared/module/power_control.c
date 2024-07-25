#include "power_control.h"
#include <math.h>

#define EVENT_MASK_PWR_UPDATE		EVENT_MASK(0)
#define EVENT_MASK_PDN_BTN			EVENT_MASK(1)
#define EVENT_MASK_RESET			EVENT_MASK(2)

const char* powerControlStateNames[4] = { "USB powered", "Energetic", "Exhausted", "Bat Empty" };

static void powerControlTask(void* params);
static void registerShellCommands(ShellCmdHandler* pHandler);

void PowerControlButtonIrq(PowerControl* pCtrl)
{
	chSysLockFromISR();
	chEvtSignalI(pCtrl->pTask, EVENT_MASK_PDN_BTN);
	chSysUnlockFromISR();
}

void PowerControlInit(PowerControl* pCtrl, PowerControlData* pInit, tprio_t taskPrio)
{
	pCtrl->data = *pInit;

	chEvtObjectInit(&pCtrl->eventSource);

	ShellCmdHandlerInit(&pCtrl->cmdHandler, pCtrl);
	registerShellCommands(&pCtrl->cmdHandler);

	pCtrl->batCells = 6;
	pCtrl->cellMin = 3.70f;
	pCtrl->cellEnergetic = 3.80f;
	pCtrl->cellMax = 4.20f;

	GPIOInitData gpioInit;
	gpioInit.alternate = 0;
	gpioInit.pupd = GPIO_PUPD_UP;
	gpioInit.mode = GPIO_MODE_INPUT;
	GPIOInit(pInit->usbPoweredPin.pPort, pInit->usbPoweredPin.pin, &gpioInit);

	gpioInit.mode = GPIO_MODE_EXTI;
	gpioInit.extiTrigger = GPIO_EXTI_TRIG_FALLING;
	GPIOInit(pInit->powerDownRequestPin.pPort, pInit->powerDownRequestPin.pin, &gpioInit);

	GPIOPinSet(pInit->powerKillPin);

	gpioInit.mode = GPIO_MODE_OUTPUT;
	gpioInit.otype = GPIO_OTYPE_OPEN_DRAIN;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	GPIOInit(pInit->powerKillPin.pPort, pInit->powerKillPin.pin, &gpioInit);

	pCtrl->pTask = chThdCreateStatic(pCtrl->waTask, sizeof(pCtrl->waTask), taskPrio, &powerControlTask, pCtrl);
}

void PowerControlKill(PowerControl* pCtrl)
{
	GPIOPinReset(pCtrl->data.powerKillPin);

	chThdSleepMilliseconds(100);

	GPIOPinSet(pCtrl->data.powerKillPin);

	chThdSleepMilliseconds(100);

	chEvtSignal(pCtrl->pTask, EVENT_MASK_RESET);

	// This function can actually return if the robot is USB powered
}

static uint8_t isPowerButtonPressed(PowerControl* pCtrl)
{
	uint8_t isPressed = 1;

	for(uint8_t i = 0; i < 5; i++)
	{
		chThdSleepMilliseconds(10);

		uint8_t pdnState = GPIOPinRead(pCtrl->data.powerDownRequestPin);
		if(pdnState != 0)
		{
			isPressed = 0;
			break;
		}
	}

	return isPressed;
}

static void powerMeasUpdate(PowerControl* pCtrl)
{
	PwrINA226Measurement meas;
	PwrINA226Get(pCtrl->data.pPwrMon, &meas);

	pCtrl->iCur = meas.shunt_mV / 2.0f;
	pCtrl->vBat = meas.bus_V;

	float cellMinWithLoad = 3.65f - pCtrl->iCur * pCtrl->data.cellInternalResistance * 1.5f;
	float cellEnergeticWithLoad = 3.70f - pCtrl->iCur * pCtrl->data.cellInternalResistance;

	pCtrl->cellMin = fmaxf(cellMinWithLoad, 3.35f);
	pCtrl->cellEnergetic = fmaxf(cellEnergeticWithLoad, 3.5f);

	if(pCtrl->vBat < 15.0f)
		pCtrl->batCells = 4;
	else
		pCtrl->batCells = 6;
}

static void powerControlTask(void* params)
{
	PowerControl* pCtrl = (PowerControl*)params;

	chRegSetThreadName("PWR_CTRL");

	event_listener_t pwrListener;
	chEvtRegisterMask(&pCtrl->data.pPwrMon->eventSource, &pwrListener, EVENT_MASK_PWR_UPDATE);

	uint8_t usbPowered = GPIOPinRead(pCtrl->data.usbPoweredPin);
	uint8_t batEmpty = 0;
	uint8_t exhausted = 0;

	systime_t lastPowerGoodTime = chVTGetSystemTimeX();
	systime_t lastEnergeticTime = chVTGetSystemTimeX();

	pCtrl->state = POWER_CONTROL_STATE_USB_POWERED;

	while(1)
	{
		eventmask_t events = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(10));

		if(events & EVENT_MASK_PWR_UPDATE)
		{
			powerMeasUpdate(pCtrl);
		}

		if(events & EVENT_MASK_PDN_BTN)
		{
			if(isPowerButtonPressed(pCtrl))
			{
				batEmpty = 1;
				chThdSleepMilliseconds(10);

				chEvtBroadcastFlags(&pCtrl->eventSource, POWER_CONTROL_EVENT_SHUTDOWN_REQUESTED);
			}
		}

		if(events & EVENT_MASK_RESET)
		{
			batEmpty = 0;
			exhausted = 0;
		}

		// Update power states
		usbPowered = GPIOPinRead(pCtrl->data.usbPoweredPin);

		if(pCtrl->vBat > pCtrl->cellEnergetic*pCtrl->batCells || usbPowered)
			lastEnergeticTime = chVTGetSystemTimeX();

		if(pCtrl->vBat > pCtrl->cellMin*pCtrl->batCells || usbPowered)
			lastPowerGoodTime = chVTGetSystemTimeX();

		if(chVTTimeElapsedSinceX(lastPowerGoodTime) > TIME_S2I(10) && batEmpty == 0)
			batEmpty = 1;

		if(chVTTimeElapsedSinceX(lastEnergeticTime) > TIME_S2I(10) && exhausted == 0)
			exhausted = 1;

		if(usbPowered)
		{
			pCtrl->state = POWER_CONTROL_STATE_USB_POWERED;
		}
		else if(batEmpty)
		{
			pCtrl->state = POWER_CONTROL_STATE_BAT_EMPTY;
		}
		else if(exhausted)
		{
			pCtrl->state = POWER_CONTROL_STATE_EXHAUSTED;
		}
		else
		{
			pCtrl->state = POWER_CONTROL_STATE_ENERGETIC;
		}

		// Force shutdown if bat is super empty
		if(batEmpty && pCtrl->vBat < 3.3f*pCtrl->batCells)
			PowerControlKill(pCtrl);

		if(chThdShouldTerminateX())
			chThdExit(0);
	}
}

SHELL_CMD(show, "Show power data summary");

SHELL_CMD_IMPL(show)
{
	(void)argc; (void)argv;
	PowerControl* pPwr = (PowerControl*)pUser;

	printf("V_bat: %fV (%huC)\r\n", pPwr->vBat, (uint16_t) pPwr->batCells);
	printf("I_cur: %fA\r\n", pPwr->iCur);
	printf("State: %s\r\n", powerControlStateNames[pPwr->state]);
	printf("Update Rate: %.2f\r\n", pPwr->data.pPwrMon->profiler.updateRate_Hz);
}

SHELL_CMD(shutdown, "Power-off main battery supply, if not USB-powered this will turn off the robot");

SHELL_CMD_IMPL(shutdown)
{
	(void)argc; (void)argv;
	PowerControl* pPwr = (PowerControl*)pUser;

	PowerControlKill(pPwr);
}

static void registerShellCommands(ShellCmdHandler* pHandler)
{
	ShellCmdAdd(pHandler, show_command);
	ShellCmdAdd(pHandler, shutdown_command);
}
