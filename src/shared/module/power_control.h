#pragma once

#include "hal/init_hal.h"
#include "drv/pwr_ina226.h"
#include "util/shell_cmd.h"

#define POWER_CONTROL_EVENT_SHUTDOWN_REQUESTED EVENT_MASK(0)

#define POWER_CONTROL_STATE_USB_POWERED		0
#define POWER_CONTROL_STATE_ENERGETIC		1
#define POWER_CONTROL_STATE_EXHAUSTED		2
#define POWER_CONTROL_STATE_BAT_EMPTY		3

typedef struct _PowerControlData
{
	GPIOPin usbPoweredPin;
	GPIOPin powerDownRequestPin;
	GPIOPin powerKillPin;
	PwrINA226* pPwrMon;

	float cellInternalResistance;
} PowerControlData;

typedef struct _PowerControl
{
	PowerControlData data;

	uint32_t state;

	float vBat;
	float iCur;

	uint8_t batCells;
	float cellMin;
	float cellEnergetic;
	float cellMax;

	ShellCmdHandler cmdHandler;

	// OS data
	THD_WORKING_AREA(waTask, 256);
	thread_t* pTask;

	event_source_t eventSource;
} PowerControl;

extern const char* powerControlStateNames[4];

void PowerControlButtonIrq(PowerControl* pCtrl);
void PowerControlInit(PowerControl* pCtrl, PowerControlData* pInit, tprio_t taskPrio);
void PowerControlKill(PowerControl* pCtrl);
