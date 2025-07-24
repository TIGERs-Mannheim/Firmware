#pragma once

#include "gfx.h"
#include "wifi.h"
#include "ext_commands.h"

typedef struct PACKED _PresenterMainStatus
{
	struct
	{
		float cap;	// [V]
		float max;	// [V]
		float chg;	// [A]
		float temp; // [�C]
	} kicker;

	struct
	{
		float bat; // [V]
		float min; // [V]
		float max; // [V]
		float cur; // [A]
		uint8_t usbPowered;
	} power;

	struct
	{
		uint16_t level[2]; // IR ADC average
		uint16_t threshold;
		uint8_t txDamaged;
		uint8_t rxDamaged;
	} barrier;

	const ExtUpdateProgress* pExtUpdateProgress;
} PresenterMainStatus;

typedef void (*MainStatusKickerDischargeCallback)();

GHandle MainStatusCreate(MainStatusKickerDischargeCallback dischageCallback, int16_t idSelectWindowId);
void MainStatusKdUpdate(PresenterMainStatus* pFb);
void MainStatusHwIdUpdate(uint8_t id);
void MainStatusWifiUpdate(PresenterWifiStat* pStat);
