#pragma once

#include "drv/adc_kicker.h"
#include "drv/mcu_dribbler.h"
#include "hal/timer_opm.h"
#include "math/ema_filter.h"
#include "util/config.h"
#include "util/shell_cmd.h"

#define KICKER_ERROR_KICK_NOT_INSTALLED 0x01
#define KICKER_ERROR_STRAIGHT_DMG		0x02
#define KICKER_ERROR_CHIP_DMG			0x04
#define KICKER_ERROR_CHG_OVERHEATED		0x08
#define KICKER_ERROR_CHG_DMG			0x10
#define KICKER_ERROR_IR_TX_DMG			0x20
#define KICKER_ERROR_IR_RX_DMG			0x40

#define KICKER_CHG_MODE_IDLE			0
#define KICKER_CHG_MODE_AUTO_CHARGE		1
#define KICKER_CHG_MODE_AUTO_DISCHARGE	2

extern const char* kickerErrorStrings[];

typedef struct PACKED _KickerConfig
{
	float maxVoltage_V;
	float chgHysteresis_V;
	uint32_t cooldown_ms;
	float kickVoltageLimit; // ignore kick cmds below kickVoltageLimit*curVoltage

	float straightCoeffs[3];  // constant, linear, quadratic
	float straightMinTime_ms; // kick duration shorter than this will be set to this value

	float chipCoeffs[3];  // constant, linear, quadratic
	float chipMinTime_ms; // kick duration shorter than this will be set to this value

	float dribbleStraightCoeffs[2]; // constant, linear
	float dribbleChipCoeffs[2]; // constant, linear
} KickerConfig;

typedef struct _KickerData
{
	ADCKicker* pAdcKicker;
	McuDribbler* pMcuDribbler;
	TimerOpm* pTimerStraight;
	TimerOpm* pTimerChip;
	GPIOPin chargePin;

	KickerConfig* pConfig;
	uint16_t configId;
	uint16_t configVersion;
	const char* pConfigName;
} KickerData;

typedef struct _Kicker
{
	ADCKicker* pAdcKicker;
	McuDribbler* pMcuDribbler;
	TimerOpm* pTimerStraight;
	TimerOpm* pTimerChip;
	GPIOPin chargePin;

	THD_WORKING_AREA(waTask, 512);
	thread_t* pTask;

	virtual_timer_t cooldownTimer;

	KickerConfig* pConfig;
	ConfigFileDesc cfgDesc;
	ConfigFile* pConfigFile;

	uint16_t errorFlags;

	ShellCmdHandler cmdHandler;

	struct
	{
		uint8_t state;
		mutex_t writeMutex;

		// Following values can be modified during states: idle, armed, force kick
		uint8_t device;
		float duration_s;
		uint32_t cooldown_ms;

		// Following value is valid from state: kicking
		float capVoltageBeforeKick_V;

		// Following value is valid in state: done
		float lastCapVoltageUsed_V;
		uint8_t numKicks;
	} kick;

	struct
	{
		uint32_t timestamp_us;
		systime_t tLastUpdate;

		EMAFilter capAvgFast;
		EMAFilter capAvgSlow;
		float boardTemp_degC;

		systime_t tChargeCheckStart;
		float capVoltageAtChargeCheckStart_V;

		systime_t tInhibitChargeStart;

		uint8_t chargeOn;

		uint16_t mode;
	} charger;

	struct
	{
		uint32_t timestamp_us;

		EMAFilter emaAutoIrOn;
		EMAFilter emaAutoIrOff;
		float irAutoThreshold;

		EMAFilter emaIrOn;
		EMAFilter emaIrOff;

		uint8_t isInterrupted;
	} barrier;
} Kicker;

void	KickerInit(Kicker* pKicker, KickerData* pInit, tprio_t prio);
int16_t	KickerArmDuration(Kicker* pKicker, float duration_s, uint8_t device, uint8_t forceKick);
int16_t	KickerArmSpeed(Kicker* pKicker, float speed_mDs, uint8_t device, uint8_t forceKick, float dribbleForce_N);
int16_t	KickerDisarm(Kicker* pKicker);
void	KickerSetChargeMode(Kicker* pKicker, uint16_t chargeMode);
void	KickerSetMaxVoltage(Kicker* pKicker, float vMax_V);
void	KickerSetCooldown(Kicker* pKicker, uint32_t cooldown_ms);
uint8_t	KickerIsCharged(Kicker* pKicker);
