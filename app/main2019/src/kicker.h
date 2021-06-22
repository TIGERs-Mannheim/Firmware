/*
 * kicker.h
 *
 *  Created on: 09.01.2019
 *      Author: AndreR
 */

#pragma once

#include "ch.h"
#include "util/ema_filter.h"
#include "util/config.h"
#include "commands.h"

#define KICKER_EVENT_QUEUE_SIZE 10

typedef struct PACKED _KickerInternalConfig
{
	float maxVoltage;		// [V]
	float chgHysteresis;	// [V]
	uint32_t cooldown;		// [ms]
	float kickVoltageLimit; // ignore kick cmds below kickVoltageLimit*curVoltage

	float kickOffset;
	float kickFactor;
	float kickMinTimeMs;

	float chipOffset;
	float chipFactor;
	float chipMinTimeMs;
} KickerInternalConfig;

typedef struct _Kicker
{
	uint8_t installed;
	uint8_t v2017;

	float vCap;
	float tEnv;

	uint8_t autoCharge;
	uint8_t state;

	uint8_t autoDischarge;

	uint8_t interrupted;

	float vIrOn;
	float vIrOff;

	float stress;

	uint32_t lastKickDone;

	uint8_t kickCounter;

	mailbox_t eventQueue;
	msg_t eventQueueData[KICKER_EVENT_QUEUE_SIZE];

	EMAFilter capAvg;

	EMAFilter emaAutoIrOn;
	EMAFilter emaAutoIrOff;
	float irAutoThreshold;

	EMAFilter emaIrOn;
	EMAFilter emaIrOff;

	uint16_t barrierTxDamaged;
	uint16_t barrierRxDamaged;

	KickerInternalConfig config;
	ConfigFile* pConfigFile;

	uint16_t straightDamaged;
	uint16_t chipDamaged;
	uint16_t chargeOverheated;

	uint32_t overheatStartTime;

	float vCapBeforeKick;
	uint8_t lastKickDevice;
	uint32_t lastKickDurationUs;
	float lastCapUsage;

	struct _armed
	{
		uint8_t isActive;
		uint8_t device;
		float speed;
	} armed;

	uint32_t lastCapChgUpdate;
} Kicker;

extern Kicker kicker;

void KickerInit();
void KickerTask(void* params);
void KickerADCUpdate(float vCap, float iChg);
void KickerIrUpdate(float vIrOn, float vIrOff);

void KickerFireSpeed(float speed, uint8_t device);
void KickerFire(float duration, uint8_t device);
void KickerArmSpeed(float speed, uint8_t device);
void KickerArm(float duration, uint8_t device);
void KickerDisarm();
uint8_t KickerIsReadyToShoot();
uint8_t KickerIsCharged();

void KickerEnableAutoRecharge(uint8_t enable);
void KickerEnableCharge(uint8_t enable);
void KickerAutoDischarge();

void KickerSetMaxVoltage(float vMax);
void KickerSetCooldown(uint32_t cooldown);
