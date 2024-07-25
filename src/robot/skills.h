/*
 * skills.h
 *
 *  Created on: 24.06.2015
 *      Author: AndreR
 */

#pragma once

#include "commands.h"
#include "ch.h"
#include "log_msgs.h"
#include "util/config.h"
#include <math.h>

typedef struct _SkillInput
{
	const RobotSensors* pSensors;
	const RobotAuxData* pAux;
	const RobotCtrlState* pState;
	const RobotCtrlOutput* pLastCtrlOut;
	const RobotCtrlReference* pLastCtrlRef;
	void* pData;
	uint8_t dataUpdated; // 1 if pData contains new data from a MatchCtrl command
} SkillInput;

typedef void(*SkillInit)(const SkillInput* pInput);
typedef void(*SkillFunction)(const SkillInput* pInput, SkillOutput* pOutput);
typedef void(*SkillExit)();

typedef struct _SkillInstance
{
	SkillInit pInit;
	SkillFunction pSkill;
	SkillExit pExit;
} SkillInstance;

typedef struct _SkillInstanceId
{
	uint8_t id;

	SkillInstance* pInstance;
} SkillInstanceId;

typedef struct _Skills
{
	struct
	{
		mutex_t mutex;
		uint8_t skillId;
		uint8_t data[SYSTEM_MATCH_CTRL_USER_DATA_SIZE];
	} input;

	uint8_t inputDataUpdated;

	SkillInstanceId* pInstanceId;

	uint8_t numSkills;
} Skills;

extern Skills skills;

void SkillsInit();
void SkillsReset();
void SkillsSetInput(uint8_t skillId, const uint8_t* pData);
void SkillsExecute(const RobotSensors* pSensors, const RobotAuxData* pAux, const RobotCtrlState* pState,
		const RobotCtrlOutput* pLastCtrlOut, const RobotCtrlReference* pLastCtrlRef, SkillOutput* pOutput);
