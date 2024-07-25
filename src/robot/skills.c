/*
 * skills.c
 *
 *  Created on: 24.06.2015
 *      Author: AndreR
 */

#include "skill_basics.h"
#include "skill_sine.h"
#include "skills.h"
#include "skill_fast_pos.h"
#include "skill_circle_ball.h"
#include "skill_kick_ball.h"
#include "intercom_constants.h"
#include <string.h>

Skills skills;

SkillInstanceId skillInstances[] = {
	{	0, &skillEmergency },
	{	1, &skillWheelVel },
	{   2, &skillLocalVel },
	{   3, &skillGlobalVel },
	{   4, &skillGlobalPos },
	{	5, &skillGlobalVelAndOrient },
	{   6, &skillSine },
	{   7, &skillFastPos.instance },
	{   8, &skillCircleBall },
	{  10, &skillLocalForce },
	{  19, &skillKickBallInstance },
};

void SkillsInit()
{
	chMtxObjectInit(&skills.input.mutex);

	skills.numSkills = sizeof(skillInstances)/sizeof(skillInstances[0]);
	skills.pInstanceId = &skillInstances[0];

	SkillFastPosConfigInit();
}

void SkillsSetInput(uint8_t skillId, const uint8_t* pData)
{
	chMtxLock(&skills.input.mutex);

	skills.input.skillId = skillId;
	if(pData)
		memcpy(skills.input.data, pData, SYSTEM_MATCH_CTRL_USER_DATA_SIZE);

	skills.inputDataUpdated = 1;

	chMtxUnlock(&skills.input.mutex);
}

void SkillsReset()
{
	chMtxLock(&skills.input.mutex);

	skills.pInstanceId = 0;
	skills.input.skillId = 0;

	chMtxUnlock(&skills.input.mutex);
}

// this is called in robot task after state estimation and before control
void SkillsExecute(const RobotSensors* pSensors, const RobotAuxData* pAux, const RobotCtrlState* pState,
		const RobotCtrlOutput* pLastCtrlOut, const RobotCtrlReference* pLastCtrlRef, SkillOutput* pOutput)
{
	chMtxLock(&skills.input.mutex);

	SkillInput input;
	input.pSensors = pSensors;
	input.pAux = pAux;
	input.pState = pState;
	input.pLastCtrlOut = pLastCtrlOut;
	input.pLastCtrlRef = pLastCtrlRef;
	input.pData = skills.input.data;
	input.dataUpdated = skills.inputDataUpdated;
	skills.inputDataUpdated = 0;

	SkillInstanceId* pNewInstance = 0;

	// find new skill instance
	for(uint16_t i = 0; i < skills.numSkills; i++)
	{
		if(skillInstances[i].id == skills.input.skillId)
		{
			// found the new one
			pNewInstance = &skillInstances[i];
			break;
		}
	}

	// check if a skill change is required
	if(skills.pInstanceId != pNewInstance)
	{
		// call exit function if specified
		if(skills.pInstanceId && skills.pInstanceId->pInstance->pExit)
			(*skills.pInstanceId->pInstance->pExit)();

		// assign new skill
		skills.pInstanceId = pNewInstance;

		// call init function if specified
		if(skills.pInstanceId && skills.pInstanceId->pInstance->pInit)
			(*skills.pInstanceId->pInstance->pInit)(&input);
	}

	if(skills.pInstanceId)
	{
		if(skills.pInstanceId->pInstance->pSkill)
		{
			(*skills.pInstanceId->pInstance->pSkill)(&input, pOutput);
		}
	}
	else
	{
		pOutput->drive.modeXY = DRIVE_MODE_OFF;
		pOutput->drive.modeW = DRIVE_MODE_OFF;
		pOutput->dribbler.mode = DRIBBLER_MODE_OFF;
		pOutput->dribbler.velocity = 0;
		pOutput->kicker.mode = KICKER_MODE_DISARM;
	}

	chMtxUnlock(&skills.input.mutex);
}
