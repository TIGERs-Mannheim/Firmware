/*
 * skill_sine.c
 *
 *  Created on: 25.06.2015
 *      Author: AndreR
 */

#include "skill_sine.h"
#include "skill_basics.h"
#include "arm_math.h"
#include "hal/sys_time.h"
#include "robot/robot.h"

static void sineInit(const SkillInput* pInput);
static void sineRun(const SkillInput* pInput, SkillOutput* pOutput);

SkillInstance skillSine = { &sineInit, &sineRun, 0 };

static struct _SkillSine
{
	float tStart;
} sine;

static void sineInit(const SkillInput* pInput)
{
	(void)pInput;

	sine.tStart = SysTime();
}

static void sineRun(const SkillInput* pInput, SkillOutput* pOutput)
{
	SkillSineInput* pSine = (SkillSineInput*)pInput->pData;

	float freq = pSine->freq*0.001f;

	float tNow = SysTime()-sine.tStart;

	float val = arm_sin_f32(2.0f*M_PI*tNow*freq);
	float valC = arm_cos_f32(2.0f*M_PI*tNow*freq);

	if(tNow < 0.25f*1.0f/freq)
		valC = 0;

	pOutput->drive.localVel[0] = pSine->vel[0]*0.001f*valC;
	pOutput->drive.localVel[1] = pSine->vel[1]*0.001f*val;
	pOutput->drive.localVel[2] = pSine->vel[2]*0.001f*val;

	pOutput->drive.limits.velMaxXY = 1.0f;
	pOutput->drive.limits.velMaxW = 1.0f;
	pOutput->drive.limits.accMaxXY = LOCAL_VEL_MAX_ACC_XY;
	pOutput->drive.limits.accMaxW = LOCAL_VEL_MAX_ACC_W;
	pOutput->drive.limits.strictVelLimit = 0;

	RobotMathLocalVelToMotorVel(pOutput->drive.localVel, pOutput->drive.wheelVel);
	for(uint8_t i = 0; i < 4; i++)
		pOutput->drive.wheelVel[i] *= CTRL_MOTOR_TO_WHEEL_RATIO;

	pOutput->drive.modeXY = DRIVE_MODE_LOCAL_VEL;
	pOutput->drive.modeW = DRIVE_MODE_LOCAL_VEL;
}
