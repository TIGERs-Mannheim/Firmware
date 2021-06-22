/*
 * skill_basics.h
 *
 *  Created on: 24.06.2015
 *      Author: AndreR
 */

#pragma once

#include <main/skills.h>

#define BASIC_KD_KICK_FLAGS_DEVICE_MASK	0x01
#define BASIC_KD_KICK_FLAGS_EXTRA_MASK	0x0E
#define BASIC_KD_KICK_FLAGS_MODE_MASK	0xF0

#define GLOBAL_POS_MAX_VEL_XY	5.0f
#define GLOBAL_POS_MAX_VEL_W	30.0f
#define GLOBAL_POS_MAX_ACC_XY	10.0f
#define GLOBAL_POS_MAX_ACC_W	100.0f

#define LOCAL_VEL_MAX_ACC_XY	10.0f
#define LOCAL_VEL_MAX_ACC_W		100.0f

extern SkillInstance skillEmergency;
extern SkillInstance skillLocalVel;
extern SkillInstance skillGlobalVel;
extern SkillInstance skillGlobalPos;
extern SkillInstance skillWheelVel;
extern SkillInstance skillGlobalVelAndOrient;
extern SkillInstance skillLocalForce;

typedef struct PACKED _BasicKDInput
{
	uint8_t kickSpeed;		// [0.04 m/s]
	uint8_t kickFlags;		// mode, dev
	uint8_t dribblerSpeed;	// [rpm/100]
} BasicKDInput;

typedef struct PACKED _LocalVelInput
{
	int16_t xyw[3];

	uint8_t accMaxXY;
	uint8_t accMaxW;
	uint8_t jerkMaxXY;
	uint8_t jerkMaxW;

	BasicKDInput kd;
} VelInput;

typedef struct PACKED _GlobalPosInput
{
	int16_t xyw[3];

	uint8_t velMaxXY;
	uint8_t velMaxW;
	uint8_t accMaxXY;
	uint8_t accMaxW;

	BasicKDInput kd;

	int8_t primaryDirection;
} GlobalPosInput;

typedef struct _EmergencyData
{
	float outVelLocal[3];
	float decrement[3];
} SkillEmergencyData;

extern SkillEmergencyData emergency;

void SkillBasicsParseKDInput(const SkillInput* pInput, const BasicKDInput* pKD, SkillOutput* pOutput);
