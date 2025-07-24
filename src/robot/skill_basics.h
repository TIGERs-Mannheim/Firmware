#pragma once

#include "robot/skills.h"

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
	//  0: Kick Speed, 9 bits, [0.02m/s] or [25us], max. 10.22m/s or 12.775ms
	//  9: Kick Device, 1 bit, one of KICKER_DEVICE_
	// 10: Kick Mode, 2 bits, one of KICKER_MODE_
	// 12: Dribbler Speed, 6 bits, [0.125m/s], max. 7.875m/s
	// 18: Dribbler Force, 6 bits, [0.25N], max. 15.75N
	// => 24 bits in total
	uint8_t data[3];
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

void SkillBasicsParseKDInput(const BasicKDInput* pKD, SkillOutput* pOutput);
void SkillBasicsSetKDInputKicker(BasicKDInput* pKD, uint8_t kickMode, uint8_t kickDevice, float kickSpeed);
void SkillBasicsSetKDInputDribbler(BasicKDInput* pKD, float dribbleSpeed_mDs, float maxForce_N);
