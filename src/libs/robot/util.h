/*
 * util.h
 *
 *  Created on: 29.05.2015
 *      Author: AndreR
 */

#ifndef CTRL_UTIL_H_
#define CTRL_UTIL_H_

#include <stdint.h>

typedef struct _CtrlUtilPerfConstants
{
	float Uacc; // voltage used for acceleration (XY)
	float UaccW; // voltage used for acceleration (W)
	float cosVMin; // cosine of minimial velocity angle
	float sinVMin; // sine of minimal velocity angle
} CtrlUtilPerfConstants;

void CtrlUtilTurnLocal2Global(float globalOrientation, float localX, float localY, float* pGlobalX, float* pGlobalY);
void CtrlUtilTurnGlobal2Local(float globalOrientation, float globalX, float globalY, float* pLocalX, float* pLocalY);
void CtrlUtilRotate(float angle, float inX, float inY, float* pOut);

void CtrlUtilLocalVelToMotorVel(const float* pLocal, float* pMotor, uint8_t useASR);
void CtrlUtilMotorVelToLocalVel(const float* pMotor, float* pLocal);

void CtrlUtilLocalForceToMotorTorque(float* pForceXYW, float* pMot);
void CtrlUtilLocalAccToMotorTorque(float* pAccXYW, float* pMot);
void CtrlUtilMotorTorqueToLocalForce(const float* pMot, float* pForceXYW);

void CtrlUtilAccelerationFromMotorVoltages(float* pVolMot, float* pAccXYW);
void CtrlUtilMotorVoltagesFromAcceleration(float* pAccXYW, float* pVolMot);
void CtrlUtilVelocityFromMotorVoltages(float* pVolMot, float* pVelXYZ);
void CtrlUtilMotorVoltagesFromVelocity(float* pVelXYZ, float* pVolMot);

uint8_t CtrlUtilScaleAccelerationToStiction(float* pAccXYW, float stiction, uint8_t onlyIfNeeded);
uint8_t CtrlUtilScaleVelocityToVoltage(float* pVelXYW, float Umax, uint8_t onlyIfNeeded, float* pVolMot);
uint8_t CtrlUtilScaleAccelerationToVoltage(float* pAccXYW, float* pVolMotUsed,
		float Umax, uint8_t useFP, uint8_t useFM, uint8_t onlyIfNeeded);

void CtrlUtilGetPerfConstants(float accMax, float accMaxW, CtrlUtilPerfConstants* pConst);
void CtrlUtilGetVMax(const CtrlUtilPerfConstants* pPerf, float Utotal, float* pVelMax, float* pVelMaxW);
void CtrlUtilGetVMaxRaw(const CtrlUtilPerfConstants* pPerf, float Utotal, float* pVelMax, float* pVelMaxW);

void CtrlUtilGetDefaultBotPosition(uint8_t botNumber, float* pPos);

float CtrlUtilGetSlippage(float* pMotorVel);
void CtrlUtilCorrectSlippage(float* pMotorIn, float* pMotorOut);

#endif /* CTRL_UTIL_H_ */
