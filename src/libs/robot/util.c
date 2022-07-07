/*
 * util.c
 *
 *  Created on: 29.05.2015
 *      Author: AndreR
 */

#include "ctrl.h"
#include "util.h"
#include "util/arm_mat_util_f32.h"

static float findVMinAngle();
static float findAMinAngle();
static float posMin(float* pIn, uint16_t size);
static float absMax(float* pIn, uint16_t size, uint16_t* pIndex);

void CtrlUtilTurnLocal2Global(float globalOrientation, float localX, float localY, float* pGlobalX, float* pGlobalY)
{
	float localToGlobalAngle = -PI / 2.0f + globalOrientation;
	float sinA = arm_sin_f32(localToGlobalAngle);
	float cosA = arm_cos_f32(localToGlobalAngle);

	// turn to global system
	*pGlobalX = localX * cosA - localY * sinA;
	*pGlobalY = localY * cosA + localX * sinA;
}

void CtrlUtilTurnGlobal2Local(float globalOrientation, float globalX, float globalY, float* pLocalX, float* pLocalY)
{
	float globalToLocalAngle = PI / 2.0f - globalOrientation;
	float sinA = arm_sin_f32(globalToLocalAngle);
	float cosA = arm_cos_f32(globalToLocalAngle);

	// turn to local system
	*pLocalX = globalX * cosA - globalY * sinA;
	*pLocalY = globalY * cosA + globalX * sinA;
}

void CtrlUtilRotate(float angle, float inX, float inY, float* pOut)
{
	float sinA = arm_sin_f32(angle);
	float cosA = arm_cos_f32(angle);

	// turn to local system
	pOut[0] = inX * cosA - inY * sinA;
	pOut[1] = inY * cosA + inX * sinA;
}

void CtrlUtilLocalVelToMotorVel(const float* pLocal, float* pMotor, uint8_t useASR)
{
	float local[3] = {pLocal[0], pLocal[1], pLocal[2]};
	arm_matrix_instance_f32 matLocal = {3, 1, local};
	arm_matrix_instance_f32 matMot = {4, 1, pMotor};

	arm_mat_mult_f32(&ctrl.matXYW2Motor, &matLocal, &matMot);

	if(useASR)
		CtrlUtilCorrectSlippage(matMot.pData, matMot.pData);

	// convert speed over ground to motor output
	for(uint8_t i = 0; i < 4; i++)
		pMotor[i] *= CTRL_WHEEL_TO_MOTOR_RATIO * 1/botParams.physical.wheelRadius;
}

void CtrlUtilMotorVelToLocalVel(const float* pMotor, float* pLocal)
{
	float motor[4] = {pMotor[0], pMotor[1], pMotor[2], pMotor[3]};
	arm_matrix_instance_f32 matMot = { 4, 1, motor };

	for(uint8_t i = 0; i < 4; i++)
		motor[i] *= botParams.physical.wheelRadius*CTRL_MOTOR_TO_WHEEL_RATIO;	// value is now speed over ground [m/s]

	// convert to local velocity
	arm_matrix_instance_f32 matLocal = {3, 1, pLocal};
	arm_mat_mult_f32(&ctrl.matMotor2XYW, &matMot, &matLocal);
}

void CtrlUtilMotorVoltagesFromVelocity(float* pVelXYZ, float* pVolMot)
{
	arm_matrix_instance_f32 matVel = {3, 1, pVelXYZ};
	arm_matrix_instance_f32 matVol = {4, 1, pVolMot};

	arm_mat_mult_f32(&ctrl.matXYW2Motor, &matVel, &matVol);

	const float k_w = CTRL_WHEEL_TO_MOTOR_RATIO/botParams.physical.wheelRadius;

	for(uint8_t i = 0; i < 4; i++)
		pVolMot[i] *= k_w*CTRL_MOTOR_KE;
}

void CtrlUtilVelocityFromMotorVoltages(float* pVolMot, float* pVelXYZ)
{
	arm_matrix_instance_f32 matVelMot = {4, 1, (float[4]){}};
	arm_matrix_instance_f32 matVelXYZ = {3, 1, pVelXYZ};

	const float k_w = CTRL_WHEEL_TO_MOTOR_RATIO/botParams.physical.wheelRadius;

	for(uint8_t i = 0; i < 4; i++)
		MAT_ELEMENT(matVelMot, i, 0) = pVolMot[i]*1.0f/(CTRL_MOTOR_KE*k_w);

	arm_mat_mult_f32(&ctrl.matMotor2XYW, &matVelMot, &matVelXYZ);
}

void CtrlUtilLocalAccToMotorTorque(float* pAccXYW, float* pMot)
{
	float forceXYW[3];
	forceXYW[0] = pAccXYW[0]*botParams.physical.mass;
	forceXYW[1] = pAccXYW[1]*botParams.physical.mass;
	forceXYW[2] = pAccXYW[2]*ctrl.momentOfInertia;

	CtrlUtilLocalForceToMotorTorque(forceXYW, pMot);
}

void CtrlUtilLocalForceToMotorTorque(float* pForceXYW, float* pMot)
{
	arm_matrix_instance_f32 matForceXYW = {3, 1, pForceXYW};
	arm_matrix_instance_f32 matMot = {4, 1, pMot};

	arm_mat_mult_f32(&ctrl.matForceXYW2Motor, &matForceXYW, &matMot);

	for(uint8_t i = 0; i < 4; i++)
		pMot[i] *= botParams.physical.wheelRadius*CTRL_MOTOR_TO_WHEEL_RATIO;
}

void CtrlUtilMotorTorqueToLocalForce(const float* pMot, float* pForceXYW)
{
	arm_matrix_instance_f32 matForceXYW = {3, 1, pForceXYW};
	arm_matrix_instance_f32 matMot = {4, 1, (float[4]){}};

	for(uint8_t i = 0; i < 4; i++)
		matMot.pData[i] = pMot[i] * 1.0f/(botParams.physical.wheelRadius*CTRL_MOTOR_TO_WHEEL_RATIO);

	arm_mat_mult_f32(&ctrl.matForceMotor2XYW, &matMot, &matForceXYW);
}

void CtrlUtilMotorVoltagesFromAcceleration(float* pAccXYW, float* pVolMot)
{
	arm_matrix_instance_f32 matForceXYW = {3, 1, (float[3]){}};
	arm_matrix_instance_f32 matVolMot = {4, 1, pVolMot};

	MAT_ELEMENT(matForceXYW, 0, 0) = pAccXYW[0]*botParams.physical.mass;
	MAT_ELEMENT(matForceXYW, 1, 0) = pAccXYW[1]*botParams.physical.mass;
	MAT_ELEMENT(matForceXYW, 2, 0) = pAccXYW[2]*ctrl.momentOfInertia;

	arm_mat_mult_f32(&ctrl.matForceXYW2Motor, &matForceXYW, &matVolMot);

	for(uint8_t i = 0; i < 4; i++)
		pVolMot[i] *= botParams.physical.wheelRadius/CTRL_WHEEL_TO_MOTOR_RATIO*CTRL_MOTOR_R/CTRL_MOTOR_KM;
}

void CtrlUtilAccelerationFromMotorVoltages(float* pVolMot, float* pAccXYW)
{
	arm_matrix_instance_f32 matForceMot = {4, 1, (float[4]){}};
	arm_matrix_instance_f32 matAccXYW = {3, 1, pAccXYW};

	for(uint8_t i = 0; i < 4; i++)
		MAT_ELEMENT(matForceMot, i, 0) = pVolMot[i]*CTRL_MOTOR_KM/CTRL_MOTOR_R*CTRL_WHEEL_TO_MOTOR_RATIO/botParams.physical.wheelRadius;

	arm_mat_mult_f32(&ctrl.matForceMotor2XYW, &matForceMot, &matAccXYW);

	pAccXYW[0] /= botParams.physical.mass;
	pAccXYW[1] /= botParams.physical.mass;
	pAccXYW[2] /= ctrl.momentOfInertia;
}

/**
 * Determines maximum acceleration given that a specific voltage is already in use (for holding velocity)
 *
 * @param pAccXYW [In/Out] Input acceleration which is scaled
 * @param pVolMostUsed [In/Out] Voltage which is already used, output of used voltage after scaling acc
 * @param Umax maximum allowed voltage per motor (usually battery level)
 * @param useFP Use positive factors in calculation
 * @param useFM Use negative factors in calculation
 *
 * @return 1 if scaling took place
 */
uint8_t CtrlUtilScaleAccelerationToVoltage(float* pAccXYW, float* pVolMotUsed,
		float Umax, uint8_t useFP, uint8_t useFM, uint8_t onlyIfNeeded)
{
	float volMotFromAcc[4];
	float factors[8];

	CtrlUtilMotorVoltagesFromAcceleration(pAccXYW, volMotFromAcc);

	if(onlyIfNeeded)
	{
		uint8_t exceedingUMax = 0;
		for(uint8_t i = 0; i < 4; i++)
		{
			if(fabsf(volMotFromAcc[i]+pVolMotUsed[i]) > Umax)
			{
				exceedingUMax = 1;
				break;
			}
		}

		if(exceedingUMax == 0)
			return 0;
	}

	memset(factors, 0, sizeof(float)*8);

	if(useFP)
	{
		for(uint8_t i = 0; i < 4; i++)
			factors[i] = (Umax-pVolMotUsed[i])/volMotFromAcc[i];
	}

	if(useFM)
	{
		for(uint8_t i = 0; i < 4; i++)
			factors[i+4] = (-Umax-pVolMotUsed[i])/volMotFromAcc[i];
	}

	float fMin = posMin(factors, 8);
	if(isinf(fMin))
		fMin = 0;

	for(uint8_t i = 0; i < 4; i++)
	{
		volMotFromAcc[i] *= fMin;
		pVolMotUsed[i] += volMotFromAcc[i];
	}

	CtrlUtilAccelerationFromMotorVoltages(volMotFromAcc, pAccXYW);

	return 1;
}

uint8_t CtrlUtilScaleVelocityToVoltage(float* pVelXYW, float Umax, uint8_t onlyIfNeeded, float* pVolMot)
{
	CtrlUtilMotorVoltagesFromVelocity(pVelXYW, pVolMot);

	float max = absMax(pVolMot, 4, 0);
	if(max == 0.0f || (onlyIfNeeded && max < Umax))
		return 0;

	float factor = Umax/max;

	for(uint8_t i = 0; i < 4; i++)
		pVolMot[i] *= factor;

	CtrlUtilVelocityFromMotorVoltages(pVolMot, pVelXYW);

	return 1;
}

uint8_t CtrlUtilScaleAccelerationToStiction(float* pAccXYW, float stiction, uint8_t onlyIfNeeded)
{
	float factors[4];

	if(onlyIfNeeded)
	{
		// do we need to scale at all?
		arm_matrix_instance_f32 matDownForces = {4, 1, (float[4]){}};
		arm_matrix_instance_f32 matWheelForces = {4, 1, (float[4]){}};
		arm_matrix_instance_f32 matForcesXYZ = {3, 1, (float[3]){pAccXYW[0]*botParams.physical.mass, pAccXYW[1]*botParams.physical.mass, botParams.physical.mass*9.81f}};

		arm_mat_mult_f32(&ctrl.matForceXYZ2DownForce, &matForcesXYZ, &matDownForces);

		MAT_ELEMENT(matForcesXYZ, 2, 0) = pAccXYW[2]*ctrl.momentOfInertia;

		arm_mat_mult_f32(&ctrl.matForceXYW2Motor, &matForcesXYZ, &matWheelForces);

		uint8_t exceedingDownForce = 0;
		for(uint8_t i = 0; i < 4; i++)
		{
			if(matDownForces.pData[i] < fabsf(matWheelForces.pData[i]))
			{
				exceedingDownForce = 1;
				break;
			}
		}

		if(exceedingDownForce == 0)
			return 0;	// no need to scale, forces within physical limits
	}

	for(uint8_t i = 0; i < 4; i++)
	{
		//f(i) = (C_wz(i,3)*m*9.81*stic)/(abs(C_FMp(i,:)*(a.*MI))-C_wz(i,1)*a(1)*m*stic-C_wz(i,2)*a(2)*m*stic);
		float f = fabsf(MAT_ELEMENT(ctrl.matForceXYW2Motor, i, 0)*pAccXYW[0]*botParams.physical.mass
				+MAT_ELEMENT(ctrl.matForceXYW2Motor, i, 1)*pAccXYW[1]*botParams.physical.mass
				+MAT_ELEMENT(ctrl.matForceXYW2Motor, i, 2)*pAccXYW[2]*ctrl.momentOfInertia);

		float denom = f - MAT_ELEMENT(ctrl.matForceXYZ2DownForce, i, 0)*pAccXYW[0]*botParams.physical.mass*stiction
				- MAT_ELEMENT(ctrl.matForceXYZ2DownForce, i, 1)*pAccXYW[1]*botParams.physical.mass*stiction;

		if(denom == 0.0f)
			factors[i] = 1.0f;
		else
			factors[i] = (MAT_ELEMENT(ctrl.matForceXYZ2DownForce, i, 2)*botParams.physical.mass*9.81f*stiction)/denom;
	}

	float fMin = posMin(factors, 4);

	for(uint8_t i = 0; i < 4; i++)
		pAccXYW[i] *= fMin;

	return 1;
}

void CtrlUtilGetPerfConstants(float accMax, float accMaxW, CtrlUtilPerfConstants* pConst)
{
	float vMinAngle = findVMinAngle();
	float aMinAngle = findAMinAngle();

	float accW[] = {0, 0, accMaxW};
	float vol[4];
	CtrlUtilMotorVoltagesFromAcceleration(accW, vol);
	pConst->UaccW = absMax(vol, 4, 0);

	float cosAMin = arm_cos_f32(aMinAngle);
	float sinAMin = arm_sin_f32(aMinAngle);
	float accXY[] = {accMax*cosAMin, accMax*sinAMin, 0};
	CtrlUtilMotorVoltagesFromAcceleration(accXY, vol);
	pConst->Uacc = absMax(vol, 4, 0);

	pConst->cosVMin = arm_cos_f32(vMinAngle);
	pConst->sinVMin = arm_sin_f32(vMinAngle);
}

void CtrlUtilGetVMax(const CtrlUtilPerfConstants* pPerf, float Utotal, float* pVelMax, float* pVelMaxW)
{
	float vol[4];
	float Uremain = Utotal - pPerf->Uacc - pPerf->UaccW;
	float velXY[] = {pPerf->cosVMin, pPerf->sinVMin, 0};

//	uint8_t xyPercent = ctrl.botLimits.xyVelPercent;
//	if(xyPercent > 99)
//		xyPercent = 99;
//
//	float xyFactor = xyPercent*0.01f;
//	float wFactor = 1.0f-xyFactor;

	float xyFactor = 1;
	float wFactor = 1;

	CtrlUtilScaleVelocityToVoltage(velXY, Uremain*xyFactor, 0, vol);
	*pVelMax = sqrtf(velXY[0]*velXY[0]+velXY[1]*velXY[1]);

	float velW[] = {0, 0, 1};
	CtrlUtilScaleVelocityToVoltage(velW, Uremain*wFactor, 0, vol);
	*pVelMaxW = velW[2];
}

void CtrlUtilGetVMaxRaw(const CtrlUtilPerfConstants* pPerf, float Utotal, float* pVelMax, float* pVelMaxW)
{
	float vol[4];
	float Uremain = Utotal;
	float velXY[] = {pPerf->cosVMin, pPerf->sinVMin, 0};

	CtrlUtilScaleVelocityToVoltage(velXY, Uremain, 0, vol);
	*pVelMax = sqrtf(velXY[0]*velXY[0]+velXY[1]*velXY[1]);

	float velW[] = {0, 0, 1};
	CtrlUtilScaleVelocityToVoltage(velW, Uremain, 0, vol);
	*pVelMaxW = velW[2];
}

void CtrlUtilGetDefaultBotPosition(uint8_t botNumber, float* pPos)
{
	// initial bot position depends on botID and should be out of field
	if(botNumber > CMD_BOT_HALF_MIN_1)
	{
		botNumber -= CMD_BOT_HALF_MIN_1;

		// that's a blue bot, put him on -Y
		pPos[1] = -6.0f;
	}
	else
	{
		// a yellow bot, put him on +Y
		pPos[1] = 6.0f;
	}
	pPos[0] = -3.2f+(botNumber*0.4);
	pPos[2] = 0;
}

// takes 4 motor or encoder values
float CtrlUtilGetSlippage(float* pMotorVel)
{
	float slip;
	arm_dot_prod_f32(pMotorVel, ctrl.matMotorNullDir.pData, 4, &slip);

	return slip;
}

// subtract the motor vector in the null direction from the wrong values to
// project them back on the energy-saving hyperplane
void CtrlUtilCorrectSlippage(float* pMotorIn, float* pMotorOut)
{
	float* pMotorNullDir = ctrl.matMotorNullDir.pData;

	float slip = CtrlUtilGetSlippage(pMotorIn);

	pMotorOut[0] = pMotorIn[0] - slip*pMotorNullDir[0];
	pMotorOut[1] = pMotorIn[1] - slip*pMotorNullDir[1];
	pMotorOut[2] = pMotorIn[2] - slip*pMotorNullDir[2];
	pMotorOut[3] = pMotorIn[3] - slip*pMotorNullDir[3];
}

// ### Helper Functions ###
static float findVMinAngle()
{
	float minAngle;
	float vMin = INFINITY;
	float volMot[4];

	for(minAngle = 0; minAngle < (M_PI/2); minAngle += (M_PI/20000.0f))
	{
		float sinMin = arm_sin_f32(minAngle);
		float cosMin = arm_cos_f32(minAngle);

		float vel[] = {cosMin, sinMin, 0};

		CtrlUtilScaleVelocityToVoltage(vel, 1.0f, 0, volMot);

		float vAbs = sqrtf(vel[0]*vel[0]+vel[1]*vel[1]);

		if(vAbs > vMin)
			break;

		vMin = vAbs;
	}

	return minAngle;
}

static float findAMinAngle()
{
	float minAngle;
	float aMin = INFINITY;

	float volUsed[] = {0, 0, 0, 0};

	for(minAngle = 0; minAngle < (M_PI/2); minAngle += (M_PI/20000.0f))
	{
		float sinMin = arm_sin_f32(minAngle);
		float cosMin = arm_cos_f32(minAngle);

		float acc[] = {cosMin, sinMin, 0};

		CtrlUtilScaleAccelerationToVoltage(acc, volUsed, 1.0f, 1, 1, 0);

		float aAbs = sqrtf(acc[0]*acc[0]+acc[1]*acc[1]);

		if(aAbs > aMin)
			break;

		aMin = aAbs;
	}

	return minAngle;
}

// get minimum value of all values on pIn which is greater than zero
static float posMin(float* pIn, uint16_t size)
{
	float result = INFINITY;

	for(uint16_t i = 0; i < size; i++)
	{
		if(pIn[i] > 0 && pIn[i] < result)
			result = pIn[i];
	}

	return result;
}

// get absolute maximum value
static float absMax(float* pIn, uint16_t size, uint16_t* pIndex)
{
	float result = 0;
	uint16_t index = 0;

	for(uint16_t i = 0; i < size; i++)
	{
		float v = fabsf(pIn[i]);
		if(v > result)
		{
			result = v;
			index = i;
		}
	}

	if(pIndex)
		*pIndex = index;

	return result;
}
