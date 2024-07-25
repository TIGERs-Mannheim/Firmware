#include "robot_math.h"
#include "math.h"
#include "robot.h"
#include "math/arm_mat_util_f32.h"
#include "math/angle_math.h"

RobotMath robotMath;

static float pXYW2Motor[12];
static float pMotor2XYW[12];
static float pForceXYW2Motor[12];
static float pForceMotor2XYW[12];
static float pMotorNullDir[4];
static float pXYW2Subwheel[12];
static float pGroundForces2ForceZMomentsXY[12];
static float pForceZMomentsXY2GroundForces[12];
static float pForceNullDir[4];


void RobotMathUpdate(const RobotSpecs* pSpecs)
{
	// construct matrices
	robotMath.matXYW2Motor = (arm_matrix_instance_f32){4, 3, pXYW2Motor};
	robotMath.matMotor2XYW = (arm_matrix_instance_f32){3, 4, pMotor2XYW};
	robotMath.matForceXYW2Motor = (arm_matrix_instance_f32){4, 3, pForceXYW2Motor};
	robotMath.matForceMotor2XYW = (arm_matrix_instance_f32){3, 4, pForceMotor2XYW};
	robotMath.matMotorNullDir = (arm_matrix_instance_f32){4, 1, pMotorNullDir};
	robotMath.matXYW2Subwheel = (arm_matrix_instance_f32){4, 3, pXYW2Subwheel};
	robotMath.matGroundForces2ForceZMomentsXY = (arm_matrix_instance_f32){3, 4, pGroundForces2ForceZMomentsXY};
	robotMath.matForceZMomentsXY2GroundForces = (arm_matrix_instance_f32){4, 3, pForceZMomentsXY2GroundForces};
	robotMath.matForceNullDir = (arm_matrix_instance_f32){4, 1, pForceNullDir};

	float frontAngle_deg = pSpecs->physical.frontAngle_deg;
	float backAngle_deg = pSpecs->physical.backAngle_deg;
	float botRadius_m = pSpecs->physical.botRadius_m;
	float mass_kg = pSpecs->physical.mass_kg;

	robotMath.momentOfInertia_kg_m2 = pSpecs->physical.massDistributionZ*mass_kg*botRadius_m*botRadius_m;

	float alpha_rad = AngleDeg2Rad(frontAngle_deg);
	float beta_rad = AngleDeg2Rad(backAngle_deg);

	robotMath.theta_rad[0] = alpha_rad;
	robotMath.theta_rad[1] = PI-alpha_rad;
	robotMath.theta_rad[2] = PI+beta_rad;
	robotMath.theta_rad[3] = 2*PI-beta_rad;

	for(uint8_t i = 0; i < 4; i++)
	{
		// construct matrix for XYW velocity to motor velocity conversion
		MAT_ELEMENT(robotMath.matXYW2Motor, i, 0) = -sinf(robotMath.theta_rad[i]);
		MAT_ELEMENT(robotMath.matXYW2Motor, i, 1) = cosf(robotMath.theta_rad[i]);
		MAT_ELEMENT(robotMath.matXYW2Motor, i, 2) = botRadius_m;

		// construct matrix for XYW velocity to subwheel velocity conversion
		MAT_ELEMENT(robotMath.matXYW2Subwheel, i, 0) = cosf(robotMath.theta_rad[i]);
		MAT_ELEMENT(robotMath.matXYW2Subwheel, i, 1) = sinf(robotMath.theta_rad[i]);
		MAT_ELEMENT(robotMath.matXYW2Subwheel, i, 2) = 0;

		// construct matrix for motor forces to XYW forces/moment conversion
		MAT_ELEMENT(robotMath.matForceMotor2XYW, 0, i) = -sinf(robotMath.theta_rad[i]);
		MAT_ELEMENT(robotMath.matForceMotor2XYW, 1, i) = cosf(robotMath.theta_rad[i]);
		MAT_ELEMENT(robotMath.matForceMotor2XYW, 2, i) = botRadius_m;

		// construct matrix for ground forces to z force and xy moments
		MAT_ELEMENT(robotMath.matGroundForces2ForceZMomentsXY, 0, i) = 1.0f;
		MAT_ELEMENT(robotMath.matGroundForces2ForceZMomentsXY, 1, i) = sinf(robotMath.theta_rad[i]) * botRadius_m - pSpecs->physical.centerOfGravity_m[1];
		MAT_ELEMENT(robotMath.matGroundForces2ForceZMomentsXY, 2, i) = -cosf(robotMath.theta_rad[i]) * botRadius_m - pSpecs->physical.centerOfGravity_m[0];
	}

	arm_mat_pinv(&robotMath.matXYW2Motor, &robotMath.matMotor2XYW);
	arm_mat_pinv(&robotMath.matForceMotor2XYW, &robotMath.matForceXYW2Motor);
	arm_mat_pinv(&robotMath.matGroundForces2ForceZMomentsXY, &robotMath.matForceZMomentsXY2GroundForces);

	// Construct motor null vector
	// this simple method is only valid for robots with an equal angle between the side wheels
	pMotorNullDir[0] = MAT_ELEMENT(robotMath.matMotor2XYW, 1, 2);
	pMotorNullDir[1] = MAT_ELEMENT(robotMath.matMotor2XYW, 1, 3);
	pMotorNullDir[2] = MAT_ELEMENT(robotMath.matMotor2XYW, 1, 1);
	pMotorNullDir[3] = MAT_ELEMENT(robotMath.matMotor2XYW, 1, 0);
	float length;
	arm_power_f32(pMotorNullDir, 4, &length);
	length = sqrtf(length);
	arm_scale_f32(pMotorNullDir, 1/length, pMotorNullDir, 4);

	// Construct force null vector
	pForceNullDir[0] = MAT_ELEMENT(robotMath.matForceMotor2XYW, 1, 2);
	pForceNullDir[1] = MAT_ELEMENT(robotMath.matForceMotor2XYW, 1, 3);
	pForceNullDir[2] = MAT_ELEMENT(robotMath.matForceMotor2XYW, 1, 1);
	pForceNullDir[3] = MAT_ELEMENT(robotMath.matForceMotor2XYW, 1, 0);
	arm_power_f32(pForceNullDir, 4, &length);
	length = sqrtf(length);
	arm_scale_f32(pForceNullDir, 1/length, pForceNullDir, 4);
}

void RobotMathLocalVelToMotorVel(const float* pLocal, float* pMotor)
{
	float local[3] = {pLocal[0], pLocal[1], pLocal[2]};
	arm_matrix_instance_f32 matLocal = {3, 1, local};
	arm_matrix_instance_f32 matMot = {4, 1, pMotor};

	arm_mat_mult_f32(&robotMath.matXYW2Motor, &matLocal, &matMot);

	// convert speed over ground to motor output
	for(uint8_t i = 0; i < 4; i++)
		pMotor[i] *= CTRL_WHEEL_TO_MOTOR_RATIO * 1/robot.specs.physical.wheelRadius_m;
}

void RobotMathMotorVelToLocalVel(const float* pMotor, float* pLocal)
{
	float motor[4] = {pMotor[0], pMotor[1], pMotor[2], pMotor[3]};
	arm_matrix_instance_f32 matMot = { 4, 1, motor };

	for(uint8_t i = 0; i < 4; i++)
		motor[i] *= robot.specs.physical.wheelRadius_m*CTRL_MOTOR_TO_WHEEL_RATIO;	// value is now speed over ground [m/s]

	// convert to local velocity
	arm_matrix_instance_f32 matLocal = {3, 1, pLocal};
	arm_mat_mult_f32(&robotMath.matMotor2XYW, &matMot, &matLocal);
}


void RobotMathLocalAccToMotorTorque(float* pAccXYW, float* pMot)
{
	float forceXYW[3];
	forceXYW[0] = pAccXYW[0]*robot.specs.physical.mass_kg;
	forceXYW[1] = pAccXYW[1]*robot.specs.physical.mass_kg;
	forceXYW[2] = pAccXYW[2]*robotMath.momentOfInertia_kg_m2;

	RobotMathLocalForceToMotorTorque(forceXYW, pMot);
}

void RobotMathLocalForceToMotorTorque(float* pForceXYW, float* pMot)
{
	arm_matrix_instance_f32 matForceXYW = {3, 1, pForceXYW};
	arm_matrix_instance_f32 matMot = {4, 1, pMot};

	arm_mat_mult_f32(&robotMath.matForceXYW2Motor, &matForceXYW, &matMot);

	for(uint8_t i = 0; i < 4; i++)
		pMot[i] *= robot.specs.physical.wheelRadius_m*CTRL_MOTOR_TO_WHEEL_RATIO;
}

void RobotMathMotorTorqueToLocalForce(const float* pMot, float* pForceXYW)
{
	arm_matrix_instance_f32 matForceXYW = {3, 1, pForceXYW};
	arm_matrix_instance_f32 matMot = {4, 1, (float[4]){}};

	for(uint8_t i = 0; i < 4; i++)
		matMot.pData[i] = pMot[i] * 1.0f/(robot.specs.physical.wheelRadius_m*CTRL_MOTOR_TO_WHEEL_RATIO);

	arm_mat_mult_f32(&robotMath.matForceMotor2XYW, &matMot, &matForceXYW);
}

void RobotMathMotorVoltagesFromVelocity(float* pVelXYZ, float* pVolMot)
{
	arm_matrix_instance_f32 matVel = {3, 1, pVelXYZ};
	arm_matrix_instance_f32 matVol = {4, 1, pVolMot};

	arm_mat_mult_f32(&robotMath.matXYW2Motor, &matVel, &matVol);

	const float k_w = CTRL_WHEEL_TO_MOTOR_RATIO/robot.specs.physical.wheelRadius_m;

	for(uint8_t i = 0; i < 4; i++)
		pVolMot[i] *= k_w*CTRL_MOTOR_KE;
}

void RobotMathVelocityFromMotorVoltages(float* pVolMot, float* pVelXYZ)
{
	arm_matrix_instance_f32 matVelMot = {4, 1, (float[4]){}};
	arm_matrix_instance_f32 matVelXYZ = {3, 1, pVelXYZ};

	const float k_w = CTRL_WHEEL_TO_MOTOR_RATIO/robot.specs.physical.wheelRadius_m;

	for(uint8_t i = 0; i < 4; i++)
		MAT_ELEMENT(matVelMot, i, 0) = pVolMot[i]*1.0f/(CTRL_MOTOR_KE*k_w);

	arm_mat_mult_f32(&robotMath.matMotor2XYW, &matVelMot, &matVelXYZ);
}

void RobotMathMotorVoltagesFromAcceleration(float* pAccXYW, float* pVolMot)
{
	arm_matrix_instance_f32 matForceXYW = {3, 1, (float[3]){}};
	arm_matrix_instance_f32 matVolMot = {4, 1, pVolMot};

	MAT_ELEMENT(matForceXYW, 0, 0) = pAccXYW[0]*robot.specs.physical.mass_kg;
	MAT_ELEMENT(matForceXYW, 1, 0) = pAccXYW[1]*robot.specs.physical.mass_kg;
	MAT_ELEMENT(matForceXYW, 2, 0) = pAccXYW[2]*robotMath.momentOfInertia_kg_m2;

	arm_mat_mult_f32(&robotMath.matForceXYW2Motor, &matForceXYW, &matVolMot);

	for(uint8_t i = 0; i < 4; i++)
		pVolMot[i] *= robot.specs.physical.wheelRadius_m/CTRL_WHEEL_TO_MOTOR_RATIO*CTRL_MOTOR_R/CTRL_MOTOR_KM;
}

void RobotMathAccelerationFromMotorVoltages(float* pVolMot, float* pAccXYW)
{
	arm_matrix_instance_f32 matForceMot = {4, 1, (float[4]){}};
	arm_matrix_instance_f32 matAccXYW = {3, 1, pAccXYW};

	for(uint8_t i = 0; i < 4; i++)
		MAT_ELEMENT(matForceMot, i, 0) = pVolMot[i]*CTRL_MOTOR_KM/CTRL_MOTOR_R*CTRL_WHEEL_TO_MOTOR_RATIO/robot.specs.physical.wheelRadius_m;

	arm_mat_mult_f32(&robotMath.matForceMotor2XYW, &matForceMot, &matAccXYW);

	pAccXYW[0] /= robot.specs.physical.mass_kg;
	pAccXYW[1] /= robot.specs.physical.mass_kg;
	pAccXYW[2] /= robotMath.momentOfInertia_kg_m2;
}

void RobotMathForceZMomentsXYToGroundForces(const float* pForceZMomentsXY, float* pGroundForces)
{
	float zuv[3] = { pForceZMomentsXY[0], pForceZMomentsXY[1], pForceZMomentsXY[2] };
	arm_matrix_instance_f32 matZUV = { 3, 1, zuv };

	arm_matrix_instance_f32 matGround = {4, 1, pGroundForces};
	arm_mat_mult_f32(&robotMath.matForceZMomentsXY2GroundForces, &matZUV, &matGround);
}

void RobotMathGetDefaultBotPosition(uint8_t botNumber, float* pPos)
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
