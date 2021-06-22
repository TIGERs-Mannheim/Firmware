/*
 * motors.h
 *
 *  Created on: 02.11.2015
 *      Author: AndreR
 */

#pragma once

#include "ch.h"
#include "util/abg_filter.h"
#include "util/ema_filter.h"
#include "util/pid.h"
#include "arm_math.h"
#include "util/config.h"

#define MOTORS_DRIBBLER_BAR_RPM_TO_MOTOR_RADS 0.05235987755983f // (1/60.0f*1/2.0f*2.0f*M_PI)
#define MOTORS_DRIBBLER_MOTOR_RADS_TO_BAR_RPM 19.09859317102744f

#define MOTORS_DRIB_FILT_NUM_TAPS 21
#define MOTORS_DRIB_FILT_DELAY	((MOTORS_DRIB_FILT_NUM_TAPS-1)/2)

#pragma pack(push, 1)
typedef struct _MotorsConfig
{
	float Kp;
	float Ki;
	float Kd;
	float Ktp;
	float Ktd;

	float K;
	float T;
} MotorsConfig;

#pragma pack(pop)

typedef struct _Motors
{
	uint8_t driveMode;

	struct _drive
	{
		// Hardware
		GPIO_TypeDef* pFaultPort;
		uint16_t ff1;
		uint16_t ff2;
		uint8_t faultStatus;

		GPIO_TypeDef* pDirPort;
		uint16_t dirPin;

		volatile uint32_t* pCCR;
		volatile uint32_t* pEncCNT;

		// Velocity measuremet with encoders (no filter)
		float velocity; // motor rad/s
		uint32_t velocityTimestamp;
		float lastEncPos;

		// Control
		PID pid;

		float setVelocity;	// motor rad/s
		float setVoltage;	// set by user, activated by DRIVE_MODE_VOLTAGE
		float setAcceleration;	// motor rad/s^2
		float setTorque; // motor Nm

		float outVoltage;	// voltage applied to motor by any control method
		uint32_t outVoltageTimestamp;

		float estCurrent; // based on set voltage and velocity measurement: i = (U-K*v)/R
	} drive[4];

	struct _dribbler
	{
		uint8_t mode;

		float lastVelMeas;
		float bias;
		float velocity;	// motor rad/s
		systime_t lastUpdateTime;
		uint8_t measurementCounter;

		arm_fir_instance_f32 velFilter;
		float velFiltState[MOTORS_DRIB_FILT_NUM_TAPS];

		PID pid;

		float setVelocity;	// motor rad/s
		float setVoltage;	// set by user, activated by DRIBBLER_MODE_VOLTAGE

		float outVoltage;	// voltage from any control method
		uint32_t outVoltageTimestamp;
	} dribbler;

	float motDamping[4]; // N*m*s
	FlashFile* pMotDampingFile;

	MotorsConfig config;
	ConfigFile* pConfigFileMotors;
} Motors;

extern Motors motors;

void MotorsInit();
void MotorsTask(void* params);

void MotorsSetDampingCoefficient(uint8_t motorId, float damp);

void MotorsDriveOff();
void MotorsDriveSetVoltage(uint8_t motor, float vol);
void MotorsDriveSetVelocity(uint8_t motor, float vel);
void MotorsDrivePrintFault();
void MotorsDrivePrintEncoders();

void MotorsDribblerOff();
void MotorsDribblerSetVoltage(float vol);
void MotorsDribblerSetVelocity(float velRPM);
