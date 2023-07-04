/*
 * motors_com.h
 *
 *  Created on: 05.01.2019
 *      Author: AndreR
 */

#pragma once

#include "util/uart_fifo.h"
#include "util/st_bootloader.h"
#include "util/config.h"
#include "util/pid.h"
#include "log_msgs.h"

#define MOTORS_DRIBBLER_BAR_RPM_TO_MOTOR_RADS 0.1047197551196598f // rpm *�1/60 *�2 * %pi
#define MOTORS_DRIBBLER_MOTOR_RADS_TO_BAR_RPM 9.549296585513721f

typedef struct _MotorsVelConfig
{
	float driveKp;
	float driveKi;
	float driveAntiJitter;
	float dribblerKp;
	float dribblerKi;
} MotorsVelConfig;

typedef struct _MotorsFocConfig
{
	float driveDKp;
	float driveDKi;
	float driveQKp;
	float driveQKi;

	float dribblerDKp;
	float dribblerDKi;
	float dribblerQKp;
	float dribblerQKi;
} MotorsFocConfig;

typedef struct _MotorsSingle
{
	// connection
	UARTFifo uart;
	uint8_t portExTarget;

	// data exchange
	MotorExchangeMISO miso;
	MotorExchangeMOSI mosi;

	// input from motor
	float cpuLoad;
	float avgSupplyVoltage1ms;
	float avgTemperature1ms;
	float avgCurrentUVW1ms[3];
	float avgCurrentDQ1ms[2];
	float avgVoltageDQ1ms[2];
	float currentOffset;
	float encoderVelocity; // motor [rad/s]
	float hallVelocity; // motor [rad/s]
	uint32_t rxTime;

	// output to motor
	uint8_t mode;
	float setVoltage[2]; // [V]
	float setCurrent[2]; // [A]
	float setVelocity; // [rad/s]
	uint32_t txTime;
	uint8_t hallOnly;

	STBootloader bootloader;
	THD_WORKING_AREA(waFlashTask, 256);
	STBootloaderFlashResult flashResult;
	thread_t* pFlashTask;
	uint32_t flashParam;
	int16_t flashFuncResult;
} MotorsSingle;

typedef struct _Motors
{
	MotorsSingle motor[5];

	uint8_t recordMotor;
	uint8_t recordMode;
	uint8_t commKillMotor;

	MotorsVelConfig velConfig;
	ConfigFile* pVelConfigFile;

	MotorsFocConfig focConfig;
	ConfigFile* pFocConfigFile;
} Motors;

extern Motors motors;

void MotorsInit();
void MotorsTask(void* params);

void MotorsSetOff(uint8_t motor);
void MotorsSetVoltageAB(uint8_t motor, float voltageA, float voltageB);
void MotorsSetElectricalAngle(uint8_t motor, float angle, float voltage);
void MotorsSetVoltageDQ(uint8_t motor, float voltageD, float voltageQ);
void MotorsSetCurrentDVoltageQ(uint8_t motor, float currentD, float voltageQ);
void MotorsSetCurrentDQ(uint8_t motor, float currentD, float currentQ);
void MotorsSetVelocity(uint8_t motor, float velocity, float currentD, float currentQ);

void MotorsSetPIGainsCurrentDDrive(float kp, float ki);
void MotorsSetPIGainsCurrentDDribbler(float kp, float ki);
void MotorsSetPIGainsCurrentQDrive(float kp, float ki);
void MotorsSetPIGainsCurrentQDribbler(float kp, float ki);
void MotorsSetPIGainsVelocityDrive(float kp, float ki);
void MotorsSetPIGainsVelocityDribbler(float kp, float ki);

void MotorsDebugRecord(uint8_t id, uint8_t recordMode);
void MotorsDebugPrintRecording();
void MotorsDebugGetRecord(arm_matrix_instance_q15* pMat);
