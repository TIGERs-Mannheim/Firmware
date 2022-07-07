/*

 * ctrl_tigga.h
 *
 *  Created on: 26.02.2015
 *      Author: AndreR
 */

// TIGGA is pretty clever, he can control a robot :)

#pragma once

#include <arm_math.h>
#include <stdint.h>
#include "ctrl.h"
#include "util/config.h"
#include "util/ema_filter.h"
#include "util/trajectory.h"
#include "util/traj_generator.h"
#include "util/pid.h"
#include "util/lag_element.h"
#include "util/ekf.h"
#include "util/delay_element.h"

#pragma pack(push, 1)
typedef struct _CtrlTiggaConfigFilter
{
	float posNoiseXY;
	float posNoiseW;
	float velNoiseXY;

	float visNoiseXY;
	float visNoiseW;

	float outlierMaxVelXY;
	float outlierMaxVelW;

	float trackingCoeff;

	uint8_t visCaptureDelay;
	uint8_t visProcessingDelayMax;
	uint8_t ctrlToGyrAccDelay;

	uint16_t visionTimeoutMs;
} CtrlTiggaConfigFilter;

typedef struct _CtrlTiggaConfigPID
{
	struct
	{
		float k;
		float max;
		float antiJitterThreshold;
	} posXY;

	struct
	{
		float k;
		float max;
		float antiJitterThreshold;
	} posW;

	struct
	{
		float k;
		float max;
	} velXY;

	struct
	{
		float k;
		float max;
	} velW;

	struct
	{
		uint8_t enable;
		uint16_t startVel;
		uint16_t range;
	} rotationScaling;
} CtrlTiggaConfigPID;

typedef struct _CtrlTiggaConfigModel
{
	float coulombXY;
	float viscousXY;
	float coulombW;
	float viscousW;
	float efficiencyXY;
	float efficiencyW;

	float xNum;
	float xDenom;
	float yNum;
	float yDenom;
} CtrlTiggaConfigModel;

typedef struct _CtrlTiggaConfigCompass
{
	float hardIronX;
	float hardIronY;
	float softIronCos;
	float softIronSin;
	float softEllipseEcc;
	float fieldOffset; //Rotation of field in relation to compass rotation
} CtrlTiggaConfigCompass;
#pragma pack(pop)

typedef struct _CtrlTiggaVisionEntry
{
	float pos[3];
	uint8_t state;
} CtrlTiggaVisionEntry;

#define CTRL_TIGGA_MAX_BUFFER_SIZE 200

typedef struct _CtrlTigga
{
	// state estimation stuff
	float mechState[8];

	EKF ekf;
	float ekfData[EKF_DATA_SIZE(5, 3, 3, 5)];

	float encState[8]; // state only updated by encoders

	// buffers
	struct _vision
	{
		float lastVisionOrient;
		int32_t turns;
	} vision;

	DelayElement delayGyrAcc; // sensor data. EKF => INS
	float delayGyrAccData[CTRL_TIGGA_MAX_BUFFER_SIZE][3]; // [gyr, acc_x, acc_y]

	DelayElement delayIns; // INS estimate. EKF => INS
	float delayInsData[CTRL_TIGGA_MAX_BUFFER_SIZE][5];

	DelayElement delayVision; // sensor data. EKF => INS
	CtrlTiggaVisionEntry delayVisionData[CTRL_TIGGA_MAX_BUFFER_SIZE];

	DelayElement delayCtrlOut; // buffered ctrl output. INS => now
	float delayCtrlOutData[CTRL_TIGGA_MAX_BUFFER_SIZE][3];

	// control stuff
	TrajGenerator trajGen;

	uint16_t visionOnline;
	uint16_t noVisionCounter;

	uint32_t lastVisionTime;

	uint16_t motorOffCounter;

	uint32_t lastInitTime;

	uint32_t numLateVisionMeasurements;
	uint32_t maxVisionDelay;

	EMAFilter emaBias[3];
	float bias[3];

	float virtualVelOutput[3];

	CtrlTiggaConfigFilter configFilter;
	ConfigFile* pConfigFileFilter;

	CtrlTiggaConfigPID configPid;
	ConfigFile* pConfigFilePid;

	CtrlTiggaConfigCompass configCompass;
	ConfigFile* pConfigFileCompass;

	CtrlTiggaConfigModel configModel;
	ConfigFile* pConfigFileModel;
} CtrlTigga;

extern CtrlTigga ctrlTigga;
extern CtrlInstance ctrlTiggaInstance;

void CtrlTiggaInitCfg();
