/*
 * log_msgs.h
 *
 *  Created on: 21.10.2014
 *      Author: AndreR
 */

#pragma once

#include <stdint.h>

#ifndef PACKED
#define PACKED __attribute__((packed))
#endif

// fundamental header for all entries in the log file
typedef struct PACKED _LogChunkHeader
{
	uint16_t type;
	uint16_t length;
} LogChunkHeader;

// the header for all log entries, contains the LogChunkHeader
typedef struct PACKED _LogEntryHeader
{
	uint16_t type;
	uint16_t length;
	uint32_t timestamp;
} LogEntryHeader;

// --- LOG DEFINITIONS ---
typedef struct PACKED _LogRawI16XYZ
{
	LogEntryHeader header;
	int16_t xyz[3];
} LogRawI16XYZ;

#define MOT_FR 0
#define MOT_FL 1
#define MOT_RL 2
#define MOT_RR 3

typedef struct PACKED _RobotSensors
{
	LogEntryHeader header;

	struct PACKED
	{
		uint32_t time;
		float vel[4];		// angular motor velocity [rad/s]
	} enc;

	struct PACKED
	{
		uint32_t updated;
		uint32_t time;
		float rotVel[3];	// around X, Y, Z [rad/s]
	} gyr;

	struct PACKED
	{
		uint32_t updated;
		uint32_t time;
		float linAcc[3];	// X, Y, Z [m^2/s]
	} acc;

	struct PACKED
	{
		uint32_t updated;
		uint32_t time;		// local arrival time (systime)
		uint32_t delay;		// t [us]
		float pos[3];		// X, Y, Z [m]
		uint32_t camId;
		uint32_t noVision;
	} vision;

	struct PACKED
	{
		float voltage;		// battery voltage [V]
		float current;		// main current consumption [A]
		uint32_t batEmpty;
		uint32_t exhausted;
	} power;

	struct PACKED
	{
		uint32_t time;
		float vol[4];		// [V]
	} motorVol;

	struct PACKED
	{
		float level;		// [V]
		float chgCurrent;	// [A]
		uint32_t kickCounter;
		uint32_t flags;
	} kicker;

	struct PACKED
	{
		float hallSpeed;	// [rad/s] at motor from hall sensors
		float auxSpeed;		// [rad/s] at motor from other sensor or model estimate
		float temp;			// [degree C]
		float voltage;
		float current;
		uint32_t overheated;
		uint32_t hallPos;
	} dribbler;

	struct PACKED
	{
		float level[2];		// (on|off) [V]
		uint32_t interrupted;
		uint32_t ballDetected;
		float estimatedBallPosition[2]; // [mm]
	} ir;

	struct PACKED
	{
		uint32_t time;
		float cur[4];		// [A]
	} motorCur;

	struct PACKED
	{
		uint32_t updated;
		uint32_t time; // center of frame exposure time

		// all coordinates in map (global) frame
		float pos[3];
		float vel[3];
		float linePos[2];
		float lineDir[2];
		uint32_t trackerId;
	} ball;

	struct PACKED
	{
		uint32_t updated;
		uint32_t time;		// local arrival time (systime)
		float strength[3];  // [mT]
		float temp;         // [degree C]
	} mag;

	struct PACKED
	{
		uint32_t id;		// 0 - 15 yellow IDs, 16 - 31 blue IDs
		uint32_t flags;		// working blobs, pattern ident system malfunction
	} pattern;

	struct PACKED
	{
		uint32_t updated;
		uint32_t time;

		uint32_t validColumns;
		float avgHeight;
		float avgYBottom;
		uint32_t isMostlyWhite;
	} pointDist;
} RobotSensors;

typedef struct PACKED _RobotAuxData
{
	// no header here, this data is not logged

	struct PACKED
	{
		uint32_t numCells;
		float cellVoltageMin;
		float cellVoltageMax;
	} power;

	struct PACKED
	{
		float maxLevel;
	} kicker;

	struct PACKED
	{
		uint32_t v2016;
	} physical;

	struct PACKED
	{
		uint32_t installed;
	} ext;
} RobotAuxData;

#define BALL_STATE_UNKNOWN			0
#define BALL_STATE_DEAD_RECKONING	1
#define BALL_STATE_FAR_DETECTION	2 // usually from camera
#define BALL_STATE_AT_BARRIER		3 // barrier interrupted, may be at dribbler
#define BALL_STATE_ACTIVE_DRIBBLING	4 // Force is exerted on the ball

#define DRIBBLER_STATE_OFF			0
#define DRIBBLER_STATE_IDLE			1 // dribbler is spinning but not exerting force on the ball
#define DRIBBLER_STATE_WEAK			2 // low traction on ball
#define DRIBBLER_STATE_STRONG		3 // high traction on ball

typedef struct PACKED _RobotCtrlState
{
	LogEntryHeader header;

	float pos[3];	// [m]
	float vel[3];	// [m/s]
	float acc[3];
	float magZ;     // [rad]

	uint32_t posUpdated;
	uint32_t velUpdated;
	uint32_t accUpdated;

	float dribblerCur; // [A]
	float dribblerKf;  // motor friction constant estimation [Nm*s]
	float dribblerVel; // [m/s] Dribbling bar surface speed
	float dribblerForce; // [N] Dribbling bar surface force estimation (compensated for viscous friction)
	uint32_t dribblerState; // One of DRIBBLER_STATE_

	uint32_t ballState; // One of BALL_STATE_
	float ballPos[2]; // [m] in local "dribbler frame", zero is centered in front of dribbler, uses dribbler collisions for limiting position
	float ballVel[2]; // [m/s] in local "dribbler frame", not respecting any collisions
} RobotCtrlState;

#define MOT_CTRL_OFF			0
#define MOT_CTRL_VELOCITY		1
#define MOT_CTRL_VOLTAGE		2
//#define MOT_CTRL_VEL_ACC		3 // deprecated
#define MOT_CTRL_TORQUE			4
#define MOT_CTRL_VEL_TORQUE		5
#define MOT_CTRL_ANGLE			6

typedef struct PACKED _RobotCtrlOutput
{
	LogEntryHeader header;

	float motorVel[4];	// angular motor velocity [rad/s]
	float motorVoltage[4];	// Motor voltage [V], -CCW, +CW
	float motorAcc[4];	// angular acceleration [rad/s^2]. Used together with velocity
	uint32_t ctrlMode;
	float motorTorque[4]; // [Nm]

	float dribblerVoltage; // [V], +Backspin, -Topspin
	float dribblerVel; // [rad/s] Angular motor velocity limit
	float dribblerTorque; // [Nm] Motor torque limit
	uint32_t dribblerMode; // One of DRIBBLER_MODE_ (from commands.h)
} RobotCtrlOutput;

// output from trajectory generator and dynamic dribbler settings
typedef struct PACKED _RobotCtrlReference
{
	LogEntryHeader header;

	float trajPos[3];
	float trajVelLocal[3];
	float trajAccLocal[3];

	float dribblerVel; // [m/s] Dribbling bar surface speed
	float dribblerForce; // [N] Dribbling bar surface force
} RobotCtrlReference;

typedef struct PACKED _RobotDriveLimits
{
	float velMaxXY;
	float accMaxXY;

	float velMaxW;
	float accMaxW;

	uint32_t strictVelLimit;
} RobotDriveLimits;

typedef struct PACKED _RobotPerformance
{
	LogEntryHeader header;

	uint32_t inputTime;
	uint32_t estimationTime;
	uint32_t skillTime;
	uint32_t controlTime;
	uint32_t outputTime;
	uint32_t miscTime;
} RobotPerformance;

typedef struct PACKED _DriveInput
{
	float pos[3];			// [m]
	float localVel[3];		// [m/s]
	float wheelVel[4];		// [rad/s] @ wheel
	float localForce[3];	// [N] (XY) and [Nm] (W)
	uint32_t modeXY;
	uint32_t modeW;
	float primaryDirection; // [rad]

	RobotDriveLimits limits;
} DriveInput;

typedef struct PACKED _DribblerInput
{
	float velocity;		// [m/s] Dribbling bar surface speed
	float maxForce;		// [N] max. surface force, only used in speed mode
	float voltage;		// [V]
	uint32_t mode;
} DribblerInput;

typedef struct PACKED _KickerInput
{
	uint32_t mode;
	uint32_t device;
	float speed;		// [m/s]
} KickerInput;

typedef struct PACKED _SkillOutput
{
	LogEntryHeader header;

	DriveInput drive;
	DribblerInput dribbler;
	KickerInput kicker;
} SkillOutput;

typedef struct PACKED _NetStats
{
	LogEntryHeader header;

	float rxPeriod; // [s]
	float rssi;	// [dBm]

	uint16_t txPackets;
	uint16_t txBytes;

	uint16_t rxPackets;
	uint16_t rxBytes;
	uint16_t rxPacketsLost;

	uint16_t txHLPacketsLost;
	uint16_t rxHLPacketsLost;
} NetStats;

#define MOTOR_EXCHANGE_MISO_MAX_MEAS 20

#define MOTOR_EXCHANGE_MISO_FLAG_ENCODER_INSTALLED		0x01
#define MOTOR_EXCHANGE_MISO_FLAG_BREAK_ACTIVE			0x02
#define MOTOR_EXCHANGE_MISO_FLAG_D_CUR_OVERLOAD			0x04
#define MOTOR_EXCHANGE_MISO_FLAG_Q_CUR_OVERLOAD			0x08
#define MOTOR_EXCHANCE_MISO_FLAG_VEL_OVERLOAD			0x10
#define MOTOR_EXCHANGE_MISO_FLAG_RX_TIMEOUT				0x20
#define MOTOR_EXCHANGE_MISO_FLAG_CUR_OFFSET_ABNORMAL	0x40

typedef struct PACKED _MotorExchangeMISO
{
	uint8_t hasDebugData;
	uint8_t hallPos;
	int32_t hallComTime;
	uint16_t hallInvalidTransitions;

	uint16_t encPos; // electrical position in encoder ticks
	int16_t encDelta; // ticks within 1ms
	int16_t encCorrection; // -32768 to 32767 (one electrical revolution)
	uint16_t encTicks; // ticks per revolution, should be fix if nothing is broken
	uint16_t encPrescaler; // encoder prescaler in interpolation mode (no encoder installed)

	uint16_t flags;

	uint16_t idleTicksNoLoad;
	uint16_t idleTicksActive;

	uint16_t avgSupplyVoltage; // [mV]
	int16_t avgTemperature; // [deci-degree Celsius]

	int16_t avgCurrentUVW[3];
	int16_t avgCurrentDQ[2];
	int16_t avgVoltageDQ[2];
	int16_t velCtrlOutputCurrent; // [mA]
	int16_t currentOffset; // [mA], may only update if motors are off

	int16_t currentMeas[MOTOR_EXCHANGE_MISO_MAX_MEAS][2];
} MotorExchangeMISO;

#define MOTOR_MODE_OFF					0
#define MOTOR_MODE_VOLT_AB				1
#define MOTOR_MODE_VOLT_DQ				2
#define MOTOR_MODE_CUR_D_VOLT_Q			3
#define MOTOR_MODE_CUR_DQ				4
#define MOTOR_MODE_SPEED				5

#define MOTOR_EXCHANGE_MOSI_FLAG_HALL_ONLY		0x01 // force to not use encoders or hall interpolation

#define MOTOR_EXCHANGE_MOSI_RECORD_NONE				0
#define MOTOR_EXCHANGE_MOSI_RECORD_CUR_AB			1
#define MOTOR_EXCHANGE_MOSI_RECORD_CUR_DQ			2
#define MOTOR_EXCHANGE_MOSI_RECORD_VOLT_AB_CUR_DQ	3 // Ua^2+Ub^2, Id^2+Iq^2
#define MOTOR_EXCHANGE_MOSI_RECORD_SET_Q_ENC_DETLA	4
#define MOTOR_EXCHANGE_MOSI_RECORD_CUR_Q_SET_MEAS	5

typedef struct PACKED _MotorExchangeMOSI
{
	uint8_t motorMode;
	uint8_t recordMode;
	uint16_t _reserved;

	int32_t input[2]; // AB or DQ, [mV] for voltages, [mA] for currents
	int16_t encDeltaSetpoint; // +-2048

	uint16_t flags;

	int32_t curCtrlDKp; // S7.10
	int32_t curCtrlDKi; // S5.13
	int32_t curCtrlQKp; // S7.10
	int32_t curCtrlQKi; // S5.13
	int32_t velCtrlKp; // S7.10
	int32_t velCtrlKi; // S5.13
	int32_t velCtrlAntiJitter; // 0-2048 (corresponding to encDeltaSetpoint)

	uint16_t resistance; // U4.6 [Ohm]
	uint16_t backEmfConstantInv; // U10.2 [rad/(V*s)]
} MotorExchangeMOSI;

typedef struct PACKED _IrExchangeMISO
{
	uint16_t channels[14];
	uint8_t config;
	uint8_t xorChecksum;
	uint16_t idleTicksNoLoad;
	uint16_t idleTicksActive;
} IrExchangeMISO;

typedef struct PACKED _ReceiveBall
{
	LogEntryHeader header;

	float sumatraBallPos[2];
	uint32_t sumatraCaptureTime;

	float robotPiBallPos[2];
	uint32_t robotPiCaptureTime;

	float sumatraReceivePose[3];
	float kickerReceivePos[2];
	float robotReceivePos[2];
} ReceiveBall;

// LOGFILE descriptions
enum ElementType
{
	UINT8 = 0,
	INT8,
	UINT16,
	INT16,
	UINT32,
	INT32,
	FLOAT,
	DOUBLE,
};

// describes a single structure element
typedef struct PACKED _ElementDesc
{
	uint8_t type;
	const char* pName;	// 60 characters max
	const char* pUnit;
	const char* pDesc;
} ElementDesc;

// describes a message
typedef struct PACKED _LogMessageDesc
{
	uint16_t msgId;
	const char* pName;
	uint16_t numElements;
	ElementDesc* elements;
} LogMessageDesc;

extern const LogMessageDesc logMessageDescriptions[10];
