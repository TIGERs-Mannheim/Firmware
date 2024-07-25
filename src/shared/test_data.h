#pragma once

#include <stdint.h>

// IDs of all available tests
typedef enum _TestId
{
	TEST_ID_MOT_IDENT_ELEC,
	TEST_ID_MOT_IDENT_MECH,
	TEST_ID_MOT_PHASE_RES,
	TEST_ID_MOT_TRACTION,
	TEST_ID_KICKER,
	TEST_ID_IMU_CALIB,
	TEST_ID_DRIBBLE_ROTATION,
	TEST_ID_ROT_IDENT,
	TEST_ID_BAT_RES,
	TEST_ID_BAT_DISCHARGE,
	TEST_ID_FRONT_LEDS,
	TEST_ID_DRIBBLE_IDLE,
} TestId;

// Test-specific defines
#define TEST_REASONING_RESULT_OK		0
#define TEST_REASONING_RESULT_WARNING	1
#define TEST_REASONING_RESULT_BAD		2

// Test-specific argument structures
typedef struct _TestArgMot
{
	uint8_t motorId;
} TestArgMot;

typedef struct _TestArgDribbleRotation
{
	float dribblerCurrent_A;
	float dribblerSpeed_rpm;
} TestArgDribbleRotation;

// Test-specific result structures
typedef struct _TestResultMotIdentMech
{
	float damping;	// Drive train damping [Nms]
	float inertia;	// Drive train inertia [Nm]

	float cur;		// current at 8V
	float speed;	// speed at 8V
	uint8_t motorId;

	struct
	{
		uint8_t damping;
		uint8_t inertia;
		uint8_t cur;
		uint8_t vel;
	} reasoning;
} TestResultMotIdentMech;

typedef struct _TestResultKicker
{
	float chargingSpeed; // V/s
	float straightVoltageDrop; // V
	float chipVoltageDrop; //V

	struct
	{
		uint8_t charge;
		uint8_t straight;
		uint8_t chip;
	} reasoning;
} TestResultKicker;
