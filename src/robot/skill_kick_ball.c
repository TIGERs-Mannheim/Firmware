/**
 * This is a workshop skeleton file.
 * Please do not commit changes to it to master!
 */

#include "skill_kick_ball.h"
#include "skill_basics.h"
#include "robot/robot.h"

/**
 * Include some utility headers.
 * Please check them out for useful functions.
 */
#include "math/angle_math.h"
#include "math/clamp.h"
#include "math/line.h"
#include "math/vector.h"
#include "math/map_to_range.h"
#include "hal/sys_time.h"

/**
 * Bot skill input from Sumatra.
 * This struct is reduced in size as much as possible as the skill input data may only be a maximum of 16 bytes.
 * This may lead to some strange units in some cases and even tricks to use every last bit efficiently.
 */
typedef struct PACKED _SkillKickBallInput
{
	int16_t ballPos[2]; // Ball position [mm]
	int16_t targetPosAndFieldSizeLSB[2]; // Upper 14 bits: targetPos [mm], lower 2 bits: field size LSBs

	uint8_t velMaxXY;
	uint8_t velMaxW;
	uint8_t accMaxXY;
	uint8_t accMaxW;

	BasicKDInput kd;

	uint8_t fieldSizeMSB; // upper 4 bits: field half size along Y, lower 4 bits: field half size along X. MSBs. 6 bits in total per dimension. [m / 8]
} SkillKickBallInput;

/** Forward declerations of skill functions. */
static void init(const SkillInput* pInput);
static void run(const SkillInput* pInput, SkillOutput* pOutput);
static void quit();

/** Additional functions for better code readability. */
static void parseInputData(const SkillInput* pInput, SkillOutput* pOutput);

/** Definition of the skill object, there exists only one instance of it */
SkillInstance skillKickBallInstance = { &init, &run, &quit };

/** Definition of the skill data object. */
SkillKickBall skillKickBall;

/** This function is called once when the skill is executed for the first time (i.e. another skill was executed previously) */
static void init(const SkillInput* pInput)
{
	(void)pInput; // pInput is not used, this suppresses a corresponding compiler warning
}

/**
 * This function is called every control cycle as long as the skill is active (1kHz).
 *
 * Note: Bot skills cannot finish and cannot notify the skill system that they are "done".
 * Use a state machine with a final state which does nothing if needed.
 */
static void run(const SkillInput* pInput, SkillOutput* pOutput)
{
	// This method parses all input data and puts some of it to the kickBall struct.
	parseInputData(pInput, pOutput);

	// This is an example output. It will make the robot slowly rotate in place.
	// To really kick the ball you should use drive.pos fields and DRIVE_MODE_GLOBAL_POS.
	pOutput->drive.localVel[0] = 0.0f;
	pOutput->drive.localVel[1] = 0.0f;
	pOutput->drive.localVel[2] = 0.5f;

	pOutput->drive.modeXY = DRIVE_MODE_LOCAL_VEL;
	pOutput->drive.modeW = DRIVE_MODE_LOCAL_VEL;

	// TODO: Implement your kick ball skill code here!

	// Hint: check out the global botParams struct to retrieve physical properties of the bot.
}

/**
 * This method is called once before the skill system starts to execute a different skill.
 * Rarely used by bot skills.
 */
static void quit()
{
}

/**
 * This function converts the weird units in the skill input struct to SI units.
 * We use those weird units to reduce the necessary amount of data that will be transmitted via radio
 * It also applies movement limits and parses kicker and dribbler input.
 */
static void parseInputData(const SkillInput* pInput, SkillOutput* pOutput)
{
	// We know the input struct type for this skill, so this cast is safe
	SkillKickBallInput* pKickBall = (SkillKickBallInput*)pInput->pData;

	// This sets all members of the kicker and dribbler part of the output and handles unit conversion/bit unstuffing.
	// You may overwrite pOutput->dribbler and pOutput->kicker afterwards to adjust it to your needs.
	SkillBasicsParseKDInput(pInput, &pKickBall->kd, pOutput);

	// Ensure we do not set any zero movement limits, control doesn't like that
	if(pKickBall->velMaxXY == 0)
		pKickBall->velMaxXY = 1;

	if(pKickBall->velMaxW == 0)
		pKickBall->velMaxW = 1;

	if(pKickBall->accMaxXY == 0)
		pKickBall->accMaxXY = 1;

	if(pKickBall->accMaxW == 0)
		pKickBall->accMaxW = 1;

	// Convert strange movement limit units to SI units
	pOutput->drive.limits.velMaxXY = ((float)pKickBall->velMaxXY)*(GLOBAL_POS_MAX_VEL_XY/255.0f);
	pOutput->drive.limits.velMaxW = ((float)pKickBall->velMaxW)*(GLOBAL_POS_MAX_VEL_W/255.0f);
	pOutput->drive.limits.accMaxXY = ((float)pKickBall->accMaxXY)*(GLOBAL_POS_MAX_ACC_XY/255.0f);
	pOutput->drive.limits.accMaxW = ((float)pKickBall->accMaxW)*(GLOBAL_POS_MAX_ACC_W/255.0f);

	// Gather bits for field size from different fields
	uint16_t fieldHalfSizeBits[2];
	fieldHalfSizeBits[0] = ((pKickBall->fieldSizeMSB & 0x0F) << 2) | (pKickBall->targetPosAndFieldSizeLSB[0] & 0x03);
	fieldHalfSizeBits[1] = ((pKickBall->fieldSizeMSB & 0xF0) >> 2) | (pKickBall->targetPosAndFieldSizeLSB[1] & 0x03);

	skillKickBall.fieldHalfSize[0] = fieldHalfSizeBits[0] * 0.125f;
	skillKickBall.fieldHalfSize[1] = fieldHalfSizeBits[1] * 0.125f;

	// Convert ball position to SI units
	skillKickBall.ballPos[0] = pKickBall->ballPos[0] * 0.001f;
	skillKickBall.ballPos[1] = pKickBall->ballPos[1] * 0.001f;

	// Convert target position bits to SI units
	skillKickBall.targetPos[0] = (pKickBall->targetPosAndFieldSizeLSB[0] >> 2) * 0.001f;
	skillKickBall.targetPos[1] = (pKickBall->targetPosAndFieldSizeLSB[1] >> 2) * 0.001f;
}
