/**
 * This is a workshop skeleton file.
 * Please do not commit changes to it to master!
 */

#pragma once

#include "skills.h"

/**
 * A struct which can be used to store variables across skill init/run/exit calls.
 * This is made globally available for inspection and checks in cli.c
 */
typedef struct _SkillKickBall
{
	float ballPos[2]; // [m]
	float targetPos[2]; // [m]
	float fieldHalfSize[2]; // [m]
} SkillKickBall;

extern SkillKickBall skillKickBall;

/** The actual skill instance of this skill. */
extern SkillInstance skillKickBallInstance;

