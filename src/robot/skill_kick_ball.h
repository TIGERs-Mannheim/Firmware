/**
 * This is a workshop skeleton file.
 * Please do not commit changes to it to master!
 */

#pragma once

#include "skills.h"

/**
 * A struct which can be used to store variables across skill init/run/exit calls.
 * This is made globally available for inspection and checks in a shell command handler.
 */
typedef struct _SkillKickBall
{
	float ballPos_m[2];
	float targetPos_m[2];
	float fieldHalfSize_m[2];
} SkillKickBall;

extern SkillKickBall skillKickBall;

/** The actual skill instance of this skill. */
extern SkillInstance skillKickBallInstance;

