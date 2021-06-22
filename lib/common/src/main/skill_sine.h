/*
 * skill_sine.h
 *
 *  Created on: 25.06.2015
 *      Author: AndreR
 */

#ifndef SKILL_SINE_H_
#define SKILL_SINE_H_

#include <main/skills.h>

typedef struct PACKED _SkillSineInput
{
	int16_t vel[3];	// [mm/s, mrad/s]
	uint16_t freq;	// [mHz]
} SkillSineInput;

extern SkillInstance skillSine;

#endif /* SKILL_SINE_H_ */
