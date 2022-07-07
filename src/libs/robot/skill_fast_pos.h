/*
 * skill_fast_pos.h
 *
 *  Created on: 04.10.2016
 *      Author: AndreR
 */

#pragma once

#include "skills.h"

typedef struct _SkillFastPosConfig
{
	uint8_t backwardOnly;
} SkillFastPosConfig;

typedef struct _SkillFastPos
{
	SkillInstance instance;
	SkillFastPosConfig config;
	ConfigFile* pConfigFile;
} SkillFastPos;

extern SkillFastPos skillFastPos;

void SkillFastPosConfigInit();
