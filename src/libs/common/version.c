/*
 * version.c
 *
 *  Created on: 01.01.2019
 *      Author: AndreR
 */

#include "version.h"
#include "git_version.h"

uint32_t VersionGet()
{
	return (GIT_VERSION_MAJOR << 24) | (GIT_VERSION_MINOR << 16) | (GIT_VERSION_PATCH << 8) | GIT_IS_DIRTY;
}

const char* VersionGetString()
{
	return GIT_VERSION_STR;
}
