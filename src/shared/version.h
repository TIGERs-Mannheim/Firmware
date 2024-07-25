/*
 * version.h
 *
 *  Created on: 04.02.2018
 *      Author: AndreR
 */

#pragma once

#include <stdint.h>
#include "git_version.h"

uint32_t VersionGet();
const char* VersionGetString();
