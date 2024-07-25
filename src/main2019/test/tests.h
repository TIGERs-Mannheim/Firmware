#pragma once

#include "ch.h"
#include "util/shell_cmd.h"

typedef struct _Tests
{
	ShellCmdHandler cmdHandler;
} Tests;

extern Tests tests;

void TestsInit(tprio_t taskPrio);
