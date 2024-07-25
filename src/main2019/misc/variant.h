// So, you are a variant...
#pragma once

#include "util/config.h"
#include "util/shell_cmd.h"

typedef struct _VariantConfig
{
	uint8_t hasPlanetaryGear;
} VariantConfig;

typedef struct _Variant
{
	VariantConfig config;
	ConfigFile* pConfigFile;

	ShellCmdHandler cmdHandler;
} Variant;

extern Variant variant;

void VariantInit();
