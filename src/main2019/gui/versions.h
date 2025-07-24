#pragma once

#include "gfx.h"
#include "util/log_file.h"
#include "util/st_bootloader.h"
#include "ext_commands.h"

GHandle VersionsCreate();
void	VersionsUpdateExt(ExtRobotPiVersion* pRobotPiVersion, uint8_t installed);
void	VersionsUpdateMotor(uint8_t motorId, STBootloaderFlashResult* pFlashResult);
void	VersionsUpdateIr(STBootloaderFlashResult* pFlashResult);
