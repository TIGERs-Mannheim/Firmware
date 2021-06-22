/*
 * ir_gui.h
 *
 *  Created on: 26.01.2020
 *      Author: MariusM
 */

#pragma once

#include "gfx.h"

GHandle IrGuiCreate();
void IrGuiUpdate(const IRData* pData, const RobotCtrlState* pState);
