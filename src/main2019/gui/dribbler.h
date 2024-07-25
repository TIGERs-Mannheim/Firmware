#pragma once

#include "gfx.h"
#include "drv/mcu_motor.h"
#include "drv/mcu_dribbler.h"
#include "log_msgs.h"
#include "robot/fusion_ekf.h"

GHandle DribblerCreate();
void DribblerUpdate(const McuMotor* pMot, const McuDribbler* pDrib, const RobotCtrlState* pState, const RobotCtrlReference* pRef);
