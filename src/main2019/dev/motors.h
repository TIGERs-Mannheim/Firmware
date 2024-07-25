#pragma once

#include "drv/mcu_motor.h"
#include "util/shell_cmd.h"

typedef struct _DevMotors
{
	McuMotor mcu[5];

	MotorParams motorParamsDrive;
	McuMotorFocConfig motorFocConfigDrive;
	McuMotorVelConfig motorVelConfigDrive;
	const McuMotorCtrlParams motorCtrlParamsDrive;

	ConfigFile* pMotorsVelConfigDriveFile;
	ConfigFile* pMotorsFocConfigDriveFile;

	MotorParams motorParamsDribbler;
	McuMotorFocConfig motorFocConfigDribbler;
	McuMotorVelConfig motorVelConfigDribbler;
	const McuMotorCtrlParams motorCtrlParamsDribbler;

	ConfigFile* pMotorsVelConfigDribblerFile;
	ConfigFile* pMotorsFocConfigDribblerFile;
} DevMotors;

extern DevMotors devMotors;

void DevMotorsCfgInit(uint8_t hasPlanetaryGear);
void DevMotorInit(uint8_t id, UARTFifo* pUart, GPIOPinInterface* pRstPin, GPIOPinInterface* pBootPin, tprio_t taskPrio);
void DevMotorRegisterShellCommands(ShellCmdHandler* pHandler);
