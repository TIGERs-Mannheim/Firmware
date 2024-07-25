#include "motors.h"
#include "struct_ids.h"
#include "generated/mcu_motor_code.h"
#include <stddef.h>

DevMotors devMotors = {
	.motorParamsDrive = { // Nanotec DF45L024048 65W
		.Km = 0.0225f * 1.5f,
		.Kn = 1.0f/0.0225f,
		.Ke = 0.0225f,
		.Kf = 8.85475e-6f,
		.R = 0.4875f, // motor cur measured: 0.325 => * 1.5
		.L = 195e-6f, // motor cur measured: 130e-6 => * 1.5
		.pp = 8,
	},
	.motorFocConfigDrive = {
		.curDKp = 0.5f,
		.curDKi = 0.06f,
		.curQKp = 0.5f,
		.curQKi = 0.06f,
	},
	.motorVelConfigDrive = {
		.velKp = 300.0f,
		.velKi = 0,
		.velAntiJitter = 1.5f,
	},
	.motorCtrlParamsDrive = {
		// encDelta = rads * 1/(2*float(%pi)) * 23040 * 1/1000;
		// => encDelta = 3.666929888837269 * rad per sec
		// => rad per sec = encDelta * 0.272707695624114
		.speedRawToRadPerSec = 0.272707695624114f,
		.curGainRealToMotor = (10800.0f/(4096.0f*4.0f)),
		.velGainRealToMotor = (4096.0f/(10800.0f*2.0f)),
		.maxCurrent_A = 9.0f,
	},

	.motorParamsDribbler = { // Moons ECU22048H18 55W
		.Km = 0.0077f,
		.Kn = 1.0f/0.0077f,
		.Ke = 0.0077f,
		.Kf = 0.000008126f,
		.R = 0.48f,
		.L = 35e-6f,
		.pp = 1,
	},
	.motorFocConfigDribbler = {
		.curDKp = 0.0f, // not used by dribbler control
		.curDKi = 0.0f, // not used by dribbler control
		.curQKp = 0.143f,
		.curQKi = 0.05f,
	},
	.motorVelConfigDribbler = {
		.velKp = 25.0f,
		.velKi = 0.0f,
		.velAntiJitter = 0.0f,
	},
	.motorCtrlParamsDribbler = {
		.speedRawToRadPerSec = (1024.0f/1000.0f),
		.curGainRealToMotor = (10800.0f/(4096.0f*8.0f)),
		.velGainRealToMotor = (4096.0f/10800.0f),
		.maxCurrent_A = 9.0f,
	},
};

static ConfigFileDesc configFileDescMotorFocConfigDrive = { SID_CFG_MOTORS_FOC_DRIVE, 0, "drive/foc", 0, 0 };
static ConfigFileDesc configFileDescMotorVelConfigDrive;
static ConfigFileDesc configFileDescMotorFocConfigDribbler = { SID_CFG_MOTORS_FOC_DRIBBLER, 0, "dribbler/foc", 0, 0 };
static ConfigFileDesc configFileDescMotorVelConfigDribbler = { SID_CFG_MOTORS_VEL_DRIBBLER, 0, "dribbler/vel", 0, 0 };

#define MOTOR_RECORD_SIZE 10000
int16_t motorDebugRecord[5][MOTOR_RECORD_SIZE*2] __attribute__((aligned(4), section(".nocache")));

static void motorConfigUpdateCallback(uint16_t cfgId)
{
	if(cfgId == SID_CFG_MOTORS_FOC_DRIBBLER)
	{
		if(McuMotorClampFocConfig(&devMotors.motorFocConfigDribbler, &devMotors.motorCtrlParamsDribbler))
			ConfigNotifyUpdate(devMotors.pMotorsFocConfigDribblerFile);
	}

	if(cfgId == SID_CFG_MOTORS_VEL_DRIBBLER)
	{
		if(McuMotorClampVelConfig(&devMotors.motorVelConfigDribbler, &devMotors.motorCtrlParamsDribbler))
			ConfigNotifyUpdate(devMotors.pMotorsVelConfigDribblerFile);
	}

	if(cfgId == SID_CFG_MOTORS_FOC_DRIVE)
	{
		if(McuMotorClampFocConfig(&devMotors.motorFocConfigDrive, &devMotors.motorCtrlParamsDrive))
			ConfigNotifyUpdate(devMotors.pMotorsFocConfigDriveFile);
	}

	if(cfgId >= SID_CFG_MOTORS_VEL_DRIVE_V2020_DD && cfgId <= SID_CFG_MOTORS_VEL_DRIVE_V_LAST)
	{
		if(McuMotorClampVelConfig(&devMotors.motorVelConfigDrive, &devMotors.motorCtrlParamsDrive))
			ConfigNotifyUpdate(devMotors.pMotorsVelConfigDriveFile);
	}
}

void DevMotorsCfgInit(uint8_t hasPlanetaryGear)
{
	McuMotorFocConfigDescInit(&configFileDescMotorFocConfigDribbler);
	McuMotorVelConfigDescInit(&configFileDescMotorVelConfigDribbler);
	McuMotorFocConfigDescInit(&configFileDescMotorFocConfigDrive);
	McuMotorVelConfigDescInit(&configFileDescMotorVelConfigDrive);

	if(hasPlanetaryGear)
	{
		configFileDescMotorVelConfigDrive.cfgId = SID_CFG_MOTORS_VEL_DRIVE_V2020_PG;
		configFileDescMotorVelConfigDrive.pName = "drive_PG/vel";
		configFileDescMotorVelConfigDrive.version = 0;

		devMotors.motorVelConfigDrive.velKp = 150.0f;
	}
	else
	{
		configFileDescMotorVelConfigDrive.cfgId = SID_CFG_MOTORS_VEL_DRIVE_V2020_DD;
		configFileDescMotorVelConfigDrive.pName = "drive_DD/vel";
		configFileDescMotorVelConfigDrive.version = 0;
	}

	devMotors.pMotorsFocConfigDribblerFile = ConfigOpenOrCreate(&configFileDescMotorFocConfigDribbler, &devMotors.motorFocConfigDribbler, sizeof(devMotors.motorFocConfigDribbler), &motorConfigUpdateCallback, CONFIG_FILE_FLAG_INTERNAL);
	devMotors.pMotorsVelConfigDribblerFile = ConfigOpenOrCreate(&configFileDescMotorVelConfigDribbler, &devMotors.motorVelConfigDribbler, sizeof(devMotors.motorVelConfigDribbler), &motorConfigUpdateCallback, CONFIG_FILE_FLAG_INTERNAL);
	devMotors.pMotorsFocConfigDriveFile = ConfigOpenOrCreate(&configFileDescMotorFocConfigDrive, &devMotors.motorFocConfigDrive, sizeof(devMotors.motorFocConfigDrive), &motorConfigUpdateCallback, 0);
	devMotors.pMotorsVelConfigDriveFile = ConfigOpenOrCreate(&configFileDescMotorVelConfigDrive, &devMotors.motorVelConfigDrive, sizeof(devMotors.motorVelConfigDrive), &motorConfigUpdateCallback, 0);
}

void DevMotorInit(uint8_t id, UARTFifo* pUart, GPIOPinInterface* pRstPin, GPIOPinInterface* pBootPin, tprio_t taskPrio)
{
	McuMotorData motInit;
	motInit.pFwProgram = mcu_motor_code_data;
	motInit.fwSize = mcu_motor_code_size;
	if(id < 4)
	{
		motInit.pMotorParams = &devMotors.motorParamsDrive;
		motInit.pCtrlParams = &devMotors.motorCtrlParamsDrive;
		motInit.pFocConfig = &devMotors.motorFocConfigDrive;
		motInit.pVelConfig = &devMotors.motorVelConfigDrive;
	}
	else
	{
		motInit.pMotorParams = &devMotors.motorParamsDribbler;
		motInit.pCtrlParams = &devMotors.motorCtrlParamsDribbler;
		motInit.pFocConfig = &devMotors.motorFocConfigDribbler;
		motInit.pVelConfig = &devMotors.motorVelConfigDribbler;
	}

	motInit.motorId = id;
	motInit.pUart = pUart;
	motInit.pRstPin = pRstPin;
	motInit.pBootPin = pBootPin;
	motInit.pDebugRecordData = motorDebugRecord[id];
	motInit.debugRecordSize = MOTOR_RECORD_SIZE;
	McuMotorInit(&devMotors.mcu[id], &motInit, taskPrio);
}

SHELL_CMD(mot, "Motor subsystem commands",
	SHELL_ARG(id, "Target motor <0-4 | drv> (0=FR, 1=FL, 2=RL, 3=RR, 4=DRIB, drv=all drive motors)"),
	SHELL_ARG(cmd, "Subsystem command and arguments")
);

SHELL_CMD_IMPL(mot)
{
	(void)pUser;

	if(strcmp(argv[1], "drv") == 0)
	{
		for(uint8_t mot = 0; mot < 4; mot++)
			ShellCmdForward(&devMotors.mcu[mot].cmdHandler, argc-2, argv+2);
	}
	else
	{
		int id = atoi(argv[1]);

		if(id > 4)
		{
			fprintf(stderr, "Invalid motor id\r\n");
			return;
		}

		ShellCmdForward(&devMotors.mcu[id].cmdHandler, argc-2, argv+2);
	}
}

SHELL_CMD(drv_cfg_set, "Change drive motor controller parameters",
	SHELL_ARG(param, "Available parameters: curd_p, curd_i, curq_p, curq_i, vel_p, vel_i, vel_aj"),
	SHELL_ARG(value, "New value")
);

typedef struct _CfgMapEntry
{
	const char* pName;
	float* pValue;
} CfgMapEntry;

static const CfgMapEntry driveConfigMap[] = {
	{ "curd_p", &devMotors.motorFocConfigDrive.curDKp },
	{ "curd_i", &devMotors.motorFocConfigDrive.curDKi },
	{ "curq_p", &devMotors.motorFocConfigDrive.curQKp },
	{ "curq_i", &devMotors.motorFocConfigDrive.curQKi },
	{ "vel_p", &devMotors.motorVelConfigDrive.velKp },
	{ "vel_i", &devMotors.motorVelConfigDrive.velKi },
	{ "vel_aj", &devMotors.motorVelConfigDrive.velAntiJitter },
};

SHELL_CMD_IMPL(drv_cfg_set)
{
	(void)pUser; (void)argc;

	const char* pParam = argv[1];
	float value = atof(argv[2]);

	size_t numParams = sizeof(driveConfigMap) / sizeof(driveConfigMap[0]);

	for(size_t i = 0; i < numParams; i++)
	{
		if(strcmp(pParam, driveConfigMap[i].pName) == 0)
		{
			*driveConfigMap[i].pValue = value;

			McuMotorClampFocConfig(&devMotors.motorFocConfigDrive, &devMotors.motorCtrlParamsDrive);
			McuMotorClampVelConfig(&devMotors.motorVelConfigDrive, &devMotors.motorCtrlParamsDrive);

			ConfigNotifyUpdate(devMotors.pMotorsFocConfigDriveFile);
			ConfigNotifyUpdate(devMotors.pMotorsVelConfigDriveFile);

			printf("Drive parameter %s set to %.3f\r\n", pParam, value);

			return;
		}
	}

	fprintf(stderr, "Invalid parameter: %s\r\n", pParam);
}

SHELL_CMD(drv_cfg, "Show drive motor controller parameters");

SHELL_CMD_IMPL(drv_cfg)
{
	(void)pUser; (void)argc; (void)argv;

	printf("=== Drive Motors ===\r\n");
	printf("Current D: P: %6.3f, I: %6.3f\r\n", devMotors.motorFocConfigDrive.curDKp, devMotors.motorFocConfigDrive.curDKi);
	printf("Current Q: P: %6.3f, I: %6.3f\r\n", devMotors.motorFocConfigDrive.curQKp, devMotors.motorFocConfigDrive.curQKi);
	printf("Velocity:  P: %6.3f, I: %6.3f, AJ: %.3f\r\n", devMotors.motorVelConfigDrive.velKp, devMotors.motorVelConfigDrive.velKi, devMotors.motorVelConfigDrive.velAntiJitter);
}

SHELL_CMD(drib_cfg_set, "Change dribbler motor controller parameters",
	SHELL_ARG(param, "Available parameters: curq_p, curq_i, vel_p, vel_i"),
	SHELL_ARG(value, "New value")
);

static const CfgMapEntry dribblerConfigMap[] = {
	{ "curq_p", &devMotors.motorFocConfigDribbler.curQKp },
	{ "curq_i", &devMotors.motorFocConfigDribbler.curQKi },
	{ "vel_p", &devMotors.motorVelConfigDribbler.velKp },
	{ "vel_i", &devMotors.motorVelConfigDribbler.velKi },
};

SHELL_CMD_IMPL(drib_cfg_set)
{
	(void)pUser; (void)argc;

	const char* pParam = argv[1];
	float value = atof(argv[2]);

	size_t numParams = sizeof(dribblerConfigMap) / sizeof(dribblerConfigMap[0]);

	for(size_t i = 0; i < numParams; i++)
	{
		if(strcmp(pParam, dribblerConfigMap[i].pName) == 0)
		{
			*dribblerConfigMap[i].pValue = value;

			McuMotorClampFocConfig(&devMotors.motorFocConfigDribbler, &devMotors.motorCtrlParamsDribbler);
			McuMotorClampVelConfig(&devMotors.motorVelConfigDribbler, &devMotors.motorCtrlParamsDribbler);

			ConfigNotifyUpdate(devMotors.pMotorsFocConfigDribblerFile);
			ConfigNotifyUpdate(devMotors.pMotorsVelConfigDribblerFile);

			printf("Dribbler parameter %s set to %.3f\r\n", pParam, value);

			return;
		}
	}

	fprintf(stderr, "Invalid parameter: %s\r\n", pParam);
}

SHELL_CMD(drib_cfg, "Show dribbler motor controller parameters");

SHELL_CMD_IMPL(drib_cfg)
{
	(void)pUser; (void)argc; (void)argv;

	printf("=== Dribbler ===\r\n");
	printf("Current Q: P: %6.3f, I: %6.3f\r\n", devMotors.motorFocConfigDribbler.curQKp, devMotors.motorFocConfigDribbler.curQKi);
	printf("Velocity:  P: %6.3f, I: %6.3f\r\n", devMotors.motorVelConfigDribbler.velKp, devMotors.motorVelConfigDribbler.velKi);
}

void DevMotorRegisterShellCommands(ShellCmdHandler* pHandler)
{
	ShellCmdAdd(pHandler, mot_command);
	ShellCmdAdd(pHandler, drv_cfg_command);
	ShellCmdAdd(pHandler, drv_cfg_set_command);
	ShellCmdAdd(pHandler, drib_cfg_command);
	ShellCmdAdd(pHandler, drib_cfg_set_command);
}
