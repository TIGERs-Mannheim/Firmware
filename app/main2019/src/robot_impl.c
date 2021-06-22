/*
 * robot_impl.c
 *
 *  Created on: 12.01.2019
 *      Author: AndreR
 */

#include "main/robot.h"
#include "main/ctrl.h"
#include "main/ctrl_panthera.h"
#include "main/fusion_ekf.h"
#include "struct_ids.h"

#include "hal/buzzer.h"
#include "hal/led.h"

#include "util/sys_time.h"
#include "util/log_file.h"

#include "kicker.h"
#include "power.h"
#include "spi4.h"
#include "motors.h"
#include "ext.h"
#include "pattern_ident.h"
#include "ir.h"
#include "inventory.h"
#include "robot_pi.h"

BotParameters botParams = {
	.driveTrain = {
		.motor2WheelRatio = 1.0f,
		.wheel2MotorRatio = 1.0f,
		.motor = { // Nanotec DF45L024048 65W
			.Km = 0.002764f * 8.0f * 1.5f,
			.Kn = 1.0f/(0.002764f * 8.0f),
			.Ke = 0.002764f * 8.0f,
			.Kf = 0.00000727f,
			.R = 0.492f, // motor cur measured: 0.328 => * 1.5
			.L = 227e-6f, // motor cur measured: 151e-6 => * 1.5
		}
	},
	.dribbler = {
		.motor2BarRatio = 1.0f,
		.bar2MotorRatio = 1.0f,
		.barDiameter = 0.011f,
		.motor = { // Moons ECU22048H18 55W
			.Km = 0.008f,
			.Kn = 1.0f/0.008f,
			.Ke = 0.008f,
			.Kf = 0.00000279f,
			.R = 0.5f,
			.L = 52e-6f,
		}
	},
	.physical = { 0.03f, 31.0f, 45.0f, 0.079f, 2.46f , 0.093f },
	.v2016 = 0,
};

static FusionEKFConfig configFusionEKF = {
	.posNoiseXY = 0.001f,
	.posNoiseW = 0.001f,
	.velNoiseXY = 0.01f,
	.visNoiseXY = 0.05f,
	.visNoiseW = 0.1f,
	.outlierMaxVelXY = 3.0f,
	.outlierMaxVelW = 3.0f,
	.trackingCoeff = 1.0f,
	.visCaptureDelay = 15,
	.fusionHorizon = 25,
	.visionTimeoutMs = 1000,
    .irPosNoise = 1000.0f,
    .irMeasNoiseX = 1.0f,
    .irMeasNoiseY = 4.0f,
    .irOutlierMaxVel = 500.0f,
    .irTimeoutMs = 500,
};

static CtrlPantheraConfigModel configModel = {
	.fric = {
		// determined by roll out in Y direction (no rotation compensation, torque == 0), must have plateau phase
		// required data: motor set torque (XYW local output force / absolute force), XY state local velocity (absolute velocity)
		.coulombXY = 2.65f,
		.viscousXY = 0.0f,
		.coulombW = 0.097f, // determined by roll out in W direction (torque == 0), must have plateau phase
		.viscousW = 0.0f,
		.efficiencyXY = 0.72f, // roll out in Y
		.efficiencyW = 0.72f, // roll out in W
	},
	.enc = {
		.K = { 0.95f, 1.0f },
		.T = { 0.095f, 0.05f },
	}
};

static CtrlPantheraConfigCtrl configCtrl = {
	.posXY = { 8.0f, 1.0f, 0.05f },
	.posW = { 4.0f, 8.0f, 0.05f },
	.velW = { 1.0f, 2.0f },
};

static CtrlPantheraConfigDribbler configDribbler = {
	.ctrlGain = 0.01f,
	.iZero = 3.0f,
	.deltaSpeed = 1.0f,
	.deltaCurrent = 1.0f,
	.fwdAccCur = -0.5f,
	.fwdBrkCur =  0.0f,
	.revAccCur =  0.5f,
	.revBrkCur =  0.0f
};

void RobotImplInit()
{
	ctrlPantheraConfigModelDesc.cfgId = SID_CFG_CTRL_PANTHERA_MODEL_V2020;
	ctrlPantheraConfigModelDesc.version = 1;
	ctrlPantheraConfigModelDesc.pName = "v2020/model";

	ctrlPantheraConfigCtrlDesc.cfgId = SID_CFG_CTRL_PANTHERA_CTRL_V2020;
	ctrlPantheraConfigCtrlDesc.version = 1;
	ctrlPantheraConfigCtrlDesc.pName = "v2020/ctrl";

	ctrlPantheraConfigDribblerDesc.cfgId = SID_CFG_CTRL_PANTHERA_DRIBBLER_V2020;
	ctrlPantheraConfigDribblerDesc.version = 3;
	ctrlPantheraConfigDribblerDesc.pName = "v2020/dribbler";

	fusionEKFConfigDesc.cfgId = SID_CFG_FUSION_EKF_V2020;
	fusionEKFConfigDesc.version = 1;
	fusionEKFConfigDesc.pName = "v2020/ekf";

	ConfigOpenOrCreate(&ctrlPantheraConfigModelDesc, &configModel, sizeof(CtrlPantheraConfigModel), 0, 0);
	ConfigOpenOrCreate(&ctrlPantheraConfigCtrlDesc, &configCtrl, sizeof(CtrlPantheraConfigCtrl), 0, 0);
	ConfigOpenOrCreate(&ctrlPantheraConfigDribblerDesc, &configDribbler, sizeof(CtrlPantheraConfigDribbler), 0, 0);
	ConfigOpenOrCreate(&fusionEKFConfigDesc, &configFusionEKF, sizeof(FusionEKFConfig), &FusionEKFConfigUpdateCallback, 0);

	CtrlPantheraInit(&configCtrl, &configModel, &configDribbler, &configFusionEKF);
}

void RobotImplUpdateSensors(RobotSensors* pSensors)
{
	const InventoryImuCalibration* pImuCalib = InventoryGetImuCalibration();

	// motors
	memset(&pSensors->enc, 0, sizeof(pSensors->enc));
	pSensors->enc.time = SysTimeUSec();
	pSensors->motorVol.time = SysTimeUSec();

	for(uint8_t m = 0; m < 4; m++)
	{
		pSensors->enc.vel[m] = motors.motor[m].encoderVelocity;
		pSensors->motorVol.vol[m] = motors.motor[m].avgVoltageDQ1ms[1];
	}

	// motor current
	memset(&pSensors->motorCur, 0, sizeof(pSensors->motorCur));
	pSensors->motorCur.time = SysTimeUSec();
	pSensors->motorCur.cur[0] = motors.motor[0].avgCurrentDQ1ms[1];
	pSensors->motorCur.cur[1] = motors.motor[1].avgCurrentDQ1ms[1];
	pSensors->motorCur.cur[2] = motors.motor[2].avgCurrentDQ1ms[1];
	pSensors->motorCur.cur[3] = motors.motor[3].avgCurrentDQ1ms[1];

	// power
	pSensors->power.voltage = power.vBat;
	pSensors->power.current = power.iCur;
	pSensors->power.batEmpty = power.batEmpty;
	pSensors->power.exhausted = power.exhausted;

	// IMU
	pSensors->acc.linAcc[0] = spi4.imu.acc[0] - pImuCalib->accBias[0] - pImuCalib->accTiltBias[0];
	pSensors->acc.linAcc[1] = spi4.imu.acc[1] - pImuCalib->accBias[1] - pImuCalib->accTiltBias[1];
	pSensors->acc.linAcc[2] = spi4.imu.acc[2] - pImuCalib->accBias[2];
	pSensors->acc.time = SysTimeUSec();
	pSensors->acc.updated = spi4.imu.updated;

	pSensors->gyr.rotVel[0] = spi4.imu.gyr[0] - pImuCalib->gyrBias[0];
	pSensors->gyr.rotVel[1] = spi4.imu.gyr[1] - pImuCalib->gyrBias[1];
	pSensors->gyr.rotVel[2] = spi4.imu.gyr[2] - pImuCalib->gyrBias[2];
	pSensors->gyr.time = SysTimeUSec();
	pSensors->gyr.updated = spi4.imu.updated;

	spi4.imu.updated = 0;

	// Magnetometer
	pSensors->mag.strength[0] = spi4.mag.strength[0] - pImuCalib->magBias[0];
	pSensors->mag.strength[1] = spi4.mag.strength[1] - pImuCalib->magBias[1];
	pSensors->mag.strength[2] = spi4.mag.strength[2] - pImuCalib->magBias[2];
	pSensors->mag.time = SysTimeUSec();
	pSensors->mag.temp = spi4.mag.temp;
	pSensors->mag.updated = spi4.mag.updated;

	spi4.mag.updated = 0;

	// Vision - SKIPPED (filled by robot base implementation)

	// Kicker
	pSensors->kicker.level = kicker.vCap;
	pSensors->kicker.chgCurrent = 0.0f;
	pSensors->kicker.kickCounter = kicker.kickCounter;
	pSensors->kicker.flags = 0;
	if(KickerIsReadyToShoot())
		pSensors->kicker.flags |= KICKER_FLAGS_READY;

	if((kicker.barrierTxDamaged || kicker.barrierRxDamaged) && ir.irData.ballDetected == 0)
		pSensors->kicker.flags |= KICKER_FLAGS_DMG_BARRIER;

	if(kicker.straightDamaged)
		pSensors->kicker.flags |= KICKER_FLAGS_DMG_STRAIGHT;

	if(kicker.chipDamaged)
		pSensors->kicker.flags |= KICKER_FLAGS_DMG_CHIP;

	if(kicker.chargeOverheated)
		pSensors->kicker.flags |= KICKER_FLAGS_DMG_CHARGE;

	if(kicker.v2017)
		pSensors->kicker.flags |= KICKER_FLAGS_V2017;

	// Dribbler
	pSensors->dribbler.speed = motors.motor[4].hallVelocity;
	pSensors->dribbler.voltage = motors.motor[4].setVoltage[1];
	pSensors->dribbler.hallPos = motors.motor[4].miso.hallPos;
	pSensors->dribbler.current = motors.motor[4].avgCurrentDQ1ms[1];
	pSensors->dribbler.temp = ir.dribblerTemperature;
	pSensors->dribbler.overheated = ir.dribblerOverheated;

	// Barrier
	pSensors->ir.level[0] = kicker.vIrOff;
	pSensors->ir.level[1] = kicker.vIrOn;
	pSensors->ir.interrupted = kicker.interrupted;

	// IR Array
	pSensors->ir.ballDetected = ir.irData.ballDetected;
	pSensors->ir.estimatedBallPosition[0] = ir.irData.estimatedBallPosition[0] * 10.0f; // convert cm to mm
	pSensors->ir.estimatedBallPosition[1] = ir.irData.estimatedBallPosition[1] * 10.0f; // convert cm to mm

	// Pattern identification system
	pSensors->pattern.id = patternIdent.detectedId;
	pSensors->pattern.flags = PatternIdentGetAndClearFlags();

	// Ball Detector
	RobotPiUpdateBallSensorData(pSensors);

	if(SysTimeUSec() - robotPi.ballDetections.timestampUs < 1000000)
		botParams.extInstalled = 1;
	else
		botParams.extInstalled = 0;
}

void RobotImplUpdateAuxData(RobotAuxData* pAux)
{
	// power
	pAux->power.cellVoltageMax = power.cellMax;
	pAux->power.cellVoltageMin = power.cellMin;
	pAux->power.numCells = power.batCells;

	// kicker
	pAux->kicker.maxLevel = kicker.config.maxVoltage;

	// physical
	pAux->physical.dribblerDistance = botParams.physical.dribblerDistance;
}

uint8_t RobotImplGetHardwareId()
{
	const InventoryEntry* pInv = InventoryGetEntry();

	if(pInv)
		return pInv->hwId;

	return 0xFF;
}

void RobotImplWriteLogMsg(void* pMsg)
{
	LogFileWrite(pMsg);
}

void RobotImplMotorOutput(const RobotCtrlOutput* pMotor)
{
	for(uint8_t motorId = 0; motorId < 4; motorId++)
	{
		switch(pMotor->ctrlMode)
		{
			case MOT_CTRL_VELOCITY:
			{
				MotorsSetVelocity(motorId, pMotor->motorVel[motorId], 0.0f, 0.0f);
			}
			break;
			case MOT_CTRL_VOLTAGE:
			{
				MotorsSetVoltageDQ(motorId, 0.0f, pMotor->motorVoltage[motorId]);
			}
			break;
			case MOT_CTRL_TORQUE:
			{
				MotorsSetCurrentDQ(motorId, 0.0f,
						pMotor->motorTorque[motorId]/botParams.driveTrain.motor.Km);
			}
			break;
			case MOT_CTRL_VEL_TORQUE:
			{
				MotorsSetVelocity(motorId, pMotor->motorVel[motorId], 0.0f,
						pMotor->motorTorque[motorId]/botParams.driveTrain.motor.Km);
			}
			break;
			default:
			{
				MotorsSetOff(motorId);
			}
			break;
		}
	}
}

void RobotImplKickerOutput(const KickerInput* pKicker)
{
	switch(pKicker->mode)
	{
		case KICKER_MODE_FORCE:
			KickerFireSpeed(pKicker->speed, pKicker->device);
			break;
		case KICKER_MODE_ARM:
			KickerArmSpeed(pKicker->speed, pKicker->device);
			break;
		case KICKER_MODE_DISARM:
			KickerDisarm();
			break;
		case KICKER_MODE_ARM_TIME:
			KickerArm(pKicker->speed, pKicker->device);
			break;
	}
}

void RobotImplDribblerOutput(const RobotCtrlOutput* pCtrlOut)
{
	switch(pCtrlOut->dribblerMode)
	{
		case DRIBBLER_MODE_VOLTAGE:
			MotorsSetVoltageDQ(4, 0.0f, pCtrlOut->dribblerVoltage);
			break;
		default:
			MotorsSetOff(4);
			break;
	}
}

void RobotImplBuzzerPlay(uint16_t seqId)
{
	BuzzerPlayId(seqId);
}

void RobotImplKickerAutoCharge(uint8_t enable)
{
	if(enable)
	{
		KickerEnableAutoRecharge(1);
		KickerEnableCharge(1);
	}
	else
	{
		KickerAutoDischarge();
	}
}

void RobotImplSetLEDs(uint8_t state)
{
	switch(state)
	{
		case SYSTEM_MATCH_CTRL_FLAGS_LED_OFF:
			LEDLeftSet(0, 0, 0, 0.01);
			LEDRightSet(0, 0, 0, 0.01);
			break;
		case SYSTEM_MATCH_CTRL_FLAGS_LED_RED:
			LEDLeftSet(0.5f, 0, 0, 0);
			LEDRightSet(0.5f, 0, 0, 0);
			break;
		case SYSTEM_MATCH_CTRL_FLAGS_LED_GREEN:
			LEDLeftSet(0, 0.3f, 0, 0);
			LEDRightSet(0, 0.3f, 0, 0);
			break;
		case SYSTEM_MATCH_CTRL_FLAGS_LED_BLUE:
			LEDLeftSet(0, 0, 0.4f, 0);
			LEDRightSet(0, 0, 0.4f, 0);
			break;
		case SYSTEM_MATCH_CTRL_FLAGS_LED_WHITE:
			LEDLeftSet(0, 0, 0, 0.5f);
			LEDRightSet(0, 0, 0, 0.5f);
			break;
		case SYSTEM_MATCH_CTRL_FLAGS_LED_LIGHT_BLUE:
			LEDLeftSet(0, 0.25f, 0.25f, 0);
			LEDRightSet(0, 0.25f, 0.25f, 0);
			break;
		case SYSTEM_MATCH_CTRL_FLAGS_LED_GOLD:
			LEDLeftSet(0.3f, 0.15f, 0, 0);
			LEDRightSet(0.3f, 0.15f, 0, 0);
			break;
		case SYSTEM_MATCH_CTRL_FLAGS_LED_PURPLE:
			LEDLeftSet(0.25f, 0, 0.25f, 0);
			LEDRightSet(0.25f, 0, 0.25f, 0);
			break;
		default:
			LEDLeftSet(0, 0.1f, 0, 0);
			LEDRightSet(0, 0, 0.1f, 0);
			break;
	}
}

void RobotImplExtSendMatchFeedback(const SystemMatchFeedback* pFeedback)
{
	static uint32_t lastFeedbackTime = 0;

	if((SysTimeUSec() - lastFeedbackTime) < 10000)
		return;

	lastFeedbackTime = SysTimeUSec();

	PacketHeader header;
	header.section = SECTION_SYSTEM;
	header.cmd = CMD_SYSTEM_MATCH_FEEDBACK;
	ExtSendPacket(&header, pFeedback, sizeof(SystemMatchFeedback));
}
