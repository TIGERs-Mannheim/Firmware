/*
 * robot_impl.c
 *
 *  Created on: 20.07.2016
 *      Author: AndreR
 */

#include "robot/robot.h"
#include "robot/ctrl.h"
#include "robot/ctrl_panthera.h"
#include "struct_ids.h"
#include "util/log_file.h"
#include "hal/motors.h"
#include "hal/kicker.h"
#include "hal/led.h"
#include "spi4.h"
#include "spi6.h"
#include "power.h"
#include "util/sys_time.h"
#include "robot/network.h"
#include "hal/buzzer.h"
#include "ext.h"

BotParameters botParams = {
	.driveTrain = {
		.motor2WheelRatio = (18.0f/60.0f),
		.wheel2MotorRatio = (60.0f/18.0f),
		.motor = { // Maxon EC 45 flat 50W (339285)
			.Km = 0.0251f,
			.Kn = (380.0f*2.0f*PI/60.0f),
			.Ke = (60.0f/(380.0f*2.0f*PI)),
			.Kf = 0.94e-4f,
			.R = 0.464f,
			.L = 322e-6f,
		}
	},
	.dribbler = {
		.motor2BarRatio = 50.0f/30.0f,
		.bar2MotorRatio = 30.0f/50.0f,
		.barDiameter = 0.014f,
		.motor = { // Maxon EC-max 22 25W (283856)
			.Km = 0.0091f,
			.Kn = 1.0f/0.0091f,
			.Ke = 0.0091f,
			.Kf = 0.00000135f,
			.R = 0.955f,
			.L = 49.8e-6f,
		}
	},
	.physical = { 0.0285f, 30.0f, 45.0f, 0.083f, 2.3f, 0.094f },
	.v2016 = 1,
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
	.emaAccelT = 0.01f,
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
		.coulombXY = 4.44f,
		.viscousXY = 0.0f,
		.coulombW = 0.14f, // determined by roll out in W direction (torque == 0), must have plateau phase
		.viscousW = 0.0f,
		.efficiencyXY = 0.37f, // roll out in Y
		.efficiencyW = 0.29f, // roll out in W
	},
	.enc = {
		.K = { 0.79f, 0.9f },
		.T = { 0.155f, 0.06f },
	}
};

static CtrlPantheraConfigCtrl configCtrl = {
	.posXY = { 8.0f, 1.0f, 0.05f },
	.posW = { 4.0f, 8.0f, 0.05f },
	.velW = { 0.0f, 2.0f },
	.dribblerCurrent2Acc = 0.0f,
};

void RobotImplInit()
{
	ctrlPantheraConfigModelDesc.cfgId = SID_CFG_CTRL_PANTHERA_MODEL_V2016;
	ctrlPantheraConfigModelDesc.version = 0;
	ctrlPantheraConfigModelDesc.pName = "v2016/model";

	ctrlPantheraConfigCtrlDesc.cfgId = SID_CFG_CTRL_PANTHERA_CTRL_V2016;
	ctrlPantheraConfigCtrlDesc.version = 0;
	ctrlPantheraConfigCtrlDesc.pName = "v2016/ctrl";

	fusionEKFConfigDesc.cfgId = SID_CFG_FUSION_EKF_V2016;
	fusionEKFConfigDesc.version = 3;
	fusionEKFConfigDesc.pName = "v2016/ekf";

	ConfigOpenOrCreate(&ctrlPantheraConfigModelDesc, &configModel, sizeof(CtrlPantheraConfigModel), 0, 0);
	ConfigOpenOrCreate(&ctrlPantheraConfigCtrlDesc, &configCtrl, sizeof(CtrlPantheraConfigCtrl), 0, 0);
	ConfigOpenOrCreate(&fusionEKFConfigDesc, &configFusionEKF, sizeof(FusionEKFConfig), &FusionEKFConfigUpdateCallback, 0);

	CtrlPantheraInit(&configCtrl, &configModel, &configFusionEKF);
}

void RobotImplUpdateSensors(RobotSensors* pSensors)
{
	// motors
	pSensors->enc.time = motors.drive[0].velocityTimestamp;
	pSensors->motorVol.time = motors.drive[0].outVoltageTimestamp;
	pSensors->motorCur.time = motors.drive[0].velocityTimestamp;

	for(uint8_t m = 0; m < 4; m++)
	{
		pSensors->enc.vel[m] = motors.drive[m].velocity;
		pSensors->motorVol.vol[m] = motors.drive[m].outVoltage;
		pSensors->motorCur.cur[m] = motors.drive[m].estCurrent;
	}

	// power
	pSensors->power.voltage = power.vBat;
	pSensors->power.current = power.iCur;
	pSensors->power.batEmpty = power.batEmpty;
	pSensors->power.exhausted = power.exhausted;

	// IMU
	pSensors->acc.linAcc[0] = spi6.imu.acc[1];
	pSensors->acc.linAcc[1] = -spi6.imu.acc[0];
	pSensors->acc.linAcc[2] = spi6.imu.acc[2];
	pSensors->acc.time = SysTimeUSec();
	pSensors->acc.updated = 1;

	pSensors->gyr.rotVel[0] = spi6.imu.gyr[1];
	pSensors->gyr.rotVel[1] = -spi6.imu.gyr[0];
	pSensors->gyr.rotVel[2] = spi6.imu.gyr[2];
	pSensors->gyr.time = SysTimeUSec();
	pSensors->gyr.updated = 1;

	// Vision - SKIPPED (filled by robot base implementation)

	// Kicker
	pSensors->kicker.level = kicker.vCap;
	pSensors->kicker.chgCurrent = kicker.iChg;
	pSensors->kicker.kickCounter = kicker.kickCounter;
	pSensors->kicker.flags = 0;
	if(KickerIsReadyToShoot())
		pSensors->kicker.flags |= KICKER_FLAGS_READY;

	if(kicker.barrierDamaged)
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
	pSensors->dribbler.hallSpeed = motors.dribbler.velocity;
	pSensors->dribbler.voltage = motors.dribbler.outVoltage;
	pSensors->dribbler.hallPos = motors.dribbler.measurementCounter;
	pSensors->dribbler.auxSpeed = motors.dribbler.velocity;
	pSensors->dribbler.current = 0;
	pSensors->dribbler.temp = 0;
	pSensors->dribbler.overheated = 0;

	// Barrier
	pSensors->ir.level[0] = kicker.vIrOff;
	pSensors->ir.level[1] = kicker.vIrOn;
	pSensors->ir.interrupted = kicker.interrupted;

	// IR Array (set default values)
	pSensors->ir.ballDetected = 0;
	pSensors->ir.estimatedBallPosition[0] = 0;
	pSensors->ir.estimatedBallPosition[1] = 0;

	// Ball Detector (set default values)
	pSensors->ball.pos[0] = NAN;
	pSensors->ball.pos[1] = NAN;
	pSensors->ball.pos[2] = NAN;
	pSensors->ball.updated = 0;
}

void RobotImplUpdateAuxData(RobotAuxData* pAux)
{
	// power
	pAux->power.cellVoltageMax = 4.20f;
	pAux->power.cellVoltageMin = 3.40f;
	pAux->power.numCells = power.batCells;

	// kicker
	pAux->kicker.maxLevel = kicker.config.maxVoltage;

	// physical
	pAux->physical.dribblerDistance = botParams.physical.dribblerDistance;
}

uint8_t RobotImplGetHardwareId()
{
	return 0xFF;
}

void RobotImplWriteLogMsg(void* pMsg)
{
	LogFileWrite(pMsg);
}

void RobotImplMotorOutput(const RobotCtrlOutput* pMotor)
{
	motors.driveMode = pMotor->ctrlMode;

	for(uint8_t i = 0; i < 4; i++)
	{
		motors.drive[i].setVelocity = pMotor->motorVel[i];
		motors.drive[i].setVoltage = pMotor->motorVoltage[i];
		motors.drive[i].setAcceleration = pMotor->motorAcc[i];
		motors.drive[i].setTorque = pMotor->motorTorque[i];
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

void RobotImplDribblerOutput(const DribblerInput* pDribbler)
{
	switch(pDribbler->mode)
	{
		case DRIBBLER_MODE_VOLTAGE:
			MotorsDribblerSetVoltage(pDribbler->voltage);
			break;
		case DRIBBLER_MODE_SPEED:
			MotorsDribblerSetVelocity(pDribbler->speed * 60.0f/(2.0f*M_PI));
			break;
		default:
			MotorsDribblerOff();
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
		case SYSTEM_MATCH_CTRL_FLAGS_LED_RED:
			LEDSet(LED_LEFT_RED | LED_RIGHT_RED);
			break;
		case SYSTEM_MATCH_CTRL_FLAGS_LED_GREEN:
			LEDSet(LED_LEFT_GREEN | LED_RIGHT_GREEN);
			break;
		case SYSTEM_MATCH_CTRL_FLAGS_LED_GOLD:
			LEDSet(LED_LEFT_RED);
			break;
		default:
			LEDSet(0);
			break;
	}
}

int16_t RobotImplExtSendPacket(PacketHeader* pHeader, const void* _pData, uint16_t dataLength)
{
	return ExtSendPacket(pHeader, _pData, dataLength);
}

void RobotImplSetEnabledDetectors(uint32_t detectors)
{
	(void)detectors;
}
