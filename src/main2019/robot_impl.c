/*
 * robot_impl.c
 *
 *  Created on: 12.01.2019
 *      Author: AndreR
 */

#include "robot/robot.h"
#include "robot/ctrl_panthera.h"
#include "robot/fusion_ekf.h"
#include "robot/network.h"
#include "struct_ids.h"

#include "hal/sys_time.h"

#include "dev/adc_kicker.h"
#include "dev/buzzer.h"
#include "dev/mag.h"
#include "dev/leds_front.h"
#include "dev/imu.h"
#include "dev/power_mon.h"
#include "dev/drib_ir.h"
#include "dev/motors.h"
#include "dev/raspberry_pi.h"

#include "util/log_file.h"

#include "misc/inventory.h"
#include "misc/variant.h"
#include "tiger_bot.h"

RobotSpecs botParams = {
	.driveTrain = {
		.motor2WheelRatio = 1.0f,
		.wheel2MotorRatio = 1.0f,
	},
	.dribbler = {
		.motor2BarRatio = 1.0f,
		.bar2MotorRatio = 1.0f,
		.barDiameter = 0.011f,
	},
	.physical = {
		.wheelRadius_m = 0.032f,
		.frontAngle_deg = 31.0f,
		.backAngle_deg = 45.0f,
		.botRadius_m = 0.079f,
		.mass_kg = 2.78f,
		.dribblerDistance_m = 0.093f,
		.dribblerWidth_m = 0.055f,
		.centerOfGravity_m = { 0, -0.005f, 0.049f },
		.massDistributionZ = 0.7f,
	},
};

static FusionEKFConfig configFusionEKF = {
	.posNoiseXY = 0.001f,
	.posNoiseW = 0.001f,
	.velNoiseXY = 0.005f,
	.visNoiseXY = 0.05f,
	.visNoiseW = 0.1f,
	.outlierMaxVelXY = 3.0f,
	.outlierMaxVelW = 3.0f,
	.trackingCoeff = 1.0f,
	.visCaptureDelay = 20,
	.fusionHorizon = 35,
	.visionTimeoutMs = 1000,
	.emaAccelT = 0.005f,
	.ballCenteringFactor = 0.1f,
	.ballCenteringVelLimit = 0.02f,
	.dribblerStrongOn = 70,
	.dribblerStrongOff = 40,
    .ballTimeoutMs = 2000,
	.activeDribblingForce_mN = 500,
};

static CtrlPantheraConfigModel configModelDirectDrive = {
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

static CtrlPantheraConfigModel configModelPlanetaryGear = {
	.fric = {
		// determined by roll out in Y direction (no rotation compensation, torque == 0), must have plateau phase
		// required data: motor set torque (XYW local output force / absolute force), XY state local velocity (absolute velocity)
		.coulombXY = 5.3f,
		.viscousXY = 0.0f,
		.coulombW = 0.2f, // determined by roll out in W direction (torque == 0), must have plateau phase
		.viscousW = 0.0f,
		.efficiencyXY = 0.54f, // roll out in Y
		.efficiencyW = 0.44f, // roll out in W
	},
	.enc = {
		.K = { 0.91f, 0.95f },
		.T = { 0.082f, 0.038f },
	}
};

static CtrlPantheraConfigCtrlDrive configCtrlDirectDrive = {
	.posXY = { 8.0f, 2.0f, 0.01f },
	.posW = { 8.0f, 8.0f, 0.01f },
	.velW = { 2.0f, 4.0f },
	.torqueVectoringStrength = 0.0f,
};

static CtrlPantheraConfigCtrlDrive configCtrlPlanetaryGear = {
	.posXY = { 8.0f, 2.0f, 0.01f },
	.posW = { 8.0f, 8.0f, 0.01f },
	.velW = { 1.0f, 4.0f },
	.torqueVectoringStrength = 1.0f,
};

static CtrlPantheraConfigCtrlDribbler configCtrlDribbler = {
	.forceLimitAngle = -45,
	.forceFromFriction = 2000,
	.forceDropRate = 10000,
	.forceMinVal_mN = 500,
	.velLimit = 8000,
	.velInc = 2000,
	.velDec = 4000,
	.achievedFractionMin = 25,
	.achievedFractionMax = 75,
};

void RobotImplGetSpecs(RobotSpecs* pSpecs)
{
	memcpy(&botParams.driveTrain.motor, &devMotors.motorParamsDrive, sizeof(MotorParams));
	memcpy(&botParams.dribbler.motor, &devMotors.motorParamsDribbler, sizeof(MotorParams));
	memcpy(pSpecs, &botParams, sizeof(RobotSpecs));

	if(variant.config.hasPlanetaryGear)
	{
		pSpecs->driveTrain.motor2WheelRatio = -7.0f / 25.0f;
		pSpecs->driveTrain.wheel2MotorRatio = -25.0f / 7.0f;
	}
}

void RobotImplInit()
{
	fusionEKFConfigDesc.cfgId = SID_CFG_FUSION_EKF_V2020;
	fusionEKFConfigDesc.version = 10;
	fusionEKFConfigDesc.pName = "v2020/ekf";

	ConfigOpenOrCreate(&fusionEKFConfigDesc, &configFusionEKF, sizeof(FusionEKFConfig), &FusionEKFConfigUpdateCallback, 0);

	ctrlPantheraConfigCtrlDribblerDesc.cfgId = SID_CFG_CTRL_PANTHERA_DRIBBLER_V2020;
	ctrlPantheraConfigCtrlDribblerDesc.version = 1;
	ctrlPantheraConfigCtrlDribblerDesc.pName = "v2020/drib";

	ConfigOpenOrCreate(&ctrlPantheraConfigCtrlDribblerDesc, &configCtrlDribbler, sizeof(CtrlPantheraConfigCtrlDribbler), 0, 0);

	float defaultPos[3];
	RobotMathGetDefaultBotPosition(network.config.botId, defaultPos);

	if(variant.config.hasPlanetaryGear)
	{
		ctrlPantheraConfigModelDesc.cfgId = SID_CFG_CTRL_PANTHERA_MODEL_V2020_PG;
		ctrlPantheraConfigModelDesc.version = 1;
		ctrlPantheraConfigModelDesc.pName = "v2020_PG/model";

		ctrlPantheraConfigCtrlDriveDesc.cfgId = SID_CFG_CTRL_PANTHERA_CTRL_V2020_PG;
		ctrlPantheraConfigCtrlDriveDesc.version = 1;
		ctrlPantheraConfigCtrlDriveDesc.pName = "v2020_PG/ctrl";

		ConfigOpenOrCreate(&ctrlPantheraConfigModelDesc, &configModelPlanetaryGear, sizeof(CtrlPantheraConfigModel), 0, 0);
		ConfigOpenOrCreate(&ctrlPantheraConfigCtrlDriveDesc, &configCtrlPlanetaryGear, sizeof(CtrlPantheraConfigCtrlDrive), 0, 0);

		CtrlPantheraInit(&configCtrlPlanetaryGear, &configCtrlDribbler, &configModelPlanetaryGear, &configFusionEKF, defaultPos);
	}
	else
	{
		ctrlPantheraConfigModelDesc.cfgId = SID_CFG_CTRL_PANTHERA_MODEL_V2020_DD;
		ctrlPantheraConfigModelDesc.version = 1;
		ctrlPantheraConfigModelDesc.pName = "v2020/model";

		ctrlPantheraConfigCtrlDriveDesc.cfgId = SID_CFG_CTRL_PANTHERA_CTRL_V2020_DD;
		ctrlPantheraConfigCtrlDriveDesc.version = 6;
		ctrlPantheraConfigCtrlDriveDesc.pName = "v2020/ctrl";

		ConfigOpenOrCreate(&ctrlPantheraConfigModelDesc, &configModelDirectDrive, sizeof(CtrlPantheraConfigModel), 0, 0);
		ConfigOpenOrCreate(&ctrlPantheraConfigCtrlDriveDesc, &configCtrlDirectDrive, sizeof(CtrlPantheraConfigCtrlDrive), 0, 0);

		CtrlPantheraInit(&configCtrlDirectDrive, &configCtrlDribbler, &configModelDirectDrive, &configFusionEKF, defaultPos);
	}
}

void RobotImplUpdateSensors(RobotSensors* pSensors)
{
	const InventoryImuCalibration* pImuCalib = InventoryGetImuCalibration();

	McuMotorMeasurement mot[5];
	for(uint8_t m = 0; m < 5; m++)
		McuMotorGet(&devMotors.mcu[m], &mot[m]);

	// motors
	memset(&pSensors->enc, 0, sizeof(pSensors->enc));
	pSensors->enc.time = SysTimeUSec();
	pSensors->motorVol.time = SysTimeUSec();

	for(uint8_t m = 0; m < 4; m++)
	{
		pSensors->enc.vel[m] = mot[m].encoderVelocity_radDs;
		pSensors->motorVol.vol[m] = mot[m].avgVoltageDQ1ms_V[1];
	}

	// motor current
	memset(&pSensors->motorCur, 0, sizeof(pSensors->motorCur));
	pSensors->motorCur.time = SysTimeUSec();
	pSensors->motorCur.cur[0] = mot[0].avgCurrentDQ1ms_A[1];
	pSensors->motorCur.cur[1] = mot[1].avgCurrentDQ1ms_A[1];
	pSensors->motorCur.cur[2] = mot[2].avgCurrentDQ1ms_A[1];
	pSensors->motorCur.cur[3] = mot[3].avgCurrentDQ1ms_A[1];

	// power
	pSensors->power.voltage = tigerBot.powerControl.vBat;
	pSensors->power.current = tigerBot.powerControl.iCur;
	pSensors->power.batEmpty = tigerBot.powerControl.state == POWER_CONTROL_STATE_BAT_EMPTY;
	pSensors->power.exhausted = tigerBot.powerControl.state == POWER_CONTROL_STATE_EXHAUSTED || tigerBot.powerControl.state == POWER_CONTROL_STATE_BAT_EMPTY;

	// IMU
	ImuICM20689Measurement imu;
	ImuICM20689Get(&devImu, &imu);

	pSensors->acc.linAcc[0] = imu.acc_mDs2[0] - pImuCalib->accBias[0] - pImuCalib->accTiltBias[0];
	pSensors->acc.linAcc[1] = imu.acc_mDs2[1] - pImuCalib->accBias[1] - pImuCalib->accTiltBias[1];
	pSensors->acc.linAcc[2] = imu.acc_mDs2[2] - pImuCalib->accBias[2];
	pSensors->acc.updated = pSensors->acc.time != imu.timestamp_us;
	pSensors->acc.time = imu.timestamp_us;

	pSensors->gyr.rotVel[0] = imu.gyr_radDs[0] - pImuCalib->gyrBias[0];
	pSensors->gyr.rotVel[1] = imu.gyr_radDs[1] - pImuCalib->gyrBias[1];
	pSensors->gyr.rotVel[2] = imu.gyr_radDs[2] - pImuCalib->gyrBias[2];
	pSensors->gyr.updated = pSensors->gyr.time != imu.timestamp_us;
	pSensors->gyr.time = imu.timestamp_us;

	// Magnetometer
	MagLIS3Measurement mag;
	MagLIS3Get(&devMag, &mag);

	pSensors->mag.strength[0] = mag.strength_uT[0] - pImuCalib->magBias[0];
	pSensors->mag.strength[1] = mag.strength_uT[1] - pImuCalib->magBias[1];
	pSensors->mag.strength[2] = mag.strength_uT[2] - pImuCalib->magBias[2];
	pSensors->mag.temp = mag.temp_degC;
	pSensors->mag.updated = pSensors->mag.time != mag.timestamp_us;
	pSensors->mag.time = mag.timestamp_us;

	// Vision - SKIPPED (filled by robot base implementation)

	// Dribbler MCU
	McuDribblerMeasurement mcuDribbler;
	McuDribblerGet(&devDribIr, &mcuDribbler);

	// Kicker
	const Kicker* pKicker = &tigerBot.kicker;

	pSensors->kicker.level = pKicker->charger.capAvgFast.value;
	pSensors->kicker.chgCurrent = 0.0f;
	pSensors->kicker.kickCounter = pKicker->kick.numKicks;
	pSensors->kicker.flags = KICKER_FLAGS_V2017;
	if(KickerIsCharged(&tigerBot.kicker))
		pSensors->kicker.flags |= KICKER_FLAGS_READY;

	if((pKicker->errorFlags & (KICKER_ERROR_IR_RX_DMG | KICKER_ERROR_IR_TX_DMG)) && mcuDribbler.isEstimatedBallPosValid == 0)
		pSensors->kicker.flags |= KICKER_FLAGS_DMG_BARRIER;

	if(pKicker->errorFlags & KICKER_ERROR_STRAIGHT_DMG)
		pSensors->kicker.flags |= KICKER_FLAGS_DMG_STRAIGHT;

	if(pKicker->errorFlags & KICKER_ERROR_CHIP_DMG)
		pSensors->kicker.flags |= KICKER_FLAGS_DMG_CHIP;

	if(pKicker->errorFlags & KICKER_ERROR_CHG_OVERHEATED)
		pSensors->kicker.flags |= KICKER_FLAGS_DMG_CHARGE;

	// Dribbler
	pSensors->dribbler.hallSpeed = mot[4].hallVelocity_radDs;
	pSensors->dribbler.auxSpeed = mot[4].encoderVelocity_radDs; // estimate from model
	pSensors->dribbler.voltage = mot[4].avgVoltageDQ1ms_V[1];
	pSensors->dribbler.hallPos = mot[4].hallPos;
	pSensors->dribbler.current = mot[4].avgCurrentDQ1ms_A[1];
	pSensors->dribbler.temp = mcuDribbler.dribblerTemp_degC;

	if(mcuDribbler.dribblerTemp_degC < 70.0f)
		pSensors->dribbler.overheated = 0;

	if(mcuDribbler.dribblerTemp_degC > 80.0f)
		pSensors->dribbler.overheated = 1;

	// Barrier
	pSensors->ir.level[0] = mcuDribbler.barrierOff_V;
	pSensors->ir.level[1] = mcuDribbler.barrierOn_V;
	pSensors->ir.interrupted = pKicker->barrier.isInterrupted;

	// IR Array
	pSensors->ir.ballDetected = mcuDribbler.isEstimatedBallPosValid;
	pSensors->ir.estimatedBallPosition[0] = mcuDribbler.estimatedBallPos_cm[0] * 10.0f; // convert cm to mm
	pSensors->ir.estimatedBallPosition[1] = mcuDribbler.estimatedBallPos_cm[1] * 10.0f; // convert cm to mm

	// Pattern identification system
	PatternIdentDetection patternIdent;
	PatternIdentGet(&tigerBot.patternIdent, &patternIdent);

	pSensors->pattern.id = patternIdent.detectedId;
	pSensors->pattern.flags = patternIdent.badBlobs;
	if(patternIdent.isSystemPresent)
		pSensors->pattern.flags |= PATTERN_IDENT_FLAGS_SYSTEM_PRESENT;

	// Robot Pi
	RobotPiUpdateSensorData(&tigerBot.robotPi, pSensors);
}

void RobotImplUpdateAuxData(RobotAuxData* pAux)
{
	// power
	pAux->power.cellVoltageMax = tigerBot.powerControl.cellMax;
	pAux->power.cellVoltageMin = tigerBot.powerControl.cellMin;
	pAux->power.numCells = tigerBot.powerControl.batCells;

	// kicker
	pAux->kicker.maxLevel = tigerBot.kicker.pConfig->maxVoltage_V;

	// physical
	pAux->physical.v2016 = 0;

	// ext
	if(SysTimeUSec() - tigerBot.robotPi.ballDetections.timestampUs < 1000000)
		pAux->ext.installed = 1;
	else
		pAux->ext.installed = 0;
}

void RobotImplUpdateNetStats(NetStats* pStats)
{
	RadioBotGetNetStats(&tigerBot.radioBot, pStats);
}

void RobotImplStateEstimation(const RobotSensors* pSensors, const RobotCtrlOutput* pLastCtrlOut, const RobotCtrlReference* pLastCtrlRef, RobotCtrlState* pState)
{
	FusionEKFUpdate(pSensors, pLastCtrlOut, pLastCtrlRef, pState);

	RobotPiSetRobotState(&tigerBot.robotPi, pState);
}

void RobotImplControl(const RobotCtrlState* pState, const SkillOutput* pInput, RobotCtrlOutput* pOutput, RobotCtrlReference* pRef)
{
	CtrlPantheraUpdate(pState, pInput, pOutput, pRef);
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
				McuMotorSetVelocity(&devMotors.mcu[motorId], pMotor->motorVel[motorId], 0.0f, 0.0f);
			}
			break;
			case MOT_CTRL_VOLTAGE:
			{
				McuMotorSetVoltageDQ(&devMotors.mcu[motorId], 0.0f, pMotor->motorVoltage[motorId]);
			}
			break;
			case MOT_CTRL_TORQUE:
			{
				McuMotorSetCurrentDQ(&devMotors.mcu[motorId], 0.0f,
						pMotor->motorTorque[motorId]/robot.specs.driveTrain.motor.Km);
			}
			break;
			case MOT_CTRL_VEL_TORQUE:
			{
				McuMotorSetVelocity(&devMotors.mcu[motorId], pMotor->motorVel[motorId], 0.0f,
						pMotor->motorTorque[motorId]/robot.specs.driveTrain.motor.Km);
			}
			break;
			default:
			{
				McuMotorSetOff(&devMotors.mcu[motorId]);
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
			KickerArmSpeed(&tigerBot.kicker, pKicker->speed, pKicker->device, 1, robot.state.ballState == BALL_STATE_ACTIVE_DRIBBLING ? robot.state.dribblerForce : 0.0f);
			break;
		case KICKER_MODE_ARM:
			KickerArmSpeed(&tigerBot.kicker, pKicker->speed, pKicker->device, 0, robot.state.ballState == BALL_STATE_ACTIVE_DRIBBLING ? robot.state.dribblerForce : 0.0f);
			break;
		case KICKER_MODE_DISARM:
			KickerDisarm(&tigerBot.kicker);
			break;
		case KICKER_MODE_ARM_TIME:
			KickerArmDuration(&tigerBot.kicker, pKicker->speed, pKicker->device, 0);
			break;
	}
}

void RobotImplDribblerOutput(const RobotCtrlOutput* pCtrlOut)
{
	switch(pCtrlOut->dribblerMode)
	{
		case DRIBBLER_MODE_VOLTAGE:
			McuMotorSetVoltageDQ(&devMotors.mcu[4], 0.0f, pCtrlOut->dribblerVoltage);
			break;
		case DRIBBLER_MODE_SPEED:
			McuMotorSetVelocity(&devMotors.mcu[4], pCtrlOut->dribblerVel, 0.0f, pCtrlOut->dribblerTorque/robot.specs.dribbler.motor.Km);
			break;
		default:
			McuMotorSetOff(&devMotors.mcu[4]);
			break;
	}
}

void RobotImplBuzzerPlay(uint16_t seqId)
{
	BuzzerPlayId(&devBuzzer, seqId);
}

void RobotImplKickerAutoCharge(uint8_t enable)
{
	if(enable)
	{
		KickerSetChargeMode(&tigerBot.kicker, KICKER_CHG_MODE_AUTO_CHARGE);
	}
	else
	{
		KickerSetChargeMode(&tigerBot.kicker, KICKER_CHG_MODE_AUTO_DISCHARGE);
	}
}

void RobotImplSetLEDs(uint8_t state)
{
	switch(state)
	{
		case SYSTEM_MATCH_CTRL_FLAGS_LED_OFF:
			LEDRGBWSet(&devLedFrontLeft, 0, 0, 0, 0.01);
			LEDRGBWSet(&devLedFrontRight, 0, 0, 0, 0.01);
			break;
		case SYSTEM_MATCH_CTRL_FLAGS_LED_RED:
			LEDRGBWSet(&devLedFrontLeft, 0.5f, 0, 0, 0);
			LEDRGBWSet(&devLedFrontRight, 0.5f, 0, 0, 0);
			break;
		case SYSTEM_MATCH_CTRL_FLAGS_LED_GREEN:
			LEDRGBWSet(&devLedFrontLeft, 0, 0.3f, 0, 0);
			LEDRGBWSet(&devLedFrontRight, 0, 0.3f, 0, 0);
			break;
		case SYSTEM_MATCH_CTRL_FLAGS_LED_BLUE:
			LEDRGBWSet(&devLedFrontLeft, 0, 0, 0.4f, 0);
			LEDRGBWSet(&devLedFrontRight, 0, 0, 0.4f, 0);
			break;
		case SYSTEM_MATCH_CTRL_FLAGS_LED_WHITE:
			LEDRGBWSet(&devLedFrontLeft, 0, 0, 0, 0.5f);
			LEDRGBWSet(&devLedFrontRight, 0, 0, 0, 0.5f);
			break;
		case SYSTEM_MATCH_CTRL_FLAGS_LED_LIGHT_BLUE:
			LEDRGBWSet(&devLedFrontLeft, 0, 0.25f, 0.25f, 0);
			LEDRGBWSet(&devLedFrontRight, 0, 0.25f, 0.25f, 0);
			break;
		case SYSTEM_MATCH_CTRL_FLAGS_LED_GOLD:
			LEDRGBWSet(&devLedFrontLeft, 0.3f, 0.15f, 0, 0);
			LEDRGBWSet(&devLedFrontRight, 0.3f, 0.15f, 0, 0);
			break;
		case SYSTEM_MATCH_CTRL_FLAGS_LED_PURPLE:
			LEDRGBWSet(&devLedFrontLeft, 0.25f, 0, 0.25f, 0);
			LEDRGBWSet(&devLedFrontRight, 0.25f, 0, 0.25f, 0);
			break;
		default:
			LEDRGBWSet(&devLedFrontLeft, 0, 0.1f, 0, 0);
			LEDRGBWSet(&devLedFrontRight, 0, 0, 0.1f, 0);
			break;
	}
}

int16_t RobotImplExtSendPacket(PacketHeader* pHeader, const void* _pData, uint16_t dataLength)
{
	return MpuExtSendPacket(&devRaspberryPi, pHeader, _pData, dataLength);
}

void RobotImplSetEnabledDetectors(uint32_t detectors)
{
	tigerBot.robotPi.enabledSteps = detectors;
}
