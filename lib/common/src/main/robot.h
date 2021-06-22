/*
 * robot.h
 *
 *  Created on: 19.07.2016
 *      Author: AndreR
 */

#pragma once

#include "log_msgs.h"
#include "commands.h"
#include "util/config.h"
#include "intercom_constants.h"
#include "util/lag_element.h"

// hardware subsystems
#define ROBOT_SYSTEM_DRIVE		0x0001
#define ROBOT_SYSTEM_DRIBBLER	0x0002
#define ROBOT_SYSTEM_KICKER		0x0004
#define ROBOT_SYSTEM_BUZZER		0x0008
#define ROBOT_SYSTEM_LEDS		0x0010

// software components
#define ROBOT_SYSTEM_STATE_EST	0x0020
#define ROBOT_SYSTEM_SKILLS		0x0040
#define ROBOT_SYSTEM_CONTROL	0x0080

#define ROBOT_SYSTEM_ALL_MASK	0xFFFF

#define ACQ_MODE_NONE				0
#define ACQ_MODE_MOTOR_MODEL		1
#define ACQ_MODE_BOT_MODEL			2
#define ACQ_MODE_DELAYS				3
#define ACQ_MODE_BOT_MODEL_V2		4

#define ROBOT_EVENT_QUEUE_SIZE 10

typedef struct _Robot
{
	mailbox_t eventQueue;
	msg_t eventQueueData[ROBOT_EVENT_QUEUE_SIZE];

	RobotSensors sensors;
	RobotCtrlState state;
	SkillOutput skillOutput;
	RobotCtrlOutput ctrlOutput;
	RobotCtrlReference ctrlRef;
	NetStats netStats;
	RobotPerformance performance;
	RobotAuxData aux;

	uint8_t feedbackStateUpdated[2];

	uint32_t lastLogTime;

	uint8_t bsOnline;
	uint8_t sumatraOnline;
	uint8_t coverPresent;

	PresenterWifiStat wifiStat;

	uint32_t enabledSystems;

	uint16_t mode;

	uint16_t dataAcquisitionMode;

	LagElementPT1 accZLag;
	float accZ;

	LagElementPT1 gyrZLag;
	float gyrZ;

	struct _ext
	{
		uint16_t active;
		uint32_t lastMatchCtrlTime;
		SystemMatchCtrl matchCtrl;
	} ext;
} Robot;

extern Robot robot;

void RobotInit();
void RobotTask(void* params);
void RobotSetIdleMode();
void RobotEnableNetwork();
void RobotTestModeEnable(uint8_t enable);
void RobotTestModeSetSystems(uint32_t enabledSystems);
void RobotTestModeStopAll();

void RobotImplInit();
void RobotImplUpdateSensors(RobotSensors* pSensors);
void RobotImplUpdateAuxData(RobotAuxData* pAux);
void RobotImplWriteLogMsg(void* pMsg);
void RobotImplMotorOutput(const RobotCtrlOutput* pMotor);
void RobotImplKickerOutput(const KickerInput* pKicker);
void RobotImplDribblerOutput(const RobotCtrlOutput* pDribbler);
void RobotImplBuzzerPlay(uint16_t seqId);
void RobotImplSetLEDs(uint8_t state);
void RobotImplKickerAutoCharge(uint8_t enable);
void RobotImplExtSendMatchFeedback(const SystemMatchFeedback* pFeedback);
uint8_t RobotImplGetHardwareId();
