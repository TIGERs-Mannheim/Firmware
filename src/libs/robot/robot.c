/*
 * robot.c
 *
 *  Created on: 19.07.2016
 *      Author: AndreR
 */

#include "robot.h"
#include "commands.h"
#include "struct_ids.h"
#include "skills.h"
#include "ctrl.h"
#include "util/sys_time.h"
#include "network.h"
#include "version.h"
#include "util/boot.h"
#include "robot/ctrl_panthera.h"
#include "util/angle_math.h"
#include "util/vector.h"

Robot robot;

#define ROBOT_MODE_IDLE			0
#define ROBOT_MODE_READY		1
#define ROBOT_MODE_LOW_POWER	2
#define ROBOT_MODE_TEST			3

#define ROBOT_EVENT_BAT_EMPTY		0
#define ROBOT_EVENT_VISION_RECEIVED	1
#define ROBOT_EVENT_ENTER_TEST		2
#define ROBOT_EVENT_EXIT_TEST		3
#define ROBOT_EVENT_RESET			4
#define ROBOT_EVENT_NET_ENABLE		5

static void clearUpdateFlags();
static void doLogging();
static void sendMatchFeedback();
static void parseMatchCtrl();
static void connectionCheck();
static void processEvent(uint32_t event);
static void stateWatchdog();
static void handleDataAcquisitionMode();
static void detectCover();

static void initLogFields()
{
	memset(&robot.sensors, 0, sizeof(robot.sensors));
	memset(&robot.state, 0, sizeof(robot.state));
	memset(&robot.skillOutput, 0, sizeof(robot.skillOutput));
	memset(&robot.ctrlOutput, 0, sizeof(robot.ctrlOutput));
	memset(&robot.ctrlRef, 0, sizeof(robot.ctrlRef));
	memset(&robot.netStats, 0, sizeof(robot.netStats));
	memset(&robot.performance, 0, sizeof(robot.performance));

	robot.sensors.header.type = SID_SENSORS;
	robot.sensors.header.length = sizeof(RobotSensors);
	robot.state.header.type = SID_CTRL_STATE;
	robot.state.header.length = sizeof(RobotCtrlState);
	robot.skillOutput.header.type = SID_SKILL_OUTPUT;
	robot.skillOutput.header.length = sizeof(SkillOutput);
	robot.ctrlOutput.header.type = SID_CTRL_OUTPUT;
	robot.ctrlOutput.header.length = sizeof(RobotCtrlOutput);
	robot.ctrlRef.header.type = SID_CTRL_REF;
	robot.ctrlRef.header.length = sizeof(RobotCtrlReference);
	robot.netStats.header.type = SID_NETSTATS;
	robot.netStats.header.length = sizeof(NetStats);
	robot.performance.header.type = SID_ROBOT_PERFORMANCE;
	robot.performance.header.length = sizeof(RobotPerformance);
}

void RobotInit()
{
	robot.mode = ROBOT_MODE_IDLE;
	robot.enabledSystems = ROBOT_SYSTEM_BUZZER | ROBOT_SYSTEM_LEDS | ROBOT_SYSTEM_STATE_EST;

	chMBObjectInit(&robot.eventQueue, robot.eventQueueData, ROBOT_EVENT_QUEUE_SIZE);

	LagElementPT1Init(&robot.accZLag, 1, 0.5f, 0.001f);
	robot.accZLag.state[0] = 20.0f;
	robot.accZLag.state[1] = 20.0f;
	robot.accZLag.state[2] = 20.0f;
	robot.accZLag.state[3] = 20.0f;

	LagElementPT1Init(&robot.gyrZLag, 1, 0.5f, 0.001f);

	robot.ext.lastMatchCtrlTime = SysTimeUSec()-5000000;

	initLogFields();

	SkillsInit();
	CtrlInit();
}

void RobotTask(void* params)
{
	(void)params;

	chRegSetThreadName("Robot");

	RobotImplInit();

	uint32_t event;
	systime_t prev = chVTGetSystemTimeX();
	systime_t next = prev+US2ST(1000);

	while(1)
	{
		uint32_t tStart = SysTimeCycleCounter();
		robot.sensors.header.timestamp = SysTimeUSec();
		robot.performance.header.timestamp = SysTimeUSec();

		// fetch updated sensor data from all subsystems
		RobotImplUpdateSensors(&robot.sensors);
		RobotImplUpdateAuxData(&robot.aux);

		// Check if this robot has a working cover on it
		detectCover();

		// check connection to BS and command sender
		connectionCheck();

		// get skill data, parse flags, read vision pos (if available)
		parseMatchCtrl();

		// Check if a new controller is selected
		CtrlCheckControllerChange(&robot.ctrlOutput);

		uint32_t tInput = SysTimeCycleCounter();
		robot.performance.inputTime = SysTimeCycleCounterDiffUSec(tStart, tInput);

		// run state estimation with sensor input
		if(robot.enabledSystems & ROBOT_SYSTEM_STATE_EST)
			CtrlStateEstimation(&robot.sensors, &robot.ctrlOutput, &robot.state);

		uint32_t tEstimation = SysTimeCycleCounter();
		robot.state.header.timestamp = SysTimeUSec();
		robot.performance.estimationTime = SysTimeCycleCounterDiffUSec(tInput, tEstimation);

		// execute skill system with updated state
		if(robot.enabledSystems & ROBOT_SYSTEM_SKILLS)
			SkillsExecute(&robot.sensors, &robot.aux, &robot.state, &robot.ctrlOutput, &robot.ctrlRef, &robot.skillOutput);

		uint32_t tSkills = SysTimeCycleCounter();
		robot.skillOutput.header.timestamp = SysTimeUSec();
		robot.performance.skillTime = SysTimeCycleCounterDiffUSec(tEstimation, tSkills);

		// run control algorithms for driving
		if(robot.enabledSystems & ROBOT_SYSTEM_CONTROL)
			CtrlControl(&robot.state, &robot.skillOutput.drive, &robot.ctrlOutput, &robot.ctrlRef);

		uint32_t tCtrl = SysTimeCycleCounter();
		robot.ctrlOutput.header.timestamp = SysTimeUSec();
		robot.ctrlRef.header.timestamp = SysTimeUSec();
		robot.performance.controlTime = SysTimeCycleCounterDiffUSec(tSkills, tCtrl);

		// Apply all actuator outputs
		if(robot.enabledSystems & ROBOT_SYSTEM_DRIVE)
			RobotImplMotorOutput(&robot.ctrlOutput);

		if(robot.enabledSystems & ROBOT_SYSTEM_KICKER)
			RobotImplKickerOutput(&robot.skillOutput.kicker);

		if(robot.enabledSystems & ROBOT_SYSTEM_DRIBBLER)
			RobotImplDribblerOutput(&robot.skillOutput.dribbler);

		uint32_t tOut = SysTimeCycleCounter();
		robot.performance.outputTime = SysTimeCycleCounterDiffUSec(tCtrl, tOut);

		// do sensor/state/output logging
		doLogging();

		// send high sample rate acquisition mode data
		handleDataAcquisitionMode();

		// send match feedback to Sumatra
		sendMatchFeedback();

		// battery check
		if(robot.sensors.power.batEmpty)
			chMBPost(&robot.eventQueue, ROBOT_EVENT_BAT_EMPTY, TIME_IMMEDIATE);

		// state watchdog (checks Z Acc)
		stateWatchdog();

		// clear all updated flags in sensor data
		clearUpdateFlags();

		// process all events
		while(chMBFetch(&robot.eventQueue, (msg_t*)&event, TIME_IMMEDIATE) == MSG_OK)
		{
			processEvent(event);
		}

		uint32_t tMisc = SysTimeCycleCounter();
		robot.performance.miscTime = SysTimeCycleCounterDiffUSec(tOut, tMisc);

		// wait until next loop
		prev = chThdSleepUntilWindowed(prev, next);
		next += US2ST(1000);
	}
}

void RobotSetIdleMode()
{
	chMBPost(&robot.eventQueue, ROBOT_EVENT_RESET, TIME_IMMEDIATE);
}

void RobotTestModeEnable(uint8_t enable)
{
	if(enable)
		chMBPost(&robot.eventQueue, ROBOT_EVENT_ENTER_TEST, TIME_INFINITE);
	else
		chMBPost(&robot.eventQueue, ROBOT_EVENT_EXIT_TEST, TIME_INFINITE);

	chThdSleepMilliseconds(10);
}

void RobotTestModeSetSystems(uint32_t enabledSystems)
{
	if(robot.mode == ROBOT_MODE_TEST)
		robot.enabledSystems = enabledSystems;

	chThdSleepMilliseconds(10);
}

static void stop()
{
	initLogFields();

	CtrlCheckControllerChange(&robot.ctrlOutput);

	RobotImplMotorOutput(&robot.ctrlOutput);
	RobotImplKickerOutput(&robot.skillOutput.kicker);
	RobotImplDribblerOutput(&robot.skillOutput.dribbler);
}

void RobotTestModeStopAll()
{
	if(robot.mode != ROBOT_MODE_TEST)
		return;

	RobotTestModeSetSystems(0);

	stop();

	RobotImplBuzzerPlay(0);
	RobotImplKickerAutoCharge(0);
	RobotImplSetLEDs(0);
}

void RobotEnableNetwork()
{
	chMBPost(&robot.eventQueue, ROBOT_EVENT_NET_ENABLE, TIME_IMMEDIATE);
}

void RobotSendVersionInfo()
{
	PacketHeader header;
	header.section = SECTION_SYSTEM;
	header.cmd = CMD_SYSTEM_VERSION;

	SystemVersion version;
	version.version = VersionGet();
	version.gitRef = GIT_SHA1_SHORT;

	NetworkSendPacketReliable(&header, &version, sizeof(SystemVersion));
}

static void processEvent(uint32_t event)
{
	if(robot.mode == ROBOT_MODE_LOW_POWER)
		return;

	if(event == ROBOT_EVENT_NET_ENABLE)
	{
		network.linkDisabled = 0;
	}

	if(event == ROBOT_EVENT_BAT_EMPTY)
	{
		if(robot.mode == ROBOT_MODE_READY)
			robot.enabledSystems = ROBOT_SYSTEM_STATE_EST;
		else
			robot.enabledSystems = 0;

		stop();

		RobotImplKickerAutoCharge(0);
		RobotImplBuzzerPlay(BUZZ_BEEP_FAST);

		robot.mode = ROBOT_MODE_LOW_POWER;
	}

	if(event == ROBOT_EVENT_ENTER_TEST)
	{
		robot.mode = ROBOT_MODE_TEST;
	}

	if(robot.mode == ROBOT_MODE_TEST && event == ROBOT_EVENT_EXIT_TEST)
	{
		robot.mode = ROBOT_MODE_IDLE;

		robot.enabledSystems = ROBOT_SYSTEM_BUZZER | ROBOT_SYSTEM_LEDS | ROBOT_SYSTEM_STATE_EST;

		ctrl.forceControllerReload = 1;

		stop();
	}

	if(robot.mode == ROBOT_MODE_IDLE && event == ROBOT_EVENT_VISION_RECEIVED)
	{
		robot.mode = ROBOT_MODE_READY;

		robot.enabledSystems = ROBOT_SYSTEM_ALL_MASK;
	}

	if(event == ROBOT_EVENT_RESET)
	{
		robot.mode = ROBOT_MODE_IDLE;

		robot.enabledSystems = ROBOT_SYSTEM_BUZZER | ROBOT_SYSTEM_LEDS | ROBOT_SYSTEM_STATE_EST;

		ctrl.forceControllerReload = 1;

		stop();
	}
}

static void doLogging()
{
	RobotImplWriteLogMsg(&robot.sensors);
	RobotImplWriteLogMsg(&robot.state);
	RobotImplWriteLogMsg(&robot.skillOutput);
	RobotImplWriteLogMsg(&robot.ctrlOutput);
	RobotImplWriteLogMsg(&robot.ctrlRef);
	RobotImplWriteLogMsg(&robot.performance);

	if(ctrl.pInstance && ctrl.pInstance->pLog)
		RobotImplWriteLogMsg(ctrl.pInstance->pLog);

	static uint8_t netStatCnt = 0;
	if(netStatCnt == 100)
	{
		netStatCnt = 0;
		RobotImplWriteLogMsg(&robot.netStats);
	}
	++netStatCnt;
}

static uint16_t gatherFeatures()
{
	uint16_t features = 0;

	if(botParams.v2016)
		features |= SYSTEM_MATCH_FEEDBACK_FEATURE_V2016;

	if(botParams.extInstalled)
		features |= SYSTEM_MATCH_FEEDBACK_FEATURE_EXT_BOARD;

	if((robot.sensors.kicker.flags & KICKER_FLAGS_DMG_BARRIER) == 0)
		features |= SYSTEM_MATCH_FEEDBACK_FEATURE_BARRIER;

	if(robot.sensors.kicker.flags & KICKER_FLAGS_V2017)
		features |= SYSTEM_MATCH_FEEDBACK_FEATURE_KICKER_V2017;

	if(robot.sensors.power.exhausted == 0)
		features |= SYSTEM_MATCH_FEEDBACK_FEATURE_ENERGETIC;

	if(robot.sensors.power.batEmpty == 1)
		return features;

	const uint32_t moveSystems = ROBOT_SYSTEM_SKILLS | ROBOT_SYSTEM_DRIVE | ROBOT_SYSTEM_CONTROL;
	if((robot.enabledSystems & moveSystems) == moveSystems)
		features |= SYSTEM_MATCH_FEEDBACK_FEATURE_MOVE;

	const uint32_t dribbleSystems = ROBOT_SYSTEM_SKILLS | ROBOT_SYSTEM_DRIBBLER;
	if((robot.enabledSystems & dribbleSystems) == dribbleSystems && robot.sensors.dribbler.overheated == 0)
		features |= SYSTEM_MATCH_FEEDBACK_FEATURE_DRIBBLE;

	const uint32_t kickSystems = ROBOT_SYSTEM_SKILLS | ROBOT_SYSTEM_KICKER;
	if((robot.enabledSystems & kickSystems) == kickSystems)
	{
		if((robot.sensors.kicker.flags & KICKER_FLAGS_DMG_STRAIGHT) == 0)
			features |= SYSTEM_MATCH_FEEDBACK_FEATURE_STRAIGHT;

		if((robot.sensors.kicker.flags & KICKER_FLAGS_DMG_CHIP) == 0)
			features |= SYSTEM_MATCH_FEEDBACK_FEATURE_CHIP;

		if((robot.sensors.kicker.flags & KICKER_FLAGS_DMG_CHARGE) == 0)
			features |= SYSTEM_MATCH_FEEDBACK_FEATURE_CHARGE;
	}

	if(	(robot.sensors.pattern.flags & PATTERN_IDENT_FLAGS_SYSTEM_PRESENT) &&
		((robot.sensors.pattern.flags & PATTERN_IDENT_FLAGS_BAD_BLOB_MASK) == 0))
	{
		features |= SYSTEM_MATCH_FEEDBACK_FEATURE_COVER;
	}

	return features;
}

static void sendMatchFeedback()
{
	if(robot.state.posUpdated)
		robot.feedbackStateUpdated[0] = 1;
	if(robot.state.velUpdated)
		robot.feedbackStateUpdated[1] = 1;

	// check if there are other packets to be send, if not => send a feedback
	uint8_t txActive = network.queue.txFifo.numPackets > 0 || network.queue.txBufUsed > 0;

	// also use a timeout of 100ms to send a feedback at at least 10Hz
	if(txActive && (SysTimeUSec() - robot.lastLogTime) < 100000)
		return;

	robot.lastLogTime = SysTimeUSec();

	SystemMatchFeedback feedback;
	memset(&feedback, 0, sizeof(SystemMatchFeedback));

	const float maxBatVoltage = robot.aux.power.cellVoltageMax*robot.aux.power.numCells;
	const float minBatVoltage = robot.aux.power.cellVoltageMin*robot.aux.power.numCells;

	feedback.batteryLevel = (uint8_t)(robot.sensors.power.voltage*10.0f + 0.5f);
	feedback.batteryPercent = (uint8_t)fmaxf(0, (robot.sensors.power.voltage - minBatVoltage)/(maxBatVoltage - minBatVoltage)*255.0f);
	feedback.kickerLevel = robot.sensors.kicker.level;
	feedback.kickerMax = robot.aux.kicker.maxLevel;

	uint32_t dribSpeed = fmaxf(0, (robot.state.dribblerVel * 60.0f/(2.0f*M_PI) + 250.0f) * 0.002f);
	uint32_t dribCur = fmaxf(0, (robot.state.dribblerCur + 0.125f) * 4.0f);

	feedback.dribblerState = (dribSpeed << 2) | (dribCur & 0x03);
	feedback.flags = (robot.sensors.ir.interrupted << 7) | ((dribCur >> 2) & 0x0F) | ((robot.sensors.kicker.kickCounter & 0x01) << 4);

	// low 30 med 55 high 80 overheated
	if(robot.sensors.dribbler.overheated)
		feedback.flags |= (3 << 5);
	else if(robot.sensors.dribbler.temp < 30.0f)
		feedback.flags |= (0 << 5);
	else if(robot.sensors.dribbler.temp < 55.0f)
		feedback.flags |= (1 << 5);
	else
		feedback.flags |= (2 << 5);
	feedback.hardwareId = RobotImplGetHardwareId();
	feedback.features = gatherFeatures() | (robot.mode << 12);

	// project reported position and velocity one BS interval ahead
	float dt = fmaxf(0.025f, network.avgRxPeriod.value) + 0.005f;

	if(robot.feedbackStateUpdated[0])
	{
		robot.feedbackStateUpdated[0] = 0;

		float futurePos[2];
		futurePos[0] = robot.state.pos[0] + dt*robot.state.vel[0] + 0.5f*dt*dt*robot.state.acc[0];
		futurePos[1] = robot.state.pos[1] + dt*robot.state.vel[1] + 0.5f*dt*dt*robot.state.acc[1];

		feedback.curPosition[0] = (int16_t)(futurePos[0]*1000.0f);
		feedback.curPosition[1] = (int16_t)(futurePos[1]*1000.0f);
		feedback.curPosition[2] = (int16_t)(AngleNormalize(robot.state.pos[2])*1000.0f);
	}
	else
	{
		feedback.curPosition[0] = SYSTEM_MATCH_FEEDBACK_UNUSED_FIELD;
		feedback.curPosition[1] = SYSTEM_MATCH_FEEDBACK_UNUSED_FIELD;
		feedback.curPosition[2] = SYSTEM_MATCH_FEEDBACK_UNUSED_FIELD;
	}

	if(robot.feedbackStateUpdated[1])
	{
		robot.feedbackStateUpdated[1] = 0;

		float futureVel[2];
		futureVel[0] = robot.state.vel[0] + dt*robot.state.acc[0];
		futureVel[1] = robot.state.vel[1] + dt*robot.state.acc[1];

		feedback.curVelocity[0] = (int16_t)(futureVel[0]*1000.0f);
		feedback.curVelocity[1] = (int16_t)(futureVel[1]*1000.0f);
		feedback.curVelocity[2] = (int16_t)(robot.state.vel[2]*1000.0f);
	}
	else
	{
		feedback.curVelocity[0] = SYSTEM_MATCH_FEEDBACK_UNUSED_FIELD;
		feedback.curVelocity[1] = SYSTEM_MATCH_FEEDBACK_UNUSED_FIELD;
		feedback.curVelocity[2] = SYSTEM_MATCH_FEEDBACK_UNUSED_FIELD;
	}

	if(robot.state.ballIrState == 2)
	{
		float ballPosGlobal[2];
		CtrlUtilTurnLocal2Global(robot.state.pos[2], robot.state.ballIrPos[0]*0.001f, robot.state.ballIrPos[1]*0.001f + robot.aux.physical.dribblerDistance, ballPosGlobal, ballPosGlobal+1);
		feedback.ballPosition[0] = (int16_t)(ballPosGlobal[0] + robot.state.pos[0] * 1000.0f);
		feedback.ballPosition[1] = (int16_t)(ballPosGlobal[1] + robot.state.pos[1] * 1000.0f);
		feedback.ballPosAge = 0;
	}
	else
	{
		uint32_t ballPosAgeMs = (SysTimeUSec() - robot.sensors.ball.time + 500)/1000;
		if(ballPosAgeMs > 255 || isnan(robot.sensors.ball.pos[0]) || isnan(robot.sensors.ball.pos[1]))
		{
			ballPosAgeMs = 255;
			feedback.ballPosition[0] = SYSTEM_MATCH_FEEDBACK_UNUSED_FIELD;
			feedback.ballPosition[1] = SYSTEM_MATCH_FEEDBACK_UNUSED_FIELD;
		}
		else
		{
			feedback.ballPosition[0] = (int16_t)(robot.sensors.ball.pos[0]*1000.0f);
			feedback.ballPosition[1] = (int16_t)(robot.sensors.ball.pos[1]*1000.0f);
		}

		feedback.ballPosAge = ballPosAgeMs;
	}

	PacketHeader header;
	header.section = SECTION_SYSTEM;
	header.cmd = CMD_SYSTEM_MATCH_FEEDBACK;
	NetworkSendPacket(&header, (uint8_t*) &feedback, sizeof(SystemMatchFeedback));
}

static void clearUpdateFlags()
{
	robot.sensors.gyr.updated = 0;
	robot.sensors.acc.updated = 0;
	robot.sensors.vision.updated = 0;
	robot.sensors.mag.updated = 0;
	robot.sensors.ball.updated = 0;
	robot.state.posUpdated = 0;
	robot.state.velUpdated = 0;
}

static void detectCover()
{
	uint8_t coverPresent = 0;
	if(robot.sensors.pattern.flags & PATTERN_IDENT_FLAGS_SYSTEM_PRESENT)
	{
		if((robot.sensors.pattern.flags & PATTERN_IDENT_FLAGS_BAD_BLOB_MASK) == 0)
			coverPresent = 1;
	}
	else
	{
		coverPresent = 1;
	}

	robot.coverPresent = coverPresent;

	if(coverPresent && network.linkDisabled)
		chMBPost(&robot.eventQueue, ROBOT_EVENT_NET_ENABLE, TIME_IMMEDIATE);
}

static void parseMatchCtrl()
{
	static uint8_t lastSong;

	// prefer match ctrl cmd from primary (nRF24) interface
	uint32_t matchCtrlTime = network.matchCtrlTime;
	SystemMatchCtrl* pCtrl = &network.lastMatchCtrlCmd;

	// only take external command interface if sumatra is offline
	if(!robot.sumatraOnline && robot.ext.active)
	{
		pCtrl = &robot.ext.matchCtrl;
		matchCtrlTime = robot.ext.lastMatchCtrlTime;
	}

	if(robot.sensors.vision.time != matchCtrlTime)
	{
		if(pCtrl->curPosition[0] != SYSTEM_MATCH_CTRL_UNUSED_FIELD)
		{
			uint32_t delay = pCtrl->posDelay;
			delay = (delay*25000)/100;	// delay now in [us]

			robot.sensors.vision.pos[0] = pCtrl->curPosition[0]*0.001f;
			robot.sensors.vision.pos[1] = pCtrl->curPosition[1]*0.001f;
			robot.sensors.vision.pos[2] = pCtrl->curPosition[2]*0.001f;
			robot.sensors.vision.time = matchCtrlTime;
			robot.sensors.vision.delay = delay;
			robot.sensors.vision.camId = pCtrl->camId & 0x7F;
			robot.sensors.vision.updated = 1;
			robot.sensors.vision.noVision = 0;

			if(robot.coverPresent)
				chMBPost(&robot.eventQueue, ROBOT_EVENT_VISION_RECEIVED, TIME_IMMEDIATE);
		}

		if(pCtrl->camId & SYSTEM_MATCH_CTRL_CAM_ID_FLAG_NO_VISION)
		{
			robot.sensors.vision.time = matchCtrlTime;
			robot.sensors.vision.updated = 0;
			robot.sensors.vision.noVision = 1;

			if(robot.coverPresent)
				chMBPost(&robot.eventQueue, ROBOT_EVENT_VISION_RECEIVED, TIME_IMMEDIATE);
		}
	}

	if(robot.enabledSystems & ROBOT_SYSTEM_KICKER)
	{
		if(pCtrl->flags & SYSTEM_MATCH_CTRL_FLAGS_KICKER_AUTOCHG)
			RobotImplKickerAutoCharge(1);
		else
			RobotImplKickerAutoCharge(0);
	}

	uint8_t song = pCtrl->flags & SYSTEM_MATCH_CTRL_FLAGS_SONG_MASK;

	if(song != lastSong && (robot.enabledSystems & ROBOT_SYSTEM_BUZZER))
		RobotImplBuzzerPlay(song);

	lastSong = song;

	uint8_t leds = (pCtrl->flags & SYSTEM_MATCH_CTRL_FLAGS_LED_MASK) >> 4;
	if(robot.enabledSystems & ROBOT_SYSTEM_LEDS)
		RobotImplSetLEDs(leds);

	if(pCtrl->flags & SYSTEM_MATCH_CTRL_FLAGS_STRICT_VEL_LIMIT)
		robot.skillOutput.drive.limits.strictVelLimit = 1;
	else
		robot.skillOutput.drive.limits.strictVelLimit = 0;

	if(robot.enabledSystems & ROBOT_SYSTEM_SKILLS)
	{
		if(robot.sumatraOnline == 0 && robot.ext.active == 0)
			SkillsSetInput(0, 0);
		else
			SkillsSetInput(pCtrl->skillId, pCtrl->skillData);
	}
}

static void connectionCheck()
{
	if(network.avgRxPeriod.value > 0.08f && robot.bsOnline)
	{
		robot.bsOnline = 0;
	}

	if(network.avgRxPeriod.value < 0.08f && !robot.bsOnline)
	{
		robot.bsOnline = 1;
	}

	uint32_t matchCtrlTime = network.matchCtrlTime;
	uint32_t now = SysTimeUSec();

	if((int32_t)(now - matchCtrlTime) > 1000000)
	{
		network.matchCtrlTime = now - 10000000;

		if(robot.sumatraOnline)
		{
			robot.sumatraOnline = 0;

			if(robot.enabledSystems & ROBOT_SYSTEM_BUZZER)
				RobotImplBuzzerPlay(BUZZ_DOWN50);
		}
	}
	else
	{
		if(robot.sumatraOnline == 0)
		{
			robot.sumatraOnline = 1;

			RobotSendVersionInfo();

			if(robot.enabledSystems & ROBOT_SYSTEM_BUZZER)
				RobotImplBuzzerPlay(BUZZ_UP50);
		}
	}

	uint32_t extCtrlTime = robot.ext.lastMatchCtrlTime;

	if((int32_t)(now - extCtrlTime) > 1000000)
	{
		robot.ext.lastMatchCtrlTime =  now - 10000000;

		if(robot.ext.active)
		{
			robot.ext.active = 0;

			if(robot.enabledSystems & ROBOT_SYSTEM_BUZZER)
				RobotImplBuzzerPlay(BUZZ_DOWN50);
		}
	}
	else
	{
		if(robot.ext.active == 0)
		{
			robot.ext.active = 1;

			if(robot.enabledSystems & ROBOT_SYSTEM_BUZZER)
				RobotImplBuzzerPlay(BUZZ_UP50);
		}
	}

	robot.wifiStat.linkDisabled = network.linkDisabled;
	robot.wifiStat.bsOnline = robot.bsOnline;
	robot.wifiStat.sumatraOnline = robot.sumatraOnline;
	robot.wifiStat.updateFreq = 1.0f/network.avgRxPeriod.value;
	robot.wifiStat.visionDelay = robot.sensors.vision.delay;
	robot.wifiStat.rssi = network.avgRssi.value;

	robot.netStats.header.timestamp = SysTimeUSec();
	robot.netStats.rxPeriod = network.avgRxPeriod.value;
	robot.netStats.rssi = network.avgRssi.value;
	robot.netStats.rxPackets = network.queue.stats.rfIO.rxPackets;
	robot.netStats.rxBytes = network.queue.stats.rfIO.rxBytes;
	robot.netStats.txPackets = network.queue.stats.rfIO.txPackets;
	robot.netStats.txBytes = network.queue.stats.rfIO.txBytes;
	robot.netStats.rxPacketsLost = network.queue.stats.rfFeedback.rxPacketsLost;
	robot.netStats.rxHLPacketsLost = network.queue.stats.queueFeedback.rxPacketsLost;
	robot.netStats.txHLPacketsLost = network.queue.stats.queueFeedback.txPacketsLost;
}


static void stateWatchdog()
{
	// robot tipped over watchdog
	if(robot.sensors.acc.updated)
		robot.accZ = LagElementPT1Process(&robot.accZLag, robot.sensors.acc.linAcc[2]);

	if(robot.accZ < 3.0f)
		chMBPost(&robot.eventQueue, ROBOT_EVENT_RESET, TIME_IMMEDIATE);

	// rotating too fast, too long watchdog
	const float ROT_VEL_Z_LIMIT = 30.0f; // sensor saturation @ 34.89rad/s

	if(robot.sensors.gyr.updated)
	{
		robot.gyrZ = LagElementPT1Process(&robot.gyrZLag, robot.sensors.gyr.rotVel[2]);

		if(fabsf(robot.sensors.gyr.rotVel[2]) > ROT_VEL_Z_LIMIT)
			RobotImplBuzzerPlay(BUZZ_DOWN100);
	}

	if(fabsf(robot.gyrZ) > ROT_VEL_Z_LIMIT)
		chMBPost(&robot.eventQueue, ROBOT_EVENT_RESET, TIME_IMMEDIATE);

	// broken state estimation watchdog
	int posNaN = isnanf(robot.state.pos[0]) || isnanf(robot.state.pos[1]) || isnanf(robot.state.pos[2]);
	int velNaN = isnanf(robot.state.vel[0]) || isnanf(robot.state.vel[1]) || isnanf(robot.state.vel[2]);
	int accNaN = isnanf(robot.state.acc[0]) || isnanf(robot.state.acc[1]) || isnanf(robot.state.acc[2]);

	if(posNaN || velNaN || accNaN)
	{
		chMBPost(&robot.eventQueue, ROBOT_EVENT_RESET, TIME_IMMEDIATE);
	}
}

static void handleDataAcquisitionMode()
{
	// check if there are other packets to be send, if not => send a feedback
	uint8_t txActive = network.queue.txFifo.numPackets > 0 || network.queue.txBufUsed > 0;
	if(txActive)
		return;

	switch(robot.dataAcquisitionMode)
	{
		case ACQ_MODE_MOTOR_MODEL:
		{
			DataAcqMotorModel mm;
			mm.timestamp = robot.sensors.motorVol.time;
			for(uint8_t i = 0; i < 4; i++)
			{
				mm.motVol[i] = (int16_t)(robot.sensors.motorVol.vol[i]*1000.0f);
				mm.motVel[i] = (int16_t)(robot.sensors.enc.vel[i]*40.0f);
			}

			PacketHeader header;
			header.section = SECTION_DATA_ACQ;
			header.cmd = CMD_DATA_ACQ_MOTOR_MODEL;

			NetworkSendPacket(&header, &mm, sizeof(DataAcqMotorModel));
		}
		break;
		case ACQ_MODE_BOT_MODEL:
		{
			DataAcqBotModel bm;
			bm.timestamp = robot.ctrlOutput.header.timestamp;

			float outVel[3];
			CtrlUtilMotorVelToLocalVel(robot.ctrlOutput.motorVel, outVel);

			bm.outVel[0] = (int16_t)(outVel[0]*1000.0f);
			bm.outVel[1] = (int16_t)(outVel[1]*1000.0f);
			bm.outVel[2] = (int16_t)(outVel[2]*100.0f);

			for(uint8_t i = 0; i < 3; i++)
				bm.visPos[i] = (int16_t)(robot.sensors.vision.pos[i]*1000.0f);

			bm.visionTime = robot.sensors.vision.time - robot.sensors.vision.delay;

			PacketHeader header;
			header.section = SECTION_DATA_ACQ;
			header.cmd = CMD_DATA_ACQ_BOT_MODEL;

			NetworkSendPacket(&header, &bm, sizeof(DataAcqBotModel));
		}
		break;
		case ACQ_MODE_DELAYS:
		{
			DataAcqDelays de;
			de.timestamp = robot.sensors.header.timestamp;
			de.visPosW = (int16_t)(robot.sensors.vision.pos[2]*1000.0f);
			de.visionTime = robot.sensors.vision.time - robot.sensors.vision.delay;

			float outVel[3];
			CtrlUtilMotorVelToLocalVel(robot.ctrlOutput.motorVel, outVel);

			de.outVelW = (int16_t)(outVel[2]*1000.0f);
			de.gyrVel = (int16_t)(robot.sensors.gyr.rotVel[2]*1000.0f);

			PacketHeader header;
			header.section = SECTION_DATA_ACQ;
			header.cmd = CMD_DATA_ACQ_DELAYS;

			NetworkSendPacket(&header, &de, sizeof(DataAcqDelays));
		}
		break;
		case ACQ_MODE_BOT_MODEL_V2:
		{
			DataAcqBotModelV2 bm2;

			bm2.timestamp = robot.ctrlOutput.header.timestamp;

			float encVel[3];
			CtrlUtilMotorVelToLocalVel(robot.sensors.enc.vel, encVel);

			bm2.encVel[0] = (int16_t)(encVel[0]*1000.0f);
			bm2.encVel[1] = (int16_t)(encVel[1]*1000.0f);
			bm2.encVel[2] = (int16_t)(encVel[2]*100.0f);

			float localStateVel[2];
			CtrlUtilTurnGlobal2Local(robot.state.pos[2], robot.state.vel[0], robot.state.vel[1], localStateVel, localStateVel+1);

			bm2.stateVel[0] = (int16_t)(localStateVel[0]*1000.0f);
			bm2.stateVel[1] = (int16_t)(localStateVel[1]*1000.0f);
			bm2.stateVel[2] = (int16_t)(robot.state.vel[2]*100.0f);

			float motorTorque[4];
			for(uint8_t i = 0; i < 4; i++)
				motorTorque[i] = robot.sensors.motorCur.cur[i] * botParams.driveTrain.motor.Km;

			float localForce[3];
			CtrlUtilMotorTorqueToLocalForce(motorTorque, localForce);

			bm2.outForce[0] = (int16_t)(localForce[0]*100.0f);
			bm2.outForce[1] = (int16_t)(localForce[1]*100.0f);
			bm2.outForce[2] = (int16_t)(localForce[2]*1000.0f);

			uint16_t effXY = (uint16_t)(ctrlPanthera.pConfigModel->fric.efficiencyXY*4096.0f);
			uint16_t effW = (uint16_t)(ctrlPanthera.pConfigModel->fric.efficiencyW*4096.0f);

			bm2.efficiency[0] = effXY & 0xFF;
			bm2.efficiency[1] = ((effXY & 0xF00) >> 8) | ((effW & 0x0F) << 4);
			bm2.efficiency[2] = (effW & 0xFF0) >> 4;

			uint8_t modeXY = 0;

			if(robot.skillOutput.drive.modeXY == DRIVE_MODE_LOCAL_FORCE)
			{
				modeXY = 1;
			}
			else if(robot.skillOutput.drive.modeXY == DRIVE_MODE_LOCAL_VEL ||
					robot.skillOutput.drive.modeXY == DRIVE_MODE_GLOBAL_POS ||
					robot.skillOutput.drive.modeXY == DRIVE_MODE_GLOBAL_POS_ASYNC)
			{
				const float* pTrajVel = robot.ctrlRef.trajVelLocal;
				const float* pTrajAcc = robot.ctrlRef.trajAccLocal;
				const float trajVelAbs = sqrtf(pTrajVel[0]*pTrajVel[0] + pTrajVel[1]*pTrajVel[1]);
				const float trajAccAbs = sqrtf(pTrajAcc[0]*pTrajAcc[0] + pTrajAcc[1]*pTrajAcc[1]);

				if(trajAccAbs < 1e-3f)
				{
					if(trajVelAbs < 1e-3f)
					{
						// stopped
						modeXY = 2;
					}
					else
					{
						// plateau
						modeXY = 3;
					}
				}
				else
				{
					float accDotVel;
					arm_dot_prod_f32(pTrajAcc, pTrajVel, 2, &accDotVel);
					if(accDotVel > 0)
					{
						// accelerating
						modeXY = 4;
					}
					else
					{
						// decelerating
						modeXY = 5;
					}
				}
			}

			uint8_t modeW = 0;

			if(robot.skillOutput.drive.modeW == DRIVE_MODE_LOCAL_FORCE)
			{
				modeW = 1;
			}
			else if(robot.skillOutput.drive.modeW == DRIVE_MODE_LOCAL_VEL ||
					robot.skillOutput.drive.modeW == DRIVE_MODE_GLOBAL_POS ||
					robot.skillOutput.drive.modeW == DRIVE_MODE_GLOBAL_POS_ASYNC)
			{
				const float trajVel = robot.ctrlRef.trajVelLocal[2];
				const float trajAcc = robot.ctrlRef.trajAccLocal[2];
				const float trajVelAbs = fabsf(trajVel);
				const float trajAccAbs = fabsf(trajAcc);

				if(trajAccAbs < 1e-3f)
				{
					if(trajVelAbs < 1e-3f)
					{
						// stopped
						modeW = 2;
					}
					else
					{
						// plateau
						modeW = 3;
					}
				}
				else
				{
					if(trajVel*trajAcc > 0)
					{
						// accelerating
						modeW = 4;
					}
					else
					{
						// decelerating
						modeW = 5;
					}
				}
			}

			bm2.mode = modeXY | (modeW << 4);

			PacketHeader header;
			header.section = SECTION_DATA_ACQ;
			header.cmd = CMD_DATA_ACQ_BOT_MODEL_V2;

			NetworkSendPacket(&header, &bm2, sizeof(DataAcqBotModelV2));
		}
		break;
	}
}
