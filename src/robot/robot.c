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
#include "hal/sys_time.h"
#include "network.h"
#include "version.h"
#include "robot/ctrl_panthera.h"
#include "math/angle_math.h"
#include "math/vector.h"
#include "robot_math.h"

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

static void specsUpdateCallback(uint16_t cfgId)
{
	if(cfgId == SID_CFG_CTRL_PHYSICAL_PARAMS)
	{
		RobotMathUpdate(&robot.specs);
	}
}

void RobotInit()
{
	robot.mode = ROBOT_MODE_IDLE;
	robot.enabledSystems = ROBOT_SYSTEM_BUZZER | ROBOT_SYSTEM_LEDS | ROBOT_SYSTEM_STATE_EST;

	chMBObjectInit(&robot.eventQueue, robot.eventQueueData, ROBOT_EVENT_QUEUE_SIZE);

	RobotImplGetSpecs(&robot.specs);

	const uint8_t configFileFlagsLogOnly = CONFIG_FILE_FLAG_INTERNAL | CONFIG_FILE_FLAG_VOLATILE;

	robot.pPhysicalParamsConfig= ConfigOpenOrCreate(&robotSpecsConfigDescPhysical, &robot.specs.physical, sizeof(PhysicalParams), &specsUpdateCallback, configFileFlagsLogOnly);
	robot.pDriveTrainParamsConfig = ConfigOpenOrCreate(&robotSpecsConfigDescDriveTrain, &robot.specs.driveTrain, sizeof(DriveTrainParams), &specsUpdateCallback, configFileFlagsLogOnly);
	robot.pDribblerParamsConfig = ConfigOpenOrCreate(&robotSpecsConfigDescDribbler, &robot.specs.dribbler, sizeof(DribblerParams), &specsUpdateCallback, configFileFlagsLogOnly);

	ConfigNotifyUpdate(robot.pPhysicalParamsConfig);

	LagElementPT1Init(&robot.accZLag, 1, 0.5f, 0.001f);
	robot.accZLag.state[0] = 20.0f;
	robot.accZLag.state[1] = 20.0f;
	robot.accZLag.state[2] = 20.0f;
	robot.accZLag.state[3] = 20.0f;

	LagElementPT1Init(&robot.gyrZLag, 1, 0.5f, 0.001f);

	initLogFields();

	SkillsInit();
}

void RobotTask(void* params)
{
	(void)params;

	chRegSetThreadName("Robot");

	RobotImplInit();

	uint32_t event;
	systime_t prev = chVTGetSystemTimeX();
	systime_t next = prev+TIME_US2I(1000);

	while(1)
	{
		uint32_t tStart = SysTimeCycleCounter();
		robot.sensors.header.timestamp = SysTimeUSec();
		robot.performance.header.timestamp = SysTimeUSec();

		// fetch updated sensor data from all subsystems
		RobotImplUpdateSensors(&robot.sensors);
		RobotImplUpdateAuxData(&robot.aux);
		RobotImplUpdateNetStats(&robot.netStats);

		// Check if this robot has a working cover on it
		detectCover();

		// check connection to BS and command sender
		connectionCheck();

		// get skill data, parse flags, read vision pos (if available)
		parseMatchCtrl();

		uint32_t tInput = SysTimeCycleCounter();
		robot.performance.inputTime = SysTimeCycleCounterDiffUSec(tStart, tInput);

		// run state estimation with sensor input
		if(robot.enabledSystems & ROBOT_SYSTEM_STATE_EST)
			RobotImplStateEstimation(&robot.sensors, &robot.ctrlOutput, &robot.ctrlRef, &robot.state);

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
			RobotImplControl(&robot.state, &robot.skillOutput, &robot.ctrlOutput, &robot.ctrlRef);

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
			RobotImplDribblerOutput(&robot.ctrlOutput);

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
			chMBPostTimeout(&robot.eventQueue, ROBOT_EVENT_BAT_EMPTY, TIME_IMMEDIATE);

		// state watchdog (checks Z Acc)
		stateWatchdog();

		// clear all updated flags in sensor data
		clearUpdateFlags();

		// process all events
		while(chMBFetchTimeout(&robot.eventQueue, (msg_t*)&event, TIME_IMMEDIATE) == MSG_OK)
		{
			processEvent(event);
		}

		uint32_t tMisc = SysTimeCycleCounter();
		robot.performance.miscTime = SysTimeCycleCounterDiffUSec(tOut, tMisc);

		// wait until next loop
		prev = chThdSleepUntilWindowed(prev, next);
		next += TIME_US2I(1000);
	}
}

void RobotSetIdleMode()
{
	chMBPostTimeout(&robot.eventQueue, ROBOT_EVENT_RESET, TIME_IMMEDIATE);
}

void RobotTestModeEnable(uint8_t enable)
{
	if(enable)
		chMBPostTimeout(&robot.eventQueue, ROBOT_EVENT_ENTER_TEST, TIME_INFINITE);
	else
		chMBPostTimeout(&robot.eventQueue, ROBOT_EVENT_EXIT_TEST, TIME_INFINITE);

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

	RobotImplInit();

	RobotImplMotorOutput(&robot.ctrlOutput);
	RobotImplKickerOutput(&robot.skillOutput.kicker);
	RobotImplDribblerOutput(&robot.ctrlOutput);

	SkillsReset();
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
	chMBPostTimeout(&robot.eventQueue, ROBOT_EVENT_NET_ENABLE, TIME_IMMEDIATE);
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
	if(event == ROBOT_EVENT_RESET)
	{
		robot.mode = ROBOT_MODE_IDLE;

		robot.enabledSystems = ROBOT_SYSTEM_BUZZER | ROBOT_SYSTEM_LEDS | ROBOT_SYSTEM_STATE_EST;

		stop();
	}

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

		stop();
	}

	if(robot.mode == ROBOT_MODE_IDLE && event == ROBOT_EVENT_VISION_RECEIVED)
	{
		robot.mode = ROBOT_MODE_READY;

		robot.enabledSystems = ROBOT_SYSTEM_ALL_MASK;
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

	if(robot.aux.physical.v2016)
		features |= SYSTEM_MATCH_FEEDBACK_FEATURE_V2016;

	if(robot.aux.ext.installed)
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
	uint8_t txActive = !NetworkImplIsSendBufferEmpty();

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

	uint32_t dribSpeed = fmaxf(0, robot.state.dribblerVel * 4.0f + 0.5f);
	uint32_t dribState = robot.state.dribblerState & 0x03;

	feedback.dribblerState = (dribSpeed << 2) | dribState;
	feedback.flags = (robot.sensors.ir.interrupted << 7) | ((robot.sensors.kicker.kickCounter & 0x01) << 4) | (robot.state.ballState & 0x07);

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

	// Use EKF data for Sumatra feedback, this matches Sumatras timepoint most closely
	const FusionEKFTimeSlot* pTimeSlot = &fusionEKF.timeSlots[(fusionEKF.timeSlotNow - fusionEKF.pConfig->visCaptureDelay)%FUSION_EKF_MAX_DELAY];

	if(robot.feedbackStateUpdated[0])
	{
		robot.feedbackStateUpdated[0] = 0;

		feedback.curPosition[0] = (int16_t)(pTimeSlot->insState[0]*1000.0f);
		feedback.curPosition[1] = (int16_t)(pTimeSlot->insState[1]*1000.0f);
		feedback.curPosition[2] = (int16_t)(AngleNormalize(pTimeSlot->insState[2])*1000.0f);
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

		float globalVel[3];
		Vector2fTurnLocal2Global(pTimeSlot->insState[2], pTimeSlot->insState[3], pTimeSlot->insState[4], &globalVel[0], &globalVel[1]);
		globalVel[2] = pTimeSlot->meas.gyrAcc[0];

		feedback.curVelocity[0] = (int16_t)(globalVel[0]*1000.0f);
		feedback.curVelocity[1] = (int16_t)(globalVel[1]*1000.0f);
		feedback.curVelocity[2] = (int16_t)(globalVel[2]*1000.0f);
	}
	else
	{
		feedback.curVelocity[0] = SYSTEM_MATCH_FEEDBACK_UNUSED_FIELD;
		feedback.curVelocity[1] = SYSTEM_MATCH_FEEDBACK_UNUSED_FIELD;
		feedback.curVelocity[2] = SYSTEM_MATCH_FEEDBACK_UNUSED_FIELD;
	}

	if(robot.state.ballState >= BALL_STATE_AT_BARRIER)
	{
		float ballPosGlobal[2];
		Vector2fTurnLocal2Global(robot.state.pos[2], robot.state.ballPos[0], robot.state.ballPos[1] + robot.specs.physical.dribblerDistance_m, ballPosGlobal, ballPosGlobal+1);
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
		chMBPostTimeout(&robot.eventQueue, ROBOT_EVENT_NET_ENABLE, TIME_IMMEDIATE);
}

static void parseMatchCtrl()
{
	static uint8_t lastSong;

	// prefer match ctrl cmd from primary wireless interface
	SystemMatchCtrl matchCtrl;
	uint32_t matchCtrlTime;
	uint8_t matchCtrlUpdated;

	chMtxLock(&network.matchCtrlMutex);

	memcpy(&matchCtrl, &network.lastMatchCtrlCmd, sizeof(SystemMatchCtrl));
	matchCtrlTime = network.matchCtrlTime;
	matchCtrlUpdated = network.matchCtrlUpdated;
	network.matchCtrlUpdated = 0;

	chMtxUnlock(&network.matchCtrlMutex);

	if(robot.sensors.vision.time != matchCtrlTime)
	{
		if(matchCtrl.curPosition[0] != SYSTEM_MATCH_CTRL_UNUSED_FIELD)
		{
			uint32_t delay = matchCtrl.posDelay;
			delay = (delay*25000)/100;	// delay now in [us]

			robot.sensors.vision.pos[0] = matchCtrl.curPosition[0]*0.001f;
			robot.sensors.vision.pos[1] = matchCtrl.curPosition[1]*0.001f;
			robot.sensors.vision.pos[2] = matchCtrl.curPosition[2]*0.001f;
			robot.sensors.vision.time = matchCtrlTime;
			robot.sensors.vision.delay = delay;
			robot.sensors.vision.camId = matchCtrl.camId & 0x7F;
			robot.sensors.vision.updated = 1;
			robot.sensors.vision.noVision = 0;

			if(robot.coverPresent)
				chMBPostTimeout(&robot.eventQueue, ROBOT_EVENT_VISION_RECEIVED, TIME_IMMEDIATE);
		}

		if(matchCtrl.camId & SYSTEM_MATCH_CTRL_CAM_ID_FLAG_NO_VISION)
		{
			robot.sensors.vision.time = matchCtrlTime;
			robot.sensors.vision.updated = 0;
			robot.sensors.vision.noVision = 1;

			if(robot.coverPresent)
				chMBPostTimeout(&robot.eventQueue, ROBOT_EVENT_VISION_RECEIVED, TIME_IMMEDIATE);
		}
	}

	if(robot.enabledSystems & ROBOT_SYSTEM_KICKER)
	{
		if(matchCtrl.flags & SYSTEM_MATCH_CTRL_FLAGS_KICKER_AUTOCHG)
			RobotImplKickerAutoCharge(1);
		else
			RobotImplKickerAutoCharge(0);
	}

	uint8_t song = matchCtrl.flags & SYSTEM_MATCH_CTRL_FLAGS_SONG_MASK;

	if(song != lastSong && (robot.enabledSystems & ROBOT_SYSTEM_BUZZER))
		RobotImplBuzzerPlay(song);

	lastSong = song;

	uint8_t leds = (matchCtrl.flags & SYSTEM_MATCH_CTRL_FLAGS_LED_MASK) >> 4;
	if(robot.enabledSystems & ROBOT_SYSTEM_LEDS)
		RobotImplSetLEDs(leds);

	if(matchCtrl.flags & SYSTEM_MATCH_CTRL_FLAGS_STRICT_VEL_LIMIT)
		robot.skillOutput.drive.limits.strictVelLimit = 1;
	else
		robot.skillOutput.drive.limits.strictVelLimit = 0;

	if(robot.enabledSystems & ROBOT_SYSTEM_SKILLS)
	{
		if(robot.sumatraOnline == 0)
			SkillsSetInput(0, 0);
		else if(matchCtrlUpdated)
			SkillsSetInput(matchCtrl.skillId, matchCtrl.skillData);
	}
}

static void connectionCheck()
{
	uint8_t baseOnline = NetworkImplIsBaseOnline();
	if(!baseOnline && robot.bsOnline)
		robot.bsOnline = 0;

	if(baseOnline && !robot.bsOnline)
		robot.bsOnline = 1;

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
}

static uint8_t isArrayNaN(const float* pData, uint32_t size)
{
	for(uint32_t i = 0; i < size; i++)
	{
		if(isnanf(pData[i]))
			return 1;
	}

	return 0;
}

static void stateWatchdog()
{
	// robot tipped over watchdog
	if(robot.sensors.acc.updated)
		robot.accZ = LagElementPT1Process(&robot.accZLag, robot.sensors.acc.linAcc[2]);

	if(robot.accZ < 3.0f)
		chMBPostTimeout(&robot.eventQueue, ROBOT_EVENT_RESET, TIME_IMMEDIATE);

	// rotating too fast, too long watchdog
	const float ROT_VEL_Z_LIMIT = 30.0f; // sensor saturation @ 34.89rad/s

	if(robot.sensors.gyr.updated)
	{
		robot.gyrZ = LagElementPT1Process(&robot.gyrZLag, robot.sensors.gyr.rotVel[2]);

		if(fabsf(robot.sensors.gyr.rotVel[2]) > ROT_VEL_Z_LIMIT)
			RobotImplBuzzerPlay(BUZZ_DOWN100);
	}

	if(fabsf(robot.gyrZ) > ROT_VEL_Z_LIMIT)
		chMBPostTimeout(&robot.eventQueue, ROBOT_EVENT_RESET, TIME_IMMEDIATE);

	// broken state estimation watchdog
	int posNaN = isnanf(robot.state.pos[0]) || isnanf(robot.state.pos[1]) || isnanf(robot.state.pos[2]);
	int velNaN = isnanf(robot.state.vel[0]) || isnanf(robot.state.vel[1]) || isnanf(robot.state.vel[2]);
	int accNaN = isnanf(robot.state.acc[0]) || isnanf(robot.state.acc[1]) || isnanf(robot.state.acc[2]);

	if(posNaN || velNaN || accNaN)
	{
		chMBPostTimeout(&robot.eventQueue, ROBOT_EVENT_RESET, TIME_IMMEDIATE);
	}

	// Check skill output for NaNs
	int skillDriveNaN = isArrayNaN(robot.skillOutput.drive.pos, 3) || isArrayNaN(robot.skillOutput.drive.localVel, 3) ||
			isArrayNaN(robot.skillOutput.drive.localForce, 3) || isArrayNaN(robot.skillOutput.drive.wheelVel, 4) ||
			isnanf(robot.skillOutput.drive.primaryDirection);

	int skillDribblerNaN = isnanf(robot.skillOutput.dribbler.maxForce) || isnanf(robot.skillOutput.dribbler.velocity) ||
			isnanf(robot.skillOutput.dribbler.voltage);

	int skillKickerNaN = isnanf(robot.skillOutput.kicker.speed);

	if(skillDriveNaN || skillDribblerNaN || skillKickerNaN)
	{
		chMBPostTimeout(&robot.eventQueue, ROBOT_EVENT_RESET, TIME_IMMEDIATE);
	}

	// Check control output and reference for NaNs
	int isCtrlOutNaN = isArrayNaN(robot.ctrlOutput.motorAcc, 4) || isArrayNaN(robot.ctrlOutput.motorTorque, 4) ||
			isArrayNaN(robot.ctrlOutput.motorVel, 4) || isArrayNaN(robot.ctrlOutput.motorVoltage, 4);

	int isCtrlRefNaN = isArrayNaN(robot.ctrlRef.trajAccLocal, 3) || isArrayNaN(robot.ctrlRef.trajPos, 3) ||
			isArrayNaN(robot.ctrlRef.trajVelLocal, 3);

	if(isCtrlOutNaN || isCtrlRefNaN)
	{
		chMBPostTimeout(&robot.eventQueue, ROBOT_EVENT_RESET, TIME_IMMEDIATE);
	}
}

static void handleDataAcquisitionMode()
{
	// check if there are other packets to be send, if not => send a feedback
	uint8_t txActive = !NetworkImplIsSendBufferEmpty();
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
			RobotMathMotorVelToLocalVel(robot.ctrlOutput.motorVel, outVel);

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
			RobotMathMotorVelToLocalVel(robot.ctrlOutput.motorVel, outVel);

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
			RobotMathMotorVelToLocalVel(robot.sensors.enc.vel, encVel);

			bm2.encVel[0] = (int16_t)(encVel[0]*1000.0f);
			bm2.encVel[1] = (int16_t)(encVel[1]*1000.0f);
			bm2.encVel[2] = (int16_t)(encVel[2]*100.0f);

			float localStateVel[2];
			Vector2fTurnGlobal2Local(robot.state.pos[2], robot.state.vel[0], robot.state.vel[1], localStateVel, localStateVel+1);

			bm2.stateVel[0] = (int16_t)(localStateVel[0]*1000.0f);
			bm2.stateVel[1] = (int16_t)(localStateVel[1]*1000.0f);
			bm2.stateVel[2] = (int16_t)(robot.state.vel[2]*100.0f);

			float motorTorque[4];
			for(uint8_t i = 0; i < 4; i++)
				motorTorque[i] = robot.sensors.motorCur.cur[i] * robot.specs.driveTrain.motor.Km;

			float localForce[3];
			RobotMathMotorTorqueToLocalForce(motorTorque, localForce);

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
