/*
 * commands.h
 *
 *  Created on: 24.05.2010
 *      Author: AndreR
 */

#pragma once

#include <stdint.h>
#include "util/float16_t.h"

#ifndef PACKED
#define PACKED __attribute__ ((packed))
#endif

#define PACKET_HEADER_SIZE 2
typedef union PACKED _PacketHeader
{
	struct
	{
		uint8_t cmd;
		uint8_t section;
	};

	uint8_t _data[PACKET_HEADER_SIZE];
} PacketHeader;

#define PACKET_HEADER_EX_SIZE 4
typedef union PACKED _PacketHeaderEx
{
	struct
	{
		uint8_t cmd;
		uint8_t section;
		uint16_t seq;
	};

	uint8_t _data[PACKET_HEADER_EX_SIZE];
} PacketHeaderEx;

// SECTIONS
#define SECTION_SYSTEM			0
#define SECTION_DECT			1	// obsolete
#define SECTION_MOTOR			2
#define SECTION_KICKER			4
#define SECTION_MOVEMENT		5	// obsolete
#define SECTION_MULTICAST		6	// obsolete
#define SECTION_CTRL			7
#define SECTION_BASE_STATION	8
#define SECTION_BOOTLOADER		9
#define SECTION_SKILL			10	// obsolete
#define SECTION_CONFIG			11
#define SECTION_STATUS			12
#define SECTION_DATA_ACQ		13
#define SECTION_EXT				14

#define SECTION_EXTENDED_PACKET	0x80

// SYSTEM
#define CMD_SYSTEM_POWER_LOG			2
#define CMD_SYSTEM_STATUS_MOVEMENT		3
#define CMD_SYSTEM_SET_IDENTITY			4
#define CMD_SYSTEM_ANNOUNCEMENT			5
#define CMD_SYSTEM_PING					6
#define CMD_SYSTEM_PONG					7
#define CMD_SYSTEM_BIG_PING				8
#define CMD_SYSTEM_SET_LOGS				9
#define CMD_SYSTEM_CONSOLE_PRINT		10
#define CMD_SYSTEM_CONSOLE_COMMAND		11
#define CMD_SYSTEM_FEATURES				12	// there is no struct for this command, it just consists of a list of [feature, state] pairs
#define CMD_SYSTEM_MATCH_CTRL			13
#define CMD_SYSTEM_MATCH_FEEDBACK		14
#define CMD_SYSTEM_PERFORMANCE			15
#define CMD_SYSTEM_QUERY				16
#define CMD_SYSTEM_TIMESYNC				17
#define CMD_SYSTEM_VERSION				18
#define CMD_SYSTEM_STATUS_V2			0x80
#define CMD_SYSTEM_STATUS_EXT			0x81
#define CMD_SYSTEM_ACK					0xF0

// KICKER
#define CMD_KICKER_KICKV1				1
#define CMD_KICKER_STATUS				2
#define CMD_KICKER_SET_AUTOLOAD			3
#define CMD_KICKER_SET_MAX_CAP_LEVEL	4
#define CMD_KICKER_CHARGE				5
#define CMD_KICKER_SET_DUTY_CYCLE		6
#define CMD_KICKER_KICKV2				7
#define CMD_KICKER_STATUSV2				8
#define CMD_KICKER_CHARGE_MANUAL		9
#define CMD_KICKER_CHARGE_AUTO			10
#define CMD_KICKER_IR_LOG				11
#define CMD_KICKER_STATUSV3				12
#define CMD_KICKER_CONFIG				13

// MOTOR
#define CMD_MOTOR_MOVE					0
#define CMD_MOTOR_DRIBBLE				1
#define CMD_MOTOR_SET_PID_PARAMS		2
#define CMD_MOTOR_SET_MODE				3
#define CMD_MOTOR_SET_MANUAL			4
#define CMD_MOTOR_SET_PID_SP			5
#define CMD_MOTOR_PID_LOG				6
#define CMD_MOTOR_SET_PARAMS			7
#define CMD_MOTOR_MOVE_V2				8

// CONTROL
#define CMD_CTRL_VISION_POS				0
#define CMD_CTRL_SPLINE_2D				1
#define CMD_CTRL_SPLINE_1D				2
#define CMD_CTRL_SET_FILTER_PARAMS		3
#define CMD_CTRL_SET_PID_PARAMS			4
#define CMD_CTRL_SET_CONTROLLER_TYPE	5
#define CMD_CTRL_VISION_TRACKED_OBJ		6
#define CMD_CTRL_RESET					7

// BASE_STATION
#define CMD_BASE_STATION_ACOMMAND		0
#define CMD_BASE_STATION_PING			1
#define CMD_BASE_STATION_AUTH			2
#define CMD_BASE_STATION_STATS			3
#define CMD_BASE_STATION_CONFIG			4
#define CMD_BASE_STATION_VISION_CONFIG	5
#define CMD_BASE_STATION_WIFI_STATS		6
#define CMD_BASE_STATION_ETH_STATS		7
#define CMD_BASE_STATION_CONFIG_V2		8
#define CMD_BASE_STATION_CAM_VIEWPORT	9
#define CMD_BASE_STATION_CONFIG_V3		10

// BOOTLOADER
#define CMD_BOOTLOADER_CHECK			0
#define CMD_BOOTLOADER_REQUEST_SIZE		1
#define CMD_BOOTLOADER_REQUEST_CRC32	2
#define CMD_BOOTLOADER_REQUEST_DATA		3
#define CMD_BOOTLOADER_SIZE				4
#define CMD_BOOTLOADER_CRC32			5
#define CMD_BOOTLOADER_DATA				6

// CONFIG
#define CMD_CONFIG_QUERY_FILE_LIST		0
#define CMD_CONFIG_FILE_STRUCTURE		1
#define CMD_CONFIG_ITEM_DESC			2
#define CMD_CONFIG_READ					3
#define CMD_CONFIG_WRITE				4

// STATUS
#define CMD_STATUS_FEEDBACK_PID			0

// DATA_ACQ
#define CMD_DATA_ACQ_MOTOR_MODEL		0
#define CMD_DATA_ACQ_BOT_MODEL			1
#define CMD_DATA_ACQ_DELAYS				2
#define CMD_DATA_ACQ_SET_MODE			4
#define CMD_DATA_ACQ_BOT_MODEL_V2		5

// BOT COUNT
#define CMD_BOT_COUNT_TOTAL 32
#define CMD_BOT_LAST_ID (CMD_BOT_COUNT_TOTAL-1)
#define CMD_BOT_COUNT_HALF (CMD_BOT_COUNT_TOTAL/2)
#define CMD_BOT_HALF_MIN_1 (CMD_BOT_COUNT_TOTAL/2-1)

//EXT
#define CMD_EXT_BALL_DETECTIONS			0
#define CMD_EXT_SHUTDOWN				1
#define CMD_EXT_ROBOT_PI_VERSION		2
#define CMD_EXT_CAMERA_CONFIG			4
#define CMD_EXT_CAMERA_PREVIEW_LINE_160	5
#define CMD_EXT_CAMERA_PREVIEW_CONFIG	6
#define CMD_EXT_CAMERA_CONTROL			7
#define CMD_EXT_CAMERA_TRIGGER_CAPTURE	8
#define CMD_EXT_CAMERA_STATS			9
#define CMD_EXT_CAMERA_INTRINSICS		10
#define CMD_EXT_CAMERA_EXTRINSICS		11
#define CMD_EXT_REMOTE_TIME				12
#define CMD_EXT_ROBOT_STATE				13
#define CMD_EXT_BALL_DETECTION_CONFIG	14
#define CMD_EXT_BALL_TRACKER_CONFIG		15
#define CMD_EXT_COLOR_THRESHOLDS		16
#define CMD_EXT_FIELD_INFO				17
#define CMD_EXT_POSITION_DETECTION		18
#define CMD_EXT_GOAL_DETECTIONS			19

typedef struct PACKED _ConfigFileStructure
{
	uint16_t id;
	uint16_t version;
// + data (uint8_t list of items)
} ConfigFileStructure;

typedef struct PACKED _ConfigItemDesc
{
	uint16_t id;
	uint8_t element;	// 0xFF for filename, item description otherwise
// + zero terminated string
} ConfigItemDesc;

typedef struct PACKED _ConfigReadWrite
{
	uint16_t id;
// + optional write data
} ConfigReadWrite;

typedef struct PACKED _BootloaderRequestSize
{
	uint8_t procId;
} BootloaderRequestSize;

typedef struct PACKED _BootloaderRequestCRC32
{
	uint8_t procId;
	uint32_t startAddr;
	uint32_t endAddr;
} BootloaderRequestCRC32;

typedef struct PACKED _BootloaderRequestData
{
	uint8_t procId;
	uint32_t offset;
	uint32_t size;
} BootloaderRequestData;

typedef struct PACKED _BootloaderSize
{
	uint8_t procId;
	uint32_t size;
} BootloaderSize;

typedef struct PACKED _BootloaderCRC32
{
	uint8_t procId;
	uint32_t startAddr;
	uint32_t endAddr;
	uint32_t crc;
} BootloaderCRC32;

typedef struct PACKED _BootloaderData
{
	uint8_t procId;
	uint32_t offset;
} BootloaderData;

typedef struct PACKED _SystemPing	// 4
{
	uint32_t id;
} SystemPing;

typedef struct PACKED _SystemPong	// 4
{
	uint32_t id;
} SystemPong;

typedef struct PACKED _SystemConsolePrint	// 1 + n
{
	uint8_t source;		// 1
} SystemConsolePrint;

typedef struct PACKED _SystemConsoleCommand	// 1 + n
{
	uint8_t target;		// 1
} SystemConsoleCommand;

#define DRIVE_MODE_OFF				0
#define DRIVE_MODE_WHEEL_VEL		1
#define DRIVE_MODE_LOCAL_VEL		2
#define DRIVE_MODE_GLOBAL_POS		3
#define DRIVE_MODE_GLOBAL_POS_ASYNC	4
#define DRIVE_MODE_LOCAL_FORCE		5

#define DRIBBLER_MODE_OFF		0
#define DRIBBLER_MODE_VOLTAGE	1
#define DRIBBLER_MODE_SPEED		2

#define KICKER_DEVICE_STRAIGHT	0
#define KICKER_DEVICE_CHIP		1

#define KICKER_MODE_DISARM		0
#define KICKER_MODE_ARM			1
#define KICKER_MODE_FORCE		2
#define KICKER_MODE_ARM_TIME	3
#define KICKER_MODE_ARM_AIM		4

#define KICKER_FLAGS_READY			0x01
#define KICKER_FLAGS_DMG_BARRIER	0x02
#define KICKER_FLAGS_DMG_STRAIGHT	0x04
#define KICKER_FLAGS_DMG_CHIP		0x08
#define KICKER_FLAGS_DMG_CHARGE		0x10
#define KICKER_FLAGS_V2017			0x20

#define PATTERN_IDENT_FLAGS_BAD_CENTER			0x0001
#define PATTERN_IDENT_FLAGS_BAD_TOP_LEFT		0x0002
#define PATTERN_IDENT_FLAGS_BAD_TOP_RIGHT		0x0004
#define PATTERN_IDENT_FLAGS_BAD_BOTTOM_LEFT		0x0008
#define PATTERN_IDENT_FLAGS_BAD_BOTTOM_RIGHT	0x0010
#define PATTERN_IDENT_FLAGS_BAD_BLOB_MASK		0x001F
#define PATTERN_IDENT_FLAGS_SYSTEM_PRESENT		0x0100
#define PATTERN_IDENT_FLAGS_COVER_DETECTED		0x4000
#define PATTERN_IDENT_FLAGS_COVER_LOST			0x8000

#define BUZZ_NONE				0
#define BUZZ_UPDOWN20			1
#define BUZZ_SONG_FINAL			2
#define BUZZ_SONG_ELEVATOR		3
#define BUZZ_SONG_EYE_LEAD		4
#define BUZZ_SONG_EYE_FOLLOW	5
#define BUZZ_SONG_CANTINA		6
#define BUZZ_TADA				7
#define BUZZ_UP20				8
#define BUZZ_UP50				9
#define BUZZ_UP100				10
#define BUZZ_DOWN50				11
#define BUZZ_DOWN100			12
#define BUZZ_BEEP_FAST			13
#define BUZZ_DOUBLE_SLOW		14

#define SYSTEM_MATCH_CTRL_FLAGS_LED_OFF				0
#define SYSTEM_MATCH_CTRL_FLAGS_LED_RED				1
#define SYSTEM_MATCH_CTRL_FLAGS_LED_GREEN			2
#define SYSTEM_MATCH_CTRL_FLAGS_LED_BLUE			3
#define SYSTEM_MATCH_CTRL_FLAGS_LED_WHITE			4
#define SYSTEM_MATCH_CTRL_FLAGS_LED_LIGHT_BLUE		5
#define SYSTEM_MATCH_CTRL_FLAGS_LED_GOLD			6
#define SYSTEM_MATCH_CTRL_FLAGS_LED_PURPLE			7

#define SYSTEM_MATCH_CTRL_FLAGS_SONG_MASK			0x07
#define SYSTEM_MATCH_CTRL_FLAGS_KICKER_AUTOCHG		0x08
#define SYSTEM_MATCH_CTRL_FLAGS_LED_MASK			0x70
#define SYSTEM_MATCH_CTRL_FLAGS_STRICT_VEL_LIMIT	0x80

#define SYSTEM_MATCH_CTRL_UNUSED_FIELD				0x7FFF

#define SYSTEM_MATCH_CTRL_USER_DATA_SIZE			16

#define SYSTEM_MATCH_CTRL_CAM_ID_FLAG_NO_VISION		0x80

typedef struct PACKED _SystemMatchCtrl	// 26
{
	int16_t curPosition[3];	// [mm]				6
	uint8_t posDelay;		// [Q6.2 ms]		1	// maxVal 63.75ms
	uint8_t camId;			// Vision camera ID	1
	uint8_t flags;			//					1 // kicker auto charge, songs/cheer, LEDs, vel limit
	uint8_t skillId;		//					1
	uint8_t skillData[SYSTEM_MATCH_CTRL_USER_DATA_SIZE];	// skill dependent	16 = 26 B + 4B Header/COBS = 30B
} SystemMatchCtrl;

#define SYSTEM_MATCH_FEEDBACK_UNUSED_FIELD			0x7FFF
#define SYSTEM_MATCH_FEEDBACK_FLAGS_BARRIER_MASK	0x80
#define SYSTEM_MATCH_FEEDBACK_FLAGS_DRIB_TEMP_MASK	0x60 // low, medium, high, overheated
#define SYSTEM_MATCH_FEEDBACK_FLAGS_KICK_MASK		0x10
#define SYSTEM_MATCH_FEEDBACK_FLAGS_DRIB_CUR_MASK	0x0F

#define SYSTEM_MATCH_FEEDBACK_FEATURE_MOVE			0x0001
#define SYSTEM_MATCH_FEEDBACK_FEATURE_DRIBBLE		0x0002
#define SYSTEM_MATCH_FEEDBACK_FEATURE_STRAIGHT		0x0004
#define SYSTEM_MATCH_FEEDBACK_FEATURE_CHIP			0x0008
#define SYSTEM_MATCH_FEEDBACK_FEATURE_BARRIER		0x0010
#define SYSTEM_MATCH_FEEDBACK_FEATURE_V2016			0x0020
#define SYSTEM_MATCH_FEEDBACK_FEATURE_EXT_BOARD		0x0040
#define SYSTEM_MATCH_FEEDBACK_FEATURE_CHARGE		0x0080
#define SYSTEM_MATCH_FEEDBACK_FEATURE_KICKER_V2017	0x0100
#define SYSTEM_MATCH_FEEDBACK_FEATURE_ENERGETIC		0x0200
#define SYSTEM_MATCH_FEEDBACK_FEATURE_COVER			0x0400
#define SYSTEM_MATCH_FEEDBACK_FEATURE_MODE_MASK		0xF000

typedef struct PACKED _SystemMatchFeedback	// 26
{
	int16_t curPosition[3];		// [mm]			6
	int16_t curVelocity[3];		// [mm/s]		6
	uint8_t kickerLevel;		// [V]			1
	uint8_t kickerMax;			// [V]			1
	uint8_t dribblerState;		// [rpm/500]	1	includes two current LSBs (others are in flags)
	uint8_t batteryLevel;		// [dV]			1
	uint8_t batteryPercent;		// 255 = 100%	1
	uint8_t flags;				//				1
	uint16_t features;			//				2	movement, dribbler, barrier, straight, chip
	uint8_t hardwareId;			//				1
	uint8_t ballPosAge;			// [ms]			1
	int16_t ballPosition[2];	// [mm]			4
} SystemMatchFeedback;

typedef struct PACKED _SystemVersion // 8
{
	uint32_t version;	// 4 [major.minor.patch.dirty]
	uint32_t gitRef;	// 4 [SHA1]
} SystemVersion;

#define SYSTEM_QUERY_PID_PARAMS			0
#define SYSTEM_QUERY_STRUCTURE			1
#define SYSTEM_QUERY_EZ_SENSOR			2
#define SYSTEM_QUERY_EX_STATE			3
#define SYSTEM_QUERY_CONTROLLER_TYPE	4
#define SYSTEM_QUERY_KICKER_CONFIG		5

typedef struct PACKED _SystemQuery
{
	uint16_t parameterId;
} SystemQuery;

typedef struct PACKED _SystemPerformance	// 12
{
	uint16_t accMax;	// [mm/s^2]
	uint16_t accMaxW;	// [crad/s^2]
	uint16_t brkMax;	// [mm/s^2]
	uint16_t brkMaxW;	// [crad/s^2]
	uint16_t velMax;	// [mm/s]
	uint16_t velMaxW;	// [crad/s]
} SystemPerformance;

typedef struct PACKED _StatusFeedbackPid // 20
{
	int16_t t;
	int16_t in[3];
	int16_t out[3];
	int16_t set[3];
} StatusFeedbackPid;

//#define CTRL_TYPE_FUSION_VEL	1 // obsolete
#define CTRL_TYPE_FUSION		2
#define CTRL_TYPE_MOTOR			3
#define CTRL_TYPE_CALIBRATE		4
#define CTRL_TYPE_TIGGA			5

typedef struct PACKED _CtrlSetControllerType // 1
{
	uint8_t controllerId;
} CtrlSetControllerType;

typedef struct PACKED _DataAcqMotorModel // 20
{
	uint32_t timestamp;		// [us]			4
	int16_t motVol[4];		// [mV]			8
	int16_t motVel[4];		// [rad/s*40]	8
} DataAcqMotorModel;

typedef struct PACKED _DataAcqBotModel // 20
{
	uint32_t timestamp;		// [us]					4
	uint32_t visionTime;	// [us]					4
	int16_t outVel[3];		// [mm/s, crad/s]		6
	int16_t visPos[3];		// [mm, mrad]			6
} DataAcqBotModel;

typedef struct PACKED _DataAcqDelays // 14
{
	uint32_t timestamp;		// [us]		4
	uint32_t visionTime;	// [us]		4
	int16_t outVelW;		// [mrad/s]	2
	int16_t visPosW;		// [mrad]	2
	int16_t gyrVel;			// [mrad/s]	2
} DataAcqDelays;

typedef struct PACKED _DataAcqBotModelV2 // 26
{
	uint32_t timestamp;		// [us]				4
	int16_t stateVel[3];	// [mm/s, crad/s]	6
	int16_t encVel[3];		// [mm/s, crad/s]	6
	int16_t outForce[3];	// [cN, mNm]		6
	uint8_t efficiency[3];	// [12bit | 12bit]	3
	uint8_t mode;			// [acc,dec,...]	1
} DataAcqBotModelV2;

typedef struct PACKED _DataAcqSetMode // 1
{
	uint8_t mode;
} DataAcqSetMode;

typedef struct PACKED _BaseStationACommand
{
	uint8_t id;
	uint8_t* pCommand;
} BaseStationACommand;

typedef struct PACKED _BaseStationPing
{
	uint32_t id;
	uint8_t* pData;
} BaseStationPing;

typedef struct PACKED _BaseStationAuth	// 4
{
	uint32_t key;	// 0x42424242
} BaseStationAuth;

typedef struct PACKED _BaseStationConfigV2
{
	uint8_t visionIp[4];
	uint16_t visionPort;

	struct PACKED _module
	{
		uint8_t channel;		// = frequency
		uint8_t speed;			// 0 = 250k, 1 = 1M, 2 = 2M
		uint8_t maxBots;		// 1 - 32
		uint8_t fixedRuntime;
		uint32_t timeout;		// in [ms]
	} module[2];

	struct PACKED _rst
	{
		uint8_t enabled;
		uint8_t rate;		// [Hz]
		uint16_t port;
	} rst;
} BaseStationConfigV2;

typedef struct PACKED _BaseStationConfigV3
{
	uint8_t visionIp[4];
	uint16_t visionPort;

	uint8_t channel;		// = frequency
	uint8_t maxBots;		// 1 - 32
	uint8_t fixedRuntime;
} BaseStationConfigV3;

typedef struct PACKED _BaseStationWifiStats	// 624
{
	struct PACKED _bot
	{
		uint8_t botId;

		uint8_t queueStats[22];

		int16_t botRssi; // [dBm*0.1]
		int16_t bsRssi;  // [dBm*0.1]
	} bots[32];

	uint16_t updateRate;	// [Hz]
} BaseStationWifiStats;

typedef struct PACKED _BaseStationEthStats
{
	uint32_t txFrames;
	uint32_t txBytes;

	uint32_t rxFrames;
	uint32_t rxBytes;

	uint16_t rxFramesDmaOverrun;

	uint8_t ntpSync;
} BaseStationEthStats;

typedef struct PACKED _BaseStationCamViewport
{
	uint8_t camId;
	int16_t minX;
	int16_t minY;
	int16_t maxX;
	int16_t maxY;
} BaseStationCamViewport;

#define EXT_BALL_DETECTIONS_MAX_BALLS 10
typedef struct PACKED _ExtBallDetections
{
    uint32_t timestampUs;
    float robotPos[3]; // X, Y, W at given timestamp

    struct PACKED _balls
    {
        // 3D position in global frame (map frame)
        // sorted by: distance to robot, unit: [m]
        float pos[3];
        float vel[3];
        float linePos[2];
        float lineDir[2];
        uint32_t trackerId;
    } balls[EXT_BALL_DETECTIONS_MAX_BALLS];

    uint8_t numBalls;
} ExtBallDetections;

#define EXT_SHUTDOWN_KEY 0xA5B6C7D8
typedef struct PACKED _ExtShutdown
{
	uint32_t key; // must be equal to EXT_SHUTDOWN_KEY in order to initiate shutdown of RPi
} ExtShutdown;

typedef struct PACKED _ExtRobotPiVersion
{
	// version number
	uint8_t major;
	uint8_t minor;

	// compile/build time
	char date[24];
} ExtRobotPiVersion;

typedef struct PACKED _ExtCameraConfig
{
	uint8_t expAutoMode;
	float expAnalogGain; // 1.0 - 8.0
	float expDigitalGain; // 1.0 - 2.0
	uint32_t expTimeUs; // maximum depends on framerate, if auto exposure is on this is the upper limit
	uint8_t wbAutoMode;
	float wbRedGain; // 0.0 - 8.0, typical 0.9 - 1.9
	float wbBlueGain; // 0.0 - 8.0, typical 0.9 - 1.9
} ExtCameraConfig;

typedef struct PACKED _ExtCameraPreviewLine160
{
	uint16_t row;
	uint16_t data[160];
} ExtCameraPreviewLine160;

typedef struct PACKED _ExtCameraPreviewConfig
{
	uint8_t enable;
} ExtCameraPreviewConfig;

typedef struct PACKED _ExtCameraControl
{
	uint8_t resolution; // 0 - low, 1 - medium, 2 - high
	uint8_t recording;
} ExtCameraControl;

typedef struct PACKED _ExtCameraStats
{
	uint16_t width;
	uint16_t height;

	float dtMin;
	float dtMax;
	float dtAvg;
	float dtDev;

	float rtMin;
	float rtMax;
	float rtAvg;
	float rtDev;

	uint8_t recording;
	char recordFilename[32];
	uint32_t recordSize;
	float recordDuration;

	uint32_t imagesTaken;
	char imageFilename[32];
} ExtCameraStats;

typedef struct PACKED _ExtCameraIntrinsics
{
	uint16_t resolution[2]; // resolution used during calibration (preferably native resolution)
	float focalLength; // in pixels, at calibration resolution. Assumed to be equal for X and Y.
	float principalPoint[2]; // in pixels, at calibration resolution.
	float radialDistortion[2];
} ExtCameraIntrinsics;

typedef struct PACKED _ExtCameraExtrinsics
{
	float cameraTiltDeg; // degree downward from horizontal plane (e.g. +20 is slightly downward)
	float cameraPos[3]; // translation from robot base to camera
	float groundClearance; // distance from floor to robot base (bottom side of robot's base plate)
} ExtCameraExtrinsics;

typedef struct PACKED _ExtRemoteTime
{
	uint32_t timestampUs;
} ExtRemoteTime;

typedef struct PACKED _ExtRobotState
{
	uint32_t timestampUs;

	float posGlobal[3];
	float velGlobal[3];
	float accGlobal[3];
} ExtRobotState;

typedef struct PACKED _ExtBallDetectionConfig
{
	float topImageSkipFactor;
	uint32_t minBlobArea;
	float ballDiameter;
	float greedyCircleFactor;
	uint32_t maxTrackers;
	float trackerTimeout;
} ExtBallDetectionConfig;

typedef struct PACKED _ExtBallTrackerConfig
{
	float modelError;
	float measError;
	float maxVelocity;
	uint32_t historySize;
} ExtBallTrackerConfig;

#define EXT_COLOR_THRESHOLDS_ID_ORANGE 0
#define EXT_COLOR_THRESHOLDS_ID_WHITE 1
#define EXT_COLOR_THRESHOLDS_ID_GREEN 2
typedef struct PACKED _ExtColorThresholds
{
    uint8_t colorId;
    uint8_t y[2];
    uint8_t u[2];
    uint8_t v[2];
} ExtColorThresholds;

typedef struct _ExtFieldInfo
{
	uint32_t timestampUs;

	float fieldSize[2]; // x,y in m inclusive field margin
	float botOrientation; // in rad relative to field
} ExtFieldInfo;

typedef struct _ExtPositionDetection
{
	uint32_t timestampUs;

	float pos[2]; // x,y in m in field coordinates
} ExtPositionDetection;

#define EXT_GOAL_DETECTIONS_MAX_GOALS 1
typedef struct _ExtGoalDetections
{
    uint32_t timestampUs;

    struct _goals
    {
        float leftAngle; // in rad (towards negative)
        float rightAngle; // in rad (towards positive)
    } goals[EXT_GOAL_DETECTIONS_MAX_GOALS];

    uint8_t numGoals;
} ExtGoalDetections;
