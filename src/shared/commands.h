#pragma once

#include <stdint.h>

#ifndef PACKED
#define PACKED __attribute__ ((packed))
#endif

#define PACKET_HEADER_SIZE 1
typedef struct PACKED _PacketHeader
{
	uint8_t cmd;
} PacketHeader;

#define PACKET_HEADER_EX_SIZE 3
typedef struct PACKED _PacketHeaderEx
{
	uint8_t cmd;
	uint16_t seq;
} PacketHeaderEx;

#define CMD_EXTENDED_HEADER_MASK		0x80

// SYSTEM
#define CMD_SYSTEM_PING					0x01
#define CMD_SYSTEM_PONG					0x02
#define CMD_SYSTEM_CONSOLE_PRINT		0x03
#define CMD_SYSTEM_CONSOLE_COMMAND		0x04
#define CMD_SYSTEM_MATCH_CTRL			0x05
#define CMD_SYSTEM_MATCH_FEEDBACK		0x06
#define CMD_SYSTEM_VERSION				0x07
#define CMD_SYSTEM_ACK					0x08

// CONFIG
#define CMD_CONFIG_QUERY_FILE_LIST		0x10
#define CMD_CONFIG_FILE_STRUCTURE		0x11
#define CMD_CONFIG_ITEM_DESC			0x12
#define CMD_CONFIG_READ					0x13
#define CMD_CONFIG_WRITE				0x14

// DATA_ACQ
#define CMD_DATA_ACQ_MOTOR_MODEL		0x20
#define CMD_DATA_ACQ_BOT_MODEL			0x21
#define CMD_DATA_ACQ_DELAYS				0x22
#define CMD_DATA_ACQ_SET_MODE			0x23
#define CMD_DATA_ACQ_BOT_MODEL_V2		0x24

// BASE_STATION
#define CMD_BASE_STATION_ACOMMAND		0x60
#define CMD_BASE_STATION_PING			0x61
#define CMD_BASE_STATION_AUTH			0x62
#define CMD_BASE_STATION_ETH_STATS		0x64
#define CMD_BASE_STATION_CAM_VIEWPORT	0x65
#define CMD_BASE_STATION_CONFIG_V3		0x66
#define CMD_BASE_STATION_BROADCAST		0x67
#define CMD_BASE_STATION_WIFI_STATS_V2	0x68

// BOT COUNT
#define CMD_BOT_COUNT_TOTAL 32
#define CMD_BOT_LAST_ID (CMD_BOT_COUNT_TOTAL-1)
#define CMD_BOT_COUNT_HALF (CMD_BOT_COUNT_TOTAL/2)
#define CMD_BOT_HALF_MIN_1 (CMD_BOT_COUNT_TOTAL/2-1)
#define CMD_BOT_BROADCAST_ID CMD_BOT_COUNT_TOTAL

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

#define BUZZ_NONE				0
#define BUZZ_UPDOWN20			1
#define BUZZ_SONG_FINAL			2
#define BUZZ_SONG_TETRIS		3
#define BUZZ_SONG_EYE_LEAD		4
#define BUZZ_SONG_EYE_FOLLOW	5
#define BUZZ_SONG_CANTINA		6
#define BUZZ_SONG_MACARENA		7
#define BUZZ_SONG_IMPERIAL		8

#define BUZZ_TADA				32
#define BUZZ_UP20				33
#define BUZZ_UP50				34
#define BUZZ_UP100				35
#define BUZZ_DOWN50				36
#define BUZZ_DOWN100			37
#define BUZZ_BEEP_FAST			38
#define BUZZ_DOUBLE_SLOW		39
#define BUZZ_SHORT_LOW			40
#define BUZZ_SHORT_MID			41
#define BUZZ_SHORT_HIGH			42

#define SYSTEM_MATCH_CTRL_FLAGS_LED_OFF				0
#define SYSTEM_MATCH_CTRL_FLAGS_LED_RED				1
#define SYSTEM_MATCH_CTRL_FLAGS_LED_GREEN			2
#define SYSTEM_MATCH_CTRL_FLAGS_LED_BLUE			3
#define SYSTEM_MATCH_CTRL_FLAGS_LED_WHITE			4
#define SYSTEM_MATCH_CTRL_FLAGS_LED_LIGHT_BLUE		5
#define SYSTEM_MATCH_CTRL_FLAGS_LED_GOLD			6
#define SYSTEM_MATCH_CTRL_FLAGS_LED_PURPLE			7

#define SYSTEM_MATCH_CTRL_FLAGS_SONG_MASK			0x1F
#define SYSTEM_MATCH_CTRL_FLAGS_LED_MASK			0xE0

#define SYSTEM_MATCH_CTRL_UNUSED_FIELD				0x7FFF

#define SYSTEM_MATCH_CTRL_USER_DATA_SIZE			16

typedef struct PACKED _SystemMatchCtrl	// 26
{
	int16_t curPosition[3];	// [mm]				6
	uint8_t posDelay;		// [Q6.2 ms]		1	// maxVal 63.75ms
	uint8_t camId;			// Vision camera ID	1
	uint8_t flags;			//					1 // songs/cheer, LEDs
	uint8_t skillId;		//					1
	uint8_t skillData[SYSTEM_MATCH_CTRL_USER_DATA_SIZE];	// skill dependent	16 = 26 B + 4B Header/COBS = 30B
} SystemMatchCtrl;

#define SYSTEM_MATCH_FEEDBACK_UNUSED_FIELD			0x7FFF
#define SYSTEM_MATCH_FEEDBACK_FLAGS_BARRIER_MASK	0x80
#define SYSTEM_MATCH_FEEDBACK_FLAGS_DRIB_TEMP_MASK	0x60 // low, medium, high, overheated
#define SYSTEM_MATCH_FEEDBACK_FLAGS_KICK_MASK		0x10
#define SYSTEM_MATCH_FEEDBACK_FLAGS_BALL_STATE_MASK	0x07

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

#define SYSTEM_MATCH_FEEDBACK_DRIBBLER_VEL_MASK		0xFC
#define SYSTEM_MATCH_FEEDBACK_DRIBBLER_STATE_MASK	0x03 // States: off, idle, weak, strong

#define SYSTEM_MATCH_FEEDBACK_LAST_KICK_DEVICE_CHIP 0x80

typedef struct PACKED _SystemMatchFeedback	// 29
{
	int16_t curPosition[3];			// [mm]			6
	int16_t curVelocity[3];			// [mm/s]		6
	uint8_t kickerLevel;			// [V]			1
	uint8_t kickerMax;				// [V]			1
	uint8_t dribblerState;			// [m/s / 4]	1	dribbling bar surface speed, includes traction strength
	uint8_t batteryLevel;			// [dV]			1
	uint8_t batteryPercent;			// 255 = 100%	1
	uint8_t flags;					//				1
	uint16_t features;				//				2	movement, dribbler, barrier, straight, chip
	uint8_t hardwareId;				//				1
	uint8_t ballPosAge;				// [ms]			1
	int16_t ballPosition[2];		// [mm]			4
	uint8_t lastKickDuration;		// [0.05ms]		1
	uint8_t lastKickDribbleVelDev;	// [0.0625m/s]	1	vel = 7 bits, kick device = 1 bit (MSB)
	uint8_t lastKickDribbleForce;	// [0.0625N]	1
} SystemMatchFeedback;

#define BASE_STATION_BROADCAST_FLAG_KICKER_AUTOCHG		0x01
#define BASE_STATION_BROADCAST_FLAG_STRICT_VEL_LIMIT	0x02
#define BASE_STATION_BROADCAST_FLAG_NO_VISION			0x80

#define BASE_STATION_BROADCAST_UNUSED_FIELD				0x7FFF

typedef struct PACKED _BaseStationBroadcast		//	22
{
	uint8_t baseStationId;		//					1
	uint32_t allocatedBotIds;	//					4
	uint32_t unixTime;			// [s]				4
	uint8_t flags;				//					1
	int16_t ballPosition[2];	// [mm]				4
	uint8_t ballPosDelay;		// [Q6.2 ms]		1	// maxVal 63.75ms
	uint8_t ballCamId;			// Vision camera ID	1
	uint8_t fieldSize[3];		// [cm]				3	// 12 bit for X and Y each
	uint8_t boundaryWidth;		// [cm]				1
	uint8_t goalWidth;			// [cm]				1
	uint8_t goalDepth;			// [cm]				1
} BaseStationBroadcast;

typedef struct PACKED _SystemVersion // 8
{
	uint32_t version;	// 4 [major.minor.patch.dirty]
	uint32_t gitRef;	// 4 [SHA1]
} SystemVersion;

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

typedef struct PACKED _BaseStationConfigV3
{
	uint8_t visionIp[4];
	uint16_t visionPort;

	uint8_t channel;		// = frequency
	uint8_t maxBots;		// 1 - 32
	uint8_t fixedRuntime;
} BaseStationConfigV3;

typedef struct PACKED _BaseStationWifiStatsV2	// 773
{
	struct PACKED _bot
	{
		struct PACKED _buf
		{
			uint32_t rxPacketsTotal; // all received packets, excl. packets lost on air
			uint32_t rxBytesTotal; // Used over-the-air bytes (excl. zero filling)
			uint32_t rxBufferOverruns; // Lost packets due to unavailable RadioBufferEntry owned by radio to put application packet
			uint32_t rxBufferDataOverflows; // Received data exceeds array size
			uint32_t rxCobsDecodingErrors;
			uint32_t rxCrcErrors;
			uint32_t txPacketsTotal; // all TX attempts, incl. packets not received by remote
			uint32_t txBytesTotal; // Used over-the-air bytes (excl. zero filling)
			uint32_t txBufferUnderrun; // No free TX buffer entry when attempting to enqueue application packet
		} buf;

		struct PACKED _ota
		{
			uint32_t rxPacketsLost;	// identified by gaps in sequence number
			uint32_t txPacketsLost;

			int16_t rxRssi; // [dBm*0.1]
		} ota;

		uint8_t botId;
	} bots[16];

	struct PACKED _module // Stats from SX1280 directly, not for a specific bot
	{
		uint32_t rxSyncErrors;
		uint32_t rxCrcErrors;
		uint32_t rxPacketsGood;
		uint32_t rxFromOtherBase;
		uint32_t cycleDuration_us;
		uint8_t numTimeslotsUsed;
	} module;
} BaseStationWifiStatsV2;

typedef struct PACKED _BaseStationEthStats
{
	uint32_t txFrames;
	uint32_t txBytes;

	uint32_t rxFrames;
	uint32_t rxBytes;

	uint16_t rxFramesDmaOverrun;
} BaseStationEthStats;

typedef struct PACKED _BaseStationCamViewport
{
	uint8_t camId;
	int16_t minX;
	int16_t minY;
	int16_t maxX;
	int16_t maxY;
} BaseStationCamViewport;
