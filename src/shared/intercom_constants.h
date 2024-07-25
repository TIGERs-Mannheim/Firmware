/*
 * intercom_sections.h
 *
 *  Created on: 11.11.2012
 *      Author: AndreR
 */

#pragma once

#include "commands.h"

// Sections
//#define INTERCOM_SECTION_CONTROL		0x01 deprecated
//#define INTERCOM_SECTION_CONSOLE		0x02 deprecated
#define INTERCOM_SECTION_BOOTLOADER		0x04
#define INTERCOM_SECTION_PRESENTER		0x05
#define INTERCOM_SECTION_SUPPLY			0x06
#define INTERCOM_SECTION_KD				0x07
#define INTERCOM_SECTION_COMMAND		0x08
#define INTERCOM_SECTION_NETWORK		0x09
#define INTERCOM_SECTION_MOTORS			0x0A
#define INTERCOM_SECTION_UI				0x0B
#define INTERCOM_SECTION_LOGFILE		0x0C
#define INTERCOM_SECTION_CONFIG			0x0D
#define INTERCOM_SECTION_TEST			0x0E
//#define INTERCOM_SECTION_RELIABLE		0x80 deprecated

// RELIABLE
#define IC_RELIABLE_DATA		0
#define IC_RELIABLE_ACK			1

// BOOTLOADER
#define BOOTLOADER_SELECT	0
#define BOOTLOADER_ERASE	1
#define BOOTLOADER_DATA		2
#define BOOTLOADER_RESPONSE	3
#define BOOTLOADER_CHECKSUM 5
#define BOOTLOADER_ACK		6
#define BOOTLOADER_NACK		7

// CONFIG PRESENTER
#define IC_PRESENTER_CONFIG			0
#define IC_PRESENTER_KICKER_STATUS	1
#define IC_PRESENTER_DISCHARGE		2
#define IC_PRESENTER_POWER			3
#define IC_PRESENTER_CFG_MISC		4
#define IC_PRESENTER_WIFI_STAT		5

// NETWORK
#define IC_NETWORK_NORMAL	0
#define IC_NETWORK_RELIABLE	1

// UI
#define IC_UI_WIFI_BOTID	0
#define IC_UI_WIFI_SETTINGS	1
#define IC_UI_WIFI_SAVE		2

// TEST
#define IC_TEST_START					0
#define IC_TEST_PROGRESS				1
#define IC_TEST_MOT_HALL				2
#define IC_TEST_MOT_POWER				3
#define IC_TEST_MOT_DYN					4
#define IC_TEST_MOT_ENC					5
#define IC_TEST_MOT_TRACTION			6
#define IC_TEST_KICKER					8
#define IC_TEST_MOT_IDENT_ELEC			9
#define IC_TEST_MOT_IDENT_MECH			10
#define IC_TEST_MOT_IDENT_ALL			11
#define IC_TEST_MOT_PHASE_RESISTANCE	12
#define IC_TEST_ROTATION_IDENT			13
#define IC_TEST_IMU_CALIB				14
#define IC_TEST_DRIBBLE_ROTATION		15
#define IC_TEST_BAT_STORAGE				16
#define IC_TEST_FRONT_LEDS				17

// Messages
#define IC_SUPPLY_POWER_LOG	0

#define IC_KD_FEEDBACK		0
#define IC_KD_DRIBBLER		1
#define IC_KD_AUTOCHG		2
#define IC_KD_CONFIG		3
#define IC_KD_FIRE			4

#define IC_MOTORS_MAIN2MOT 	0
#define IC_MOTORS_MOT2MAIN	1
#define IC_MOTORS_TEMP		2
#define IC_MOTORS_CTRL_TEMP_MEAS 3

#define IC_LOGFILE_TIMESYNC	0
#define IC_LOGFILE_MESSAGE	1

typedef struct PACKED _PresenterMainStatus
{
	struct
	{
		float cap;	// [V]
		float max;	// [V]
		float chg;	// [A]
		float temp; // [�C]
	} kicker;

	struct
	{
		float bat;	// [V]
		float min;	// [V]
		float max;	// [V]
		float cur;	// [A]
		uint8_t usbPowered;
	} power;

	struct
	{
		uint16_t level[2];	// IR ADC average
		uint16_t threshold;
		uint8_t txDamaged;
		uint8_t rxDamaged;
	} barrier;

	const ExtUpdateProgress* pExtUpdateProgress;
} PresenterMainStatus;

typedef struct PACKED _LogFileTimeSync
{
	uint32_t time;
	uint32_t unixTime;
} LogFileTimeSync;

typedef struct PACKED _ConfigNetwork
{
	uint8_t botId;
	uint8_t channel;		// frequency
} ConfigNetwork;

typedef struct PACKED _PresenterWifiStat
{
	uint8_t linkDisabled;
	uint8_t sumatraOnline;
	uint8_t bsOnline;
	uint16_t updateFreq;	// [Hz]
	uint16_t visionDelay;	// [us]
	float rssi; // [dBm]
} PresenterWifiStat;

typedef struct PACKED _BootloaderChecksum
{
	uint8_t index;
	uint16_t seq;
	uint32_t start;
	uint32_t end;
	uint32_t crc;
} BootloaderChecksum;

typedef struct PACKED _BootloaderSeq
{
	uint8_t index;
	uint16_t seq;
} BootloaderSeq;

typedef struct PACKED _BootloaderProgramData
{
	uint8_t index;
	uint16_t seq;
	uint32_t offset;
} BootloaderProgramData;
