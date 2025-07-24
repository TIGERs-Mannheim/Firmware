#pragma once

#include "util/cobs.h"

// Absolute maximum application payload size which can be processed
#define RADIO_APP_MAX_PACKET_SIZE 128

// Maximum number of buffered packets per robot/broadcast
#define RADIO_APP_MAX_BUFFERED_PACKETS 4

// Size of each packet over-the-air
#define RADIO_AIR_PACKET_SIZE 42

// Maximum number of broadcast time slots
#define RADIO_NUM_BROADCAST_TIMESLOTS_MAX 8

// Internal/Derived settings
#define RADIO_NUM_ROBOT_CLIENTS 	32
#define RADIO_NUM_TOTAL_CLIENTS 	(RADIO_NUM_ROBOT_CLIENTS+1)

#define RADIO_APP_TRAILER_SIZE 3 // CRC16 + Zero delimiter
#define RADIO_AIR_HEADER_SIZE 3 // Header + SEQ + RX_SEQ_LOSS

#define RADIO_APP_MAX_ENCODED_PACKET_SIZE_WITH_TRAILER ((COBSMaxStuffedSize(RADIO_APP_MAX_PACKET_SIZE)) + RADIO_APP_TRAILER_SIZE)

#define RADIO_APP_GUARANTEED_SINGLE_PACKET_PAYLOAD_SIZE (RADIO_AIR_PACKET_SIZE - RADIO_APP_TRAILER_SIZE - RADIO_AIR_HEADER_SIZE - 1)

typedef struct _RadioPhyTimeouts
{
	uint16_t default_us;
	uint16_t bufferIo_us;
	uint16_t modeChange_us;
	uint16_t txDelay_us;
	uint16_t rxWait_us;
	uint16_t rxWaitLong_us;
	uint16_t txWait_us;
} RadioPhyTimeouts;

typedef struct _RadioSettingsGFSK
{
	// Modulation Params
	uint8_t bitrateAndBandwidth; // One of GFSK_BR_...
	uint8_t modulationIndex;
	uint8_t modulationShaping;

	// Packet Params
	uint8_t preambleLength;
	uint8_t crcLength; // 0..2
	uint8_t enableWhitening;
	uint8_t longPreambleCount; // 15.625us steps

	// Sync Params
	uint32_t syncWord;
	uint8_t syncWordTolerance;
} RadioSettingsGFSK;

typedef struct _RadioSettingsFLRC
{
	// Modulation Params
	uint8_t bitrateAndBandwidth;
	uint8_t codingRate;
	uint8_t modulationShaping;

	// Packet Params
	uint8_t preambleLength;
	uint8_t crcLength;
	uint8_t enableWhitening;

	// Sync Params
	uint32_t syncWord;
	uint8_t syncWordTolerance;
} RadioSettingsFLRC;

typedef struct _RadioSettingsCommon
{
	uint32_t frequency; // in [Hz]
	uint8_t txPower; // 0 - 31 => -18 - 13dBm (be very careful not to exceed PA input limits!)
	int8_t rxGain; // 1..13 => Max-54..Max LNA gain, -1 => auto-gain
	uint8_t paRampTime;
	uint8_t highSensitivityMode;
} RadioSettingsCommon;

typedef struct _RadioSettings
{
	RadioPhyTimeouts phyTimeouts;

	uint8_t packetType;

	RadioSettingsCommon settingsCommon;
	RadioSettingsGFSK settingsGfsk;
	RadioSettingsFLRC settingsFlrc;

	uint32_t connectionTimeout_us;
} RadioSettings;

extern RadioSettings radioSettings;
