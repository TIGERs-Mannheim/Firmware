#pragma once

#include "radio_phy.h"
#include "radio_module.h"
#include "util/cobs.h"

// Absolute maximum application payload size which can be processed
#define RADIO_APP_MAX_PACKET_SIZE 128

// Maximum number of buffered packets per robot/broadcast
#define RADIO_APP_MAX_BUFFERED_PACKETS 4

// Size of each packet over-the-air
#define RADIO_AIR_PACKET_SIZE 55

// Maximum number of broadcast time slots
#define RADIO_NUM_BROADCAST_TIMESLOTS_MAX 8

// Internal/Derived settings
#define RADIO_NUM_ROBOT_CLIENTS 	32
#define RADIO_NUM_TOTAL_CLIENTS 	(RADIO_NUM_ROBOT_CLIENTS+1)

#define RADIO_APP_TRAILER_SIZE 3 // CRC16 + Zero delimiter
#define RADIO_AIR_HEADER_SIZE 2 // Header + Inverted Header

#define RADIO_APP_MAX_ENCODED_PACKET_SIZE_WITH_TRAILER ((COBSMaxStuffedSize(RADIO_APP_MAX_PACKET_SIZE)) + RADIO_APP_TRAILER_SIZE)

#define RADIO_APP_GUARANTEED_SINGLE_PACKET_PAYLOAD_SIZE (RADIO_AIR_PACKET_SIZE - RADIO_APP_TRAILER_SIZE - RADIO_AIR_HEADER_SIZE - 1)

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
