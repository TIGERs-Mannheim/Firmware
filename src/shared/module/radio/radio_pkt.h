#pragma once

#include "radio_settings.h"

#define RADIO_HEADER_CLIENT_ID_MASK	0x1F
#define RADIO_HEADER_BROADCAST		0x20
#define RADIO_HEADER_DIR_FROM_BASE	0x40
#define RADIO_HEADER_RESERVED		0x80

#pragma pack(push,1)

typedef struct _RadioModuleAirPacket
{
	uint8_t header;
	uint8_t seq;
	uint8_t rxSeqLoss;
	uint8_t payload[RADIO_AIR_PACKET_SIZE-3];
} RadioModuleAirPacket;

#pragma pack(pop)
