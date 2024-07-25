#pragma once

#include "net_proto.h"

#define ARP_CACHE_SIZE 16

typedef struct _ArpCacheEntry
{
	IPv4Address ip;
	MAC mac;
	uint32_t tLastUse;
} ArpCacheEntry;

typedef struct _Arp
{
	NetProto proto;
	NetIf* pIf;

	ArpCacheEntry cache[ARP_CACHE_SIZE];

	uint32_t rxFramesDropped;
	uint32_t rxFramesGood;
	uint32_t txFamesDropped;
	uint32_t txFramesGood;
} Arp;

void ArpInit(Arp* pArp, NetIf* pIf);
int16_t ArpSend(Arp* pArp, uint8_t operation, MAC targetMAC, IPv4Address targetIP);
const MAC* ArpCacheLookup(Arp* pArp, IPv4Address ip);
