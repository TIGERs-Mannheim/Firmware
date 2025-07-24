/**
 * Multicast Domain Name System
 *
 * According to: RFC 6762
 *
 * The current implementation violates the following parts of the RFC:
 * - Response delay, not implemented (sec 6)
 * - Probing, not implemented (sec. 8.1)
 * - Announcing, not implemented (sec. 8.3)
 * - Conflict resolution, not implemented (sec. 9)
 */

#pragma once

#include "net_proto.h"
#include "dns.h"

typedef struct
{
	NetProto proto;
	NetIf* pIf;

	IPv4Address multicastIp;
	char hostnameDotLocal[64];

	DNSMessage rxMessage;
	DNSMessage txMessage;

	uint32_t rxFramesDropped;
	uint32_t rxFramesGood;
	uint32_t txFamesDropped;
	uint32_t txFramesGood;
} MDNS;

void MDNSInit(MDNS* pMdns, NetIf* pIf);
