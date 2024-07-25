#pragma once

#include "inet.h"
#include "net_buf.h"
#include "util/list.h"

typedef struct _NetPkt
{
	void* pNext; // Must be the first element, enables usage in list
	uint32_t refCount;

	struct
	{
		uint16_t type; // Higher level protocol type
		MAC src; // If empty on transmit, the host MAC is filled in
		MAC dst;
	} ethernet;

	struct
	{
		uint8_t protocol; // Higher level protocol identifier
		IPv4Address src; // If empty on transmit, the host IP is filled in
		IPv4Address dst;
		uint8_t ttl; // If zero on transmit: set to 64
		uint16_t optionLength; // Additional option bytes already in buf
	} ipv4;

	struct
	{
		uint16_t srcPort;
		uint16_t dstPort;
	} udp;

	NetBuf buf;
} NetPkt;
