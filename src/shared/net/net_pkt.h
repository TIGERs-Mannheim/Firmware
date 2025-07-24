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
		MAC src; // By default set to the host MAC
		MAC dst;
	} ethernet;

	struct
	{
		uint8_t protocol; // Higher level protocol identifier
		IPv4Address src; // By default set to the host IP
		IPv4Address dst;
		uint8_t ttl; // By default set to 64
		uint16_t optionLength; // Additional option bytes already in buf
	} ipv4;

	struct
	{
		uint16_t srcPort;
		uint16_t dstPort;
	} udp;

	NetBuf buf;
} NetPkt;
