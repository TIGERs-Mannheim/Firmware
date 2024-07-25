#pragma once

#include "net_proto.h"

typedef struct _ICMP
{
	NetProto proto;
	NetIf* pIf;

	uint32_t rxFramesDropped;
	uint32_t rxFramesGood;
	uint32_t txFamesDropped;
	uint32_t txFramesGood;

	uint16_t txEchoRequestSequenceNumber;
	uint32_t tEchoRequestSent;

	uint16_t pendingEchoRequestSeq;
} ICMP;

void ICMPInit(ICMP* pIcmp, NetIf* pIf);
int16_t ICMPSendEchoRequest(ICMP* pIcmp, IPv4Address dstIp);
