#pragma once

#include "net_proto.h"

#define IPV4_MAX_PENDING_PACKETS 2

typedef struct _IPv4
{
	NetProto proto;
	NetIf* pIf;

	uint32_t rxFramesDropped;
	uint32_t rxFramesGood;
	uint32_t txFamesDropped;
	uint32_t txFramesGood;

	uint16_t txIdentificationCounter;

	List pendingPacketList;
} IPv4;

void IPv4Init(IPv4* pIp, NetIf* pIf);
int16_t IPv4Send(IPv4* pIp, NetPkt* pPkt);
int16_t IPv4PushRouterAlertOption(NetPkt* pPkt);
void* IPv4PullOptions(NetPkt* pPkt);
void IPv4SendPendingPackets(IPv4* pIp);
