#pragma once

#include "ch.h"
#include "net_pkt.h"
#include "util/shell_cmd.h"
#include "util/list.h"
#include "hal/eth.h"

#include "ethernet.h"
#include "arp.h"
#include "ipv4.h"
#include "icmp.h"
#include "igmp.h"
#include "udp.h"

#define NET_IF_PKT_POOL_SIZE 32

typedef struct _NetIfStats
{
	uint32_t rxFramesTotal;
	uint32_t rxFramesDropped;
} NetIfStats;

typedef struct _NetIfData
{
	MAC mac;
	IPv4Address ip;
	Eth* pEth;
} NetIfData;

typedef struct _NetIf
{
	NetIfData data;
	NetIfStats stats;

	Ethernet ethernet;
	Arp arp;
	IPv4 ipv4;
	ICMP icmp;
	UDP udp;
	IGMP igmp;

	NetProto udpUser;
	List udpSockets;

	guarded_memory_pool_t pktPool;
	NetPkt pktPoolData[NET_IF_PKT_POOL_SIZE];

	ShellCmdHandler cmdHandler;

	THD_WORKING_AREA(waTask, 512);
	thread_t* pTask;
} NetIf;

void NetIfInit(NetIf* pIf, NetIfData* pInit, tprio_t prio);
void NetIfSetIp(NetIf* pIf, IPv4Address ip);
NetPkt* NetIfAllocPkt(NetIf* pIf);
void NetIfFreePkt(NetIf* pIf, NetPkt* pPkt);
