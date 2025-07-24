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
#include "dhcp.h"
#include "mdns.h"

#define NET_IF_PKT_POOL_SIZE 32

typedef struct _NetIfStats
{
	uint32_t rxFramesTotal;
	uint32_t rxFramesDropped;
} NetIfStatistics;

typedef enum
{
	NET_IF_IP_CONFIG_TYPE_STATIC,
	NET_IF_IP_CONFIG_TYPE_DHCP,
} NetIfIpConfigType;

typedef struct _NetIfData
{
	MAC mac;
	IPv4Address staticIp;
	NetIfIpConfigType ipConfigType;
	const char* pHostname;
	Eth* pEth;
} NetIfData;

typedef enum
{
	NET_IF_STATE_DISCONNECTED,
	NET_IF_STATE_CONFIGURING,
	NET_IF_STATE_CONNECTED,
} NetIfState;

typedef struct _NetIf
{
	MAC mac; // fix, will not change after init
	IPv4Address ip; // Active IP of this interface. May be statically assigned or from DHCP, or not set at all.
	const char* pHostname;

	NetIfIpConfigType ipConfigType;
	IPv4Address staticIp;

	NetIfState state;
	NetIfStatistics statistics;

	Eth* pEthDevice;
	Ethernet ethernet;
	Arp arp;
	IPv4 ipv4;
	ICMP icmp;
	UDP udp;
	IGMP igmp;
	DHCP dhcp;
	MDNS mdns;

	NetProto udpUser;
	List udpSockets;

	guarded_memory_pool_t pktPool;
	NetPkt pktPoolData[NET_IF_PKT_POOL_SIZE];

	ShellCmdHandler cmdHandler;

	THD_WORKING_AREA(waTask, 1024);
	thread_t* pTask;
} NetIf;

void NetIfInit(NetIf* pIf, NetIfData* pInit, tprio_t prio);
void NetIfSetIpConfigType(NetIf* pIf, NetIfIpConfigType type);
void NetIfSetStaticIp(NetIf* pIf, IPv4Address ip);
NetPkt* NetIfAllocPkt(NetIf* pIf);
void NetIfFreePkt(NetIf* pIf, NetPkt* pPkt);
