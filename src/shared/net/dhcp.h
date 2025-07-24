#pragma once

#include "udp.h"
#include "net_proto.h"
#include "ch.h"

#define DHCP_EVENT_CONFIGURATION_RECEIVED	EVENT_MASK(0)

typedef enum
{
	DHCP_MODE_DISABLED,
	DHCP_MODE_ACTIVE,
	DHCP_MODE_INFORM,
} DHCPMode;

typedef struct
{
	uint32_t type;
	uint32_t xid;

	struct
	{
		IPv4Address destinationIp;
		IPv4Address ciaddr;
		IPv4Address requestedIp;
		IPv4Address serverIdentifier;
	} request;

	struct
	{
		IPv4Address destinationIp;
		IPv4Address clientIp;
	} inform;

	uint32_t lastTransmitTime_s;
	uint32_t numTransmissions;
} DHCPOperation;

typedef struct
{
	IPv4Address clientIp;
	IPv4Address subnetMask;
	IPv4Address routerIp;
	IPv4Address dhcpServerIp;
	IPv4Address dnsServerIp;
	char domainName[256];
} DHCPResult;

typedef struct
{
	NetProto proto;
	NetIf* pIf;

	uint32_t state;
	uint32_t xid;
	DHCPOperation op;
	DHCPResult result;

	uint32_t renewalTime;
	uint32_t rebindTime;
	uint32_t leaseTime;
	uint8_t isInfiniteLease;

	uint32_t rxFramesDropped;
	uint32_t rxFramesGood;
	uint32_t txFamesDropped;
	uint32_t txFramesGood;

	event_source_t eventSource;

	THD_WORKING_AREA(waTask, 1024);
	thread_t* pTask;
} DHCP;

void DHCPInit(DHCP* pDhcp, NetIf* pIf);
void DHCPSetMode(DHCP* pDhcp, DHCPMode mode);
uint8_t DHCPIsResultValid(DHCP* pDhcp);
const char* DHCPGetCurrentStateName(DHCP* pDhcp);
void DHCPTriggerRenew(DHCP* pDhcp);
void DHCPTriggerRebind(DHCP* pDhcp);
void DHCPTriggerReinit(DHCP* pDhcp);
