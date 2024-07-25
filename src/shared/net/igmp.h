#pragma once

#include "net_proto.h"
#include "ch.h"

#define IGMP_MAX_JOINED_GROUPS 8
#define IGMP_V3_MAX_SOURCES_IN_LIST 64

typedef struct _IGMP IGMP;

typedef struct _IGMPMulticastGroup
{
	IPv4Address ip;
	uint32_t numReferences;
	virtual_timer_t timer;
	volatile uint8_t timerExpired;
	uint32_t numResponsesLeft;

	IPv4Address v3SourcesList[IGMP_V3_MAX_SOURCES_IN_LIST];
	size_t sourcesUsed;

	IGMP* pIgmp;
} IGMPMulticastGroup;

typedef struct _IGMPConfig
{
	uint32_t robustnessVariable;
	uint32_t queryInterval_ms;
	uint32_t queryResponseInterval_ms;
} IGMPConfig;

typedef struct _IGMP
{
	NetProto proto;
	NetIf* pIf;

	IGMPMulticastGroup groups[IGMP_MAX_JOINED_GROUPS];
	IGMPConfig config;

	volatile uint8_t isV1QuerierPresent;
	volatile uint8_t isV2QuerierPresent;
	virtual_timer_t v1QuerierTimeout;
	virtual_timer_t v2QuerierTimeout;

	virtual_timer_t v3ResponseTimer;
	uint32_t numV3ResponsesLeft;

	uint32_t rxFramesDropped;
	uint32_t rxFramesGood;
	uint32_t txFamesDropped;
	uint32_t txFramesGood;

	THD_WORKING_AREA(waTask, 512);
	thread_t* pTask;
} IGMP;

void IGMPInit(IGMP* pIgmp, NetIf* pIf, tprio_t prio);
int16_t IGMPJoinGroup(IGMP* pIgmp, IPv4Address multicastIp);
int16_t IGMPLeaveGroup(IGMP* pIgmp, IPv4Address multicastIp);
void IGMPSendAllMemberships(IGMP* pIgmp);
