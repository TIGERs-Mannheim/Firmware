/*
 * IGMP according to RFCs 1112 (IGMPv1), 2236 (IGMPv2), 3376 (IGMPv3)
 *
 * https://datatracker.ietf.org/doc/html/rfc1112
 * https://datatracker.ietf.org/doc/html/rfc2236
 * https://datatracker.ietf.org/doc/html/rfc3376
 *
 * This implementation does not support source-filtering for IGMPv3.
 */

#include "igmp.h"
#include "net_if.h"
#include "errors.h"
#include "hal/rng.h"
#include <stdarg.h>

#define IGMP_DEBUG_ON 0

#define trace(fmt, ...) \
            do { if (IGMP_DEBUG_ON) printf(fmt, ##__VA_ARGS__); } while (0)

// Mesage Types
#define IGMP_MEMBERSHIP_QUERY		0x11
#define IGMPV3_MEMBERSHIP_REPORT	0x22
#define IGMPV1_MEMBERSHIP_REPORT	0x12
#define IGMPV2_MEMBERSHIP_REPORT	0x16
#define IGMPV2_LEAVE_GROUP			0x17

// Group Record Types
#define IGMPV3_MODE_IS_INCLUDE	0x01
#define IGMPV3_MODE_IS_EXCLUDE	0x02
#define IGMPV3_CHANGE_TO_INCLUDE	0x03
#define IGMPV3_CHANGE_TO_EXCLUDE	0x04
#define IGMPV3_ALLOW_NEW_SOURCES	0x05
#define IGMPV3_BLOCK_OLD_SOURCES	0x06

#define IGMP_EVENT_GROUP_RESPONSE_TIMER_EXPIRED	EVENT_MASK(0)
#define IGMP_EVENT_V3_RESPONSE_TIMER_EXPIRED	EVENT_MASK(1)

static int16_t igmpv2SendMembershipReport(IGMP* pIgmp, IPv4Address multicastIp, uint8_t type);
int16_t igmpv3SendMembershipReportGroupSpecific(IGMPMulticastGroup* pGroup, uint8_t recordType);
static void igmpTask(void* pParam);
static void groupTimerExpired(virtual_timer_t* pTimer, void* pUser);
static NetVerdict receive(NetPkt* pPkt, void* pUser);

void IGMPInit(IGMP* pIgmp, NetIf* pIf, tprio_t prio)
{
	pIgmp->proto.pReceiveFunc = &receive;
	pIgmp->proto.pUser = pIgmp;
	pIgmp->pIf = pIf;

	for(size_t i = 0; i < IGMP_MAX_JOINED_GROUPS; i++)
	{
		pIgmp->groups[i].pIgmp = pIgmp;
		chVTObjectInit(&pIgmp->groups[i].timer);
	}

	chVTObjectInit(&pIgmp->v1QuerierTimeout);
	chVTObjectInit(&pIgmp->v2QuerierTimeout);
	chVTObjectInit(&pIgmp->v3ResponseTimer);

	pIgmp->config.robustnessVariable = 2;
	pIgmp->config.queryInterval_ms = 125000;
	pIgmp->config.queryResponseInterval_ms = 10000;

	pIgmp->pTask = chThdCreateStatic(pIgmp->waTask, sizeof(pIgmp->waTask), prio, &igmpTask, pIgmp);
}

int16_t IGMPJoinGroup(IGMP* pIgmp, IPv4Address multicastIp)
{
	IGMPMulticastGroup* pFreeGroup = 0;

	for(size_t i = 0; i < IGMP_MAX_JOINED_GROUPS; i++)
	{
		IGMPMulticastGroup* pGroup = &pIgmp->groups[i];
		if(pGroup->ip.u32 == multicastIp.u32)
		{
			pGroup->numReferences++;
			return 0;
		}

		if(pGroup->numReferences == 0 && !pFreeGroup)
			pFreeGroup = pGroup;
	}

	if(!pFreeGroup)
		return ERROR_NOT_ENOUGH_MEMORY;

	pFreeGroup->ip = multicastIp;
	pFreeGroup->numReferences = 1;
	chVTReset(&pFreeGroup->timer);
	pFreeGroup->timerExpired = 0;
	pFreeGroup->sourcesUsed = 0;

	igmpv2SendMembershipReport(pIgmp, multicastIp, IGMPV1_MEMBERSHIP_REPORT);
	igmpv2SendMembershipReport(pIgmp, multicastIp, IGMPV2_MEMBERSHIP_REPORT);
	igmpv3SendMembershipReportGroupSpecific(pFreeGroup, IGMPV3_CHANGE_TO_EXCLUDE);

	return 0;
}

int16_t IGMPLeaveGroup(IGMP* pIgmp, IPv4Address multicastIp)
{
	IGMPMulticastGroup* pLeaveGroup = 0;

	for(size_t i = 0; i < IGMP_MAX_JOINED_GROUPS; i++)
	{
		IGMPMulticastGroup* pGroup = &pIgmp->groups[i];
		if(pGroup->ip.u32 == multicastIp.u32 && pGroup->numReferences)
		{
			pGroup->numReferences--;
			if(pGroup->numReferences == 0)
			{
				pLeaveGroup = pGroup;
				break;
			}

			return 0;
		}
	}

	if(!pLeaveGroup)
		return ERROR_INVALID_PARAMETER;

	chVTReset(&pLeaveGroup->timer);
	pLeaveGroup->timerExpired = 0;
	pLeaveGroup->sourcesUsed = 0;

	if(pIgmp->isV1QuerierPresent)
		return 0; // Leave message not available in IGMPv1

	if(pIgmp->isV2QuerierPresent)
	{
		igmpv2SendMembershipReport(pIgmp, multicastIp, IGMPV2_LEAVE_GROUP);
		return 0;
	}

	igmpv3SendMembershipReportGroupSpecific(pLeaveGroup, IGMPV3_CHANGE_TO_INCLUDE);

	return 0;
}

void IGMPSendAllMemberships(IGMP* pIgmp)
{
	for(size_t i = 0; i < IGMP_MAX_JOINED_GROUPS; i++)
	{
		IGMPMulticastGroup* pGroup = &pIgmp->groups[i];
		if(pGroup->numReferences == 0)
			continue;

		chVTReset(&pGroup->timer);
		pGroup->timerExpired = 0;
		pGroup->sourcesUsed = 0;

		igmpv2SendMembershipReport(pIgmp, pGroup->ip, IGMPV1_MEMBERSHIP_REPORT);
		igmpv2SendMembershipReport(pIgmp, pGroup->ip, IGMPV2_MEMBERSHIP_REPORT);
		igmpv3SendMembershipReportGroupSpecific(pGroup, IGMPV3_CHANGE_TO_EXCLUDE);
	}
}

static int16_t igmpv2SendMembershipReport(IGMP* pIgmp, IPv4Address multicastIp, uint8_t type)
{
	NetIf* pIf = pIgmp->pIf;

	NetPkt* pPkt = NetIfAllocPkt(pIf);
	if(!pPkt)
	{
		pIgmp->txFamesDropped++;
		return ERROR_NOT_ENOUGH_MEMORY;
	}

	int16_t pushAlertResult = IPv4PushRouterAlertOption(pPkt);
	IGMPv2MembershipReport* pHeader = NetBufAdd(&pPkt->buf, sizeof(IGMPv2MembershipReport));
	if(!pHeader || pushAlertResult)
	{
		pIgmp->txFamesDropped++;
		NetIfFreePkt(pIf, pPkt);
		return ERROR_NOT_ENOUGH_MEMORY;
	}

	pHeader->type = type;
	pHeader->maxRespTime = 0;
	pHeader->groupAddress = multicastIp;

	pHeader->checksum = IPv4CalcChecksum(pHeader, sizeof(IGMPv2MembershipReport));

	pPkt->ipv4.protocol = IPV4_PROTOCOL_IGMP;
	pPkt->ipv4.ttl = 1;
	pPkt->ipv4.dst = multicastIp;

	pIgmp->txFramesGood++;

	return IPv4Send(&pIf->ipv4, pPkt);
}

static int16_t igmpv3SendMembershipGeneralResponse(IGMP* pIgmp)
{
	NetIf* pIf = pIgmp->pIf;

	NetPkt* pPkt = NetIfAllocPkt(pIf);
	if(!pPkt)
	{
		pIgmp->txFamesDropped++;
		return ERROR_NOT_ENOUGH_MEMORY;
	}

	int16_t pushAlertResult = IPv4PushRouterAlertOption(pPkt);

	void* pDataStart = NetBufGetTail(&pPkt->buf);

	IGMPv3MembershipReport* pHeader = NetBufAdd(&pPkt->buf, sizeof(IGMPv3MembershipReport));
	if(!pHeader || pushAlertResult)
	{
		pIgmp->txFamesDropped++;
		NetIfFreePkt(pIf, pPkt);
		return ERROR_NOT_ENOUGH_MEMORY;
	}

	pHeader->type = IGMPV3_MEMBERSHIP_REPORT;
	pHeader->reserved1 = 0;
	pHeader->reserved2 = 0;
	pHeader->checksum = 0;

	uint16_t numberOfGroups = 0;

	for(size_t i = 0; i < IGMP_MAX_JOINED_GROUPS; i++)
	{
		IGMPMulticastGroup* pGroup = &pIgmp->groups[i];
		if(pGroup->numReferences == 0)
			continue;

		IGMPv3GroupRecord* pRecord = NetBufAdd(&pPkt->buf, sizeof(IGMPv3GroupRecord));
		if(!pRecord)
		{
			pIgmp->txFamesDropped++;
			NetIfFreePkt(pIf, pPkt);
			return ERROR_NOT_ENOUGH_MEMORY;
		}

		pRecord->auxDataLen = 0;
		pRecord->numberOfSources = 0;
		pRecord->recordType = IGMPV3_MODE_IS_EXCLUDE;
		pRecord->multicastAddress = pGroup->ip;

		numberOfGroups++;
	}

	if(numberOfGroups == 0)
	{
		trace("IGMPv3 general query response dropped, no records\r\n");
		NetIfFreePkt(pIf, pPkt);
		return ERROR_NOT_ENOUGH_MEMORY;
	}

	pHeader->numberOfGroupRecords = htons(numberOfGroups);

	void* pDataEnd = NetBufGetTail(&pPkt->buf);

	pHeader->checksum = IPv4CalcChecksum(pHeader, pDataEnd - pDataStart);

	pPkt->ipv4.protocol = IPV4_PROTOCOL_IGMP;
	pPkt->ipv4.ttl = 1;
	pPkt->ipv4.dst = IPv4AddressSet(224, 0, 0, 22);

	pIgmp->txFramesGood++;

	return IPv4Send(&pIf->ipv4, pPkt);
}

int16_t igmpv3SendMembershipReportGroupSpecific(IGMPMulticastGroup* pGroup, uint8_t recordType)
{
	IGMP* pIgmp = pGroup->pIgmp;
	NetIf* pIf = pIgmp->pIf;

	NetPkt* pPkt = NetIfAllocPkt(pIf);
	if(!pPkt)
	{
		pIgmp->txFamesDropped++;
		return ERROR_NOT_ENOUGH_MEMORY;
	}

	int16_t pushAlertResult = IPv4PushRouterAlertOption(pPkt);

	void* pDataStart = NetBufGetTail(&pPkt->buf);

	IGMPv3MembershipReport* pHeader = NetBufAdd(&pPkt->buf, sizeof(IGMPv3MembershipReport));
	if(!pHeader || pushAlertResult)
	{
		pIgmp->txFamesDropped++;
		NetIfFreePkt(pIf, pPkt);
		return ERROR_NOT_ENOUGH_MEMORY;
	}

	pHeader->type = IGMPV3_MEMBERSHIP_REPORT;
	pHeader->reserved1 = 0;
	pHeader->reserved2 = 0;
	pHeader->checksum = 0;
	pHeader->numberOfGroupRecords = htons(1);

	IGMPv3GroupRecord* pRecord = NetBufAdd(&pPkt->buf, sizeof(IGMPv3GroupRecord));
	if(!pRecord)
	{
		pIgmp->txFamesDropped++;
		NetIfFreePkt(pIf, pPkt);
		return ERROR_NOT_ENOUGH_MEMORY;
	}

	pRecord->multicastAddress = pGroup->ip;
	pRecord->auxDataLen = 0;
	pRecord->numberOfSources = htons(pGroup->sourcesUsed);
	pRecord->recordType = recordType;

	if(pGroup->sourcesUsed)
	{
		void* pSourcesDst = NetBufAdd(&pPkt->buf, pGroup->sourcesUsed * sizeof(IPv4Address));
		if(!pSourcesDst)
		{
			pIgmp->txFamesDropped++;
			NetIfFreePkt(pIf, pPkt);
			return ERROR_NOT_ENOUGH_MEMORY;
		}

		memcpy(pSourcesDst, pGroup->v3SourcesList, pGroup->sourcesUsed * sizeof(IPv4Address));
	}

	void* pDataEnd = NetBufGetTail(&pPkt->buf);

	pHeader->checksum = IPv4CalcChecksum(pHeader, pDataEnd - pDataStart);

	pPkt->ipv4.protocol = IPV4_PROTOCOL_IGMP;
	pPkt->ipv4.ttl = 1;
	pPkt->ipv4.dst = IPv4AddressSet(224, 0, 0, 22);

	pIgmp->txFramesGood++;

	return IPv4Send(&pIf->ipv4, pPkt);
}

static void handleGroupResponse(IGMPMulticastGroup* pGroup)
{
	IGMP* pIgmp = pGroup->pIgmp;

	if(pGroup->numReferences == 0 || !pGroup->timerExpired || pGroup->numResponsesLeft == 0)
		return;

	if(pIgmp->isV1QuerierPresent)
	{
		trace("Sending IGMPv1 membership report\r\n");
		igmpv2SendMembershipReport(pIgmp, pGroup->ip, IGMPV1_MEMBERSHIP_REPORT);
	}
	else if(pIgmp->isV2QuerierPresent)
	{
		trace("Sending IGMPv2 membership report\r\n");
		igmpv2SendMembershipReport(pIgmp, pGroup->ip, IGMPV2_MEMBERSHIP_REPORT);
	}
	else
	{
		trace("Sending IGMPv3 group-specific membership report\r\n");
		igmpv3SendMembershipReportGroupSpecific(pGroup, pGroup->sourcesUsed ? IGMPV3_MODE_IS_INCLUDE : IGMPV3_MODE_IS_EXCLUDE);
	}

	pGroup->numResponsesLeft--;
	if(pGroup->numResponsesLeft)
	{
		pGroup->timerExpired = 0;
		chVTSet(&pGroup->timer, TIME_MS2I(1000), &groupTimerExpired, pGroup);
	}
	else
	{
		pGroup->sourcesUsed = 0;
	}
}

static void v1QuerierPresentTimerExpired(virtual_timer_t* pTimer, void* pUser)
{
	(void)pTimer;
	IGMP* pIgmp = pUser;
	pIgmp->isV1QuerierPresent = 0;
}

static void v2QuerierPresentTimerExpired(virtual_timer_t* pTimer, void* pUser)
{
	(void)pTimer;
	IGMP* pIgmp = pUser;
	pIgmp->isV2QuerierPresent = 0;
}

static void groupTimerExpired(virtual_timer_t* pTimer, void* pUser)
{
	(void)pTimer;
	IGMPMulticastGroup* pGroup = pUser;
	pGroup->timerExpired = 1;
	chSysLockFromISR();
	chEvtSignalI(pGroup->pIgmp->pTask, IGMP_EVENT_GROUP_RESPONSE_TIMER_EXPIRED);
	chSysUnlockFromISR();
}

static void v3ResponseTimerExpired(virtual_timer_t* pTimer, void* pUser)
{
	(void)pTimer;
	IGMP* pIgmp = pUser;
	chSysLockFromISR();
	chEvtSignalI(pIgmp->pTask, IGMP_EVENT_V3_RESPONSE_TIMER_EXPIRED);
	chSysUnlockFromISR();
}

static void igmpTask(void* pParam)
{
	IGMP* pIgmp = (IGMP*)pParam;

	chRegSetThreadName("IGMP");

	while(1)
	{
		eventmask_t events = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(10));

		if(events & IGMP_EVENT_GROUP_RESPONSE_TIMER_EXPIRED)
		{
			for(size_t i = 0; i < IGMP_MAX_JOINED_GROUPS; i++)
			{
				handleGroupResponse(&pIgmp->groups[i]);
			}
		}

		if(events & IGMP_EVENT_V3_RESPONSE_TIMER_EXPIRED)
		{
			trace("Sending IGMPv3 general membership report\r\n");

			igmpv3SendMembershipGeneralResponse(pIgmp);

			pIgmp->numV3ResponsesLeft--;
			if(pIgmp->numV3ResponsesLeft)
			{
				chVTSet(&pIgmp->v3ResponseTimer, TIME_MS2I(1000), &v3ResponseTimerExpired, pIgmp);
			}
		}
	}
}

static sysinterval_t getRandomInterval_ms(uint32_t maxInterval_ms)
{
	return (RNGGetRandomU32() % maxInterval_ms) + 1;
}

static void scheduleResponse(IGMPMulticastGroup* pGroup, uint32_t maxResponseTime_ms)
{
	if(pGroup->numReferences == 0)
		return;

	uint32_t reportDelay_ms = getRandomInterval_ms(maxResponseTime_ms);

	if(chVTIsArmed(&pGroup->timer))
	{
		return;
	}

	trace("Scheduled reponse for group %hu.%hu.%hu.%hu in %ums\r\n",
			(uint16_t)pGroup->ip.u8[0], (uint16_t)pGroup->ip.u8[1], (uint16_t)pGroup->ip.u8[2], (uint16_t)pGroup->ip.u8[3], reportDelay_ms);

	pGroup->numResponsesLeft = pGroup->pIgmp->config.robustnessVariable;
	pGroup->timerExpired = 0;
	chVTSet(&pGroup->timer, TIME_MS2I(reportDelay_ms), &groupTimerExpired, pGroup);
}

static uint32_t v3DecodeMaxRespCode_ms(uint8_t code)
{
	uint32_t tenth = 0;

	if(code & 0x80)
	{
		tenth = ((code & 0x0F) | 0x10) << (((code & 0x70) >> 4) + 3);
	}
	else
	{
		tenth = code;
	}

	return tenth * 100;
}

static void handleIGMPv1Query(IGMP* pIgmp)
{
	if(pIgmp->isV1QuerierPresent == 0)
	{
		// Compatibility mode change, stop all response and retransmission timers
		for(size_t i = 0; i < IGMP_MAX_JOINED_GROUPS; i++)
		{
			chVTReset(&pIgmp->groups[i].timer);
			pIgmp->groups[i].numResponsesLeft = 0;
			pIgmp->groups[i].timerExpired = 0;
		}
	}

	pIgmp->isV1QuerierPresent = 1;
	uint32_t olderVersionQuerierPresentTimeout_ms = pIgmp->config.robustnessVariable * pIgmp->config.queryInterval_ms + pIgmp->config.queryResponseInterval_ms;
	chVTSet(&pIgmp->v1QuerierTimeout, TIME_MS2I(olderVersionQuerierPresentTimeout_ms), &v1QuerierPresentTimerExpired, pIgmp);

	trace("IGMPv1 present timer set\r\n");

	for(size_t i = 0; i < IGMP_MAX_JOINED_GROUPS; i++)
	{
		IGMPMulticastGroup* pGroup = &pIgmp->groups[i];
		scheduleResponse(pGroup, 10000);
	}
}

static void handleIGMPv2GeneralQuery(IGMP* pIgmp, uint32_t maxResponseTime_ms)
{
	if(pIgmp->isV2QuerierPresent == 0)
	{
		// Compatibility mode change, stop all response and retransmission timers
		for(size_t i = 0; i < IGMP_MAX_JOINED_GROUPS; i++)
		{
			chVTReset(&pIgmp->groups[i].timer);
			pIgmp->groups[i].numResponsesLeft = 0;
			pIgmp->groups[i].timerExpired = 0;
		}
	}

	pIgmp->isV2QuerierPresent = 1;
	uint32_t olderVersionQuerierPresentTimeout_ms = pIgmp->config.robustnessVariable * pIgmp->config.queryInterval_ms + pIgmp->config.queryResponseInterval_ms;
	chVTSet(&pIgmp->v2QuerierTimeout, TIME_MS2I(olderVersionQuerierPresentTimeout_ms), &v2QuerierPresentTimerExpired, pIgmp);

	trace("IGMPv3 present timer set\r\n");

	for(size_t i = 0; i < IGMP_MAX_JOINED_GROUPS; i++)
	{
		IGMPMulticastGroup* pGroup = &pIgmp->groups[i];
		scheduleResponse(pGroup, maxResponseTime_ms);
	}
}

static void handleIGMPv2GroupSpecificQuery(IGMP* pIgmp, IPv4Address groupAddress, uint32_t maxResponseTime_ms)
{
	for(size_t i = 0; i < IGMP_MAX_JOINED_GROUPS; i++)
	{
		IGMPMulticastGroup* pGroup = &pIgmp->groups[i];
		if(pGroup->ip.u32 != groupAddress.u32)
			continue;

		scheduleResponse(pGroup, maxResponseTime_ms);
	}
}

static void handleIGMPv3GeneralQuery(IGMP* pIgmp, uint32_t maxResponseTime_ms)
{
	if(!chVTIsArmed(&pIgmp->v3ResponseTimer))
	{
		uint32_t reportDelay_ms = getRandomInterval_ms(maxResponseTime_ms);
		trace("Scheduled IGMPv3 response timer to %ums\r\n", reportDelay_ms);
		pIgmp->numV3ResponsesLeft = pIgmp->config.robustnessVariable;
		chVTSet(&pIgmp->v3ResponseTimer, TIME_MS2I(reportDelay_ms), &v3ResponseTimerExpired, pIgmp);
	}
}

static NetVerdict handleIGMPv3GroupSpecificQuery(IGMP* pIgmp, IGMPv3MembershipQuery* pQuery, NetBuf* pData)
{
	uint16_t numberOfSources = ntohs(pQuery->numberOfSources);
	uint32_t maxResponseTime_ms = v3DecodeMaxRespCode_ms(pQuery->maxRespCode);

	for(size_t i = 0; i < IGMP_MAX_JOINED_GROUPS; i++)
	{
		IGMPMulticastGroup* pGroup = &pIgmp->groups[i];
		if(pGroup->ip.u32 != pQuery->groupAddress.u32 || pGroup->numReferences == 0)
			continue;

		trace("Active multicast group found, sources in query: %hu\r\n", numberOfSources);

		if(chVTIsArmed(&pGroup->timer))
		{
			if(numberOfSources == 0)
			{
				pGroup->sourcesUsed = 0;
			}
			else
			{
				// append sources
				uint32_t sourcesToAdd = numberOfSources;
				if(pGroup->sourcesUsed + sourcesToAdd > IGMP_V3_MAX_SOURCES_IN_LIST)
					sourcesToAdd = IGMP_V3_MAX_SOURCES_IN_LIST - pGroup->sourcesUsed;

				uint32_t sourcesSize = sourcesToAdd * sizeof(IPv4Address);
				void* pSources = NetBufPull(pData, sourcesSize);
				if(!pSources)
				{
					pIgmp->rxFramesDropped++;
					return NET_VERDICT_DROP;
				}

				memcpy(pGroup->v3SourcesList + pGroup->sourcesUsed, pSources, sourcesSize);
				pGroup->sourcesUsed += sourcesToAdd;
			}
		}
		else
		{
			// set sources to received
			uint32_t sourcesToAdd = numberOfSources;
			if(sourcesToAdd > IGMP_V3_MAX_SOURCES_IN_LIST)
				sourcesToAdd = IGMP_V3_MAX_SOURCES_IN_LIST;

			uint32_t sourcesSize = sourcesToAdd * sizeof(IPv4Address);
			void* pSources = NetBufPull(pData, sourcesSize);
			if(!pSources)
			{
				pIgmp->rxFramesDropped++;
				return NET_VERDICT_DROP;
			}

			memcpy(pGroup->v3SourcesList, pSources, sourcesSize);
			pGroup->sourcesUsed = sourcesToAdd;

			scheduleResponse(pGroup, maxResponseTime_ms);
		}
	}

	return NET_VERDICT_OK;
}

static NetVerdict receive(NetPkt* pPkt, void* pUser)
{
	IGMP* pIgmp = (IGMP*)pUser;
	NetBuf* pData = &pPkt->buf;

	if(pPkt->ipv4.protocol != IPV4_PROTOCOL_IGMP)
		return NET_VERDICT_CONTINUE;

	IPv4PullOptions(pPkt);

	size_t dataSize = NetBufGetSizeUsed(pData);
	if(dataSize < 4)
	{
		pIgmp->rxFramesDropped++;
		return NET_VERDICT_DROP;
	}

	uint16_t checksum = IPv4CalcChecksum(NetBufGetHead(pData), dataSize);
	if(checksum != 0)
	{
		pIgmp->rxFramesDropped++;
		return NET_VERDICT_DROP;
	}

	uint8_t type = *((uint8_t*)NetBufGetHead(pData));

	if(type == IGMP_MEMBERSHIP_QUERY)
	{
		const IPv4Address allSystemsAddress = IPv4AddressSet(224, 0, 0, 1);

		trace("IGMP Membership Query received on IP %hu.%hu.%hu.%hu, dataSize: %u\r\n",
				(uint16_t)pPkt->ipv4.dst.u8[0], (uint16_t)pPkt->ipv4.dst.u8[1], (uint16_t)pPkt->ipv4.dst.u8[2], (uint16_t)pPkt->ipv4.dst.u8[3], dataSize);

		if(dataSize < sizeof(IGMPv3MembershipQuery))
		{
			// Message is short, this must be an IGMPv1 or v2 query
			IGMPv2MembershipQuery* pQuery = NetBufPull(pData, sizeof(IGMPv2MembershipQuery));
			if(!pQuery || pQuery->groupAddress.u32 != pPkt->ipv4.dst.u32)
			{
				pIgmp->rxFramesDropped++;
				return NET_VERDICT_DROP;
			}

			if(pQuery->maxRespCode == 0)
			{
				// This is an IGMPv1 query
				if(pQuery->groupAddress.u32 != 0 || pPkt->ipv4.dst.u32 != allSystemsAddress.u32)
				{
					// IGMPv1 membership queries must always have a group address of zero
					pIgmp->rxFramesDropped++;
					return NET_VERDICT_DROP;
				}

				trace("Processing IGMPv1 query\r\n");
				handleIGMPv1Query(pIgmp);
			}
			else
			{
				// IGMPv2 query
				uint32_t maxResponseTime_ms = (uint32_t)pQuery->maxRespCode * 100;

				if(pQuery->groupAddress.u32 == 0)
				{
					// General query
					if(pPkt->ipv4.dst.u32 != allSystemsAddress.u32)
					{
						// General query not targeting all-systems address
						pIgmp->rxFramesDropped++;
						return NET_VERDICT_DROP;
					}

					trace("Processing IGMPv2 general query\r\n");
					handleIGMPv2GeneralQuery(pIgmp, maxResponseTime_ms);
				}
				else
				{
					// Group-specific query
					trace("Processing IGMPv2 group-specific query\r\n");
					handleIGMPv2GroupSpecificQuery(pIgmp, pQuery->groupAddress, maxResponseTime_ms);
				}
			}
		}
		else
		{
			IGMPv3MembershipQuery* pQuery = NetBufPull(pData, sizeof(IGMPv3MembershipQuery));
			if(!pQuery)
			{
				pIgmp->rxFramesDropped++;
				return NET_VERDICT_DROP;
			}

			if(pQuery->groupAddress.u32 == 0)
			{
				// General query
				if(pPkt->ipv4.dst.u32 != allSystemsAddress.u32)
				{
					// General query not targeting all-systems address
					pIgmp->rxFramesDropped++;
					return NET_VERDICT_DROP;
				}

				trace("Processing IGMPv3 general query\r\n");

				uint32_t maxResponseTime_ms = v3DecodeMaxRespCode_ms(pQuery->maxRespCode);
				handleIGMPv3GeneralQuery(pIgmp, maxResponseTime_ms);
			}
			else
			{
				// Group-specific query
				trace("Processing IGMPv3 group-specific query\r\n");
				NetVerdict result = handleIGMPv3GroupSpecificQuery(pIgmp, pQuery, pData);
				if(result == NET_VERDICT_DROP)
					return NET_VERDICT_DROP;
			}
		}
	}

	if((type == IGMPV1_MEMBERSHIP_REPORT && pIgmp->isV1QuerierPresent) || (type == IGMPV2_MEMBERSHIP_REPORT && pIgmp->isV2QuerierPresent))
	{
		// Cancel own response if matches and in IGMPv1 mode
		trace("IGMPv1/v2 membership report received\r\n");

		IGMPv2MembershipReport* pReport = NetBufPull(pData, sizeof(IGMPv2MembershipReport));
		if(!pReport)
		{
			pIgmp->rxFramesDropped++;
			return NET_VERDICT_DROP;
		}

		for(size_t i = 0; i < IGMP_MAX_JOINED_GROUPS; i++)
		{
			IGMPMulticastGroup* pGroup = &pIgmp->groups[i];
			if(pGroup->numReferences && pGroup->ip.u32 == pReport->groupAddress.u32)
			{
				chVTReset(&pGroup->timer);
				pGroup->timerExpired = 0;
				pGroup->numResponsesLeft = 0;
				break;
			}
		}
	}

	if(type == IGMPV3_MEMBERSHIP_REPORT)
	{
		// This is in general not interesting, does not cancel any pending response and this is not a router implementation
		trace("IGMPv3 membership report received\r\n");
	}

	pIgmp->rxFramesGood++;
	return NET_VERDICT_OK;
}
