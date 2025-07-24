#include "mdns.h"
#include "igmp.h"
#include "net_if.h"

#define DNS_RECORD_TYPE_MDNS_FLUSH_CACHE_BIT	(1 << 15)

static NetVerdict receive(NetPkt* pRxPkt, void* pUser);

void MDNSInit(MDNS* pMdns, NetIf* pIf)
{
	pMdns->proto.pReceiveFunc = &receive;
	pMdns->proto.pUser = pMdns;
	pMdns->pIf = pIf;

	pMdns->multicastIp = IPv4AddressSet(224, 0, 0, 251);
	snprintf(pMdns->hostnameDotLocal, sizeof(pMdns->hostnameDotLocal), "%s.local.", pIf->pHostname);

	IGMPJoinGroup(&pIf->igmp, pMdns->multicastIp);
}

static DNSResourceRecord* getRecordTypeA(MDNS* pMdns, DNSMessage* pMsg)
{
	return DNSMessageTxCreateResourceRecoryTypeA(pMsg, pMdns->hostnameDotLocal, 120, pMdns->pIf->ip);
}

static DNSResourceRecord* getRecordTypeNSEC(MDNS* pMdns, DNSMessage* pMsg)
{
	uint8_t nsecData[5];
	*((uint16_t*)nsecData) = htons(sizeof(DNSHeader) | 0xC000); // Pointer to hostname
	nsecData[2] = 0; // Type Bit Map block number
	nsecData[3] = 1; // Type Bit Map block length in bytes
	nsecData[4] = __RBIT(1 << DNS_RECORD_TYPE_A) >> 24; // set bits for existing resource records

	return DNSMessageTxCreateResourceRecord(pMsg, pMdns->hostnameDotLocal, DNS_RECORD_TYPE_NSEC, DNS_RECORD_CLASS_IN, 120, nsecData, sizeof(nsecData));
}

static NetVerdict receive(NetPkt* pRxPkt, void* pUser)
{
	int16_t result = 0;
	MDNS* pMdns = pUser;

	if(pRxPkt->udp.dstPort != UDP_PORT_MDNS)
		return NET_VERDICT_CONTINUE;

	if(pRxPkt->ipv4.dst.u32 != pMdns->multicastIp.u32 && pRxPkt->ipv4.dst.u32 != pMdns->pIf->ip.u32)
		return NET_VERDICT_CONTINUE;

	result = DNSMessageParseRxPkt(pRxPkt, &pMdns->rxMessage);
	if(result)
	{
		pMdns->rxFramesDropped++;
		return NET_VERDICT_DROP;
	}

	DNSMessage* pRxMsg = &pMdns->rxMessage;
	const DNSHeader* pRxHeader = pRxMsg->pHeader;

	if(pRxHeader->isResponse)
	{
		// This implemention will only answer requests, not interested in responses
		pMdns->rxFramesGood++;
		return NET_VERDICT_OK;
	}

	uint8_t sendRecordTypeA = 0;
	uint8_t sendRecordTypeAAAA = 0;
	uint8_t isUnicastQuestion = pRxPkt->ipv4.dst.u32 == pMdns->pIf->ip.u32;

	for(DNSQuestion* pQuestion = ListFront(pRxMsg->questions); pQuestion; pQuestion = pQuestion->pNext)
	{
		uint8_t isQuestionForThisBase = strcasecmp(pQuestion->pName, pMdns->hostnameDotLocal) == 0;
		uint16_t qClass = pQuestion->qClass & 0x7FFF;

		if(isQuestionForThisBase && (qClass == DNS_RECORD_CLASS_IN || qClass == DNS_RECORD_CLASS_ANY))
		{
			isUnicastQuestion = !!(pQuestion->qClass & 0x8000);

			if(pQuestion->qType == DNS_RECORD_TYPE_A)
			{
				sendRecordTypeA = 1;
			}
			else if(pQuestion->qType == DNS_RECORD_TYPE_AAAA)
			{
				sendRecordTypeAAAA = 1;
			}
			else if(pQuestion->qType == DNS_RECORD_TYPE_ANY)
			{
				sendRecordTypeA = 1;
				sendRecordTypeAAAA = 1;
			}
		}
	}

	pMdns->rxFramesGood++;

	if(!sendRecordTypeA && !sendRecordTypeAAAA)
		return NET_VERDICT_OK;

	DNSMessage* pResponse = &pMdns->txMessage;

	DNSMessageTxInit(pResponse);
	pResponse->pHeader->id = 0;
	pResponse->pHeader->flags = 0;
	pResponse->pHeader->isResponse = 1;
	pResponse->pHeader->authoritiveAnswer = 1;

	DNSResourceRecord* pRecordTypeA = getRecordTypeA(pMdns, pResponse);
	DNSResourceRecord* pRecordTypeNSEC = getRecordTypeNSEC(pMdns, pResponse);

	if(!pRecordTypeA || !pRecordTypeNSEC)
	{
		pMdns->txFamesDropped++;
		return NET_VERDICT_DROP;
	}

	pRecordTypeA->rrClass |= DNS_RECORD_TYPE_MDNS_FLUSH_CACHE_BIT;
	pRecordTypeNSEC->rrClass |= DNS_RECORD_TYPE_MDNS_FLUSH_CACHE_BIT;

	if(sendRecordTypeA)
		ListPushBack(&pResponse->answerRecords, pRecordTypeA);

	if(sendRecordTypeAAAA)
		ListPushBack(&pResponse->answerRecords, pRecordTypeNSEC);
	else
		ListPushBack(&pResponse->additionalRecords, pRecordTypeNSEC);

	NetPkt* pTxPkt = NetIfAllocPkt(pMdns->pIf);
	if(!pTxPkt)
	{
		pMdns->txFamesDropped++;
		return NET_VERDICT_DROP;
	}

	pTxPkt->udp.srcPort = UDP_PORT_MDNS;
	pTxPkt->ipv4.src = pMdns->pIf->ip;
	pTxPkt->ipv4.ttl = 255;

	if(isUnicastQuestion)
	{
		pTxPkt->udp.dstPort = UDP_PORT_MDNS;
		pTxPkt->ipv4.dst = pRxPkt->ipv4.src;
	}
	else if(pRxPkt->udp.srcPort == UDP_PORT_MDNS)
	{
		pTxPkt->udp.dstPort = UDP_PORT_MDNS;
		pTxPkt->ipv4.dst = pMdns->multicastIp;
	}
	else
	{
		// Simple legacy resolver, response according to RFC 6762 section 6.7
		pTxPkt->udp.dstPort = pRxPkt->udp.srcPort;
		pTxPkt->ipv4.dst = pRxPkt->ipv4.src;

		pResponse->pHeader->id = pRxMsg->pHeader->id;

		pRecordTypeA->rrClass &= ~DNS_RECORD_TYPE_MDNS_FLUSH_CACHE_BIT;
		pRecordTypeNSEC->rrClass &= ~DNS_RECORD_TYPE_MDNS_FLUSH_CACHE_BIT;
		pRecordTypeA->ttl_s = 5;
		pRecordTypeNSEC->ttl_s = 5;

		// Questions must be repeated for conventional DNS responses
		for(DNSQuestion* pQuestion = ListFront(pRxMsg->questions); pQuestion; pQuestion = pQuestion->pNext)
		{
			DNSQuestion* pReplyQuestion = DNSMessageTxAddQuestion(pResponse, pQuestion->pName, pQuestion->qType, pQuestion->qClass);
			if(!pReplyQuestion)
			{
				NetIfFreePkt(pMdns->pIf, pTxPkt);
				pMdns->txFamesDropped++;
				return NET_VERDICT_DROP;
			}

			ListPushBack(&pResponse->questions, pReplyQuestion);
		}
	}

	result = DNSMessageTxBuildPacket(pResponse, pTxPkt);
	if(result)
	{
		NetIfFreePkt(pMdns->pIf, pTxPkt);
		pMdns->txFamesDropped++;
		return NET_VERDICT_DROP;
	}

	pMdns->txFramesGood++;
	return UDPSend(&pMdns->pIf->udp, pTxPkt);
}
