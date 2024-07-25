#include "icmp.h"
#include "net_if.h"
#include "errors.h"

static NetVerdict receive(NetPkt* pPkt, void* pUser);

void ICMPInit(ICMP* pIcmp, NetIf* pIf)
{
	pIcmp->proto.pReceiveFunc = &receive;
	pIcmp->proto.pUser = pIcmp;
	pIcmp->pIf = pIf;
}

int16_t ICMPSendEchoRequest(ICMP* pIcmp, IPv4Address dstIp)
{
	NetIf* pIf = pIcmp->pIf;

	NetPkt* pPkt = NetIfAllocPkt(pIf);
	if(!pPkt)
	{
		pIcmp->txFamesDropped++;
		return ERROR_NOT_ENOUGH_MEMORY;
	}

	ICMPHeader* pHeader = NetBufAdd(&pPkt->buf, sizeof(ICMPHeader));
	ICMPEcho* pEcho = NetBufAdd(&pPkt->buf, sizeof(ICMPEcho));
	if(!pHeader || !pEcho)
	{
		pIcmp->txFamesDropped++;
		NetIfFreePkt(pIf, pPkt);
		return ERROR_NOT_ENOUGH_MEMORY;
	}

	pHeader->type = ICMP_TYPE_ECHO_REQUEST;
	pHeader->code = 0;
	pHeader->checksum = 0;

	pIcmp->pendingEchoRequestSeq = pIcmp->txEchoRequestSequenceNumber;
	pIcmp->tEchoRequestSent = chVTGetSystemTimeX();

	pEcho->identifier = htons(1);
	pEcho->sequenceNumber = htons(pIcmp->txEchoRequestSequenceNumber++);

	pPkt->ipv4.protocol = IPV4_PROTOCOL_ICMP;
	pPkt->ipv4.dst = dstIp;

	pIcmp->txFramesGood++;

	return IPv4Send(&pIf->ipv4, pPkt);
}

int16_t ICMPSendEchoReply(ICMP* pIcmp, IPv4Address dstIp, uint16_t identifier, uint16_t sequenceNumber, const void* pData, size_t dataSize)
{
	NetIf* pIf = pIcmp->pIf;

	NetPkt* pPkt = NetIfAllocPkt(pIf);
	if(!pPkt)
	{
		pIcmp->txFamesDropped++;
		return ERROR_NOT_ENOUGH_MEMORY;
	}

	ICMPHeader* pHeader = NetBufAdd(&pPkt->buf, sizeof(ICMPHeader));
	ICMPEcho* pEcho = NetBufAdd(&pPkt->buf, sizeof(ICMPEcho));
	void* pPktData = NetBufAdd(&pPkt->buf, dataSize);
	if(!pHeader || !pEcho || !pPktData)
	{
		pIcmp->txFamesDropped++;
		NetIfFreePkt(pIf, pPkt);
		return ERROR_NOT_ENOUGH_MEMORY;
	}

	pHeader->type = ICMP_TYPE_ECHO_REPLY;
	pHeader->code = 0;
	pHeader->checksum = 0;

	pEcho->identifier = htons(identifier);
	pEcho->sequenceNumber = htons(sequenceNumber);

	memcpy(pPktData, pData, dataSize);

	pPkt->ipv4.protocol = IPV4_PROTOCOL_ICMP;
	pPkt->ipv4.dst = dstIp;

	pIcmp->txFramesGood++;

	return IPv4Send(&pIf->ipv4, pPkt);
}

static NetVerdict receive(NetPkt* pPkt, void* pUser)
{
	ICMP* pIcmp = (ICMP*)pUser;
	NetBuf* pData = &pPkt->buf;

	if(pPkt->ipv4.protocol != IPV4_PROTOCOL_ICMP)
		return NET_VERDICT_CONTINUE;

	IPv4PullOptions(pPkt);

	ICMPHeader* pHeader = NetBufPull(pData, sizeof(ICMPHeader));
	if(!pHeader)
	{
		pIcmp->rxFramesDropped++;
		return NET_VERDICT_DROP;
	}

	if(pHeader->type == ICMP_TYPE_ECHO_REQUEST)
	{
		ICMPEcho* pEcho = NetBufPull(pData, sizeof(ICMPEcho));

		uint16_t identifier = ntohs(pEcho->identifier);
		uint16_t sequenceNumber = ntohs(pEcho->sequenceNumber);

		ICMPSendEchoReply(pIcmp, pPkt->ipv4.src, identifier, sequenceNumber, NetBufGetHead(pData), NetBufGetSizeUsed(pData));
	}
	else if(pHeader->type == ICMP_TYPE_ECHO_REPLY)
	{
		ICMPEcho* pEcho = NetBufPull(pData, sizeof(ICMPEcho));

		uint16_t identifier = ntohs(pEcho->identifier);
		uint16_t sequenceNumber = ntohs(pEcho->sequenceNumber);

		if(identifier == 1 && sequenceNumber == pIcmp->pendingEchoRequestSeq)
		{
			uint32_t echoLatency_ms = TIME_I2MS(chVTTimeElapsedSinceX(pIcmp->tEchoRequestSent));
			printf("Ping answered by %hu.%hu.%hu.%hu after %ums\r\n",
					(uint16_t)pPkt->ipv4.src.u8[0], (uint16_t)pPkt->ipv4.src.u8[1], (uint16_t)pPkt->ipv4.src.u8[2], (uint16_t)pPkt->ipv4.src.u8[3], echoLatency_ms);
		}
	}

	pIcmp->rxFramesGood++;

	return NET_VERDICT_OK;
}
