#include "udp.h"
#include "net_if.h"
#include "errors.h"

static NetVerdict receive(NetPkt* pPkt, void* pUser);

void UDPInit(UDP* pUdp, NetIf* pIf)
{
	pUdp->proto.pReceiveFunc = &receive;
	pUdp->proto.pUser = pUdp;
	pUdp->pIf = pIf;
}

int16_t UDPSend(UDP* pUdp, NetPkt* pPkt)
{
	NetBuf* pData = &pPkt->buf;

	UDPHeader* pHeader = NetBufPush(pData, sizeof(UDPHeader));
	if(!pHeader)
	{
		pUdp->txFamesDropped++;
		NetIfFreePkt(pUdp->pIf, pPkt);
		return ERROR_NOT_ENOUGH_MEMORY;
	}

	pHeader->srcPort = htons(pPkt->udp.srcPort);
	pHeader->dstPort = htons(pPkt->udp.dstPort);
	pHeader->length = htons(NetBufGetSizeUsed(pData));
	pHeader->crc = 0;

	pPkt->ipv4.protocol = IPV4_PROTOCOL_UDP;

	pUdp->txFramesGood++;

	return IPv4Send(&pUdp->pIf->ipv4, pPkt);
}

static NetVerdict receive(NetPkt* pPkt, void* pUser)
{
	UDP* pUdp = (UDP*)pUser;
	NetBuf* pData = &pPkt->buf;

	if(pPkt->ipv4.protocol != IPV4_PROTOCOL_UDP)
		return NET_VERDICT_CONTINUE;

	IPv4PullOptions(pPkt);

	size_t dataSize = NetBufGetSizeUsed(pData);

	UDPHeader* pHeader = NetBufPull(pData, sizeof(UDPHeader));
	if(!pHeader)
	{
		pUdp->rxFramesDropped++;
		return NET_VERDICT_DROP;
	}

	uint16_t length = ntohs(pHeader->length);

	if(dataSize < length)
	{
		pUdp->rxFramesDropped++;
		return NET_VERDICT_DROP;
	}

	pPkt->udp.dstPort = ntohs(pHeader->dstPort);
	pPkt->udp.srcPort = ntohs(pHeader->srcPort);

	pUdp->rxFramesGood++;
	return NET_VERDICT_PASS_ON;
}
