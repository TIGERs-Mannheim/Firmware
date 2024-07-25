#include "ipv4.h"
#include "net_if.h"
#include "errors.h"
#include "util/log.h"

static int16_t ipv4DoSend(IPv4* pIp, NetPkt* pPkt, MAC dstMac);
static NetVerdict receive(NetPkt* pPkt, void* pUser);

void IPv4Init(IPv4* pIp, NetIf* pIf)
{
	pIp->proto.pReceiveFunc = &receive;
	pIp->proto.pUser = pIp;
	pIp->pIf = pIf;
}

int16_t IPv4Send(IPv4* pIp, NetPkt* pPkt)
{
	if((pPkt->ipv4.optionLength % 4) != 0)
	{
		pIp->txFamesDropped++;
		NetIfFreePkt(pIp->pIf, pPkt);
		return ERROR_IPV4_BAD_PACKET;
	}

	if(IPv4IsMulticastAddress(pPkt->ipv4.dst))
	{
		return ipv4DoSend(pIp, pPkt, IPv4CalcMulticastMAC(pPkt->ipv4.dst));
	}

	const MAC* pDstMac = ArpCacheLookup(&pIp->pIf->arp, pPkt->ipv4.dst);
	if(!pDstMac)
	{
		ArpSend(&pIp->pIf->arp, ARP_OPERATION_REQUEST, MACSet(255, 255, 255, 255, 255, 255), pPkt->ipv4.dst);

		ListPushBack(&pIp->pendingPacketList, pPkt);

		while(ListSize(pIp->pendingPacketList) > IPV4_MAX_PENDING_PACKETS)
		{
			NetPkt* pDropped = ListPopFront(&pIp->pendingPacketList);
			pIp->txFamesDropped++;
			NetIfFreePkt(pIp->pIf, pDropped);
		}

		return ERROR_IPV4_MAC_UNKNOWN;
	}

	return ipv4DoSend(pIp, pPkt, *pDstMac);
}

void IPv4SendPendingPackets(IPv4* pIp)
{
	NetPkt* pPkt = ListFront(pIp->pendingPacketList);

	while(pPkt)
	{
		NetPkt* pSend = pPkt;
		pPkt = pPkt->pNext;

		const MAC* pDstMac = ArpCacheLookup(&pIp->pIf->arp, pSend->ipv4.dst);
		if(pDstMac)
		{
			ListErase(&pIp->pendingPacketList, pSend);
			ipv4DoSend(pIp, pSend, *pDstMac);
		}
	}
}

int16_t IPv4PushRouterAlertOption(NetPkt* pPkt)
{
	uint8_t alertData[4];
	alertData[0] = 0b10010100;
	alertData[1] = 0b00000100;
	alertData[2] = 0;
	alertData[3] = 0;

	if(!NetBufAddMem(&pPkt->buf, alertData, sizeof(alertData)))
		return ERROR_NOT_ENOUGH_MEMORY;

	pPkt->ipv4.optionLength += sizeof(alertData);

	return 0;
}

void* IPv4PullOptions(NetPkt* pPkt)
{
	return NetBufPull(&pPkt->buf, pPkt->ipv4.optionLength);
}

static int16_t ipv4DoSend(IPv4* pIp, NetPkt* pPkt, MAC dstMac)
{
	NetBuf* pData = &pPkt->buf;

	IPv4Header* pHeader = NetBufPush(pData, sizeof(IPv4Header));
	if(!pHeader)
	{
		pIp->txFamesDropped++;
		NetIfFreePkt(pIp->pIf, pPkt);
		return ERROR_NOT_ENOUGH_MEMORY;
	}

	pHeader->version = 4;
	pHeader->ihl = 5 + pPkt->ipv4.optionLength/4;
	pHeader->tos = 0;
	pHeader->totalLength = htons(NetBufGetSizeUsed(pData));
	pHeader->identification = htons(pIp->txIdentificationCounter++);
	pHeader->flags = 2;
	pHeader->fragmentOffset = 0;
	pHeader->fragment = htons(pHeader->fragment);
	pHeader->ttl = pPkt->ipv4.ttl == 0 ? 64 : pPkt->ipv4.ttl;
	pHeader->protocol = pPkt->ipv4.protocol;

	if(pPkt->ipv4.src.u32 == 0)
		pHeader->srcIP = pIp->pIf->data.ip;
	else
		pHeader->srcIP = pPkt->ipv4.src;

	pHeader->dstIP = pPkt->ipv4.dst;
	pHeader->headerCRC = 0;

	pPkt->ethernet.type = ETHERNET_PROTOCOL_IPV4;
	pPkt->ethernet.dst = dstMac;

	pIp->txFramesGood++;

	return EthernetSend(&pIp->pIf->ethernet, pPkt);
}

static NetVerdict receive(NetPkt* pPkt, void* pUser)
{
	IPv4* pIp = (IPv4*)pUser;
	NetBuf* pData = &pPkt->buf;

	if(pPkt->ethernet.type != ETHERNET_PROTOCOL_IPV4)
		return NET_VERDICT_CONTINUE;

	size_t dataSize = NetBufGetSizeUsed(pData);

	IPv4Header* pHeader = NetBufPull(pData, sizeof(IPv4Header));
	if(!pHeader || pHeader->version != 4)
	{
		pIp->rxFramesDropped++;
		return NET_VERDICT_DROP;
	}

	pHeader->fragment = ntohs(pHeader->fragment);

	if(pHeader->fragmentOffset != 0 || (pHeader->flags & 4))
	{
		pIp->rxFramesDropped++;
		LogError("Fragmented packet");
		return NET_VERDICT_DROP;
	}

	uint16_t totalLength = ntohs(pHeader->totalLength);

	if(dataSize < totalLength)
	{
		pIp->rxFramesDropped++;
		return NET_VERDICT_DROP;
	}

	pPkt->ipv4.protocol = pHeader->protocol;
	pPkt->ipv4.src = pHeader->srcIP;
	pPkt->ipv4.dst = pHeader->dstIP;
	pPkt->ipv4.ttl = pHeader->ttl;
	pPkt->ipv4.optionLength = pHeader->ihl*4 - sizeof(IPv4Header);

	pIp->rxFramesGood++;
	return NET_VERDICT_PASS_ON;
}
