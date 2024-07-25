#include "arp.h"
#include "net_if.h"
#include "errors.h"
#include "ch.h"

static NetVerdict receive(NetPkt* pPkt, void* pUser);

void ArpInit(Arp* pArp, NetIf* pIf)
{
	pArp->proto.pReceiveFunc = &receive;
	pArp->proto.pUser = pArp;
	pArp->pIf = pIf;
}

int16_t ArpSend(Arp* pArp, uint8_t operation, MAC targetMAC, IPv4Address targetIP)
{
	NetIf* pIf = pArp->pIf;

	NetPkt* pPkt = NetIfAllocPkt(pIf);
	if(!pPkt)
	{
		pArp->txFamesDropped++;
		return ERROR_NOT_ENOUGH_MEMORY;
	}

	ARPPacketIPv4* pARP = NetBufAdd(&pPkt->buf, sizeof(ARPPacketIPv4));
	if(!pARP)
	{
		pArp->txFamesDropped++;
		NetIfFreePkt(pIf, pPkt);
		return ERROR_NOT_ENOUGH_MEMORY;
	}

	pARP->hardwareType = 1;
	pARP->protocolType = 0x0800;
	pARP->hwAddrLen = 6;
	pARP->protAddrLen = 4;
	pARP->operation = operation;
	pARP->srcHwAddr = pIf->data.mac;
	pARP->srcProtAddr = pIf->data.ip;

	if(operation == ARP_OPERATION_REQUEST)
		pARP->dstHwAddr = MACSet(0, 0, 0, 0, 0, 0);
	else
		pARP->dstHwAddr = targetMAC;

	pARP->dstProtAddr = targetIP;

	pARP->hardwareType = htons(pARP->hardwareType);
	pARP->protocolType = htons(pARP->protocolType);
	pARP->operation = htons(pARP->operation);

	pPkt->ethernet.type = ETHERNET_PROTOCOL_ARP;
	pPkt->ethernet.dst = targetMAC;

	pArp->txFramesGood++;

	return EthernetSend(&pIf->ethernet, pPkt);
}

const MAC* ArpCacheLookup(Arp* pArp, IPv4Address ip)
{
	for(size_t i = 0; i < ARP_CACHE_SIZE; i++)
	{
		if(pArp->cache[i].ip.u32 == ip.u32)
		{
			pArp->cache[i].tLastUse = chVTGetSystemTimeX();
			return &pArp->cache[i].mac;
		}
	}

	return 0;
}

static void cacheUpdate(Arp* pArp, IPv4Address ip, MAC mac)
{
	ArpCacheEntry* pTargetEntry = pArp->cache;

	for(size_t i = 0; i < ARP_CACHE_SIZE; i++)
	{
		if(pArp->cache[i].ip.u32 == ip.u32)
		{
			pTargetEntry = &pArp->cache[i];
			break;
		}

		sysinterval_t ageOld = chVTTimeElapsedSinceX(pTargetEntry->tLastUse);
		sysinterval_t ageCheck = chVTTimeElapsedSinceX(pArp->cache[i].tLastUse);

		if(ageCheck > ageOld)
		{
			pTargetEntry = &pArp->cache[i];
		}
	}

	pTargetEntry->ip = ip;
	pTargetEntry->mac = mac;
	pTargetEntry->tLastUse = chVTGetSystemTimeX();
}

static NetVerdict receive(NetPkt* pPkt, void* pUser)
{
	Arp* pArp = (Arp*)pUser;
	NetBuf* pData = &pPkt->buf;

	if(pPkt->ethernet.type != ETHERNET_PROTOCOL_ARP)
		return NET_VERDICT_CONTINUE;

	ARPPacketIPv4* pPacket = NetBufPull(pData, sizeof(ARPPacketIPv4));
	if(!pPacket)
	{
		pArp->rxFramesDropped++;
		return NET_VERDICT_DROP;
	}

	uint16_t hardwareType = ntohs(pPacket->hardwareType);
	uint16_t protocolType = ntohs(pPacket->protocolType);
	uint16_t operation = ntohs(pPacket->operation);

	if(hardwareType != 1)	// link layer protocol != ethernet?
	{
		pArp->rxFramesDropped++;
		return NET_VERDICT_DROP;
	}

	if(protocolType != 0x0800)	// protocol type != ipv4?
	{
		pArp->rxFramesDropped++;
		return NET_VERDICT_DROP;
	}

	if(pPacket->hwAddrLen != 6 || pPacket->protAddrLen != 4)
	{
		pArp->rxFramesDropped++;
		return NET_VERDICT_DROP;
	}

	if(operation == ARP_OPERATION_REQUEST && pPacket->dstProtAddr.u32 == pArp->pIf->data.ip.u32)
	{
		ArpSend(pArp, ARP_OPERATION_REPLY, pPacket->srcHwAddr, pPacket->srcProtAddr);
	}

	uint8_t isGratuitousArp = operation == ARP_OPERATION_REQUEST && pPacket->srcProtAddr.u32 == pPacket->dstProtAddr.u32;

	if(operation == ARP_OPERATION_REPLY || isGratuitousArp)
	{
		cacheUpdate(pArp, pPacket->srcProtAddr, pPacket->srcHwAddr);
		IPv4SendPendingPackets(&pArp->pIf->ipv4);
	}

	pArp->rxFramesGood++;

	return NET_VERDICT_OK;
}
