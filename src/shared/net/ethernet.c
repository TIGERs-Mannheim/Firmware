#include "ethernet.h"
#include "net_if.h"
#include "errors.h"

static NetVerdict receive(NetPkt* pPkt, void* pUser);

void EthernetInit(Ethernet* pEth, NetIf* pIf, EthernetOutputFunc outputFunc, void* pUser)
{
	pEth->proto.pReceiveFunc = &receive;
	pEth->proto.pUser = pEth;
	pEth->pIf = pIf;
	pEth->pOutputFunc = outputFunc;
	pEth->pUser = pUser;
}

int16_t EthernetSend(Ethernet* pEth, NetPkt* pPkt)
{
	NetBuf* pData = &pPkt->buf;

	EthernetHeader* pHeader = NetBufPush(pData, sizeof(EthernetHeader));
	if(!pHeader)
	{
		pEth->txFamesDropped++;
		NetIfFreePkt(pEth->pIf, pPkt);
		return ERROR_NOT_ENOUGH_MEMORY;
	}

	pHeader->type = htons(pPkt->ethernet.type);
	pHeader->dstMAC = pPkt->ethernet.dst;

	if(pPkt->ethernet.src.u16[0] == 0 && pPkt->ethernet.src.u16[1] == 0 && pPkt->ethernet.src.u16[2] == 0)
		pHeader->srcMAC = pEth->pIf->data.mac;
	else
		pHeader->srcMAC = pPkt->ethernet.src;

	pEth->txFramesGood++;

	return (*pEth->pOutputFunc)(pPkt, pEth->pUser);
}

static NetVerdict receive(NetPkt* pPkt, void* pUser)
{
	Ethernet* pEth = (Ethernet*)pUser;
	NetBuf* pData = &pPkt->buf;

	EthernetHeader* pHeader = NetBufPull(pData, sizeof(EthernetHeader));
	if(!pHeader)
	{
		pEth->rxFramesDropped++;
		return NET_VERDICT_DROP;
	}

	pPkt->ethernet.type = ntohs(pHeader->type);
	pPkt->ethernet.src = pHeader->srcMAC;
	pPkt->ethernet.dst = pHeader->dstMAC;

	pEth->rxFramesGood++;
	return NET_VERDICT_PASS_ON;
}
