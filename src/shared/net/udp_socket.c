#include "udp_socket.h"
#include "hal/rng.h"

#include <errors.h>

void UDPSocketOpen(UDPSocket* pSocket, NetIf* pInterface, uint16_t myPort)
{
	pSocket->pIf = pInterface;
	pSocket->multicastIp.u32 = 0;

	while(myPort == 0)
	{
		myPort = (RNGGetRandomU32() % 16384) + 49152;

		UDPSocket* pSocketSearch = ListFront(pInterface->udpSockets);
		while(pSocketSearch)
		{
			if(pSocketSearch->port == myPort)
			{
				myPort = 0;
				break;
			}

			pSocketSearch = pSocketSearch->pNext;
		}
	}

	pSocket->port = myPort;

	chEvtObjectInit(&pSocket->eventSource);

	ListPushBack(&pInterface->udpSockets, pSocket);
}

void UDPSocketSetMulticastGroup(UDPSocket* pSocket, IPv4Address multicastIp)
{
	if(IPv4IsMulticastAddress(pSocket->multicastIp))
		IGMPLeaveGroup(&pSocket->pIf->igmp, pSocket->multicastIp);

	if(IPv4IsMulticastAddress(multicastIp))
		IGMPJoinGroup(&pSocket->pIf->igmp, multicastIp);

	pSocket->multicastIp = multicastIp;
}

void UDPSocketClose(UDPSocket* pSocket)
{
	NetPkt* pPkt = ListFront(pSocket->rxPktQueue);

	while(pPkt)
	{
		UDPSocketFree(pSocket, pPkt);
		pPkt = pPkt->pNext;
	}

	UDPSocketSetMulticastGroup(pSocket, IPV4_ADDRESS_ZEROS);

	ListErase(&pSocket->pIf->udpSockets, pSocket);
}

NetPkt* UDPSocketAlloc(UDPSocket* pSocket, IPv4Address dstIp, uint16_t dstPort)
{
	NetPkt* pPkt = NetIfAllocPkt(pSocket->pIf);
	if(!pPkt)
		return 0;

	pPkt->udp.srcPort = pSocket->port;
	pPkt->udp.dstPort = dstPort;
	pPkt->ipv4.dst = dstIp;

	return pPkt;
}

int16_t UDPSocketSend(UDPSocket* pSocket, NetPkt* pPkt)
{
	if(pSocket->pIf->state != NET_IF_STATE_CONNECTED)
		return ERROR_RESSOURCE_UNAVAILABLE;

	pPkt->udp.srcPort = pSocket->port;

	return UDPSend(&pSocket->pIf->udp, pPkt);
}

NetPkt* UDPSocketReceive(UDPSocket* pSocket)
{
	return ListPopFront(&pSocket->rxPktQueue);
}

void UDPSocketFree(UDPSocket* pSocket, NetPkt* pPkt)
{
	NetIfFreePkt(pSocket->pIf, pPkt);
}
