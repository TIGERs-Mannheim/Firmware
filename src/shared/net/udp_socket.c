#include "udp_socket.h"
#include "hal/rng.h"

void UDPSocketOpen(UDPSocket* pSocket, NetIf* pInterface, uint16_t myPort, IPv4Address bindIp)
{
	pSocket->pIf = pInterface;
	pSocket->bindIp = bindIp;

	while(myPort == 0)
	{
		myPort = (RNGGetRandomU32() % 16384) + 49152;

		UDPSocket* pSocket = ListFront(pInterface->udpSockets);
		while(pSocket)
		{
			if(pSocket->port == myPort)
			{
				myPort = 0;
				break;
			}

			pSocket = pSocket->pNext;
		}
	}

	pSocket->port = myPort;

	chEvtObjectInit(&pSocket->eventSource);

	ListPushBack(&pInterface->udpSockets, pSocket);

	if(IPv4IsMulticastAddress(bindIp))
		IGMPJoinGroup(&pInterface->igmp, bindIp);
}

void UDPSocketSetBindIp(UDPSocket* pSocket, IPv4Address ip)
{
	if(ip.u32 == pSocket->bindIp.u32)
		return;

	if(IPv4IsMulticastAddress(pSocket->bindIp))
		IGMPLeaveGroup(&pSocket->pIf->igmp, pSocket->bindIp);

	if(IPv4IsMulticastAddress(ip))
		IGMPJoinGroup(&pSocket->pIf->igmp, ip);

	pSocket->bindIp = ip;
}

void UDPSocketClose(UDPSocket* pSocket)
{
	NetPkt* pPkt = ListFront(pSocket->rxPktQueue);

	while(pPkt)
	{
		UDPSocketFree(pSocket, pPkt);
		pPkt = pPkt->pNext;
	}

	if(IPv4IsMulticastAddress(pSocket->bindIp))
		IGMPLeaveGroup(&pSocket->pIf->igmp, pSocket->bindIp);

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
