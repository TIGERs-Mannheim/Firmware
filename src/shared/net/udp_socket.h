#pragma once

#include "net_if.h"

typedef struct _UDPSocket
{
	void* pNext;

	uint16_t port;
	IPv4Address multicastIp;

	NetIf* pIf;
	List rxPktQueue;

	event_source_t eventSource;

	uint32_t rxPktLost;
} UDPSocket;

void UDPSocketOpen(UDPSocket* pSocket, NetIf* pInterface, uint16_t myPort);
void UDPSocketClose(UDPSocket* pSocket);
void UDPSocketSetMulticastGroup(UDPSocket* pSocket, IPv4Address multicastIp);
NetPkt* UDPSocketAlloc(UDPSocket* pSocket, IPv4Address dstIp, uint16_t dstPort);
int16_t UDPSocketSend(UDPSocket* pSocket, NetPkt* pPkt);
NetPkt* UDPSocketReceive(UDPSocket* pSocket);
void UDPSocketFree(UDPSocket* pSocket, NetPkt* pPkt);
