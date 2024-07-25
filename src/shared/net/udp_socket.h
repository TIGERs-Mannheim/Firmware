#pragma once

#include "net_if.h"

typedef struct _UDPSocket
{
	void* pNext;

	uint16_t port;
	IPv4Address bindIp;

	NetIf* pIf;
	List rxPktQueue;

	event_source_t eventSource;

	uint32_t rxPktLost;
} UDPSocket;

void UDPSocketOpen(UDPSocket* pSocket, NetIf* pInterface, uint16_t myPort, IPv4Address bindIp);
void UDPSocketClose(UDPSocket* pSocket);
void UDPSocketSetBindIp(UDPSocket* pSocket, IPv4Address ip);
NetPkt* UDPSocketAlloc(UDPSocket* pSocket, IPv4Address dstIp, uint16_t dstPort);
int16_t UDPSocketSend(UDPSocket* pSocket, NetPkt* pPkt);
NetPkt* UDPSocketReceive(UDPSocket* pSocket);
void UDPSocketFree(UDPSocket* pSocket, NetPkt* pPkt);
