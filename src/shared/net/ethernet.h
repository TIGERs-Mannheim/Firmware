#pragma once

#include "net_proto.h"

typedef int16_t(*EthernetOutputFunc)(NetPkt*, void*);

typedef struct _Ethernet
{
	NetProto proto;
	NetIf* pIf;

	EthernetOutputFunc pOutputFunc;
	void* pUser;

	uint32_t rxFramesDropped;
	uint32_t rxFramesGood;
	uint32_t txFamesDropped;
	uint32_t txFramesGood;
} Ethernet;

void EthernetInit(Ethernet* pEth, NetIf* pIf, EthernetOutputFunc outputFunc, void* pUser);
int16_t EthernetSend(Ethernet* pEth, NetPkt* pPkt);
