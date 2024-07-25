#pragma once

#include "net_proto.h"

typedef struct _UDP
{
	NetProto proto;
	NetIf* pIf;

	uint32_t rxFramesDropped;
	uint32_t rxFramesGood;
	uint32_t txFamesDropped;
	uint32_t txFramesGood;
} UDP;

void UDPInit(UDP* pUdp, NetIf* pIf);
int16_t UDPSend(UDP* pUdp, NetPkt* pPkt);
