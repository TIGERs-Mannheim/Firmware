#pragma once

#include "net_pkt.h"

typedef enum NetVerdict
{
	NET_VERDICT_DROP,		// packet is bad/invalid, no further processing
	NET_VERDICT_OK,			// packet is fully consumed, processing stops
	NET_VERDICT_CONTINUE,	// protocol does not handle this packet, network stack should try other protocols on the same layer
	NET_VERDICT_PASS_ON,	// protocol partially handles this packet, it contains more data which should be passed on to higher layers
} NetVerdict;

typedef NetVerdict(*NetReceiveFunc)(NetPkt*, void*);

typedef struct _NetProto
{
	NetReceiveFunc pReceiveFunc;
	void* pUser;

	struct _NetProto* pUpperProtos;
	struct _NetProto* pNext;
} NetProto;

void NetProtoAppend(NetProto* pList, NetProto* pNew);
void NetProtoLink(NetProto* pUpper, NetProto* pLower);
NetVerdict NetProtoHandleRxPkt(NetProto* pList, NetPkt* pPkt);
