#pragma once

#include "net_pkt.h"

typedef enum NetVerdict
{
	NET_VERDICT_DROP,
	NET_VERDICT_OK,
	NET_VERDICT_CONTINUE,
	NET_VERDICT_PASS_ON,
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
