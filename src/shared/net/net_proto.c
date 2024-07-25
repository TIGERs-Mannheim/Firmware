#include "net_proto.h"

void NetProtoAppend(NetProto* pList, NetProto* pNew)
{
	NetProto* pEnd = pList;
	for(; pEnd->pNext; pEnd = pEnd->pNext);

	pEnd->pNext = pNew;
}

void NetProtoLink(NetProto* pUpper, NetProto* pLower)
{
	if(pLower->pUpperProtos)
		NetProtoAppend(pLower->pUpperProtos, pUpper);
	else
		pLower->pUpperProtos = pUpper;
}

NetVerdict NetProtoHandleRxPkt(NetProto* pList, NetPkt* pPkt)
{
	for(NetProto* pProto = pList; pProto; pProto = pProto->pNext)
	{
		NetVerdict verdict = (*pProto->pReceiveFunc)(pPkt, pProto->pUser);
		if(verdict == NET_VERDICT_CONTINUE)
			continue;

		if(verdict == NET_VERDICT_PASS_ON)
			return NetProtoHandleRxPkt(pProto->pUpperProtos, pPkt);

		return verdict;
	}

	return NET_VERDICT_DROP;
}
