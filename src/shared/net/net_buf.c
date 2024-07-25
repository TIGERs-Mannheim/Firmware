// Based on: https://github.com/zephyrproject-rtos/zephyr/blob/main/include/zephyr/net/buf.h

#include "net_buf.h"
#include <string.h>

void NetBufInit(NetBuf* pBuf)
{
	pBuf->pEnd = pBuf->data + NET_BUF_MAX_SIZE;
	pBuf->pHead = pBuf->data;
	pBuf->pTail = pBuf->data;
}

size_t NetBufGetSizeFree(NetBuf* pBuf)
{
	return pBuf->pEnd - pBuf->pTail;
}

size_t NetBufGetSizeUsed(NetBuf* pBuf)
{
	return pBuf->pTail - pBuf->pHead;
}

void* NetBufAdd(NetBuf* pBuf, size_t len)
{
	if(pBuf->pTail + len > pBuf->pEnd)
		return 0;

	uint8_t* pTail = pBuf->pTail;
	pBuf->pTail += len;

	return pTail;
}

void* NetBufAddMem(NetBuf* pBuf, const void* pData, size_t len)
{
	void* pDst = NetBufAdd(pBuf, len);
	if(!pDst)
		return 0;

	memcpy(pDst, pData, len);

	return pDst;
}

void* NetBufRemove(NetBuf* pBuf, size_t len)
{
	if(pBuf->pTail - len < pBuf->pHead)
		return 0;

	pBuf->pTail -= len;
	return pBuf->pTail;
}

void* NetBufPush(NetBuf* pBuf, size_t len)
{
	if(pBuf->pTail + len > pBuf->pEnd)
		return 0;

	size_t bytesUsed = NetBufGetSizeUsed(pBuf);
	memmove(pBuf->pHead + len, pBuf->pHead, bytesUsed);
	pBuf->pTail += len;

	return pBuf->pHead;
}

void* NetBufPull(NetBuf* pBuf, size_t len)
{
	if(pBuf->pHead + len > pBuf->pTail)
		return 0;

	uint8_t* pHead = pBuf->pHead;
	pBuf->pHead += len;
	return pHead;
}
