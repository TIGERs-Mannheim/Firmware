#pragma once

#include <stdint.h>
#include <stddef.h>

#define NET_BUF_MAX_SIZE 2048

typedef struct _NetBuf
{
	uint8_t data[NET_BUF_MAX_SIZE];
	uint8_t* pEnd;

	uint8_t* pHead;
	uint8_t* pTail;
} NetBuf;

void     NetBufInit(NetBuf* pBuf);
size_t   NetBufGetSizeFree(NetBuf* pBuf);
size_t   NetBufGetSizeUsed(NetBuf* pBuf);
static inline void* NetBufGetTail(NetBuf* pBuf) { return pBuf->pTail; }
static inline void* NetBufGetHead(NetBuf* pBuf) { return pBuf->pHead; }

void*    NetBufAdd(NetBuf* pBuf, size_t len);
void*    NetBufAddMem(NetBuf* pBuf, const void* pData, size_t len);
void*    NetBufRemove(NetBuf* pBuf, size_t len);
void*    NetBufPush(NetBuf* pBuf, size_t len);
void*    NetBufPull(NetBuf* pBuf, size_t len);
