#include "dns.h"
#include "errors.h"
#include <string.h>

typedef struct PACKED
{
	uint16_t rrType;
	uint16_t rrClass;
	uint32_t ttl; // [s]
	uint16_t dataLength;
} DNSResourceRecordDetails;

typedef struct PACKED
{
	uint16_t qType;
	uint16_t qClass;
} DNSQuestionDetails;

static void* memoryPoolAlloc(DNSMessage* pMsg, size_t size, size_t alignment)
{
	size = ((size + alignment - 1) / alignment) * alignment; // round up to alignment

	if(pMsg->memoryUsed + size > sizeof(pMsg->memoryPool))
		return 0;

	void* pResult = pMsg->memoryPool + pMsg->memoryUsed;
	pMsg->memoryUsed += size;

	return pResult;
}

static int16_t parseLabel(DNSMessage* pMsg, void* pEnd, void** ppRead, char** ppLabel)
{
	void* pStart = pMsg->pHeader;
	void* pRead = *ppRead;

	void* pLabelEnd = 0;

	*ppLabel = memoryPoolAlloc(pMsg, 0, 1);

	while(1)
	{
		if(pRead >= pEnd)
			return ERROR_DNS_BAD_PACKET;

		uint8_t length = *((uint8_t*)pRead);
		pRead++;

		if((length & 0xC0) == 0xC0)
		{
			// This is a pointer
			if(pRead >= pEnd)
				return ERROR_DNS_BAD_PACKET;

			uint32_t offset = (uint32_t)(length & 0x3F) << 8;
			offset |= *((uint8_t*)pRead);

			if(!pLabelEnd)
				pLabelEnd = pRead + 1;

			pRead = pStart + offset;
		}
		else if((length & 0xC0) == 0)
		{
			// Regular label
			if(pRead + length > pEnd)
				return ERROR_DNS_BAD_PACKET;

			if(length == 0)
			{
				// Terminating root label
				if(!pLabelEnd)
					pLabelEnd = pRead;

				char* pDst = memoryPoolAlloc(pMsg, 1, 1);
				if(!pDst)
					return ERROR_NOT_ENOUGH_MEMORY;

				*pDst = 0;

				break;
			}

			char* pDst = memoryPoolAlloc(pMsg, length + 1, 1);
			if(!pDst)
				return ERROR_NOT_ENOUGH_MEMORY;

			memcpy(pDst, pRead, length);
			pDst += length;
			*pDst = '.';

			pRead += length;
		}
		else
		{
			// Invalid type
			return ERROR_DNS_BAD_PACKET;
		}
	}

	*ppRead = pLabelEnd;

	return 0;
}

static int16_t parseQuestionEntry(DNSMessage* pMsg, void* pEnd, void** ppRead, DNSQuestion** ppResult)
{
	*ppResult = 0;

	DNSQuestion* pQuestion = memoryPoolAlloc(pMsg, sizeof(DNSQuestion), 4);
	if(!pQuestion)
		return ERROR_NOT_ENOUGH_MEMORY;

	pQuestion->pNext = 0;

	int16_t result = parseLabel(pMsg, pEnd, ppRead, &pQuestion->pName);
	if(result)
		return result;

	if(*ppRead + sizeof(DNSQuestionDetails) > pEnd)
		return ERROR_DNS_BAD_PACKET;

	const DNSQuestionDetails* pDetails = *ppRead;
	pQuestion->qType = ntohs(pDetails->qType);
	pQuestion->qClass = ntohs(pDetails->qClass);
	*ppRead += sizeof(DNSQuestionDetails);

	*ppResult = pQuestion;

	return 0;
}

static int16_t parseResourceRecord(DNSMessage* pMsg, void* pEnd, void** ppRead, DNSResourceRecord** ppResult)
{
	*ppResult = 0;

	DNSResourceRecord* pRes = memoryPoolAlloc(pMsg, sizeof(DNSResourceRecord), 4);
	if(!pRes)
		return ERROR_NOT_ENOUGH_MEMORY;

	pRes->pNext = 0;

	int16_t result = parseLabel(pMsg, pEnd, ppRead, &pRes->pName);
	if(result)
		return result;

	if(*ppRead + sizeof(DNSResourceRecordDetails) > pEnd)
		return ERROR_DNS_BAD_PACKET;

	const DNSResourceRecordDetails* pDetails = *ppRead;
	pRes->rrType = ntohs(pDetails->rrType);
	pRes->rrClass = ntohs(pDetails->rrClass);
	pRes->ttl_s = ntohl(pDetails->ttl);
	pRes->dataLength = ntohs(pDetails->dataLength);
	*ppRead += sizeof(DNSResourceRecordDetails);

	pRes->pData = *ppRead;

	*ppResult = pRes;

	return 0;
}

int16_t DNSMessageParseRxPkt(NetPkt* pPkt, DNSMessage* pMsg)
{
	int16_t result = 0;

	// Clear message, might have been used before
	pMsg->pHeader = 0;
	pMsg->questions = 0;
	pMsg->answerRecords = 0;
	pMsg->authoritiveRecords = 0;
	pMsg->additionalRecords = 0;
	pMsg->memoryUsed = 0;

	// Process DNS message header
	NetBuf* pData = &pPkt->buf;

	DNSHeader* pHeader = NetBufPull(pData, sizeof(DNSHeader));
	if(!pHeader)
		return ERROR_DNS_BAD_PACKET;

	pHeader->id = ntohs(pHeader->id);
	pHeader->flags = ntohs(pHeader->flags);
	pHeader->numQuestions = ntohs(pHeader->numQuestions);
	pHeader->numAnswers = ntohs(pHeader->numAnswers);
	pHeader->numAuthoritiveRecords = ntohs(pHeader->numAuthoritiveRecords);
	pHeader->numAdditionalRecords = ntohs(pHeader->numAdditionalRecords);

	pMsg->pHeader = pHeader;

	// Read all resource entries
	void* pEnd = NetBufGetTail(pData);
	void* pRead = NetBufGetHead(pData);

	for(uint16_t i = 0; i < pHeader->numQuestions; i++)
	{
		DNSQuestion* pQuestion;
		result = parseQuestionEntry(pMsg, pEnd, &pRead, &pQuestion);
		if(result)
			return result;

		ListPushBack(&pMsg->questions, pQuestion);
	}

	for(uint16_t i = 0; i < pHeader->numAnswers; i++)
	{
		DNSResourceRecord* pRes;
		result = parseResourceRecord(pMsg, pEnd, &pRead, &pRes);
		if(result)
			return result;

		ListPushBack(&pMsg->answerRecords, pRes);
		pRead += pRes->dataLength;
	}

	for(uint16_t i = 0; i < pHeader->numAuthoritiveRecords; i++)
	{
		DNSResourceRecord* pRes;
		result = parseResourceRecord(pMsg, pEnd, &pRead, &pRes);
		if(result)
			return result;

		ListPushBack(&pMsg->authoritiveRecords, pRes);
		pRead += pRes->dataLength;
	}

	for(uint16_t i = 0; i < pHeader->numAdditionalRecords; i++)
	{
		DNSResourceRecord* pRes;
		result = parseResourceRecord(pMsg, pEnd, &pRead, &pRes);
		if(result)
			return result;

		ListPushBack(&pMsg->additionalRecords, pRes);
		pRead += pRes->dataLength;
	}

	return 0;
}

void DNSMessageTxInit(DNSMessage* pMsg)
{
	pMsg->questions = 0;
	pMsg->answerRecords = 0;
	pMsg->authoritiveRecords = 0;
	pMsg->additionalRecords = 0;
	pMsg->memoryUsed = 0;

	pMsg->pHeader = memoryPoolAlloc(pMsg, sizeof(DNSHeader), 4);
}

DNSQuestion* DNSMessageTxAddQuestion(DNSMessage* pMsg, const char* pName, uint16_t qType, uint16_t qClass)
{
	DNSQuestion* pQuestion = memoryPoolAlloc(pMsg, sizeof(DNSQuestion), 4);
	if(!pQuestion)
		return 0;

	size_t nameLength = strlen(pName);
	char* pNameCopy = memoryPoolAlloc(pMsg, nameLength + 1, 1);
	if(!pNameCopy)
		return 0;

	memcpy(pNameCopy, pName, nameLength+1);

	pQuestion->pNext = 0;
	pQuestion->pName = pNameCopy;
	pQuestion->qType = qType;
	pQuestion->qClass = qClass;

	ListPushBack(&pMsg->questions, pQuestion);

	return pQuestion;
}

DNSResourceRecord* DNSMessageTxCreateResourceRecord(DNSMessage* pMsg, const char* pName, uint16_t rrType, uint16_t rrClass, uint32_t ttl_s, const void* pData, uint16_t dataLength)
{
	DNSResourceRecord* pRes = memoryPoolAlloc(pMsg, sizeof(DNSResourceRecord), 4);
	if(!pRes)
		return 0;

	size_t nameLength = strlen(pName);
	char* pNameCopy = memoryPoolAlloc(pMsg, nameLength + 1, 1);
	if(!pNameCopy)
		return 0;

	void* pDataCopy = memoryPoolAlloc(pMsg, dataLength, 1);
	if(!pDataCopy)
		return 0;

	memcpy(pNameCopy, pName, nameLength+1);
	memcpy(pDataCopy, pData, dataLength);

	pRes->pNext = 0;
	pRes->pName = pNameCopy;
	pRes->rrType = rrType;
	pRes->rrClass = rrClass;
	pRes->ttl_s = ttl_s;
	pRes->dataLength = dataLength;
	pRes->pData = pDataCopy;

	return pRes;
}

DNSResourceRecord* DNSMessageTxCreateResourceRecoryTypeA(DNSMessage* pMsg, const char* pName, uint32_t ttl_s, IPv4Address ip)
{
	return DNSMessageTxCreateResourceRecord(pMsg, pName, DNS_RECORD_TYPE_A, DNS_RECORD_CLASS_IN, ttl_s, &ip, sizeof(ip));
}

static int16_t encodeName(NetPkt* pPkt, const char* pName)
{
	uint32_t nameLength = strlen(pName);

	char* pBuf = NetBufAdd(&pPkt->buf, nameLength + 1);
	if(!pBuf)
		return ERROR_NOT_ENOUGH_MEMORY;

	char* pLen = pBuf;
	pBuf++;

	*pLen = 0;

	for(uint32_t i = 0; i < nameLength; i++)
	{
		if(pName[i] == '.')
		{
			pLen = pBuf;
			pBuf++;
			*pLen = 0;
		}
		else
		{
			*pBuf = pName[i];
			pBuf++;
			(*pLen)++;
		}
	}

	return 0;
}

static int16_t packetAddQuestion(NetPkt* pPkt, DNSQuestion* pQuestion)
{
	int16_t result = encodeName(pPkt, pQuestion->pName);
	if(result)
		return result;

	DNSQuestionDetails* pDetails = NetBufAdd(&pPkt->buf, sizeof(DNSQuestionDetails));
	if(!pDetails)
		return ERROR_NOT_ENOUGH_MEMORY;

	pDetails->qType = htons(pQuestion->qType);
	pDetails->qClass = htons(pQuestion->qClass);

	return 0;
}

static int16_t packetAddResourceRecord(NetPkt* pPkt, DNSResourceRecord* pRecord)
{
	int16_t result = encodeName(pPkt, pRecord->pName);
	if(result)
		return result;

	DNSResourceRecordDetails* pDetails = NetBufAdd(&pPkt->buf, sizeof(DNSResourceRecordDetails));
	if(!pDetails)
		return ERROR_NOT_ENOUGH_MEMORY;

	pDetails->rrType = htons(pRecord->rrType);
	pDetails->rrClass = htons(pRecord->rrClass);
	pDetails->ttl = htonl(pRecord->ttl_s);
	pDetails->dataLength = htons(pRecord->dataLength);

	void* pRecordData = NetBufAdd(&pPkt->buf, pRecord->dataLength);
	if(!pRecordData)
		return ERROR_NOT_ENOUGH_MEMORY;

	memcpy(pRecordData, pRecord->pData, pRecord->dataLength);

	return 0;
}

int16_t DNSMessageTxBuildPacket(DNSMessage* pMsg, NetPkt* pPkt)
{
	int16_t result = 0;

	DNSHeader* pHeader = NetBufAdd(&pPkt->buf, sizeof(DNSHeader));
	if(!pHeader)
		return ERROR_NOT_ENOUGH_MEMORY;

	pHeader->id = htons(pMsg->pHeader->id);
	pHeader->flags = htons(pMsg->pHeader->flags);
	pHeader->numQuestions = htons(ListSize(pMsg->questions));
	pHeader->numAnswers = htons(ListSize(pMsg->answerRecords));
	pHeader->numAuthoritiveRecords = htons(ListSize(pMsg->authoritiveRecords));
	pHeader->numAdditionalRecords = htons(ListSize(pMsg->additionalRecords));

	for(DNSQuestion* pQuestion = ListFront(pMsg->questions); pQuestion; pQuestion = pQuestion->pNext)
	{
		result = packetAddQuestion(pPkt, pQuestion);
		if(result)
			return result;
	}

	for(DNSResourceRecord* pRes = ListFront(pMsg->answerRecords); pRes; pRes = pRes->pNext)
	{
		result = packetAddResourceRecord(pPkt, pRes);
		if(result)
			return result;
	}

	for(DNSResourceRecord* pRes = ListFront(pMsg->authoritiveRecords); pRes; pRes = pRes->pNext)
	{
		result = packetAddResourceRecord(pPkt, pRes);
		if(result)
			return result;
	}

	for(DNSResourceRecord* pRes = ListFront(pMsg->additionalRecords); pRes; pRes = pRes->pNext)
	{
		result = packetAddResourceRecord(pPkt, pRes);
		if(result)
			return result;
	}

	return 0;
}
