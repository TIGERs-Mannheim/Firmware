/*
 * udp.c
 *
 *  Created on: 23.02.2011
 *      Author: AndreR
 */

#include "udp.h"
#include <string.h>

int16_t UDPDecode(uint8_t* pSrc, uint16_t srcLength, uint8_t** ppDecoded, uint16_t* pDecodedLength, UDPHeader** ppHeader)
{
	if(srcLength <= UDP_HEADER_SIZE)
		return ERROR_UDP_BAD_PACKET;

	*ppDecoded = pSrc + UDP_HEADER_SIZE;

	*ppHeader = (UDPHeader*)pSrc;

	(*ppHeader)->crc = ntohs((*ppHeader)->crc);
	(*ppHeader)->dstPort = ntohs((*ppHeader)->dstPort);
	(*ppHeader)->srcPort = ntohs((*ppHeader)->srcPort);
	(*ppHeader)->length = ntohs((*ppHeader)->length);

	*pDecodedLength = (*ppHeader)->length - UDP_HEADER_SIZE;

	return 0;
}

int16_t UDPEncode(uint8_t* pBuffer, uint16_t bufferLength, uint16_t dataLength, uint16_t* pEncodedLength, uint16_t srcPort, uint16_t dstPort)
{
	if(dataLength + UDP_HEADER_SIZE > bufferLength)
		return ERROR_NOT_ENOUGH_MEMORY;

	memmove(pBuffer+UDP_HEADER_SIZE, pBuffer, dataLength);

	UDPHeader* pHeader = (UDPHeader*)pBuffer;

	pHeader->srcPort = htons(srcPort);
	pHeader->dstPort = htons(dstPort);
	pHeader->length = htons(dataLength + UDP_HEADER_SIZE);
	pHeader->crc = 0;

	*pEncodedLength = dataLength + UDP_HEADER_SIZE;

	return 0;
}

