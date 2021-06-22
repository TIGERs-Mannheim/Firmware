/*
 * icmp.c
 *
 *  Created on: 18.04.2011
 *      Author: AndreR
 */

#include "icmp.h"
#include "errors.h"
#include <string.h>

int16_t ICMPDecode(uint8_t* pSrc, uint16_t srcLength, uint8_t** ppDecoded, uint16_t* pDecodedLength, ICMPHeader** ppHeader)
{
	if(srcLength <= ICMP_HEADER_SIZE)
		return ERROR_ICMP_BAD_PACKET;

	*ppHeader = (ICMPHeader*)pSrc;

	(*ppHeader)->checksum = ntohs((*ppHeader)->checksum);

	*ppDecoded = pSrc + ICMP_HEADER_SIZE;
	*pDecodedLength = srcLength - ICMP_HEADER_SIZE;

	return 0;
}

int16_t ICMPDecodeEcho(uint8_t* pSrc, uint16_t srcLength, uint8_t** ppDecoded, uint16_t* pDecodedLength, ICMPEcho** ppEcho)
{
	if(srcLength <= ICMP_ECHO_SIZE)
		return ERROR_ICMP_BAD_PACKET;

	*ppEcho = (ICMPEcho*)pSrc;

	(*ppEcho)->identifier = ntohs((*ppEcho)->identifier);
	(*ppEcho)->sequenceNumber = ntohs((*ppEcho)->sequenceNumber);

	*ppDecoded = pSrc + ICMP_ECHO_SIZE;
	*pDecodedLength = srcLength - ICMP_ECHO_SIZE;

	return 0;
}

int16_t ICMPEncode(	uint8_t* pBuffer, uint16_t bufferLength, uint16_t dataLength, uint16_t* pEncodedLength,
					uint8_t type, uint8_t code)
{
	if(dataLength + ICMP_HEADER_SIZE > bufferLength)
		return ERROR_NOT_ENOUGH_MEMORY;

	memmove(pBuffer+ICMP_HEADER_SIZE, pBuffer, dataLength);

	ICMPHeader* pHeader = (ICMPHeader*)pBuffer;

	pHeader->type = type;
	pHeader->code = code;
	pHeader->checksum = 0;

	*pEncodedLength = dataLength + ICMP_HEADER_SIZE;

	return 0;
}

int16_t ICMPEncodeEcho(	uint8_t* pBuffer, uint16_t bufferLength, uint16_t dataLength, uint16_t* pEncodedLength,
						uint16_t identifier, uint16_t sequence)
{
	if(dataLength + ICMP_ECHO_SIZE > bufferLength)
		return ERROR_NOT_ENOUGH_MEMORY;

	memmove(pBuffer+ICMP_ECHO_SIZE, pBuffer, dataLength);

	ICMPEcho* pEcho = (ICMPEcho*)pBuffer;

	pEcho->identifier = htons(identifier);
	pEcho->sequenceNumber = htons(sequence);

	*pEncodedLength = dataLength + ICMP_ECHO_SIZE;

	return 0;
}
