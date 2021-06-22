/*
 * ipv4.c
 *
 *  Created on: 20.02.2011
 *      Author: AndreR
 */

#include "ipv4.h"
#include <string.h>

int16_t IPv4Decode(uint8_t* pSrc, uint16_t srcLength, uint8_t** ppDecoded, uint16_t* pDecodedLength, IPv4Header** ppHeader)
{
	if(srcLength <= IPV4_HEADER_SIZE)
		return ERROR_IPV4_BAD_PACKET;

	*ppHeader = (IPv4Header*)pSrc;

	(*ppHeader)->totalLength = ntohs((*ppHeader)->totalLength);
	(*ppHeader)->identification = ntohs((*ppHeader)->identification);
	(*ppHeader)->headerCRC = ntohs((*ppHeader)->headerCRC);
	(*ppHeader)->fragment = ntohs((*ppHeader)->fragment);

	*ppDecoded = pSrc + (*ppHeader)->ihl*4;

	*pDecodedLength = (*ppHeader)->totalLength - (*ppHeader)->ihl*4;

	if((*ppHeader)->version != 4)
		return ERROR_IPV4_NOT_SUPPORTED;

	return 0;
}

int16_t IPv4Encode(	uint8_t* pBuffer, uint16_t bufferLength, uint16_t dataLength, uint16_t* pEncodedLength,
					uint16_t identification, IPv4Address senderIP, IPv4Address targetIP, uint8_t protocol)
{
	if(dataLength + IPV4_HEADER_SIZE > bufferLength)
		return ERROR_NOT_ENOUGH_MEMORY;

	memmove(pBuffer+IPV4_HEADER_SIZE, pBuffer, dataLength);

	IPv4Header* pHeader = (IPv4Header*)pBuffer;

	pHeader->version = 4;
	pHeader->ihl = 5;
	pHeader->tos = 0;
	pHeader->totalLength = htons(IPV4_HEADER_SIZE + dataLength);
	pHeader->identification = htons(identification);
	pHeader->flags = 2;
	pHeader->fragmentOffset = 0;
	pHeader->fragment = htons(pHeader->fragment);
	pHeader->ttl = 32;
	pHeader->protocol = protocol;
	pHeader->srcIP = senderIP;
	pHeader->dstIP = targetIP;
	pHeader->headerCRC = 0;

	*pEncodedLength = dataLength + IPV4_HEADER_SIZE;

	return 0;
}

int16_t IPv4EncodeWithOptions(	uint8_t* pBuffer, uint16_t bufferLength, uint16_t dataLength, uint16_t* pEncodedLength,
								uint16_t identification, IPv4Address senderIP, IPv4Address targetIP, uint8_t protocol,
								uint8_t* pOptionData, uint16_t optionLength)
{
	if(dataLength + optionLength + IPV4_HEADER_SIZE > bufferLength)
		return ERROR_NOT_ENOUGH_MEMORY;

	if(optionLength % 4 != 0)
		return ERROR_IPV4_BAD_PACKET;

	memmove(pBuffer+IPV4_HEADER_SIZE+optionLength, pBuffer, dataLength);

	IPv4Header* pHeader = (IPv4Header*)pBuffer;

	pHeader->version = 4;
	pHeader->ihl = 5 + optionLength/4;
	pHeader->tos = 0;
	pHeader->totalLength = htons(IPV4_HEADER_SIZE + dataLength + optionLength);
	pHeader->identification = htons(identification);
	pHeader->flags = 2;
	pHeader->fragmentOffset = 0;
	pHeader->fragment = htons(pHeader->fragment);
	pHeader->ttl = 32;
	pHeader->protocol = protocol;
	pHeader->srcIP = senderIP;
	pHeader->dstIP = targetIP;
	pHeader->headerCRC = 0;

	memcpy(pBuffer+IPV4_HEADER_SIZE, pOptionData, optionLength);

	*pEncodedLength = dataLength + IPV4_HEADER_SIZE + optionLength;

	return 0;
}

int16_t IPv4EncodeWithRouterAlertOption(uint8_t* pBuffer, uint16_t bufferLength, uint16_t dataLength,
										uint16_t* pEncodedLength, uint16_t identification,
										IPv4Address senderIP, IPv4Address targetIP, uint8_t protocol)
{
	uint8_t alertData[4];
	alertData[0] = 0b10010100;
	alertData[1] = 0b00000100;
	alertData[2] = 0;
	alertData[3] = 0;

	return IPv4EncodeWithOptions(pBuffer, bufferLength, dataLength, pEncodedLength, identification,
			senderIP, targetIP, protocol, alertData, 4);
}

