/*
 * ethernet.c
 *
 *  Created on: 20.02.2011
 *      Author: AndreR
 */

#include "ethernet.h"
#include <string.h>

int16_t EthernetDecode(uint8_t* pSrc, uint16_t srcLength, uint8_t** ppDecoded, uint16_t* pDecodedLength, FrameHeader** ppHeader)
{
	if(srcLength <= FRAME_HEADER_SIZE)
		return ERROR_ETHERNET_BAD_FRAME;

	*ppDecoded = pSrc + FRAME_HEADER_SIZE;

	*ppHeader = (FrameHeader*)pSrc;

	(*ppHeader)->type = ntohs((*ppHeader)->type);

	uint16_t type = (*ppHeader)->type;

	if(type != ETHERNET_PROTOCOL_IPV4 && type != ETHERNET_PROTOCOL_ARP)
		return ERROR_ETHERNET_UNSUPPORTED_PROTOCOL;

	*pDecodedLength = srcLength - FRAME_HEADER_SIZE;

	return 0;
}

int16_t EthernetEncode(	uint8_t* pBuffer, uint16_t bufferLength, uint16_t dataLength, uint16_t* pEncodedLength,
						MAC senderMAC, MAC targetMAC, uint16_t type)
{
	if(dataLength + FRAME_HEADER_SIZE > bufferLength)
		return ERROR_NOT_ENOUGH_MEMORY;

	// we need to prepend data, so move the current data to the right
	memmove(pBuffer+FRAME_HEADER_SIZE, pBuffer, dataLength);

	FrameHeader* pHeader = (FrameHeader*)pBuffer;
	pHeader->type = htons(type);
	pHeader->srcMAC = senderMAC;
	pHeader->dstMAC = targetMAC;

	*pEncodedLength = dataLength + FRAME_HEADER_SIZE;

	return 0;
}

