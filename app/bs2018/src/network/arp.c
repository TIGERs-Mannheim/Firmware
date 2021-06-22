/*
 * arp.c
 *
 *  Created on: 20.02.2011
 *      Author: AndreR
 */

#include "arp.h"

int16_t ARPDecode(uint8_t* pSrc, uint16_t srcLength, ARPPacketIPv4** ppPacket)
{
	if(srcLength < ARP_PACKET_SIZE)
		return ERROR_ARP_BAD_PACKET;

	*ppPacket = (ARPPacketIPv4*)pSrc;

	(*ppPacket)->hardwareType = ntohs((*ppPacket)->hardwareType);
	(*ppPacket)->protocolType = ntohs((*ppPacket)->protocolType);
	(*ppPacket)->operation = ntohs((*ppPacket)->operation);

	if((*ppPacket)->hardwareType != 1)	// link layer protocol != ethernet?
		return ERROR_ARP_NOT_SUPPORTED;

	if((*ppPacket)->protocolType != 0x0800)	// protocol type != ipv4?
		return ERROR_ARP_NOT_SUPPORTED;

	if((*ppPacket)->hwAddrLen != 6 || (*ppPacket)->protAddrLen != 4)
		return ERROR_ARP_BAD_PACKET;

	return 0;
}

int16_t ARPEncode(	uint8_t* pBuffer, uint16_t bufferLength, uint16_t* pEncodedLength,
					uint8_t operation, MAC senderMAC, IPv4Address senderIP, MAC targetMAC, IPv4Address targetIP)
{
	// ARP is top level, nothing to encapsule
	if(bufferLength < ARP_PACKET_SIZE)
		return ERROR_NOT_ENOUGH_MEMORY;

	ARPPacketIPv4* pARP = (ARPPacketIPv4*)pBuffer;

	pARP->hardwareType = 1;
	pARP->protocolType = 0x0800;
	pARP->hwAddrLen = 6;
	pARP->protAddrLen = 4;
	pARP->operation = operation;
	pARP->srcHwAddr = senderMAC;
	pARP->srcProtAddr = senderIP;
	pARP->dstHwAddr = targetMAC;
	pARP->dstProtAddr = targetIP;

	pARP->hardwareType = htons(pARP->hardwareType);
	pARP->protocolType = htons(pARP->protocolType);
	pARP->operation = htons(pARP->operation);

	*pEncodedLength = ARP_PACKET_SIZE;

	return 0;
}
