/*
 * arp.h
 *
 *  Created on: 20.02.2011
 *      Author: AndreR
 */

#ifndef __ARP_H__
#define __ARP_H__

#include "errors.h"
#include "inet.h"

int16_t ARPDecode(uint8_t* pSrc, uint16_t srcLength, ARPPacketIPv4** ppPacket);
int16_t ARPEncode(	uint8_t* pBuffer, uint16_t bufferLength, uint16_t* pEncodedLength,
					uint8_t operation, MAC senderMAC, IPv4Address senderIP, MAC targetMAC, IPv4Address targetIP);


#endif /* ARP_H_ */
