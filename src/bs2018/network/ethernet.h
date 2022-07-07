/*
 * ethernet.h
 *
 *  Created on: 20.02.2011
 *      Author: AndreR
 */

#ifndef __ETHERNET_H__
#define __ETHERNET_H__

#include "errors.h"
#include "inet.h"

#define ETHERNET_PROTOCOL_IPV4	0x0800
#define ETHERNET_PROTOCOL_ARP	0x0806

int16_t EthernetDecode(uint8_t* pSrc, uint16_t srcLength, uint8_t** ppDecoded, uint16_t* pDecodedLength, FrameHeader** ppHeader);
int16_t EthernetEncode(	uint8_t* pBuffer, uint16_t bufferLength, uint16_t dataLength, uint16_t* pEncodedLength,
						MAC senderMAC, MAC targetMAC, uint16_t type);

#endif /* ETHERNET_H_ */
