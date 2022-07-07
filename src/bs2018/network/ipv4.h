/*
 * ipv4.h
 *
 *  Created on: 20.02.2011
 *      Author: AndreR
 */

#ifndef __IPV4_H__
#define __IPV4_H__

#include "errors.h"
#include "inet.h"

#define IPV4_PROTOCOL_UDP 17
#define IPV4_PROTOCOL_IGMP 2
#define IPV4_PROTOCOL_ICMP 1

int16_t IPv4Decode(uint8_t* pSrc, uint16_t srcLength, uint8_t** ppDecoded, uint16_t* pDecodedLength, IPv4Header** ppHeader);

int16_t IPv4Encode(	uint8_t* pBuffer, uint16_t bufferLength, uint16_t dataLength, uint16_t* pEncodedLength,
					uint16_t identification, IPv4Address senderIP, IPv4Address targetIP, uint8_t protocol);

int16_t IPv4EncodeWithOptions(	uint8_t* pBuffer, uint16_t bufferLength, uint16_t dataLength, uint16_t* pEncodedLength,
								uint16_t identification, IPv4Address senderIP, IPv4Address targetIP, uint8_t protocol,
								uint8_t* pOptionData, uint16_t optionLength);

int16_t IPv4EncodeWithRouterAlertOption(uint8_t* pBuffer, uint16_t bufferLength, uint16_t dataLength,
										uint16_t* pEncodedLength, uint16_t identification,
										IPv4Address senderIP, IPv4Address targetIP, uint8_t protocol);

#endif /* IPV4_H_ */
