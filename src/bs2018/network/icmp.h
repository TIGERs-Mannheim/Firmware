/*
 * icmp.h
 *
 *  Created on: 18.04.2011
 *      Author: AndreR
 */

#ifndef __ICMP_H__
#define __ICMP_H__

#include "inet.h"

#define ICMP_TYPE_ECHO_REPLY 0
#define ICMP_TYPE_DEST_UNREACHABLE 3
#define ICMP_TYPE_ECHO_REQUEST 8

#define ICMP_PROTOCOL_UNREACHABLE 2
#define ICMP_PORT_UNREACHABLE 3

int16_t ICMPDecode(uint8_t* pSrc, uint16_t srcLength, uint8_t** ppDecoded, uint16_t* pDecodedLength, ICMPHeader** ppHeader);
int16_t ICMPDecodeEcho(uint8_t* pSrc, uint16_t srcLength, uint8_t** ppDecoded, uint16_t* pDecodedLength, ICMPEcho** ppEcho);
int16_t ICMPEncode(	uint8_t* pBuffer, uint16_t bufferLength, uint16_t dataLength, uint16_t* pEncodedLength,
					uint8_t type, uint8_t code);
int16_t ICMPEncodeEcho(	uint8_t* pBuffer, uint16_t bufferLength, uint16_t dataLength, uint16_t* pEncodedLength,
						uint16_t identifier, uint16_t sequence);

#endif /* ICMP_H_ */
