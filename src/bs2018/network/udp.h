/*
 * udp.h
 *
 *  Created on: 23.02.2011
 *      Author: AndreR
 */

#ifndef __UDP_H__
#define __UDP_H__

#include "errors.h"
#include "inet.h"

int16_t UDPDecode(uint8_t* pSrc, uint16_t srcLength, uint8_t** ppDecoded, uint16_t* pDecodedLength, UDPHeader** ppHeader);
int16_t UDPEncode(uint8_t* pBuffer, uint16_t bufferLength, uint16_t dataLength, uint16_t* pEncodedLength, uint16_t srcPort, uint16_t dstPort);

#endif /* UDP_H_ */
