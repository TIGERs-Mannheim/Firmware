/*
 * sntp.h
 *
 *  Created on: 23.03.2016
 *      Author: AndreR
 */

#ifndef NETWORK_SNTP_H_
#define NETWORK_SNTP_H_

#include "inet.h"

int16_t SNTPDecode(uint8_t* pSrc, uint16_t srcLength, NTPPacket** ppPacket);
int16_t SNTPEncode(uint8_t* pBuffer, uint16_t bufferLength, uint16_t* pEncodedLength, uint32_t unixTransmitTimestamp);

#endif /* NETWORK_SNTP_H_ */
