/*
 * sntp.c
 *
 *  Created on: 23.03.2016
 *      Author: AndreR
 */

#include "sntp.h"
#include "errors.h"
#include <string.h>
#include "util/sys_time.h"

int16_t SNTPDecode(uint8_t* pSrc, uint16_t srcLength, NTPPacket** ppPacket)
{
	if(srcLength < NTP_PACKET_SIZE)
		return ERROR_SNTP_BAD_PACKET;

	*ppPacket = (NTPPacket*)pSrc;

	(*ppPacket)->rootDelay = ntohl((*ppPacket)->rootDelay);
	(*ppPacket)->rootDispersion = ntohl((*ppPacket)->rootDispersion);
	(*ppPacket)->referenceIdentifier = ntohl((*ppPacket)->referenceIdentifier);
	(*ppPacket)->referenceTimestampSec = ntohl((*ppPacket)->referenceTimestampSec);
	(*ppPacket)->referenceTimestampFrac = ntohl((*ppPacket)->referenceTimestampFrac);
	(*ppPacket)->originateTimestampSec = ntohl((*ppPacket)->originateTimestampSec);
	(*ppPacket)->originateTimestampFrac = ntohl((*ppPacket)->originateTimestampFrac);
	(*ppPacket)->receiveTimestampSec = ntohl((*ppPacket)->receiveTimestampSec);
	(*ppPacket)->receiveTimestampFrac = ntohl((*ppPacket)->receiveTimestampFrac);
	(*ppPacket)->transmitTimestampSec = ntohl((*ppPacket)->transmitTimestampSec);
	(*ppPacket)->transmitTimestampFrac = ntohl((*ppPacket)->transmitTimestampFrac);

	if((*ppPacket)->transmitTimestampSec == 0 && (*ppPacket)->transmitTimestampFrac == 0)
		return ERROR_SNTP_BAD_PACKET;

	return 0;
}

int16_t SNTPEncode(uint8_t* pBuffer, uint16_t bufferLength, uint16_t* pEncodedLength, uint32_t unixTransmitTimestamp)
{
	// SNTP is top level, nothing to encapsulate
	if(bufferLength < NTP_PACKET_SIZE)
		return ERROR_NOT_ENOUGH_MEMORY;

	NTPPacket* pNTP = (NTPPacket*)pBuffer;

	memset(pNTP, 0, sizeof(NTPPacket));

	pNTP->versionNumber = 4; // SNTPv4
	pNTP->mode = 3; // client mode

	pNTP->transmitTimestampFrac = 0;
	pNTP->transmitTimestampSec = UNIX_TO_NTP(unixTransmitTimestamp);

	pNTP->transmitTimestampSec = htonl(pNTP->transmitTimestampSec);
	pNTP->transmitTimestampFrac = htonl(pNTP->transmitTimestampFrac);

	*pEncodedLength = NTP_PACKET_SIZE;

	return 0;
}
