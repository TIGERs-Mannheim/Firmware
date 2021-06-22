/*
 * igmpv3.h
 *
 *  Created on: 15.04.2011
 *      Author: AndreR
 */

#ifndef __IGMPV3_H__
#define __IGMPV3_H__

#include "inet.h"

#define IGMP_MEMBERSHIP_QUERY		0x11
#define IGMPV3_MEMBERSHIP_REPORT	0x22
#define IGMPV1_MEMBERSHIP_REPORT	0x12
#define IGMPV2_MEMBERSHIP_REPORT	0x16
#define IGMPV2_LEAVE_GROUP			0x17

// Group Record Types
#define IGMPV3_MODE_IS_INCLUDE	0x01
#define IGMPV3_MODE_IS_EXCLUDE	0x02
#define IGMPV3_CHANGE_TO_INCLUDE	0x03
#define IGMPV3_CHANGE_TO_EXCLUDE	0x04
#define IGMPV3_ALLOW_NEW_SOURCES	0x05
#define IGMPV3_BLOCK_OLD_SOURCES	0x06

void IGMPv3GetType(uint8_t* pSrc, uint8_t* pType);

int16_t IGMPv3DecodeMembershipQuery(uint8_t* pSrc, uint16_t srcLength, IGMPv3MembershipQuery** ppQuery,
									IPv4Address** ppSourceListStart);

int16_t IGMPv3DecodeMembershipReport(uint8_t* pSrc, uint16_t srcLength, IGMPv3MembershipReport** ppReport,
									uint8_t** ppGroupRecordStart);

int16_t IGMPv3EncodeMembershipReport(uint8_t* pBuffer, uint16_t bufferLength, uint16_t* pEncodedLengh,
									uint16_t numberOfGroupRecords, uint8_t* recordData, uint16_t recordDataLength);

uint32_t IGMPv3DecodeMaxRespCode(uint8_t code);

int16_t IGMPv2EncodeMembershipReport(uint8_t* pBuffer,
		uint16_t bufferLength, uint16_t* pEncodedLengh, const IPv4Address* pMulticastAddress);

#endif /* IGMPV3_H_ */
