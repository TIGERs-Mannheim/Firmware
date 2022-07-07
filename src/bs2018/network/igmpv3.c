/*
 * igmpv3.c
 *
 *  Created on: 15.04.2011
 *      Author: AndreR
 */

#include "igmpv3.h"
#include "errors.h"
#include <string.h>

void IGMPv3GetType(uint8_t* pSrc, uint8_t* pType)
{
	*pType = *pSrc;
}

int16_t IGMPv3DecodeMembershipQuery(uint8_t* pSrc, uint16_t srcLength, IGMPv3MembershipQuery** ppQuery,
									IPv4Address** ppSourceListStart)
{
	if(srcLength < IGMPV3_MEMBERSHIP_QUERY_SIZE)
		return ERROR_IGMP_BAD_PACKET;

	*ppSourceListStart = (IPv4Address*)(pSrc + IGMPV3_MEMBERSHIP_QUERY_SIZE);

	*ppQuery = (IGMPv3MembershipQuery*)pSrc;

	uint16_t checksum = IPv4CalcChecksum(pSrc, srcLength);

	(*ppQuery)->checksum = ntohs((*ppQuery)->checksum);
	(*ppQuery)->numberOfSources = ntohs((*ppQuery)->numberOfSources);

	if(checksum != 0)
		return ERROR_IGMP_BAD_CHECKSUM;

	return 0;
}

int16_t IGMPv3DecodeMembershipReport(uint8_t* pSrc, uint16_t srcLength, IGMPv3MembershipReport** ppReport,
									uint8_t** ppGroupRecordStart)
{
	if(srcLength < IGMPV3_MEMBERSHIP_REPORT_SIZE)
		return ERROR_IGMP_BAD_PACKET;

	*ppGroupRecordStart = pSrc + IGMPV3_MEMBERSHIP_REPORT_SIZE;

	*ppReport = (IGMPv3MembershipReport*)pSrc;

	uint16_t checksum = IPv4CalcChecksum(pSrc, srcLength);

	(*ppReport)->checksum = ntohs((*ppReport)->checksum);
	(*ppReport)->numberOfGroupRecords = ntohs((*ppReport)->numberOfGroupRecords);

	if(checksum != 0)
		return ERROR_IGMP_BAD_CHECKSUM;

	return 0;
}

int16_t IGMPv3EncodeMembershipReport(uint8_t* pBuffer, uint16_t bufferLength, uint16_t* pEncodedLengh,
									uint16_t numberOfGroupRecords, uint8_t* recordData, uint16_t recordDataLength)
{
	if(recordDataLength + IGMPV3_MEMBERSHIP_REPORT_SIZE > bufferLength)
		return ERROR_NOT_ENOUGH_MEMORY;

	if(recordDataLength % 4 != 0)
		return ERROR_IGMP_BAD_PACKET;

	*pEncodedLengh = recordDataLength + IGMPV3_MEMBERSHIP_REPORT_SIZE;

	IGMPv3MembershipReport* pReport = (IGMPv3MembershipReport*)pBuffer;

	pReport->reserved1 = 0;
	pReport->reserved2 = 0;
	pReport->checksum = 0;
	pReport->type = IGMPV3_MEMBERSHIP_REPORT;
	pReport->numberOfGroupRecords = htons(numberOfGroupRecords);

	memcpy(pBuffer + IGMPV3_MEMBERSHIP_REPORT_SIZE, recordData, recordDataLength);

	pReport->checksum = IPv4CalcChecksum(pBuffer, *pEncodedLengh);

	return 0;
}

uint32_t IGMPv3DecodeMaxRespCode(uint8_t code)
{
	uint32_t tenth = 0;

	if(code & 0x80)
	{
		tenth = ((code & 0x0F) | 0x10) << (((code & 0x70) >> 4) + 3);
	}
	else
	{
		tenth = code;
	}

	return tenth * 100;
}

int16_t IGMPv2EncodeMembershipReport(uint8_t* pBuffer, uint16_t bufferLength, uint16_t* pEncodedLengh, const IPv4Address* pMulticastAddress)
{
	if(IGMPV2_MEMBERSHIP_REPORT_SIZE > bufferLength)
		return ERROR_NOT_ENOUGH_MEMORY;

	*pEncodedLengh = IGMPV2_MEMBERSHIP_REPORT_SIZE;

	IGMPv2MembershipReport* pReport = (IGMPv2MembershipReport*)pBuffer;

	pReport->type = IGMPV2_MEMBERSHIP_REPORT;
	pReport->maxRespTime = 0;
	pReport->groupAddress.u32 = pMulticastAddress->u32;
	pReport->checksum = 0;

	pReport->checksum = IPv4CalcChecksum(pBuffer, *pEncodedLengh);

	return 0;
}
