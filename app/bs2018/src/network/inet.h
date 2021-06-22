/*
 * network.h
 *
 *  Created on: 13.02.2011
 *      Author: AndreR
 */

#ifndef __INET_H__
#define __INET_H__

#include <stdint.h>

#define PACKED __attribute__ ((packed))

#define PPPINITFCS16	0xffff	// Initial FCS value
#define PPPGOODFCS16	0xf0b8	// Good final FCS value

#define ARP_OPERATION_REQUEST	1
#define ARP_OPERATION_REPLY		2

typedef union PACKED _MAC
{
	uint8_t u8[6];

	uint16_t u16[3];

	struct PACKED
	{
		uint8_t multicast:1;	// or broadcast
		uint8_t local:1;		// locally administered
		uint8_t _unused1:6;
		uint8_t _unused2[5];
	} cast;
} MAC;

typedef union PACKED _IPv4Address
{
	uint8_t u8[4];
	uint16_t u16[2];
	uint32_t u32;
} IPv4Address;

// byte order functions
uint16_t htons(uint16_t host);
uint32_t htonl(uint32_t host);
uint16_t ntohs(uint16_t net);
uint32_t ntohl(uint32_t net);

// MAC functions
uint8_t MACCompare(MAC mac1, MAC mac2);
MAC		MACSet(uint8_t a, uint8_t b, uint8_t c, uint8_t d, uint8_t e, uint8_t f);

// IP functions
uint16_t IPv4CalcChecksum(uint8_t* pData, uint16_t length);
IPv4Address IPv4AddressSet(uint8_t a, uint8_t b, uint8_t c, uint8_t d);
MAC IPv4CalcMulticastMAC(IPv4Address addr);

// PPP checksum function
uint16_t pppfcs16(uint8_t* cp, uint16_t len, uint16_t fcs);

#define FRAME_HEADER_SIZE 14

typedef union PACKED _FrameHeader
{
	struct PACKED
	{
		MAC dstMAC;
		MAC srcMAC;
		uint16_t type;	// length of following data or 0x0806 for ARP packet
	};

	uint8_t _data[FRAME_HEADER_SIZE];
} FrameHeader;

#define IPV4_HEADER_SIZE 20

typedef union PACKED _IPv4Header
{
	struct PACKED
	{
		uint8_t ihl:4;		// ip header length, multiples of 4 byte, 5 = 20byte
		uint8_t version:4;	// 4 = IPv4
		uint8_t tos;		// Type of service (0)
		uint16_t totalLength;	// Data length + header length
		uint16_t identification;	// unique datagram number
		union PACKED
		{
			struct PACKED
			{
				uint16_t fragmentOffset:13;	// 0 of not fragmented
				uint16_t flags:3;	// 2 = don't fragment, last fragment
			};

			uint16_t fragment;
		};
		uint8_t ttl;		// default = 10
		uint8_t protocol;	// UDP = 17
		uint16_t headerCRC;
		IPv4Address srcIP;
		IPv4Address dstIP;
	};

	uint8_t _data[IPV4_HEADER_SIZE];
} IPv4Header;

#define UDP_HEADER_SIZE 8

typedef union PACKED _UDPHeader
{
	struct PACKED
	{
		uint16_t srcPort;
		uint16_t dstPort;
		uint16_t length;	// data + header
		uint16_t crc;		// cann be 0 if not used
	};

	uint8_t _data[UDP_HEADER_SIZE];
} UDPHeader;

#define ARP_PACKET_SIZE 28

typedef union PACKED _ARPPacketIPv4
{
	struct PACKED
	{
		uint16_t hardwareType;	// Link layer protocol, 1 = Ethernet
		uint16_t protocolType;	// 0x0800 = IPv4
		uint8_t hwAddrLen;		// hardware address length, MAC = 6
		uint8_t protAddrLen;	// protocol address length, IPv4 = 4
		uint16_t operation;		// 1 = request, 2 = reply
		MAC srcHwAddr;		// sender hardware address = MAC of sender
		IPv4Address srcProtAddr;	// sender protocol address = IP of sender
		MAC dstHwAddr;	// target hardware address = MAC of target, ignored in request
		IPv4Address dstProtAddr;	// target protocol address = IP of target
	};

	uint8_t _data[ARP_PACKET_SIZE];
} ARPPacketIPv4;

#define IGMPV3_MEMBERSHIP_QUERY_SIZE 12
typedef union PACKED _IGMPv3MembershipQuery
{
	struct PACKED
	{
		uint8_t type;
		uint8_t maxRespCode;
		uint16_t checksum;
		IPv4Address groupAddress;
		uint8_t QRV:3;
		uint8_t S:1;
		uint8_t resv:4;
		uint8_t QQIC;
		uint16_t numberOfSources;
	};

	uint8_t _data[IGMPV3_MEMBERSHIP_QUERY_SIZE];

	// + numberOfSources * IPv4Addresses as source filter
} IGMPv3MembershipQuery;

#define IGMPV3_MEMBERSHIP_REPORT_SIZE 8
typedef union PACKED _IGMPv3MembershipReport
{
	struct PACKED
	{
		uint8_t type;
		uint8_t reserved1;
		uint16_t checksum;
		uint16_t reserved2;
		uint16_t numberOfGroupRecords;
	};

	uint8_t _data[IGMPV3_MEMBERSHIP_REPORT_SIZE];

	// + numberOfGroupRecords * IGMPv3GroupRecord
} IGMPv3MembershipReport;

#define IGMPV3_GROUP_RECORD_SIZE 8
typedef union PACKED _IGMPv3GroupRecord
{
	struct PACKED
	{
		uint8_t recordType;
		uint8_t auxDataLen;
		uint16_t numberOfSources;
		IPv4Address multicastAddress;
	};

	uint8_t _data[IGMPV3_GROUP_RECORD_SIZE];

	// + numberOfSources * IPv4Addresses
	// + Aux Data
} IGMPv3GroupRecord;

#define IGMPV2_MEMBERSHIP_REPORT_SIZE 8
typedef union PACKED _IGMPv2MembershipReport
{
	struct PACKED
	{
		uint8_t type;
		uint8_t maxRespTime;
		uint16_t checksum;
		IPv4Address groupAddress;
	};

	uint8_t _data[IGMPV2_MEMBERSHIP_REPORT_SIZE];
} IGMPv2MembershipReport;

#define ICMP_HEADER_SIZE 4
typedef union PACKED _ICMPHeader
{
	struct PACKED
	{
		uint8_t type;
		uint8_t code;
		uint16_t checksum;
	};

	uint8_t _data[ICMP_HEADER_SIZE];
} ICMPHeader;

#define ICMP_DESTINATION_UNREACHABLE_SIZE (12+IPV4_HEADER_SIZE)
typedef union PACKED _ICMPDestinationUnreachable
{
	struct PACKED
	{
		uint32_t unused;
		IPv4Header ipHeader;
		uint8_t originalData[8];
	};

	uint8_t _data[ICMP_DESTINATION_UNREACHABLE_SIZE];

} ICMPDestinationUnreachable;

#define ICMP_ECHO_SIZE 4
typedef union PACKED _ICMPEcho
{
	struct PACKED
	{
		uint16_t identifier;
		uint16_t sequenceNumber;
	};

	uint8_t _data[ICMP_ECHO_SIZE];

	// + Data
} ICMPEcho;

#define NTP_PORT 123

#define NTP_PACKET_SIZE 48
typedef union PACKED _NTPPacket
{
	struct PACKED
	{
		uint8_t mode:3;
		uint8_t versionNumber:3;
		uint8_t leapIndicator:2;
		uint8_t stratum;
		uint8_t poll;
		uint8_t precision;

		uint32_t rootDelay;
		uint32_t rootDispersion;

		uint32_t referenceIdentifier;
		uint32_t referenceTimestampSec;	// timestamps indicate time sind 1.1.1900
		uint32_t referenceTimestampFrac;

		uint32_t originateTimestampSec;
		uint32_t originateTimestampFrac;

		uint32_t receiveTimestampSec;
		uint32_t receiveTimestampFrac;

		uint32_t transmitTimestampSec;
		uint32_t transmitTimestampFrac;
	};

	uint8_t _data[NTP_PACKET_SIZE];
} NTPPacket;

#endif /* NET_STRUCTS_H_ */
