/**
 * Domain Name System
 *
 * Only basic DNS functions. A full DNS resolver is not implemented.
 *
 * According to: RFC 1034, RFC 1035
 */

#pragma once

#include "net_pkt.h"

typedef struct
{
  void* pNext;

  char* pName;
  uint16_t rrType;
  uint16_t rrClass;
  uint32_t ttl_s;
  uint16_t dataLength;
  void* pData;
} DNSResourceRecord;

typedef struct
{
  void* pNext;

  char* pName;
  uint16_t qType;
  uint16_t qClass;
} DNSQuestion;

typedef struct
{
  DNSHeader* pHeader;

  List questions;
  List answerRecords;
  List authoritiveRecords;
  List additionalRecords;

  uint8_t memoryPool[4096];
  uint32_t memoryUsed;
} DNSMessage;

int16_t DNSMessageParseRxPkt(NetPkt* pPkt, DNSMessage* pMsg);

void DNSMessageTxInit(DNSMessage* pMsg);
DNSQuestion* DNSMessageTxAddQuestion(DNSMessage* pMsg, const char* pName, uint16_t qType, uint16_t qClass);
DNSResourceRecord* DNSMessageTxCreateResourceRecord(DNSMessage* pMsg, const char* pName, uint16_t rrType, uint16_t rrClass, uint32_t ttl_s, const void* pData, uint16_t dataLength);
DNSResourceRecord* DNSMessageTxCreateResourceRecoryTypeA(DNSMessage* pMsg, const char* pName, uint32_t ttl_s, IPv4Address ip);
int16_t DNSMessageTxBuildPacket(DNSMessage* pMsg, NetPkt* pPkt);
