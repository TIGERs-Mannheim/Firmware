#include "dhcp.h"
#include "base_station.h"
#include "udp.h"
#include "inet.h"
#include "errors.h"
#include "constants.h"
#include "hal/sys_time.h"
#include "hal/rng.h"
#include "hal/cpuid.h"

#define EVENT_MASK_CHANGE_TO_DISABLED	EVENT_MASK(0)
#define EVENT_MASK_CHANGE_TO_ACTIVE		EVENT_MASK(1)
#define EVENT_MASK_CHANGE_TO_INFORM		EVENT_MASK(2)

#define BOOTP_FLAG_BROADCAST 0x8000

#define DHCP_OPT_TAG_SUBNET_MASK 1
#define DHCP_OPT_TAG_ROUTER 3
#define DHCP_OPT_TAG_DNS_SERVER 6
#define DHCP_OPT_TAG_HOSTNAME 12
#define DHCP_OPT_TAG_DOMAIN_NAME 15
#define DHCP_OPT_TAG_BROADCAST_ADDRESS 28
#define DHCP_OPT_TAG_REQUESTED_IP_ADDRESS 50
#define DHCP_OPT_TAG_IP_ADDRESS_LEASE_TIME 51
#define DHCP_OPT_TAG_DHCP_MESSAGE_TYPE 53
#define DHCP_OPT_TAG_SERVER_IDENTIFIER 54
#define DHCP_OPT_TAG_PARAMETER_REQUEST_LIST 55
#define DHCP_OPT_TAG_MAXIMUM_DHCP_MESSAGE_SIZE 57
#define DHCP_OPT_TAG_RENEWAL_TIME_VALUE 58
#define DHCP_OPT_TAG_REBINDING_TIME_VALUE 59
#define DHCP_OPT_TAG_CLIENT_IDENTIFIER 61
#define DHCP_OPT_TAG_END 255

typedef enum
{
	STATE_DISABLED,

	// Active DHCP states
	STATE_INIT,
	STATE_SELECTING,
	STATE_REQUESTING,
	STATE_REBINDING,
	STATE_RENEWING,
	STATE_BOUND,

	// Inform DHCP states
	STATE_INFORMING,
	STATE_INFORMED,
} DHCPState;

static const char* dhcpStateNames[] = { "Disabled", "Init", "Selecting", "Requesting", "Rebinding", "Renewing", "Bound", "Informing", "Informed" };

typedef enum
{
	DHCP_OP_BOOTREQUEST = 1,
	DHCP_OP_BOOTREPLY = 2,
} DHCPOp;

typedef enum
{
	DHCP_MESSAGE_TYPE_DISCOVER = 1,
	DHCP_MESSAGE_TYPE_OFFER = 2,
	DHCP_MESSAGE_TYPE_REQUEST = 3,
	DHCP_MESSAGE_TYPE_DECLINE = 4,
	DHCP_MESSAGE_TYPE_ACK = 5,
	DHCP_MESSAGE_TYPE_NAK = 6,
	DHCP_MESSAGE_TYPE_RELEASE = 7,
	DHCP_MESSAGE_TYPE_INFORM = 8,
} DHCPMessageType;

static void dhcpTask(void* pParam);
static NetVerdict receive(NetPkt* pPkt, void* pUser);

void DHCPInit(DHCP* pDhcp, NetIf* pIf)
{
	chEvtObjectInit(&pDhcp->eventSource);

	pDhcp->state = STATE_DISABLED;

	pDhcp->proto.pReceiveFunc = &receive;
	pDhcp->proto.pUser = pDhcp;
	pDhcp->pIf = pIf;

	pDhcp->xid = RNGGetRandomU32();

	pDhcp->pTask = chThdCreateStatic(pDhcp->waTask, sizeof(pDhcp->waTask), TASK_PRIO_DHCP, &dhcpTask, pDhcp);
}

void DHCPSetMode(DHCP* pDhcp, DHCPMode mode)
{
	switch(mode)
	{
		case DHCP_MODE_DISABLED:
			chEvtSignal(pDhcp->pTask, EVENT_MASK_CHANGE_TO_DISABLED);
			break;
		case DHCP_MODE_ACTIVE:
			chEvtSignal(pDhcp->pTask, EVENT_MASK_CHANGE_TO_ACTIVE);
			break;
		case DHCP_MODE_INFORM:
			chEvtSignal(pDhcp->pTask, EVENT_MASK_CHANGE_TO_INFORM);
			break;
	}
}

uint8_t DHCPIsResultValid(DHCP* pDhcp)
{
	return pDhcp->state == STATE_BOUND || pDhcp->state == STATE_RENEWING || pDhcp->state == STATE_REBINDING || pDhcp->state == STATE_INFORMED;
}

const char* DHCPGetCurrentStateName(DHCP* pDhcp)
{
	return dhcpStateNames[pDhcp->state];
}

void DHCPTriggerRenew(DHCP* pDhcp)
{
	if(pDhcp->state != STATE_BOUND)
		return;

	pDhcp->renewalTime = SysTimeMonotonic_s() - 1;
}

void DHCPTriggerRebind(DHCP* pDhcp)
{
	if(pDhcp->state != STATE_BOUND)
		return;

	pDhcp->state = STATE_RENEWING;
	pDhcp->rebindTime = SysTimeMonotonic_s() - 1;
}

void DHCPTriggerReinit(DHCP* pDhcp)
{
	if(pDhcp->state != STATE_BOUND)
		return;

	pDhcp->state = STATE_INIT;
}

static int16_t optAddMagicCookie(NetPkt* pPkt)
{
	uint8_t* pData = NetBufAdd(&pPkt->buf, 4);
	if(!pData)
		return ERROR_NOT_ENOUGH_MEMORY;

	pData[0] = 99;
	pData[1] = 130;
	pData[2] = 83;
	pData[3] = 99;

	return 0;
}

static int16_t optAddMessageType(NetPkt* pPkt, DHCPMessageType messageType)
{
	uint8_t* pData = NetBufAdd(&pPkt->buf, 3);
	if(!pData)
		return ERROR_NOT_ENOUGH_MEMORY;

	pData[0] = DHCP_OPT_TAG_DHCP_MESSAGE_TYPE;
	pData[1] = 1;
	pData[2] = messageType;

	return 0;
}

static int16_t optAddClientIdentifierCpuId(NetPkt* pPkt)
{
	uint8_t* pData = NetBufAdd(&pPkt->buf, 25);
	if(!pData)
		return ERROR_NOT_ENOUGH_MEMORY;

	// According to RFC4361, section 6.1 and RFC3315, section 9+10
	pData[0] = DHCP_OPT_TAG_CLIENT_IDENTIFIER;
	pData[1] = 23; // length
	pData[2] = 255; // client identifier type, RFC3315-style binding identifier
	*((uint32_t*)&pData[3]) = htonl(1); // IAID
	*((uint16_t*)&pData[7]) = htons(2); // DUID-EN type
	*((uint32_t*)&pData[9]) = htonl(7616); // IANA Private Enterprise Number of STMicroelectronics

	memcpy(&pData[13], (void*)CPUIDGet(), sizeof(uint32_t)*3);

	return 0;
}

static int16_t optAddMaximumDhcpMessageSize(NetPkt* pPkt, uint16_t maxMsgSize)
{
	uint8_t* pData = NetBufAdd(&pPkt->buf, 4);
	if(!pData)
		return ERROR_NOT_ENOUGH_MEMORY;

	pData[0] = DHCP_OPT_TAG_MAXIMUM_DHCP_MESSAGE_SIZE;
	pData[1] = 2;
	*((uint16_t*)&pData[2]) = htons(maxMsgSize);

	return 0;
}

static int16_t optAddHostName(NetPkt* pPkt, const char* pHostName)
{
	uint32_t nameLength = strlen(pHostName);

	uint8_t* pData = NetBufAdd(&pPkt->buf, nameLength+2);
	if(!pData)
		return ERROR_NOT_ENOUGH_MEMORY;

	pData[0] = DHCP_OPT_TAG_HOSTNAME;
	pData[1] = nameLength;
	memcpy(&pData[2], pHostName, nameLength);

	return 0;
}

static int16_t optAddParameterRequestList(NetPkt* pPkt)
{
	uint8_t* pData = NetBufAdd(&pPkt->buf, 7);
	if(!pData)
		return ERROR_NOT_ENOUGH_MEMORY;

	pData[0] = DHCP_OPT_TAG_PARAMETER_REQUEST_LIST;
	pData[1] = 5;
	pData[2] = DHCP_OPT_TAG_SUBNET_MASK;
	pData[3] = DHCP_OPT_TAG_ROUTER;
	pData[5] = DHCP_OPT_TAG_DNS_SERVER;
	pData[6] = DHCP_OPT_TAG_DOMAIN_NAME;

	return 0;
}

static int16_t optAddRequestedIpAddress(NetPkt* pPkt, IPv4Address* pIp)
{
	uint8_t* pData = NetBufAdd(&pPkt->buf, 6);
	if(!pData)
		return ERROR_NOT_ENOUGH_MEMORY;

	pData[0] = DHCP_OPT_TAG_REQUESTED_IP_ADDRESS;
	pData[1] = 4;
	memcpy(&pData[2], pIp, sizeof(IPv4Address));

	return 0;
}

static int16_t optAddServerIdentifier(NetPkt* pPkt, IPv4Address* pIp)
{
	uint8_t* pData = NetBufAdd(&pPkt->buf, 6);
	if(!pData)
		return ERROR_NOT_ENOUGH_MEMORY;

	pData[0] = DHCP_OPT_TAG_SERVER_IDENTIFIER;
	pData[1] = 4;
	memcpy(&pData[2], pIp, sizeof(IPv4Address));

	return 0;
}

static int16_t optAddEnd(NetPkt* pPkt)
{
	uint8_t* pData = NetBufAdd(&pPkt->buf, 1);
	if(!pData)
		return ERROR_NOT_ENOUGH_MEMORY;

	pData[0] = DHCP_OPT_TAG_END;

	return 0;
}

static int16_t send(DHCP* pDHCP, DHCPOperation* pOp)
{
	NetIf* pIf = pDHCP->pIf;

	NetPkt* pPkt = NetIfAllocPkt(pDHCP->pIf);
	if(!pPkt)
	{
		pDHCP->txFamesDropped++;
		return ERROR_NOT_ENOUGH_MEMORY;
	}

	DHCPMessage* pMsg = NetBufAdd(&pPkt->buf, sizeof(DHCPMessage));
	if(!pMsg)
	{
		pDHCP->txFamesDropped++;
		NetIfFreePkt(pIf, pPkt);
		return ERROR_NOT_ENOUGH_MEMORY;
	}

	// Prepare message, common fields
	pMsg->op = DHCP_OP_BOOTREQUEST;
	pMsg->htype = 1; // Ethernet
	pMsg->hlen = 6; // 6-Byte HW address
	pMsg->hops = 0;
	pMsg->xid = htonl(pOp->xid);
	memcpy(pMsg->chaddr, &pIf->mac, sizeof(MAC));

	// Prepare common options
	int16_t result = 0;
	result |= optAddMagicCookie(pPkt);
	result |= optAddMessageType(pPkt, pOp->type);
	result |= optAddClientIdentifierCpuId(pPkt);
	result |= optAddMaximumDhcpMessageSize(pPkt, 1500);
	result |= optAddHostName(pPkt, pDHCP->pIf->pHostname);
	result |= optAddParameterRequestList(pPkt);

	// Set individual fields and options
	switch(pOp->type)
	{
		case DHCP_MESSAGE_TYPE_DISCOVER:
		{
			pMsg->flags = htons(BOOTP_FLAG_BROADCAST);
			pPkt->ipv4.src.u32 = 0;
			pPkt->ipv4.dst.u32 = 0xffffffff;
		}
		break;
		case DHCP_MESSAGE_TYPE_REQUEST:
		{
			pMsg->ciaddr = pOp->request.ciaddr;

			if(pOp->request.requestedIp.u32)
				result |= optAddRequestedIpAddress(pPkt, &pOp->request.requestedIp);

			if(pOp->request.serverIdentifier.u32)
				result |= optAddServerIdentifier(pPkt, &pOp->request.serverIdentifier);

			pPkt->ipv4.dst = pOp->request.destinationIp;

			if(IPv4IsLimitedBroadcastAddress(pOp->request.destinationIp))
			{
				pMsg->flags = htons(BOOTP_FLAG_BROADCAST);
				pPkt->ipv4.src.u32 = 0;
			}
			else
			{
				pPkt->ipv4.src = pOp->request.ciaddr;
			}
		}
		break;
		case DHCP_MESSAGE_TYPE_INFORM:
		{
			pMsg->ciaddr = pOp->inform.clientIp;
			pPkt->ipv4.src = pOp->inform.clientIp;
			pPkt->ipv4.dst = pOp->inform.destinationIp;
		}
		break;
		default:
			pDHCP->txFamesDropped++;
			NetIfFreePkt(pIf, pPkt);
			return ERROR_INVALID_PARAMETER;
	}

	result |= optAddEnd(pPkt);

	if(result)
	{
		pDHCP->txFamesDropped++;
		NetIfFreePkt(pIf, pPkt);
		return ERROR_NOT_ENOUGH_MEMORY;
	}

	pPkt->udp.srcPort = UDP_PORT_DHCP_CLIENT;
	pPkt->udp.dstPort = UDP_PORT_DHCP_SERVER;

	pOp->lastTransmitTime_s = SysTimeMonotonic_s();
	pOp->numTransmissions++;

	pDHCP->txFramesGood++;
	return UDPSend(&pDHCP->pIf->udp, pPkt);
}

static void* optFind(NetBuf* pBuf, uint8_t tag, uint8_t* pOptLen)
{
	for(uint8_t* pByte = pBuf->pHead; pByte < pBuf->pTail;)
	{
		if(*pByte == DHCP_OPT_TAG_END)
			return 0;

		uint8_t optLen = pByte[1];

		if(*pByte == tag)
		{
			*pOptLen = optLen;
			return pByte + 2;
		}

		pByte += optLen + 2;
	}

	return 0;
}

static int16_t handleOffer(DHCP* pDHCP, DHCPMessage* pMsg, NetBuf* pOptBuf)
{
	uint8_t optLen = 0;
	IPv4Address* pServerIdentifier = optFind(pOptBuf, DHCP_OPT_TAG_SERVER_IDENTIFIER, &optLen);
	if(!pServerIdentifier || optLen != 4)
		return ERROR_INVALID_PARAMETER;

	pDHCP->op.type = DHCP_MESSAGE_TYPE_REQUEST;
	pDHCP->op.xid = pDHCP->xid;
	pDHCP->op.numTransmissions = 0;
	pDHCP->op.lastTransmitTime_s = 0;
	pDHCP->op.request.destinationIp = IPV4_ADDRESS_LIMITED_BROADCAST;
	pDHCP->op.request.serverIdentifier = *pServerIdentifier;
	pDHCP->op.request.ciaddr = IPV4_ADDRESS_ZEROS;
	pDHCP->op.request.requestedIp = pMsg->yiaddr;

	return send(pDHCP, &pDHCP->op);
}

static void handleAck(DHCP* pDhcp, DHCPMessage* pMsg, NetBuf* pOptBuf)
{
	if(pMsg->yiaddr.u32)
		pDhcp->result.clientIp = pMsg->yiaddr;

	// Note: Most IP options may contain a list, but we will only take the first entry for the result.
	uint8_t optLen = 0;
	IPv4Address* pServerIdentifier = optFind(pOptBuf, DHCP_OPT_TAG_SERVER_IDENTIFIER, &optLen);
	if(pServerIdentifier && optLen == 4)
		pDhcp->result.dhcpServerIp = *pServerIdentifier;
	else
		pDhcp->result.dhcpServerIp = IPV4_ADDRESS_ZEROS;

	IPv4Address* pRouterIp = optFind(pOptBuf, DHCP_OPT_TAG_ROUTER, &optLen);
	if(pRouterIp && optLen >= 4)
		pDhcp->result.routerIp = *pRouterIp;
	else
		pDhcp->result.routerIp = IPV4_ADDRESS_ZEROS;

	IPv4Address* pSubnetMask = optFind(pOptBuf, DHCP_OPT_TAG_SUBNET_MASK, &optLen);
	if(pSubnetMask && optLen >= 4)
		pDhcp->result.subnetMask = *pSubnetMask;
	else
		pDhcp->result.subnetMask = IPV4_ADDRESS_ZEROS;

	IPv4Address* pDnsServerIp = optFind(pOptBuf, DHCP_OPT_TAG_DNS_SERVER, &optLen);
	if(pDnsServerIp && optLen >= 4)
		pDhcp->result.dnsServerIp = *pDnsServerIp;
	else
		pDhcp->result.dnsServerIp = IPV4_ADDRESS_ZEROS;

	memset(pDhcp->result.domainName, 0, sizeof(pDhcp->result.domainName));

	char* pDomainName = optFind(pOptBuf, DHCP_OPT_TAG_DOMAIN_NAME, &optLen);
	if(pDomainName && optLen >= 1)
		memcpy(pDhcp->result.domainName, pDomainName, optLen);

	// Renew, rebind, and lease time values
	uint32_t* pLeaseTime_s = optFind(pOptBuf, DHCP_OPT_TAG_IP_ADDRESS_LEASE_TIME, &optLen);
	if(pLeaseTime_s && optLen == 4)
	{
		pDhcp->leaseTime = ntohl(*pLeaseTime_s);
		if(pDhcp->leaseTime == 0xffffffff)
			pDhcp->isInfiniteLease = 1;
	}
	else
	{
		pDhcp->isInfiniteLease = 0;
		pDhcp->leaseTime = 300;
	}

	uint32_t* pRenewalTime_s = optFind(pOptBuf, DHCP_OPT_TAG_RENEWAL_TIME_VALUE, &optLen);
	if(pRenewalTime_s && optLen == 4)
		pDhcp->renewalTime = ntohl(*pRenewalTime_s);
	else
		pDhcp->renewalTime = 0.5f * pDhcp->leaseTime;

	uint32_t* pRebindTime_s = optFind(pOptBuf, DHCP_OPT_TAG_REBINDING_TIME_VALUE, &optLen);
	if(pRebindTime_s && optLen == 4)
		pDhcp->rebindTime = ntohl(*pRebindTime_s);
	else
		pDhcp->rebindTime = 0.875f * pDhcp->leaseTime;

	uint32_t now = SysTimeMonotonic_s();
	pDhcp->rebindTime += now;
	pDhcp->renewalTime += now;
	pDhcp->leaseTime += now;

	chEvtBroadcastFlags(&pDhcp->eventSource, DHCP_EVENT_CONFIGURATION_RECEIVED);

	printf("DHCP configuration received\r\n");
}

static NetVerdict receive(NetPkt* pPkt, void* pUser)
{
	DHCP* pDHCP = pUser;
	NetBuf* pData = &pPkt->buf;

	if(pPkt->udp.dstPort != UDP_PORT_DHCP_CLIENT)
		return NET_VERDICT_CONTINUE;

	DHCPMessage* pMsg = NetBufPull(pData, sizeof(DHCPMessage));
	if(!pMsg)
	{
		pDHCP->rxFramesDropped++;
		return NET_VERDICT_DROP;
	}

	// Check session id
	if(pDHCP->xid != ntohl(pMsg->xid))
	{
		pDHCP->rxFramesDropped++;
		return NET_VERDICT_DROP;
	}

	uint8_t* pMagicCookie = NetBufPull(pData, 4);
	if(!pMagicCookie)
	{
		pDHCP->rxFramesDropped++;
		return NET_VERDICT_DROP;
	}

	// Check DHCP magic cookie
	if(pMagicCookie[0] != 99 || pMagicCookie[1] != 130 || pMagicCookie[2] != 83 || pMagicCookie[3] != 99)
	{
		pDHCP->rxFramesDropped++;
		return NET_VERDICT_DROP;
	}

	// Find message type option
	uint8_t optLen = 0;
	uint8_t* pMsgType = optFind(pData, DHCP_OPT_TAG_DHCP_MESSAGE_TYPE, &optLen);
	if(!pMsgType || optLen != 1)
	{
		pDHCP->rxFramesDropped++;
		return NET_VERDICT_DROP;
	}

	switch(pDHCP->state)
	{
		case STATE_SELECTING:
			if(*pMsgType == DHCP_MESSAGE_TYPE_OFFER)
			{
				int16_t result = handleOffer(pDHCP, pMsg, pData);
				if(result)
				{
					pDHCP->rxFramesDropped++;
					return NET_VERDICT_DROP;
				}

				pDHCP->state = STATE_REQUESTING;
				pDHCP->rxFramesGood++;
				return NET_VERDICT_OK;
			}
			break;
		case STATE_REQUESTING:
		case STATE_RENEWING:
		case STATE_REBINDING:
			if(*pMsgType == DHCP_MESSAGE_TYPE_ACK)
			{
				handleAck(pDHCP, pMsg, pData);
				pDHCP->state = STATE_BOUND;
				pDHCP->rxFramesGood++;
				return NET_VERDICT_OK;
			}

			if(*pMsgType == DHCP_MESSAGE_TYPE_NAK)
			{
				pDHCP->state = STATE_INIT;

				pDHCP->rxFramesGood++;
				return NET_VERDICT_OK;
			}
			break;
		case STATE_INFORMING:
			if(*pMsgType == DHCP_MESSAGE_TYPE_ACK)
			{
				handleAck(pDHCP, pMsg, pData);
				pDHCP->state = STATE_INFORMED;
				pDHCP->rxFramesGood++;
				return NET_VERDICT_OK;
			}
			break;
		case STATE_BOUND:
		case STATE_INIT:
		case STATE_DISABLED:
		default:
			pDHCP->rxFramesDropped++;
			return NET_VERDICT_DROP;
	}

	pDHCP->rxFramesDropped++;
	return NET_VERDICT_DROP;
}

static int16_t retransmitIfRequired(DHCP* pDhcp, DHCPOperation* pOp)
{
	uint32_t tNow_s = SysTimeMonotonic_s();
	uint32_t timeSinceLastTransmission_s = tNow_s - pOp->lastTransmitTime_s;
	uint32_t timeout_s = (1 << pOp->numTransmissions);

	if(timeout_s > 8)
		timeout_s = 8;

	if(timeSinceLastTransmission_s >= timeout_s)
	{
		int16_t result = send(pDhcp, pOp);
		if(result)
			return result;

		if(pOp->numTransmissions >= 3)
			return ERROR_WAIT_TIMEOUT;
	}

	return 0;
}

static void dhcpTask(void* pParam)
{
	DHCP* pDhcp = pParam;

	chRegSetThreadName("DHCP");

	while(1)
	{
		eventmask_t events = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(50));
		uint32_t tNow_s = SysTimeMonotonic_s();

		if((events & EVENT_MASK_CHANGE_TO_ACTIVE) && (pDhcp->state == STATE_DISABLED || pDhcp->state == STATE_INFORMING || pDhcp->state == STATE_INFORMED))
		{
			if(pDhcp->result.clientIp.u32)
			{
				pDhcp->xid++;

				pDhcp->op.type = DHCP_MESSAGE_TYPE_REQUEST;
				pDhcp->op.xid = pDhcp->xid;
				pDhcp->op.numTransmissions = 0;
				pDhcp->op.lastTransmitTime_s = 0;
				pDhcp->op.request.destinationIp = IPV4_ADDRESS_LIMITED_BROADCAST;
				pDhcp->op.request.ciaddr = IPV4_ADDRESS_ZEROS;
				pDhcp->op.request.serverIdentifier = IPV4_ADDRESS_ZEROS;
				pDhcp->op.request.requestedIp = pDhcp->result.clientIp;

				pDhcp->rebindTime = tNow_s + 60;
				pDhcp->state = STATE_RENEWING;
				send(pDhcp, &pDhcp->op);
			}
			else
			{
				pDhcp->state = STATE_INIT;
			}
		}

		if(events & EVENT_MASK_CHANGE_TO_DISABLED)
		{
			pDhcp->state = STATE_DISABLED;
		}

		if(events & EVENT_MASK_CHANGE_TO_INFORM)
		{
			if(pDhcp->pIf->staticIp.u32)
			{
				pDhcp->xid++;

				pDhcp->op.type = DHCP_MESSAGE_TYPE_INFORM;
				pDhcp->op.xid = pDhcp->xid;
				pDhcp->op.numTransmissions = 0;
				pDhcp->op.lastTransmitTime_s = 0;
				pDhcp->op.inform.destinationIp = IPV4_ADDRESS_LIMITED_BROADCAST;
				pDhcp->op.inform.clientIp = pDhcp->pIf->staticIp;

				pDhcp->state = STATE_INFORMING;
				send(pDhcp, &pDhcp->op);
			}
			else
			{
				pDhcp->state = STATE_INIT;
			}
		}

		switch(pDhcp->state)
		{
			case STATE_DISABLED:
				break;
			case STATE_INIT:
				pDhcp->xid++;

				pDhcp->op.type = DHCP_MESSAGE_TYPE_DISCOVER;
				pDhcp->op.xid = pDhcp->xid;
				pDhcp->op.numTransmissions = 0;
				pDhcp->op.lastTransmitTime_s = 0;

				pDhcp->state = STATE_SELECTING;
				send(pDhcp, &pDhcp->op);
				break;
			case STATE_SELECTING:
			case STATE_REQUESTING:
				// Reply handled in receive(), check for timeout and retransmit (DISCOVER or REQUEST)
				if(retransmitIfRequired(pDhcp, &pDhcp->op))
					pDhcp->state = STATE_INIT;
				break;
			case STATE_BOUND:
				if((int32_t)(pDhcp->renewalTime - tNow_s) < 0)
				{
					pDhcp->xid++;

					pDhcp->op.type = DHCP_MESSAGE_TYPE_REQUEST;
					pDhcp->op.xid = pDhcp->xid;
					pDhcp->op.numTransmissions = 0;
					pDhcp->op.lastTransmitTime_s = 0;
					pDhcp->op.request.destinationIp = pDhcp->result.dhcpServerIp;
					pDhcp->op.request.ciaddr = pDhcp->result.clientIp;
					pDhcp->op.request.serverIdentifier = IPV4_ADDRESS_ZEROS;
					pDhcp->op.request.requestedIp = IPV4_ADDRESS_ZEROS;

					pDhcp->state = STATE_RENEWING;
					send(pDhcp, &pDhcp->op);
				}
				break;
			case STATE_RENEWING:
				if((int32_t)(pDhcp->rebindTime - tNow_s) < 0)
				{
					pDhcp->xid++;

					pDhcp->op.type = DHCP_MESSAGE_TYPE_REQUEST;
					pDhcp->op.xid = pDhcp->xid;
					pDhcp->op.numTransmissions = 0;
					pDhcp->op.lastTransmitTime_s = 0;
					pDhcp->op.request.destinationIp = IPV4_ADDRESS_LIMITED_BROADCAST;
					pDhcp->op.request.ciaddr = pDhcp->result.clientIp;
					pDhcp->op.request.serverIdentifier = IPV4_ADDRESS_ZEROS;
					pDhcp->op.request.requestedIp = IPV4_ADDRESS_ZEROS;

					pDhcp->state = STATE_REBINDING;
					send(pDhcp, &pDhcp->op);
				}
				else
				{
					// ignore excessive retransmits, at some point we move to rebinding state
					retransmitIfRequired(pDhcp, &pDhcp->op);
				}
				break;
			case STATE_REBINDING:
				if(!pDhcp->isInfiniteLease && (int32_t)(pDhcp->leaseTime - tNow_s) < 0)
				{
					// Lease expired
					pDhcp->state = STATE_INIT;
				}
				else
				{
					// ignore excessive retransmits, at some point we move to init state
					retransmitIfRequired(pDhcp, &pDhcp->op);
				}
				break;
			case STATE_INFORMING:
				retransmitIfRequired(pDhcp, &pDhcp->op);
				break;
			case STATE_INFORMED:
				if((int32_t)(pDhcp->leaseTime - tNow_s) < 0)
				{
					chEvtAddEvents(EVENT_MASK_CHANGE_TO_INFORM);
				}
				break;
			default:
				break;
		}
	}
}
