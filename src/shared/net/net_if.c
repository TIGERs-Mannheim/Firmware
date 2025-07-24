#include "net_if.h"
#include "udp_socket.h"
#include "dhcp.h"
#include "base_station.h"
#include "hal/sys_time.h"

#define EVENT_MASK_ETH			EVENT_MASK(0)
#define EVENT_MASK_DHCP			EVENT_MASK(1)
#define EVENT_MASK_RECONFIGURE	EVENT_MASK(2)
#define EVENT_MASK_CONNECTED	EVENT_MASK(3)

static void netIfTask(void* pParam);
static int16_t sendEthOutput(NetPkt* pPkt, void* pUser);
static NetVerdict receiveUdpUserPacket(NetPkt* pPkt, void* pUser);
static void registerShellCommands(ShellCmdHandler* pHandler);

void NetIfInit(NetIf* pIf, NetIfData* pInit, tprio_t prio)
{
	chGuardedPoolObjectInit(&pIf->pktPool, sizeof(NetPkt));
	chGuardedPoolLoadArray(&pIf->pktPool, pIf->pktPoolData, NET_IF_PKT_POOL_SIZE);

	pIf->mac = pInit->mac;
	pIf->ipConfigType = pInit->ipConfigType;
	pIf->pHostname = pInit->pHostname;
	pIf->staticIp = pInit->staticIp;
	pIf->pEthDevice = pInit->pEth;
	pIf->state = NET_IF_STATE_DISCONNECTED;

	EthAllowMAC(pInit->pEth, 0, pInit->mac.u8, 1, 0);	// configure our MAC address

	EthernetInit(&pIf->ethernet, pIf, &sendEthOutput, pIf);
	ArpInit(&pIf->arp, pIf);
	IPv4Init(&pIf->ipv4, pIf);
	ICMPInit(&pIf->icmp, pIf);
	UDPInit(&pIf->udp, pIf);
	IGMPInit(&pIf->igmp, pIf, prio-1);
	DHCPInit(&pIf->dhcp, pIf);
	MDNSInit(&pIf->mdns, pIf);

	pIf->udpUser.pReceiveFunc = &receiveUdpUserPacket;
	pIf->udpUser.pUser = pIf;

	NetProtoLink(&pIf->arp.proto, &pIf->ethernet.proto);
	NetProtoLink(&pIf->ipv4.proto, &pIf->ethernet.proto);
	NetProtoLink(&pIf->icmp.proto, &pIf->ipv4.proto);
	NetProtoLink(&pIf->igmp.proto, &pIf->ipv4.proto);
	NetProtoLink(&pIf->udp.proto, &pIf->ipv4.proto);
	NetProtoLink(&pIf->dhcp.proto, &pIf->udp.proto);
	NetProtoLink(&pIf->mdns.proto, &pIf->udp.proto);
	NetProtoLink(&pIf->udpUser, &pIf->udp.proto);

	ShellCmdHandlerInit(&pIf->cmdHandler, pIf);
	registerShellCommands(&pIf->cmdHandler);

	pIf->pTask = chThdCreateStatic(pIf->waTask, sizeof(pIf->waTask), prio, &netIfTask, pIf);
}

void NetIfSetStaticIp(NetIf* pIf, IPv4Address ip)
{
	pIf->staticIp = ip;

	if(pIf->ipConfigType == NET_IF_IP_CONFIG_TYPE_STATIC)
		chEvtSignal(pIf->pTask, EVENT_MASK_RECONFIGURE);
}

void NetIfSetIpConfigType(NetIf* pIf, NetIfIpConfigType type)
{
	if(pIf->ipConfigType == type)
		return;

	pIf->ipConfigType = type;
	chEvtSignal(pIf->pTask, EVENT_MASK_RECONFIGURE);
}

NetPkt* NetIfAllocPkt(NetIf* pIf)
{
	NetPkt* pPkt = chGuardedPoolAllocTimeout(&pIf->pktPool, TIME_IMMEDIATE);
	if(!pPkt)
		return 0;

	memset(pPkt, 0, sizeof(NetPkt));

	NetBufInit(&pPkt->buf);
	pPkt->refCount = 1;

	pPkt->ethernet.src = pIf->mac;
	pPkt->ipv4.src = pIf->ip;
	pPkt->ipv4.ttl = 64;

	return pPkt;
}

void NetIfFreePkt(NetIf* pIf, NetPkt* pPkt)
{
	chDbgAssert(pPkt->refCount != 0, "Double free");

	pPkt->refCount--;

	if(pPkt->refCount == 0)
		chGuardedPoolFree(&pIf->pktPool, pPkt);
}

static void netIfTask(void* pParam)
{
	NetIf* pIf = pParam;

	chRegSetThreadName("NET_IFACE");

	event_listener_t ethListener;
	event_listener_t dhcpListener;

	chEvtRegisterMask(&pIf->pEthDevice->eventSource, &ethListener, EVENT_MASK_ETH);
	chEvtRegisterMask(&pIf->dhcp.eventSource, &dhcpListener, EVENT_MASK_DHCP);

	systime_t secondaryIgmpAnnouncementStart = 0;

	while(1)
	{
		eventmask_t events = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(10));

		if(events & EVENT_MASK_ETH)
		{
			eventflags_t flags = chEvtGetAndClearFlags(&ethListener);
			if(flags & ETH_EVENT_LINK_UP)
			{
				chEvtAddEvents(EVENT_MASK_RECONFIGURE);
			}

			if(flags & ETH_EVENT_LINK_DOWN)
			{
				DHCPSetMode(&pIf->dhcp, DHCP_MODE_DISABLED);
				pIf->state = NET_IF_STATE_DISCONNECTED;
			}

			if(flags & ETH_EVENT_DATA_RECEIVED)
			{
				// No action here, ethernet frames are processed below
			}
		}

		if(events & EVENT_MASK_RECONFIGURE)
		{
			if(pIf->ipConfigType == NET_IF_IP_CONFIG_TYPE_DHCP)
			{
				DHCPSetMode(&pIf->dhcp, DHCP_MODE_ACTIVE);
			}
			else
			{
				pIf->ip = pIf->staticIp;
				DHCPSetMode(&pIf->dhcp, DHCP_MODE_INFORM);
				chEvtAddEvents(EVENT_MASK_CONNECTED);
			}

			pIf->state = NET_IF_STATE_CONFIGURING;
		}

		if(events & EVENT_MASK_DHCP)
		{
			eventflags_t flags = chEvtGetAndClearFlags(&dhcpListener);
			if(flags & DHCP_EVENT_CONFIGURATION_RECEIVED)
			{
				if(pIf->state == NET_IF_STATE_CONFIGURING)
				{
					pIf->ip = pIf->dhcp.result.clientIp;
					chEvtAddEvents(EVENT_MASK_CONNECTED);
				}
			}
		}

		if(events & EVENT_MASK_CONNECTED)
		{
			ArpSend(&pIf->arp, ARP_OPERATION_REQUEST, pIf->mac, pIf->ip); // Send gratuitous ARP
			IGMPSendAllMemberships(&pIf->igmp);
			secondaryIgmpAnnouncementStart = chVTGetSystemTimeX();
			pIf->state = NET_IF_STATE_CONNECTED;
		}

		if(secondaryIgmpAnnouncementStart)
		{
			if(chVTTimeElapsedSinceX(secondaryIgmpAnnouncementStart) > TIME_S2I(35))
			{
				IGMPSendAllMemberships(&pIf->igmp);
				secondaryIgmpAnnouncementStart = 0;
			}
		}

		NetPkt* pPkt = NetIfAllocPkt(pIf);
		while(pPkt)
		{
			size_t pktSize = 0;
			int16_t result = EthGetEthernetFrame(pIf->pEthDevice, NetBufGetTail(&pPkt->buf), NetBufGetSizeFree(&pPkt->buf), &pktSize);
			if(result == 0)
			{
				NetBufAdd(&pPkt->buf, pktSize);
				pIf->statistics.rxFramesTotal++;

				NetVerdict verdict = NetProtoHandleRxPkt(&pIf->ethernet.proto, pPkt);
				if(verdict == NET_VERDICT_DROP)
					pIf->statistics.rxFramesDropped++;
			}

			NetIfFreePkt(pIf, pPkt);

			if(result)
				break;

			pPkt = NetIfAllocPkt(pIf);
		}
	}
}

static int16_t sendEthOutput(NetPkt* pPkt, void* pUser)
{
	NetIf* pIf = pUser;

	int16_t result = EthSendEthernetFrame(pIf->pEthDevice, NetBufGetHead(&pPkt->buf), NetBufGetSizeUsed(&pPkt->buf));
	NetIfFreePkt(pIf, pPkt);

	return result;
}

static NetVerdict receiveUdpUserPacket(NetPkt* pPkt, void* pUser)
{
	NetIf* pIf = pUser;

	if(pIf->state != NET_IF_STATE_CONNECTED)
		return NET_VERDICT_CONTINUE;

	UDPSocket* pSocket = ListFront(pIf->udpSockets);
	uint8_t isPktUsed = 0;

	while(pSocket)
	{
		uint8_t isLimitedBroadcast = IPv4IsLimitedBroadcastAddress(pPkt->ipv4.dst);
		uint8_t isUnicastToThisInterface = pPkt->ipv4.dst.u32 == pIf->ip.u32;
		uint8_t isExpectedMulticastPacket = IPv4IsMulticastAddress(pSocket->multicastIp) && pSocket->multicastIp.u32 == pPkt->ipv4.dst.u32;

		if(pSocket->port == pPkt->udp.dstPort && (isLimitedBroadcast || isUnicastToThisInterface || isExpectedMulticastPacket))
		{
			if(isPktUsed)
			{
				NetPkt* pClone = NetIfAllocPkt(pIf);
				if(!pClone)
				{
					pSocket->rxPktLost++;
					continue;
				}

				memcpy(pClone, pPkt, sizeof(NetPkt));
				pPkt = pClone;
			}

			pPkt->refCount++; // Ensure packet is not freed by main network task
			ListPushBack(&pSocket->rxPktQueue, pPkt);
			chEvtBroadcast(&pSocket->eventSource);

			isPktUsed = 1;
		}

		pSocket = pSocket->pNext;
	}

	if(isPktUsed)
		return NET_VERDICT_OK;

	return NET_VERDICT_CONTINUE;
}

SHELL_CMD(config, "Show network configuration");

SHELL_CMD_IMPL(config)
{
	(void)argc; (void)argv;
	NetIf* pIf = pUser;

	static const char* netIfStateNames[] = { "Disconnected", "Configuring", "Connected" };

	printf("State: %s\r\n", netIfStateNames[pIf->state]);
	printf("MAC: %02hX-%02hX-%02hX-%02hX-%02hX-%02hX\r\n", pIf->mac.u8[0], pIf->mac.u8[1], pIf->mac.u8[2], pIf->mac.u8[3], pIf->mac.u8[4], pIf->mac.u8[5]);
	printf("Current IP: %hu.%hu.%hu.%hu\r\n", (uint16_t)pIf->ip.u8[0], (uint16_t)pIf->ip.u8[1], (uint16_t)pIf->ip.u8[2], (uint16_t)pIf->ip.u8[3]);
	printf("Static IP: %hu.%hu.%hu.%hu\r\n", (uint16_t)pIf->staticIp.u8[0], (uint16_t)pIf->staticIp.u8[1], (uint16_t)pIf->staticIp.u8[2], (uint16_t)pIf->staticIp.u8[3]);

	if(baseStation.config.base.isDHCPEnabled)
		printf("DHCP: ON\r\n");
	else
		printf("DHCP: OFF\r\n");
}

SHELL_CMD(phydbg, "Print PHY debug information");

SHELL_CMD_IMPL(phydbg)
{
	(void)argc; (void)argv;
	NetIf* pIf = pUser;

	EthPrintDebugOutput(pIf->pEthDevice);
}

SHELL_CMD(stats, "Print network interface stats");

SHELL_CMD_IMPL(stats)
{
	(void)argc; (void)argv;
	NetIf* pIf = pUser;

	chSysLock();
	size_t freePktsInPool = chGuardedPoolGetCounterI(&pIf->pktPool);
	chSysUnlock();

	printf("Packet Pool free: %u of %u\r\n", freePktsInPool, (uint32_t)NET_IF_PKT_POOL_SIZE);

	printf("--- Interface ---\r\n");
	printf("RX frames: %u total, %u dropped\r\n", pIf->statistics.rxFramesTotal, pIf->statistics.rxFramesDropped);

	printf("--- Ethernet ---\r\n");
	printf("RX frames: %u good, %u dropped\r\n", pIf->ethernet.rxFramesGood, pIf->ethernet.rxFramesDropped);
	printf("TX frames: %u good, %u dropped\r\n", pIf->ethernet.txFramesGood, pIf->ethernet.txFamesDropped);

	printf("--- ARP ---\r\n");
	printf("RX frames: %u good, %u dropped\r\n", pIf->arp.rxFramesGood, pIf->arp.rxFramesDropped);
	printf("TX frames: %u good, %u dropped\r\n", pIf->arp.txFramesGood, pIf->arp.txFamesDropped);

	printf("--- IPv4 ---\r\n");
	printf("RX frames: %u good, %u dropped\r\n", pIf->ipv4.rxFramesGood, pIf->ipv4.rxFramesDropped);
	printf("TX frames: %u good, %u dropped\r\n", pIf->ipv4.txFramesGood, pIf->ipv4.txFamesDropped);

	printf("--- ICMP ---\r\n");
	printf("RX frames: %u good, %u dropped\r\n", pIf->icmp.rxFramesGood, pIf->icmp.rxFramesDropped);
	printf("TX frames: %u good, %u dropped\r\n", pIf->icmp.txFramesGood, pIf->icmp.txFamesDropped);

	printf("--- UDP ---\r\n");
	printf("RX frames: %u good, %u dropped\r\n", pIf->udp.rxFramesGood, pIf->udp.rxFramesDropped);
	printf("TX frames: %u good, %u dropped\r\n", pIf->udp.txFramesGood, pIf->udp.txFamesDropped);

	printf("--- IGMP ---\r\n");
	printf("RX frames: %u good, %u dropped\r\n", pIf->igmp.rxFramesGood, pIf->igmp.rxFramesDropped);
	printf("TX frames: %u good, %u dropped\r\n", pIf->igmp.txFramesGood, pIf->igmp.txFamesDropped);

	printf("--- DHCP ---\r\n");
	printf("RX frames: %u good, %u dropped\r\n", pIf->dhcp.rxFramesGood, pIf->dhcp.rxFramesDropped);
	printf("TX frames: %u good, %u dropped\r\n", pIf->dhcp.txFramesGood, pIf->dhcp.txFamesDropped);

	printf("--- mDNS ---\r\n");
	printf("RX frames: %u good, %u dropped\r\n", pIf->mdns.rxFramesGood, pIf->mdns.rxFramesDropped);
	printf("TX frames: %u good, %u dropped\r\n", pIf->mdns.txFramesGood, pIf->mdns.txFamesDropped);
}

SHELL_CMD(arp, "Show ARP cache content");

SHELL_CMD_IMPL(arp)
{
	(void)argc; (void)argv;
	NetIf* pIf = pUser;

	for(size_t i = 0; i < ARP_CACHE_SIZE; i++)
	{
		ArpCacheEntry* pEntry = &pIf->arp.cache[i];

		if(pEntry->tLastUse == 0)
			continue;

		printf("%hu.%hu.%hu.%hu\t\t%02hX-%02hX-%02hX-%02hX-%02hX-%02hX\t%8ums\r\n", (uint16_t)pEntry->ip.u8[0], (uint16_t)pEntry->ip.u8[1], (uint16_t)pEntry->ip.u8[2], (uint16_t)pEntry->ip.u8[3],
				pEntry->mac.u8[0], pEntry->mac.u8[1], pEntry->mac.u8[2], pEntry->mac.u8[3], pEntry->mac.u8[4], pEntry->mac.u8[5],
				TIME_I2MS(chVTTimeElapsedSinceX(pEntry->tLastUse)));
	}
}

SHELL_CMD(ping, "Ping a network client",
	SHELL_ARG(ip, "IPv4 address of target client")
);

SHELL_CMD_IMPL(ping)
{
	(void)argc;
	NetIf* pIf = pUser;

	IPv4Address ip;
	if(IPv4ParseAddress(argv[1], &ip))
	{
		fprintf(stderr, "Unable to parse IP address\r\n");
		return;
	}

	printf("Executing ping to %hu.%hu.%hu.%hu\r\n", (uint16_t)ip.u8[0], (uint16_t)ip.u8[1], (uint16_t)ip.u8[2], (uint16_t)ip.u8[3]);

	ICMPSendEchoRequest(&pIf->icmp, ip);
}

SHELL_CMD(join, "Join a multicast group",
	SHELL_ARG(ip, "IPv4 address of multicast group")
);

SHELL_CMD_IMPL(join)
{
	(void)argc;
	NetIf* pIf = pUser;

	IPv4Address ip;
	if(IPv4ParseAddress(argv[1], &ip) || !IPv4IsMulticastAddress(ip))
	{
		fprintf(stderr, "Unable to parse IP address or not a multicast address\r\n");
		return;
	}

	printf("Joining group %hu.%hu.%hu.%hu\r\n", (uint16_t)ip.u8[0], (uint16_t)ip.u8[1], (uint16_t)ip.u8[2], (uint16_t)ip.u8[3]);

	IGMPJoinGroup(&pIf->igmp, ip);
}

SHELL_CMD(join_all, "Send membership report of all groups");

SHELL_CMD_IMPL(join_all)
{
	(void)argc; (void)argv;
	NetIf* pIf = pUser;

	printf("Sending membership report\r\n");

	IGMPSendAllMemberships(&pIf->igmp);
}

SHELL_CMD(leave, "Leave a multicast group",
	SHELL_ARG(ip, "IPv4 address of multicast group")
);

SHELL_CMD_IMPL(leave)
{
	(void)argc;
	NetIf* pIf = pUser;

	IPv4Address ip;
	if(IPv4ParseAddress(argv[1], &ip) || !IPv4IsMulticastAddress(ip))
	{
		fprintf(stderr, "Unable to parse IP address or not a multicast address\r\n");
		return;
	}

	printf("Leaving group %hu.%hu.%hu.%hu\r\n", (uint16_t)ip.u8[0], (uint16_t)ip.u8[1], (uint16_t)ip.u8[2], (uint16_t)ip.u8[3]);

	IGMPLeaveGroup(&pIf->igmp, ip);
}

SHELL_CMD(igmp, "Show currently active multicast groups");

SHELL_CMD_IMPL(igmp)
{
	(void)argc; (void)argv;
	NetIf* pIf = pUser;

	for(size_t i = 0; i < IGMP_MAX_JOINED_GROUPS; i++)
	{
		IGMPMulticastGroup* pGroup = &pIf->igmp.groups[i];
		if(pGroup->numReferences == 0)
			continue;

		printf("%s, %u refs\r\n", IPv4FormatAddress(pGroup->ip), pGroup->numReferences);
	}
}

SHELL_CMD(dhcp, "Show DHCP configuration and status");

SHELL_CMD_IMPL(dhcp)
{
	(void)argc; (void)argv; (void)argv;
	NetIf* pIf = pUser;
	DHCP* pDhcp = &pIf->dhcp;

	printf("Hostname:   %s\r\n", pIf->pHostname);
	printf("DHCP state: %s\r\n", DHCPGetCurrentStateName(pDhcp));

	if(DHCPIsResultValid(pDhcp))
	{
		printf("IP:      %s\r\n", IPv4FormatAddress(pDhcp->result.clientIp));
		printf("Netmask: %s\r\n", IPv4FormatAddress(pDhcp->result.subnetMask));
		printf("Router:  %s\r\n", IPv4FormatAddress(pDhcp->result.routerIp));
		printf("DHCP:    %s\r\n", IPv4FormatAddress(pDhcp->result.dhcpServerIp));
		printf("DNS:     %s\r\n", IPv4FormatAddress(pDhcp->result.dnsServerIp));
		printf("Domain:  %s\r\n", pDhcp->result.domainName);
	}
	else
	{
		printf("No DHCP information available.");
	}

	uint32_t now = SysTimeMonotonic_s();
	int32_t timeToRebind_s = pDhcp->rebindTime - now;
	int32_t timeToRenew_s = pDhcp->renewalTime - now;
	int32_t timeToExpire_s = pDhcp->leaseTime - now;

	printf("Renewing in:      %4dh %dmin\r\n", timeToRenew_s / 3600, (timeToRenew_s % 3600)/60);
	printf("Rebinding in:     %4dh %dmin\r\n", timeToRebind_s / 3600, (timeToRebind_s % 3600)/60);

	if(pDhcp->isInfiniteLease)
		printf("Lease expires in: never\r\n");
	else
		printf("Lease expires in: %4dh %dmin\r\n", timeToExpire_s / 3600, (timeToExpire_s % 3600)/60);
}

SHELL_CMD(dhcp_renew, "Trigger DHCP renewal");

SHELL_CMD_IMPL(dhcp_renew)
{
	(void)argc; (void)argv; (void)argv;
	NetIf* pIf = pUser;

	DHCPTriggerRenew(&pIf->dhcp);

	printf("DHCP renew triggered.\r\n");
}

SHELL_CMD(dhcp_rebind, "Trigger DHCP rebind");

SHELL_CMD_IMPL(dhcp_rebind)
{
	(void)argc; (void)argv; (void)argv;
	NetIf* pIf = pUser;

	DHCPTriggerRebind(&pIf->dhcp);

	printf("DHCP rebind triggered.\r\n");
}

SHELL_CMD(dhcp_reinit, "Restart DHCP process");

SHELL_CMD_IMPL(dhcp_reinit)
{
	(void)argc; (void)argv; (void)argv;
	NetIf* pIf = pUser;

	DHCPTriggerReinit(&pIf->dhcp);

	printf("DHCP reinit triggered.\r\n");
}

SHELL_CMD(ethstats, "Print ETH device stats");

SHELL_CMD_IMPL(ethstats)
{
	(void)argc; (void)argv;
	NetIf* pIf = pUser;

	EthStats* pStats = EthGetStats(pIf->pEthDevice);

	printf("RX frames: %u processed / %u total, bytes: %u\r\n", pStats->rxFramesProcessed, pStats->rxFrames, pStats->rxBytesProcessed);
	printf("TX frames: %u processed / %u total, bytes: %u\r\n", pStats->txFramesProcessed, pStats->txFrames, pStats->txBytesProcessed);
	printf("DMA losses: %u app, %u ctrl\r\n", pStats->rxMissedApp, pStats->rxMissedCtrl);
}

static void registerShellCommands(ShellCmdHandler* pHandler)
{
	ShellCmdAdd(pHandler, config_command);
	ShellCmdAdd(pHandler, stats_command);
	ShellCmdAdd(pHandler, arp_command);
	ShellCmdAdd(pHandler, ping_command);
	ShellCmdAdd(pHandler, join_all_command);
	ShellCmdAdd(pHandler, join_command);
	ShellCmdAdd(pHandler, leave_command);
	ShellCmdAdd(pHandler, igmp_command);
	ShellCmdAdd(pHandler, dhcp_command);
	ShellCmdAdd(pHandler, dhcp_renew_command);
	ShellCmdAdd(pHandler, dhcp_rebind_command);
	ShellCmdAdd(pHandler, dhcp_reinit_command);
	ShellCmdAdd(pHandler, ethstats_command);
	ShellCmdAdd(pHandler, phydbg_command);
}
