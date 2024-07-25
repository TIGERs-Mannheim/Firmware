#include "net_if.h"
#include "udp_socket.h"

#define EVENT_MASK_ETH	EVENT_MASK(0)

static void netIfTask(void* pParam);
static int16_t sendEthOutput(NetPkt* pPkt, void* pUser);
static NetVerdict receiveUdpUserPacket(NetPkt* pPkt, void* pUser);
static void registerShellCommands(ShellCmdHandler* pHandler);

void NetIfInit(NetIf* pIf, NetIfData* pInit, tprio_t prio)
{
	chGuardedPoolObjectInit(&pIf->pktPool, sizeof(NetPkt));
	chGuardedPoolLoadArray(&pIf->pktPool, pIf->pktPoolData, NET_IF_PKT_POOL_SIZE);

	pIf->data = *pInit;

	EthAllowMAC(pInit->pEth, 0, pInit->mac.u8, 1, 0);	// configure our MAC address

	EthernetInit(&pIf->ethernet, pIf, &sendEthOutput, pIf);
	ArpInit(&pIf->arp, pIf);
	IPv4Init(&pIf->ipv4, pIf);
	ICMPInit(&pIf->icmp, pIf);
	UDPInit(&pIf->udp, pIf);
	IGMPInit(&pIf->igmp, pIf, prio-1);

	pIf->udpUser.pReceiveFunc = &receiveUdpUserPacket;
	pIf->udpUser.pUser = pIf;

	NetProtoLink(&pIf->arp.proto, &pIf->ethernet.proto);
	NetProtoLink(&pIf->ipv4.proto, &pIf->ethernet.proto);
	NetProtoLink(&pIf->icmp.proto, &pIf->ipv4.proto);
	NetProtoLink(&pIf->igmp.proto, &pIf->ipv4.proto);
	NetProtoLink(&pIf->udp.proto, &pIf->ipv4.proto);
	NetProtoLink(&pIf->udpUser, &pIf->udp.proto);

	ShellCmdHandlerInit(&pIf->cmdHandler, pIf);
	registerShellCommands(&pIf->cmdHandler);

	pIf->pTask = chThdCreateStatic(pIf->waTask, sizeof(pIf->waTask), prio, &netIfTask, pIf);
}

void NetIfSetIp(NetIf* pIf, IPv4Address ip)
{
	pIf->data.ip = ip;
	ArpSend(&pIf->arp, ARP_OPERATION_REQUEST, pIf->data.mac, pIf->data.ip); // Send gratuitous ARP
}

NetPkt* NetIfAllocPkt(NetIf* pIf)
{
	NetPkt* pPkt = (NetPkt*)chGuardedPoolAllocTimeout(&pIf->pktPool, TIME_IMMEDIATE);
	if(!pPkt)
		return 0;

	memset(pPkt, 0, sizeof(NetPkt));

	NetBufInit(&pPkt->buf);
	pPkt->refCount = 1;

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
	NetIf* pIf = (NetIf*)pParam;

	chRegSetThreadName("NET_IFACE");

	event_listener_t ethListener;

	chEvtRegisterMask(&pIf->data.pEth->eventSource, &ethListener, EVENT_MASK_ETH);

	systime_t secondaryIgmpAnnouncementStart = 0;

	while(1)
	{
		eventmask_t events = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(10));

		if(events & EVENT_MASK_ETH)
		{
			eventflags_t flags = chEvtGetAndClearFlags(&ethListener);
			if(flags & ETH_EVENT_LINK_UP)
			{
				ArpSend(&pIf->arp, ARP_OPERATION_REQUEST, pIf->data.mac, pIf->data.ip); // Send gratuitous ARP
				IGMPSendAllMemberships(&pIf->igmp);
				secondaryIgmpAnnouncementStart = chVTGetSystemTimeX();
			}

			if(flags & ETH_EVENT_LINK_DOWN)
			{
			}

			if(flags & ETH_EVENT_DATA_RECEIVED)
			{
			}
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
			int16_t result = EthGetEthernetFrame(pIf->data.pEth, NetBufGetTail(&pPkt->buf), NetBufGetSizeFree(&pPkt->buf), &pktSize);
			if(result == 0)
			{
				NetBufAdd(&pPkt->buf, pktSize);
				pIf->stats.rxFramesTotal++;

				NetVerdict verdict = NetProtoHandleRxPkt(&pIf->ethernet.proto, pPkt);
				if(verdict == NET_VERDICT_DROP)
					pIf->stats.rxFramesDropped++;
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
	NetIf* pIf = (NetIf*)pUser;

	int16_t result = EthSendEthernetFrame(pIf->data.pEth, NetBufGetHead(&pPkt->buf), NetBufGetSizeUsed(&pPkt->buf));
	NetIfFreePkt(pIf, pPkt);

	return result;
}

static NetVerdict receiveUdpUserPacket(NetPkt* pPkt, void* pUser)
{
	NetIf* pIf = (NetIf*)pUser;

	UDPSocket* pSocket = ListFront(pIf->udpSockets);
	uint8_t isPktUsed = 0;

	while(pSocket)
	{
		if(pSocket->port == pPkt->udp.dstPort && (pSocket->bindIp.u32 == 0 || pSocket->bindIp.u32 == pPkt->ipv4.dst.u32))
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
	NetIf* pIf = (NetIf*)pUser;

	printf("IP: %hu.%hu.%hu.%hu\r\n", (uint16_t)pIf->data.ip.u8[0], (uint16_t)pIf->data.ip.u8[1], (uint16_t)pIf->data.ip.u8[2], (uint16_t)pIf->data.ip.u8[3]);
	printf("MAC: %02hX-%02hX-%02hX-%02hX-%02hX-%02hX\r\n", pIf->data.mac.u8[0], pIf->data.mac.u8[1], pIf->data.mac.u8[2], pIf->data.mac.u8[3], pIf->data.mac.u8[4], pIf->data.mac.u8[5]);
}

SHELL_CMD(phydbg, "Print PHY debug information");

SHELL_CMD_IMPL(phydbg)
{
	(void)argc; (void)argv;
	NetIf* pIf = (NetIf*)pUser;

	EthPrintDebugOutput(pIf->data.pEth);
}

SHELL_CMD(stats, "Print network interface stats");

SHELL_CMD_IMPL(stats)
{
	(void)argc; (void)argv;
	NetIf* pIf = (NetIf*)pUser;

	chSysLock();
	size_t freePktsInPool = chGuardedPoolGetCounterI(&pIf->pktPool);
	chSysUnlock();

	printf("Packet Pool free: %u of %u\r\n", freePktsInPool, (uint32_t)NET_IF_PKT_POOL_SIZE);

	printf("--- Interface ---\r\n");
	printf("RX frames: %u total, %u dropped\r\n", pIf->stats.rxFramesTotal, pIf->stats.rxFramesDropped);

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
}

SHELL_CMD(arp, "Show ARP cache content");

SHELL_CMD_IMPL(arp)
{
	(void)argc; (void)argv;
	NetIf* pIf = (NetIf*)pUser;

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
	NetIf* pIf = (NetIf*)pUser;

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
	NetIf* pIf = (NetIf*)pUser;

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
	NetIf* pIf = (NetIf*)pUser;

	printf("Sending membership report\r\n");

	IGMPSendAllMemberships(&pIf->igmp);
}

SHELL_CMD(leave, "Leave a multicast group",
	SHELL_ARG(ip, "IPv4 address of multicast group")
);

SHELL_CMD_IMPL(leave)
{
	(void)argc;
	NetIf* pIf = (NetIf*)pUser;

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
	NetIf* pIf = (NetIf*)pUser;

	for(size_t i = 0; i < IGMP_MAX_JOINED_GROUPS; i++)
	{
		IGMPMulticastGroup* pGroup = &pIf->igmp.groups[i];
		if(pGroup->numReferences == 0)
			continue;

		printf("%s, %u refs\r\n", IPv4FormatAddress(pGroup->ip), pGroup->numReferences);
	}
}

SHELL_CMD(ethstats, "Print ETH stats");

SHELL_CMD_IMPL(ethstats)
{
	(void)argc; (void)argv;
	NetIf* pIf = (NetIf*)pUser;

	EthStats* pStats = EthGetStats(pIf->data.pEth);

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
	ShellCmdAdd(pHandler, ethstats_command);
	ShellCmdAdd(pHandler, phydbg_command);
}
