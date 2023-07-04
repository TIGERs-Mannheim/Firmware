/*
 * cli.c
 *
 *  Created on: 23.10.2017
 *      Author: AndreR
 */

#include "cli.h"

#include "hal/led.h"
#include "hal/usart1.h"
#include "hal/rtc.h"

#include "util/console.h"
#include "util/flash_fs.h"
#include "util/flash.h"
#include "util/nrf24_ex.h"
#include "util/init_hal.h"
#include "util/boot.h"
#include "util/sys_time.h"
#include "util/log.h"
#include "util/terminal.h"

#include "version.h"
#include "cpu_load.h"
#include "commands.h"
#include "network.h"
#include "wifi.h"
#include "hub.h"
#include "vision.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

static void processCommand();

// IRQ Level: Task
static void consoleCallback()
{
	// called after ConsolePrint
}

static void cmdCallback(char* pBuf, uint16_t length)
{
	// process onboard
	uint8_t *pData;
	int16_t result = FifoLinReserve(&console.rxFifo, length+1, &pData);
	if(result)
		return;

	memcpy(pData, pBuf, length);
	pData[length] = 0;

	FifoLinCommit(&console.rxFifo, length+1);
}

void CLIInit()
{
	static uint8_t rxData[CONSOLE_RX_SIZE];
	static uint8_t txData[CONSOLE_TX_SIZE];

	ConsoleData conData;
	conData.pDataTx = txData;
	conData.txSize = CONSOLE_TX_SIZE;
	conData.pDataRx = rxData;
	conData.rxSize = CONSOLE_RX_SIZE;
	conData.printSize = CONSOLE_PRINT_SIZE;
	conData.txCallback = &consoleCallback;
	conData.pUserData = 0;

	ConsoleInit(conData);

	terminal.cmdCb = &cmdCallback;
}

void CLITask(void* params)
{
	(void)params;

	chRegSetThreadName("CLI");

	while(1)
	{
		// check data from main console to terminal
		while(console.txFifo.numPackets > 0)
		{
			uint8_t* pData;
			uint32_t len32;
			if(FifoLinGet(&console.txFifo, &pData, &len32))
				break;

			TerminalWrite(pData, len32);

			FifoLinDelete(&console.txFifo);
		}

		if(ConsoleIsCmdReady())
		{
			processCommand();

			ConsoleCmdProcessed();
		}

		chThdSleepMilliseconds(5);
	}
}

static void miscCommands()
{
	uint16_t u16;

	if(ConsoleCmpCmd("reset"))
	{
		BootReset();
	}

	if(ConsoleCmpCmd("bootloader"))
	{
		ConsolePrint("Selecting bootloader\r\n");

		BootSetBootloaderSelected(1);
	}

	if(ConsoleCmpCmd("test"))
	{
		ConsolePrint("Number: %d\r\n", 42);
	}

	float f = 21.0f;
	if(ConsoleScanCmd("float %f", &f) == 1)
	{
		ConsolePrint("Float: %f\r\n", f);
	}

	if(ConsoleCmpCmd("float"))
	{
		ConsolePrint("Float: %f\r\n", 42.0f);
	}

	if(ConsoleCmpCmd("cpu"))
	{
		CPULoadPrintUsage();
	}

	if(ConsoleCmpCmd("tasks"))
	{
		CPULoadPrintTasks();
	}

	if(ConsoleCmpCmd("flash list"))
	{
		FlashFSListFiles();
	}

	if(ConsoleCmpCmd("flash erase"))
	{
		ConsolePrint("Erasing flash\r\n");

		FlashErase(flashFS.flashAddr, flashFS.flashAddr+flashFS.flashSize);
	}

	if(ConsoleCmpCmd("flash compact"))
	{
		FlashFSCompact();

		ConsolePrint("Flash compacted\r\n");
	}

	if(ConsoleScanCmd("log %hu", &u16) == 1)
	{
		LogPrintAllRecent(u16);
	}

	if(ConsoleCmpCmd("time"))
	{
		ConsolePrint("SysTime: %uus\r\n", SysTimeUSec());

		struct tm* nowtm;
		time_t now = RTCGetUnixTimestamp();
		nowtm = localtime(&now);

		char tmbuf[64];
		strftime(tmbuf, sizeof tmbuf, "%Y-%m-%d %H:%M:%S", nowtm);
		ConsolePrint("%s\r\n", tmbuf);

		uint32_t dr = RTC->DR;
		uint32_t tr = RTC->TR;
		ConsolePrint("RTC Date: %08X\r\n", dr);
		ConsolePrint("RTC Time: %08X\r\n", tr);
		ConsolePrint("RTC Calib: 0x%08X\r\n", RTC->CALR);
	}

	uint32_t u32;
	if(ConsoleScanCmd("time %X", &u32) == 1)
	{
		ConsolePrint("Setting time on RTC: %06X\r\n", u32);

		RTCSetTimeBCD(u32);
	}

	if(ConsoleScanCmd("date %X", &u32) == 1)
	{
		ConsolePrint("Setting date on RTC: %06X\r\n", u32);

		RTCSetDateBCD(u32);
	}

	float f32;
	if(ConsoleScanCmd("rtc calib %f", &f32) == 1)
	{
		ConsolePrint("Adjusting RTC calibration by %.2fppm\r\n", f32);

		RTCSetCalib(f32);
	}

	if(ConsoleCmpCmd("pyhdbg"))
	{
		uint16_t bmcr = ETHPhyRead(0x00);
		uint16_t bmsr = ETHPhyRead(0x01);
		uint16_t physts = ETHPhyRead(0x10);
		uint16_t micr = ETHPhyRead(0x11);
		uint16_t misr = ETHPhyRead(0x12);

		ConsolePrint("BMCR:   0x%04X\r\n", bmcr);
		ConsolePrint("BMSR:   0x%04X\r\n", bmsr);
		ConsolePrint("PHYSTS: 0x%04X\r\n", physts);
		ConsolePrint("MICR:   0x%04X\r\n", micr);
		ConsolePrint("MISR:   0x%04X\r\n", misr);
		ConsolePrint("INT:    %u\r\n", (GPIOD->IDR & GPIO_PIN_0));
	}

	if(ConsoleCmpCmd("ethdbg"))
	{
		EthDebug();
	}

	if(ConsoleCmpCmd("viewports"))
	{
		for(uint16_t i = 0; i < VISION_MAX_CAMS; i++)
		{
			ConsolePrint("%hu: %5hd %5hd %5hd %5hd\r\n", i, vision.cams[i].minX,
				vision.cams[i].maxX, vision.cams[i].minY, vision.cams[i].maxY);
		}
	}
}

static void networkCommands()
{
	uint16_t u16;

	if(ConsoleCmpCmd("wifi stats"))
	{
		ConsolePrint("Runtime: %u\r\n", wifi.runtime);

		for(uint16_t i = 0; i < CMD_BOT_COUNT_TOTAL; i++)
		{
			if(!wifi.bots[i].online)
				continue;

			RFQueueStats* pStat = &wifi.bots[i].queue.stats;

			ConsolePrint(" %2hu       toBot    fromBot     rxLoss     txLoss\r\n", i);
			ConsolePrint("LL P %10u %10u %10u\r\n", pStat->rfIO.txPackets, pStat->rfIO.rxPackets,
					pStat->rfFeedback.rxPacketsLost);
			ConsolePrint("   B %10u %10u\r\n", pStat->rfIO.txBytes, pStat->rfIO.rxBytes);
			ConsolePrint("HL P %10u %10u %10u %10u\r\n", pStat->queueIO.txPackets, pStat->queueIO.rxPackets,
					pStat->queueFeedback.rxPacketsLost, pStat->queueFeedback.txPacketsLost);
			ConsolePrint("   B %10u %10u\r\n", pStat->queueIO.txBytes, pStat->queueIO.rxBytes);
			ConsolePrint("RSSI0: %.2fdBm\r\n", wifi.bots[i].emaRssi[0].value);
			ConsolePrint("RSSI1: %.2fdBm\r\n", wifi.bots[i].emaRssi[1].value);
			ConsolePrint("Ant Row: %hu\r\n", (uint16_t)wifi.bots[i].antTableRow);
		}
	}

	if(ConsoleScanCmd("wifi stats %hu", &u16) == 1)
	{
		if(u16 > CMD_BOT_COUNT_TOTAL)
			return;

		RFQueueStats* pStat = &wifi.bots[u16].queue.stats;

		ConsolePrint(" %2hu       toBot    fromBot     rxLoss     txLoss\r\n", u16);
		ConsolePrint("LL P %10u %10u %10u\r\n", pStat->rfIO.txPackets, pStat->rfIO.rxPackets,
				pStat->rfFeedback.rxPacketsLost);
		ConsolePrint("   B %10u %10u\r\n", pStat->rfIO.txBytes, pStat->rfIO.rxBytes);
		ConsolePrint("HL P %10u %10u %10u %10u\r\n", pStat->queueIO.txPackets, pStat->queueIO.rxPackets,
				pStat->queueFeedback.rxPacketsLost, pStat->queueFeedback.txPacketsLost);
		ConsolePrint("   B %10u %10u\r\n", pStat->queueIO.txBytes, pStat->queueIO.rxBytes);
	}

	if(ConsoleScanCmd("wifi ant %hu", &u16) == 1)
	{
		ConsolePrint("Using antenna: %hu\r\n", u16);

		SKY66112UseAntenna(&wifi.module.fem, u16);
	}

	if(ConsoleCmpCmd("calltime"))
	{
		ConsolePrint("tNow: %u\r\nwifi.lastCall: %u\r\n", SysTimeUSec(), wifi.lastCall);
	}

	if(ConsoleScanCmd("pos %hu", &u16) == 1)
	{
		if(u16 > CMD_BOT_COUNT_TOTAL)
			return;

		HubStorage* pBot = &hub.robot[u16];

		ConsolePrint("X:%8hd Y:%8hd W:%8hd\r\n", pBot->lastVisionPos[0], pBot->lastVisionPos[1], pBot->lastVisionPos[2]);
	}

	if(ConsoleCmpCmd("ntp"))
	{
		ConsolePrint("Synced: %hu\r\n", (uint16_t)network.sntp.synced);
	}
}

static void configCommands()
{
	uint16_t u16;
	IPv4Address ip;
	uint16_t ipIn[4];

	if(ConsoleCmpCmd("help"))
	{
		ConsolePrint("Help:\r\n");
		ConsolePrint("config: This prints the current config\r\n");
		ConsolePrint("vision port %%portnumber: set the portnumber of the vision to portnumber\r\n");
		ConsolePrint("vision ip %%ipblock1 %%ipblock2 %%ipblock3 %%ipblock4: set the vision ip to ipblock1.ipblock2.ipblock3.ipblock4\r\n");
		ConsolePrint("eth ip %%ipbleock1 %%ipblock2 %%ipblock3 %%ipblock4: set the basestation ip to ipblock1.ipblock2.ipblock3.ipblock4\r\n");
		ConsolePrint("eth port %%port: Set the port to listen on for match commands to port\r\n");
		ConsolePrint("eth mac %%byte: Set the last 8 bit of the basestations mac to byte\r\n");
		ConsolePrint("wifi ch %%channel: Set the channel for the basestation to channel\r\n");
		ConsolePrint("config write: write the transient configuration to the flash memory\r\n");
	}

	if(ConsoleCmpCmd("config"))
	{
		NetworkPrintConfig();
		WifiPrintConfig();
	}

	// --- VISION ---
	if(ConsoleScanCmd("vision port %hu", &u16) == 1)
	{
		NetworkSetVisionPort(u16);

		NetworkPrintConfig();
	}

	if(ConsoleScanCmd("vision ip %hu %hu %hu %hu", ipIn, ipIn+1, ipIn+2, ipIn+3) == 4)
	{
		ip.u8[0] = ipIn[0];
		ip.u8[1] = ipIn[1];
		ip.u8[2] = ipIn[2];
		ip.u8[3] = ipIn[3];
		NetworkSetVisionIP(ip);

		NetworkPrintConfig();
	}

	// --- ETHERNET ---
	if(ConsoleScanCmd("eth ip %hu %hu %hu %hu", ipIn, ipIn+1, ipIn+2, ipIn+3) == 4)
	{
		ip.u8[0] = ipIn[0];
		ip.u8[1] = ipIn[1];
		ip.u8[2] = ipIn[2];
		ip.u8[3] = ipIn[3];
		NetworkSetMyIP(ip);

		NetworkPrintConfig();
	}

	if(ConsoleScanCmd("eth port %hu", &u16) == 1)
	{
		NetworkSetMyPort(u16);

		NetworkPrintConfig();
	}

	if(ConsoleScanCmd("eth mac %hu", &u16) == 1)
	{
		NetworkSetMyMAC(u16);

		NetworkPrintConfig();
	}

	// --- WIFI ---
	if(ConsoleScanCmd("wifi ch %hu", &u16) == 1)
	{
		WifiSetChannel(u16);

		WifiPrintConfig();
	}

	if(ConsoleScanCmd("wifi fixed %hu", &u16) == 1)
	{
		wifi.cfg.fixedRuntime = u16;

		WifiPrintConfig();
	}

	if(ConsoleScanCmd("wifi bots %hu", &u16) == 1)
	{
		wifi.cfg.maxBots = u16;

		WifiPrintConfig();
	}

	if(ConsoleScanCmd("wifi timeout %hu", &u16) == 1)
	{
		wifi.cfg.timeout = u16;

		WifiPrintConfig();
	}

	if(ConsoleCmpCmd("wifi status"))
	{
		uint16_t status;
		int16_t result = SX1280GetStatus(&wifi.module.radio, &status);
		if(result)
			ConsolePrint("GetStatus failed: 0x%04hX\r\n", result);
		else
			ConsolePrint("Status: 0x%04hX\r\n", status);
	}

	if(ConsoleCmpCmd("config write"))
	{
		ConsolePrint("Writing config to FLASH...");

		NetworkSaveConfig();
		WifiSaveConfig();
//		RSTSaveConfig();

		ConsolePrint("OK\r\n");
	}

	if(ConsoleCmpCmd("touch calib"))
	{
		FlashFSDelete("touch/calib");
		BootReset();
	}
/*
	if(ConsoleScanCmd("rst %hu", &u16) == 1)
	{
		ConsolePrint("RST: %hu\r\n", u16);
		RSTEnable(u16);
	}

	if(ConsoleScanCmd("rst rate %hu", &u16) == 1)
	{
		ConsolePrint("RST rate: %huHz\r\n", u16);
		RSTSetRate(u16);
	}

	if(ConsoleScanCmd("rst port %hu", &u16) == 1)
	{
		ConsolePrint("RST port: %hu\r\n", u16);
		RSTSetPort(u16);
	}*/
}

static void processCommand()
{
#ifndef ENV_BOOT
	miscCommands();
	configCommands();
	networkCommands();
#endif

	if(ConsoleCmpCmd("version"))
	{
		ConsolePrint(VersionGetString());
		ConsolePrint("\r\nCommit SHA1: %s\r\n", GIT_SHA1);
		ConsolePrint("Commit Date: %s\r\n", GIT_COMMIT_DATE_ISO8601);
		ConsolePrint("Checksum: 0x%08X\r\n", BootGetApplicationCRC32());
	}
}
