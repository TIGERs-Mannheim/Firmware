#include "shell_commands.h"
#include "dev/shell.h"
#include "dev/touch.h"
#include "dev/sx1280.h"
#include "sys/eth0.h"
#include "sys/rtc.h"
#include "util/cpu_load.h"
#include "util/log.h"
#include "util/config.h"
#include "hal/sys_time.h"
#include "misc/inventory.h"
#include "presenter.h"
#include "version.h"
#include "base_station.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

SHELL_CMD(tasks, "List all active tasks");

SHELL_CMD_IMPL(tasks)
{
	(void)pUser; (void)argc; (void)argv;

	printf("IRQ stack free: %u\r\n", chThdGetFreeStack(0));
	CPULoadPrintTasks();
}

SHELL_CMD(log, "Print recent log events",
	SHELL_ARG(length, "Number of events per level to print")
);

SHELL_CMD_IMPL(log)
{
	(void)pUser; (void)argc;

	int numEvents = atol(argv[1]);
	LogPrintAllRecent(numEvents);
}

SHELL_CMD(stats, "Show station stats");

SHELL_CMD_IMPL(stats)
{
	(void)pUser; (void)argc; (void)argv;

	if(baseStation.sumatra.isOnline)
		printf("Sumatra: online at %s:%hu\r\n", IPv4FormatAddress(baseStation.sumatra.ip), baseStation.sumatra.port);
	else
		printf("Sumatra: offline\r\n");

	printf("RX: %u good, %u invalid, %u unauthorized, %u auth failures\r\n",
			baseStation.stats.rxGood, baseStation.stats.rxInvalid, baseStation.stats.rxUnauthorized, baseStation.stats.rxAuthFailed);
	printf("TX: %u good, %u failed\r\n", baseStation.stats.txGood, baseStation.stats.txFailed);
}

SHELL_CMD(config, "Show configuration");

SHELL_CMD_IMPL(config)
{
	(void)pUser; (void)argc; (void)argv;

	printf("Base IP:   %s:%hu\r\n", IPv4FormatAddress(baseStation.config.base.ip), baseStation.config.base.port);
	printf("Vision IP: %s:%hu\r\n", IPv4FormatAddress(baseStation.config.vision.ip), baseStation.config.vision.port);
}

SHELL_CMD(base_ip, "Set base station IP",
	SHELL_ARG(ip, "New IP to use")
);

SHELL_CMD_IMPL(base_ip)
{
	(void)pUser; (void)argc;

	IPv4Address ip;
	if(IPv4ParseAddress(argv[1], &ip))
	{
		fprintf(stderr, "Unable to parse IP address\r\n");
		return;
	}

	printf("New IP: %s\r\n", IPv4FormatAddress(ip));

	BaseStationSetIp(ip);
}

SHELL_CMD(base_port, "Set base station UDP port",
	SHELL_ARG(port, "New port to use")
);

SHELL_CMD_IMPL(base_port)
{
	(void)pUser; (void)argc;

	int port = atoi(argv[1]);
	if(port < 1024 || port > 65535)
	{
		fprintf(stderr, "Invalid port\r\n");
		return;
	}

	printf("New port: %d\r\n", port);

	BaseStationSetPort(port);
}

SHELL_CMD(vision_ip, "Set vision multicast IP",
	SHELL_ARG(ip, "New IP to use")
);

SHELL_CMD_IMPL(vision_ip)
{
	(void)pUser; (void)argc;

	IPv4Address ip;
	if(IPv4ParseAddress(argv[1], &ip))
	{
		fprintf(stderr, "Unable to parse IP address\r\n");
		return;
	}

	if(!IPv4IsMulticastAddress(ip))
	{
		fprintf(stderr, "Vision requires a multicast address\r\n");
		return;
	}

	printf("New IP: %s\r\n", IPv4FormatAddress(ip));

	BaseStationSetVisionIp(ip);
}

SHELL_CMD(vision_port, "Set vision UDP port",
	SHELL_ARG(port, "New port to use")
);

SHELL_CMD_IMPL(vision_port)
{
	(void)pUser; (void)argc;

	int port = atoi(argv[1]);
	if(port < 1024 || port > 65535)
	{
		fprintf(stderr, "Invalid port\r\n");
		return;
	}

	printf("New port: %d\r\n", port);

	BaseStationSetVisionPort(port);
}

SHELL_CMD(net, "Network subsystem commands",
	SHELL_ARG(cmd, "Subsystem command and arguments")
);

SHELL_CMD_IMPL(net)
{
	(void)pUser;

	ShellCmdForward(&baseStation.networkInterface.cmdHandler, argc-1, argv+1);
}

SHELL_CMD(vision, "SSL Vision subsystem commands",
	SHELL_ARG(cmd, "Subsystem command and arguments")
);

SHELL_CMD_IMPL(vision)
{
	(void)pUser;

	ShellCmdForward(&baseStation.vision.cmdHandler, argc-1, argv+1);
}

SHELL_CMD(rtc, "RTC subsystem commands",
	SHELL_ARG(cmd, "Subsystem command and arguments")
);

SHELL_CMD_IMPL(rtc)
{
	(void)pUser;

	ShellCmdForward(&rtc.cmdHandler, argc-1, argv+1);
}

SHELL_CMD(radio, "Radio subsystem commands",
	SHELL_ARG(cmd, "Subsystem command and arguments")
);

SHELL_CMD_IMPL(radio)
{
	(void)pUser;

	ShellCmdForward(&baseStation.radioBase.cmdHandler, argc-1, argv+1);
}

SHELL_CMD(flash, "Flash filesystem subsystem commands",
	SHELL_ARG(cmd, "Subsystem command and arguments")
);

SHELL_CMD_IMPL(flash)
{
	(void)pUser;

	ShellCmdForward(&flashFS.cmdHandler, argc-1, argv+1);
}

SHELL_CMD(reset, "Reset main microcontroller (reboot)");

SHELL_CMD_IMPL(reset)
{
	(void)pUser; (void)argc; (void)argv;

	NVIC_SystemReset();
}

SHELL_CMD(cpu, "Show CPU usage");

SHELL_CMD_IMPL(cpu)
{
	(void)pUser; (void)argc; (void)argv;

	CPULoadPrintUsage();
}

SHELL_CMD(heap, "Show heap usage");

SHELL_CMD_IMPL(heap)
{
	(void)pUser; (void)argc; (void)argv;

	PresenterPrintHeapStatus();
}

SHELL_CMD(cpuid, "Show microcontroller's CPU ID and version");

SHELL_CMD_IMPL(cpuid)
{
	(void)pUser; (void)argc; (void)argv;

	volatile uint32_t* pCPUID = (volatile uint32_t*)0x1FF0F420;
	printf("CPUID: 0x%08X 0x%08X 0x%08X\r\n", pCPUID[0], pCPUID[1], pCPUID[2]);

	uint32_t rev = (SCB->CPUID >> 20) & 0x07;
	uint32_t patch = SCB->CPUID & 0x07;
	printf("Cortex-M7 version: r%up%u\r\n", rev, patch);

	uint16_t revId = DBGMCU->IDCODE >> 16;
	const char* pIdName;
	switch(revId)
	{
		case 0x1000: pIdName = "A"; break;
		case 0x1001: pIdName = "Z"; break;
		default: pIdName = "?"; break;
	}

	printf("Revision: 0x%04hX => %s\r\n", revId, pIdName);
}

SHELL_CMD(sensors, "Sensor data summary");

SHELL_CMD_IMPL(sensors)
{
	(void)pUser; (void)argc; (void)argv;

	printf("\r\n### Touch ###\r\n");
	printf("Pressed: %hu\r\nX: %hu\r\nY: %hu\r\n", (uint16_t) touch.sensor.meas.pressed, touch.sensor.meas.x, touch.sensor.meas.y);
	printf("t_touch: %uus / %.2fHz\r\n", touch.sensor.profiler.ioDuration_us, touch.sensor.profiler.updateRate_Hz);
}

SHELL_CMD(time, "Show current system time");

SHELL_CMD_IMPL(time)
{
	(void)pUser; (void)argc; (void)argv;

	printf("Time: %uus\r\n", SysTimeUSec());

	struct tm *nowtm;
	time_t now = RTCGetUnixTimestamp();
	nowtm = localtime(&now);

	char tmbuf[64];
	strftime(tmbuf, sizeof tmbuf, "%Y-%m-%d %H:%M:%S", nowtm);
	printf("%s\r\n", tmbuf);
}

SHELL_CMD(touch_calib, "Reset touch screen calibration");

SHELL_CMD_IMPL(touch_calib)
{
	(void)pUser; (void)argc; (void)argv;

	FlashFSDelete("touch/calib");
	NVIC_SystemReset();
}

SHELL_CMD(version, "Show firmware version information");

SHELL_CMD_IMPL(version)
{
	(void)pUser; (void)argc; (void)argv;

	printf("%s\r\n", VersionGetString());
	printf("Commit SHA1: %s\r\n", GIT_SHA1);
	printf("Commit Date: %s\r\n", GIT_COMMIT_DATE_ISO8601);
}

SHELL_CMD(inventory, "Check if this base station is in the inventory");

SHELL_CMD_IMPL(inventory)
{
	(void)pUser; (void)argc; (void)argv;

	const InventoryEntry* pEntry = InventoryGetEntry();

	if(pEntry)
	{
		printf("Base listed in inventory\r\n");
		printf("HWID: %u\r\n", pEntry->hwId);
	}
	else
	{
		fprintf(stderr, "Base not in inventory!\r\n");
	}
}

void ShellCommandsInit()
{
	ShellCmdAdd(&devShell.shell.cmdHandler, version_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, reset_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, cpu_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, heap_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, tasks_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, log_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, stats_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, config_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, base_ip_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, base_port_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, vision_ip_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, vision_port_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, net_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, vision_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, radio_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, rtc_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, sensors_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, cpuid_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, time_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, flash_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, touch_calib_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, inventory_command);
}
