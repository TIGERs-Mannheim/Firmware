#include "shell_commands.h"
#include "dev/shell.h"
#include "dev/buzzer.h"
#include "dev/drib_ir.h"
#include "dev/motors.h"
#include "dev/imu.h"
#include "dev/mag.h"
#include "dev/touch.h"
#include "dev/adc_kicker.h"
#include "dev/power_mon.h"
#include "dev/leds_front.h"
#include "util/cpu_load.h"
#include "util/log.h"
#include "util/log_file.h"
#include "util/network_print.h"
#include "robot/network.h"
#include "robot/robot.h"
#include "module/power_control.h"
#include "misc/variant.h"
#include "misc/inventory.h"
#include "math/arm_mat_util_f32.h"
#include "tiger_bot.h"
#include "robot/fusion_ekf.h"
#include "robot/skills.h"
#include "presenter.h"
#include "hal/sys_time.h"
#include "version.h"
#include "test/tests.h"
#include "test/test_common.h"
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

SHELL_CMD(net, "Network subsystem commands",
	SHELL_ARG(cmd, "Subsystem command and arguments")
);

SHELL_CMD_IMPL(net)
{
	(void)pUser;

	ShellCmdForward(&network.cmdHandler, argc-1, argv+1);
}

SHELL_CMD(radio, "Radio subsystem commands",
	SHELL_ARG(cmd, "Subsystem command and arguments")
);

SHELL_CMD_IMPL(radio)
{
	(void)pUser;

	ShellCmdForward(&tigerBot.radioBot.cmdHandler, argc-1, argv+1);
}

SHELL_CMD(buzzer, "Buzzer subsystem commands",
	SHELL_ARG(cmd, "Subsystem command and arguments")
);

SHELL_CMD_IMPL(buzzer)
{
	(void)pUser;

	ShellCmdForward(&devBuzzer.cmdHandler, argc-1, argv+1);
}

SHELL_CMD(kicker, "Kicker subsystem commands",
	SHELL_ARG(cmd, "Subsystem command and arguments")
);

SHELL_CMD_IMPL(kicker)
{
	(void)pUser;

	ShellCmdForward(&tigerBot.kicker.cmdHandler, argc-1, argv+1);
}

SHELL_CMD(camera, "Camera/RPi subsystem commands",
	SHELL_ARG(cmd, "Subsystem command and arguments")
);

SHELL_CMD_IMPL(camera)
{
	(void)pUser;

	ShellCmdForward(&tigerBot.robotPi.cmdHandler, argc-1, argv+1);
}

SHELL_CMD(ir, "IR controller subsystem commands",
	SHELL_ARG(cmd, "Subsystem command and arguments")
);

SHELL_CMD_IMPL(ir)
{
	(void)pUser;

	ShellCmdForward(&devDribIr.cmdHandler, argc-1, argv+1);
}

SHELL_CMD(pwr, "Power subsystem commands",
	SHELL_ARG(cmd, "Subsystem command and arguments")
);

SHELL_CMD_IMPL(pwr)
{
	(void)pUser;

	ShellCmdForward(&tigerBot.powerControl.cmdHandler, argc-1, argv+1);
}

SHELL_CMD(pat, "Patten identification subsystem commands",
	SHELL_ARG(cmd, "Subsystem command and arguments")
);

SHELL_CMD_IMPL(pat)
{
	(void)pUser;

	ShellCmdForward(&tigerBot.patternIdent.cmdHandler, argc-1, argv+1);
}

SHELL_CMD(variant, "Variant subsystem commands",
	SHELL_ARG(cmd, "Subsystem command and arguments")
);

SHELL_CMD_IMPL(variant)
{
	(void)pUser;

	ShellCmdForward(&variant.cmdHandler, argc-1, argv+1);
}

SHELL_CMD(flash, "Flash filesystem subsystem commands",
	SHELL_ARG(cmd, "Subsystem command and arguments")
);

SHELL_CMD_IMPL(flash)
{
	(void)pUser;

	ShellCmdForward(&flashFS.cmdHandler, argc-1, argv+1);
}

SHELL_CMD(test, "Test subsystem commands",
	SHELL_ARG(cmd, "Subsystem command and arguments")
);

SHELL_CMD_IMPL(test)
{
	(void)pUser;

	ShellCmdForward(&tests.cmdHandler, argc-1, argv+1);
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

SHELL_CMD(screenshot, "Take a screenshot of the display content and write it to USB");

SHELL_CMD_IMPL(screenshot)
{
	(void)pUser; (void)argc; (void)argv;

	PresenterTakeScreenshot();
}

SHELL_CMD(cpuid, "Show microcontroller's CPU ID and version");

SHELL_CMD_IMPL(cpuid)
{
	(void)pUser; (void)argc; (void)argv;

	volatile uint32_t* pCPUID = (volatile uint32_t*)0x1FF1E800;
	printf("CPUID: 0x%08X 0x%08X 0x%08X\r\n", pCPUID[0], pCPUID[1], pCPUID[2]);

	uint32_t rev = (SCB->CPUID >> 20) & 0x07;
	uint32_t patch = SCB->CPUID & 0x07;
	printf("Cortex-M7 version: r%up%u\r\n", rev, patch);

	uint16_t revId = DBGMCU->IDCODE >> 16;
	const char* pIdName;
	switch(revId)
	{
		case 0x1001: pIdName = "Z"; break;
		case 0x1003: pIdName = "Y"; break;
		case 0x2001: pIdName = "X"; break;
		case 0x2003: pIdName = "V"; break;
		default: pIdName = "?"; break;
	}

	printf("Revision: 0x%04hX => %s\r\n", revId, pIdName);
}

SHELL_CMD(logfile, "Start a new logfile with a given name",
	SHELL_ARG(filename, "Logfile name")
);

SHELL_CMD_IMPL(logfile)
{
	(void)pUser; (void)argc;

	const char* pFilename = argv[1];

	FRESULT fresult;

	LogFileClose();

	uint32_t freeCluster;
	FATFS *fs;
	fresult = f_getfree("SD:", &freeCluster, &fs);
	if(fresult)
	{
		NetworkPrint("No media attached\n");
		fprintf(stderr, "No media attached\r\n");
		return;
	}

	uint32_t freeSectors = freeCluster * fs->csize;
	uint32_t freeSizeMB = freeSectors / ((1024 * 1024) / 512);

	NetworkPrint("Free Space: %uMB\n", freeSizeMB);
	printf("Free Space: %uMB\r\n", freeSizeMB);

	f_chdrive("SD:");
	LogFileOpen(pFilename);
}

SHELL_CMD(startlog, "Start a new logfile (auto-naming)");

SHELL_CMD_IMPL(startlog)
{
	(void)pUser; (void)argc; (void)argv;

	LogFileOpen(0);
}

SHELL_CMD(stoplog, "Close active logfile (if any)");

SHELL_CMD_IMPL(stoplog)
{
	(void)pUser; (void)argc; (void)argv;

	if(logFile.ready == 0)
	{
		NetworkPrint("No log file open!\n");
		fprintf(stderr, "No log file open!\r\n");
	}
	else
	{
		LogFileClose();
	}
}

SHELL_CMD(logstat, "Show logfile and filesystem status");

SHELL_CMD_IMPL(logstat)
{
	(void)pUser; (void)argc; (void)argv;

	FRESULT fresult;
	FATFS *fs;
	uint32_t freeCluster;

	fresult = f_getfree("SD:", &freeCluster, &fs);
	if(fresult)
	{
		fprintf(stderr, "No media attached\r\n");
		return;
	}

	uint32_t freeSectors = freeCluster * fs->csize;
	uint32_t freeSizeMB = freeSectors / ((1024 * 1024) / 512);

	printf("SD card free space: %uMB\r\n", freeSizeMB);

	if(logFile.ready == 0)
	{
		printf("No log file open.\r\n");
	}
	else
	{
		printf("File:        %s\r\n", logFile.logFilename);
		printf("Size:        %.3fMB\r\n", logFile.event.fileSize / (1024.0f*1024.0f));
		printf("Data In:     %u kB/s\r\n", logFile.event.bytesPerSecIn/1024);
		printf("Data Out:    %u kB/s\r\n", logFile.event.bytesPerSecWritten/1024);
		printf("Write speed: %u kB/s\r\n", logFile.event.writeSpeed/1024);
		printf("Data Loss:   %u kB/s\r\n", logFile.event.bytesPerSecLost/1024);
	}
}

SHELL_CMD(state, "Robot state summary");

SHELL_CMD_IMPL(state)
{
	(void)pUser; (void)argc; (void)argv;

	printf("### EKF ###\r\n");
	printf("x:\r\n");
	arm_mat_print(&fusionEKF.ekf.x);

	printf("z:\r\n");
	arm_mat_print(&fusionEKF.ekf.z);

	printf("Sigma:\r\n");
	arm_mat_print(&fusionEKF.ekf.Sigma);

	printf("Ex:\r\n");
	arm_mat_print(&fusionEKF.ekf.Ex);

	printf("timeSlot: %u\r\n", fusionEKF.timeSlotNow);
	printf("On vision: %hu\r\n", fusionEKF.vision.online);
	printf("tLastValidSample: %u\r\n", fusionEKF.vision.timeLastValidSample);
	printf("late meas: %u\r\n", fusionEKF.vision.numLateMeasurements);
	printf("No vision in network: %u\r\n", robot.sensors.vision.noVision);

	printf("### Robot State ###\r\n");
	printf("state pos: %.3f, %.3f, %.3f\r\n", robot.state.pos[0], robot.state.pos[1], robot.state.pos[2]);
	printf("state vel: %.3f, %.3f, %.3f\r\n", robot.state.vel[0], robot.state.vel[1], robot.state.vel[2]);

	printf("### Watchdog ###\r\n");
	printf("Z: %f\r\n", robot.accZ);

	printf("### Ball State ###\r\n");
	printf("State: %u\r\n", robot.state.ballState);
	printf("Pos: %.3f, %.3f\r\n", robot.state.ballPos[0], robot.state.ballPos[1]);
	printf("Vel: %.3f, %.3f\r\n", robot.state.ballVel[0], robot.state.ballVel[1]);

	printf("### Magnetometer ###\r\n");
	printf("Mag: %.4f, %.4f, %.4f\r\n", robot.sensors.mag.strength[0], robot.sensors.mag.strength[1], robot.sensors.mag.strength[2]);
	printf("ZMag: %.4f, ZState: %.4f", robot.state.magZ, robot.state.pos[2]);

	printf("### Limits ###\r\n");
	printf("XY mode: %u, W mode: %u\r\n", robot.skillOutput.drive.modeXY, robot.skillOutput.drive.modeW);
	printf("XY Acc: %f, W Acc: %f\r\n", robot.skillOutput.drive.limits.accMaxXY, robot.skillOutput.drive.limits.accMaxW);
	printf("Skill: %hu\r\n", (uint16_t) skills.input.skillId);
}

SHELL_CMD(runtime, "Runtimes of critical tasks");

SHELL_CMD_IMPL(runtime)
{
	(void)pUser; (void)argc; (void)argv;

	printf("### Robot ###\r\n");
	printf("Input:      %uus\r\n", robot.performance.inputTime);
	printf("Estimation: %uus\r\n", robot.performance.estimationTime);
	printf("Skill:      %uus\r\n", robot.performance.skillTime);
	printf("Control:    %uus\r\n", robot.performance.controlTime);
	printf("Output:     %uus\r\n", robot.performance.outputTime);
	printf("Misc:       %uus\r\n", robot.performance.miscTime);
	printf("SUM:        %uus\r\n", robot.performance.inputTime + robot.performance.estimationTime +robot.performance.skillTime
			+ robot.performance.controlTime + robot.performance.outputTime + robot.performance.miscTime);

	printf("\r\n");

	for(uint16_t i = 0; i < 5; i++)
	{
		printf("Motor%hu: %uus, %.3fHz\r\n", i, devMotors.mcu[i].profiler.ioDuration_us, devMotors.mcu[i].profiler.updateRate_Hz);
	}

	printf("\r\n### SPI2 ###\r\n");
	printf("Kicker: %uus\r\n", devAdcKicker.profiler.ioDuration_us);

	printf("\r\n### SPI4 ###\r\n");
	printf("IMU:   %uus, %.3fHz\r\n", devImu.profiler.ioDuration_us, devImu.profiler.updateRate_Hz);
	printf("Mag:   %uus, %.3fHz\r\n", devMag.profiler.ioDuration_us, devMag.profiler.updateRate_Hz);
	printf("Touch: %uus, %.3fHz\r\n", devTouch.profiler.ioDuration_us, devTouch.profiler.updateRate_Hz);
	printf("SUM:   %uus\r\n", devImu.profiler.ioDuration_us + devMag.profiler.ioDuration_us + devTouch.profiler.ioDuration_us);

	printf("\r\n### I2C2 ###\r\n");
	printf("INA226: %uus, %.3fHz\r\n", devPowerMon.profiler.ioDuration_us, devPowerMon.profiler.updateRate_Hz);

	printf("\r\n### UART8 ###\r\n");
	printf("McuDribbler: %uus, %.3fHz\r\n", devDribIr.profiler.ioDuration_us, devDribIr.profiler.updateRate_Hz);

	printf("\r\nPatternIdent: %uus, %.3fHz\r\n", tigerBot.patternIdent.profiler.ioDuration_us, tigerBot.patternIdent.profiler.updateRate_Hz);
}

SHELL_CMD(sensors, "Sensor data summary");

SHELL_CMD_IMPL(sensors)
{
	(void)pUser; (void)argc; (void)argv;

	printf("\r\n### IMU ###\r\n");
	float accNorm = sqrtf(devImu.meas.acc_mDs2[0]*devImu.meas.acc_mDs2[0]
					+ devImu.meas.acc_mDs2[1]*devImu.meas.acc_mDs2[1]
					+ devImu.meas.acc_mDs2[2]*devImu.meas.acc_mDs2[2]);
	printf("Acc: %.4f, %.4f, %.4f (%.4f)\r\n", devImu.meas.acc_mDs2[0], devImu.meas.acc_mDs2[1], devImu.meas.acc_mDs2[2], accNorm);
	printf("Gyr: %.4f, %.4f, %.4f\r\n", devImu.meas.gyr_radDs[0], devImu.meas.gyr_radDs[1], devImu.meas.gyr_radDs[2]);
	printf("Temp: %.2f\r\n", devImu.meas.temp_degC);
	printf("t_imu:  %uus\r\n", devImu.profiler.ioDuration_us);
	printf("f_imu: %.2f\r\n", devImu.profiler.updateRate_Hz);

	float magNorm = sqrtf(devMag.meas.strength_uT[0]*devMag.meas.strength_uT[0]
					+ devMag.meas.strength_uT[1]*devMag.meas.strength_uT[1]
					+ devMag.meas.strength_uT[2]*devMag.meas.strength_uT[2]);

	printf("\r\n### MAG ###\r\n");
	printf("Mag: %.4f, %.4f, %.4f (%.4f)\r\n", devMag.meas.strength_uT[0], devMag.meas.strength_uT[1], devMag.meas.strength_uT[2], magNorm);
	printf("Temp: %.2f\r\n", devMag.meas.temp_degC);
	printf("t_mag:  %uus\r\n", devMag.profiler.ioDuration_us);
	printf("f_mag: %.2f\r\n", devMag.profiler.updateRate_Hz);

	printf("\r\n### Touch ###\r\n");
	printf("Pressed: %hu\r\nX: %hu\r\nY: %hu\r\n", (uint16_t) devTouch.meas.pressed, devTouch.meas.x, devTouch.meas.y);
	printf("t_touch: %uus\r\n", devTouch.profiler.ioDuration_us);
}

SHELL_CMD(configs, "List all configs");

SHELL_CMD_IMPL(configs)
{
	(void)pUser; (void)argc; (void)argv;

	for(uint16_t i = 0; i < config.filesUsed; i++)
	{
		printf("%hu: %s\r\n", i, config.files[i].pDesc->pName);
	}
}

SHELL_CMD(update, "Start firmware update",
	SHELL_ARG(source, "usb or sd")
);

SHELL_CMD_IMPL(update)
{
	(void)pUser; (void)argc;

	if(*argv[1] == 'u')
	{
		FwUpdaterLoad(&tigerBot.fwUpdater, &tigerBot.fwLoaderUsb.source);
	}
	else if(*argv[1] == 's')
	{
		FwUpdaterLoad(&tigerBot.fwUpdater, &tigerBot.fwLoaderSdCard.source);
	}
	else
	{
		fprintf(stderr, "Unknown source\r\n");
	}
}

SHELL_CMD(time, "Show current system time");

SHELL_CMD_IMPL(time)
{
	(void)pUser; (void)argc; (void)argv;

	printf("Time: %uus\r\n", SysTimeUSec());

	struct tm *nowtm;
	time_t now = SysTimeUnix();
	nowtm = localtime(&now);

	char tmbuf[64];
	strftime(tmbuf, sizeof tmbuf, "%Y-%m-%d %H:%M:%S", nowtm);
	printf("%s\r\n", tmbuf);
}

SHELL_CMD(calibration, "Show IMU calibration status");

SHELL_CMD_IMPL(calibration)
{
	(void)pUser; (void)argc; (void)argv;

	const InventoryEntry* pEntry = InventoryGetEntry();
	if(pEntry)
	{
		printf("HW ID: %u\r\n", pEntry->hwId);

		const InventoryImuCalibration* pCalib = &pEntry->imuCalib;
		if(pCalib->calibrated)
		{
			printf("\r\n--- IMU ---\r\n");
			printf("Gyro bias: %.6f, %.6f, %.6f\r\n",
				pCalib->gyrBias[0], pCalib->gyrBias[1], pCalib->gyrBias[2]);
			printf("Acc bias:  %.6f, %.6f, %.6f\r\n",
				pCalib->accBias[0], pCalib->accBias[1], pCalib->accBias[2]);
			printf("MB tilt bias: %.6f, %.6f\r\n",
				pCalib->accTiltBias[0], pCalib->accTiltBias[1]);
			printf("Calibrated at %.2f\u00b0C\r\n", pCalib->imuCalibTemp);
			printf("\r\n--- MAG ---\r\n");
			printf("Bias: %.6f, %.6f, %.6f\r\n",
				pCalib->magBias[0], pCalib->magBias[1], pCalib->magBias[2]);
			printf("Calibrated at %.2f\u00b0C\r\n", pCalib->magCalibTemp);
		}
		else
		{
			printf("No calibration data present!\r\nPlease run 'calib imu' and update inventory.\r\n");
		}
	}
	else
	{
		printf("This robot is not listed in the inventory!\r\n");
	}
}

SHELL_CMD(inject, "Inject an event to the main robot system, triggering state changes",
	SHELL_ARG(event, "event name or 'list' to display options")
);

SHELL_CMD_IMPL(inject)
{
	(void)pUser; (void)argc;

	if(strcmp(argv[1], "list") == 0)
	{
		printf("  vision\r\n  reset\r\n  bat\r\n  network\r\n");
	}
	else if(strcmp(argv[1], "vision") == 0)
	{
		printf("Injecting vision detection\r\n");
		chMBPostTimeout(&robot.eventQueue, 1, TIME_IMMEDIATE);
	}
	else if(strcmp(argv[1], "reset") == 0)
	{
		printf("Injecting robot state reset\r\n");
		chMBPostTimeout(&robot.eventQueue, 4, TIME_IMMEDIATE);
	}
	else if(strcmp(argv[1], "bat") == 0)
	{
		printf("Injecting battery low event\r\n");
		chMBPostTimeout(&robot.eventQueue, 0, TIME_IMMEDIATE);
	}
	else if(strcmp(argv[1], "network") == 0)
	{
		printf("Injecting network enable event\r\n");
		RobotEnableNetwork();
	}
	else
	{
		fprintf(stderr, "Unknown event\r\n");
	}
}

SHELL_CMD(led, "Set front LED colors",
	SHELL_ARG(device, "left | right | both"),
	SHELL_ARG(r, "Red (0.0-1.0)"),
	SHELL_ARG(g, "Green (0.0-1.0)"),
	SHELL_ARG(b, "Blue (0.0-1.0)"),
	SHELL_ARG(w, "White (0.0-1.0)")
);

SHELL_CMD_IMPL(led)
{
	(void)pUser; (void)argc;

	float red = atof(argv[2]);
	float green = atof(argv[3]);
	float blue = atof(argv[4]);
	float white = atof(argv[5]);

	if(strcmp(argv[1], "left") == 0)
	{
		printf("Setting left LED to:\r\n");
		printf("R: %.3f, G: %.3f, B: %.3f, W: %.3f\r\n", red, green, blue, white);

		LEDRGBWSet(&devLedFrontLeft, red, green, blue, white);
	}
	else if(strcmp(argv[1], "right") == 0)
	{
		printf("Setting right LED to:\r\n");
		printf("R: %.3f, G: %.3f, B: %.3f, W: %.3f\r\n", red, green, blue, white);

		LEDRGBWSet(&devLedFrontRight, red, green, blue, white);
	}
	else if(strcmp(argv[1], "both") == 0)
	{
		printf("Setting both LEDs to:\r\n");
		printf("R: %.3f, G: %.3f, B: %.3f, W: %.3f\r\n", red, green, blue, white);

		LEDRGBWSet(&devLedFrontLeft, red, green, blue, white);
		LEDRGBWSet(&devLedFrontRight, red, green, blue, white);
	}
	else
	{
		fprintf(stderr, "Invalid argument\r\n");
	}
}

SHELL_CMD(version, "Show firmware version information");

SHELL_CMD_IMPL(version)
{
	(void)pUser; (void)argc; (void)argv;

	printf("%s\r\n", VersionGetString());
	printf("Commit SHA1: %s\r\n", GIT_SHA1);
	printf("Commit Date: %s\r\n", GIT_COMMIT_DATE_ISO8601);
}

SHELL_CMD(test_mode, "Enable robot test mode/deactivate high-level control",
	SHELL_ARG(enable, "1=enable test mode, 0=disable test mode")
);

SHELL_CMD_IMPL(test_mode)
{
	(void)pUser; (void)argc;

	int enable = atoi(argv[1]);

	if(enable)
	{
		TestModeStartup();
	}
	else
	{
		TestModeExit();
	}
}

void ShellCommandsInit()
{
	ShellCmdAdd(&devShell.shell.cmdHandler, version_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, reset_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, update_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, cpu_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, heap_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, tasks_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, log_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, calibration_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, configs_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, net_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, radio_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, buzzer_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, kicker_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, camera_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, ir_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, pwr_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, pat_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, variant_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, sensors_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, state_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, runtime_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, led_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, screenshot_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, cpuid_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, logfile_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, startlog_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, stoplog_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, logstat_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, time_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, inject_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, flash_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, test_command);
	ShellCmdAdd(&devShell.shell.cmdHandler, test_mode_command);
}
