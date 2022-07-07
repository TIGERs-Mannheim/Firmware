/*
 * cli.c
 *
 *  Created on: 29.05.2010
 *      Author: AndreR
 */

#include "cli.h"

#include "robot/ctrl_motor.h"
#include "robot/skills.h"

#include "hal/led.h"
#include "hal/uart8.h"
#include "hal/buzzer.h"
#include "hal/tft.h"
#include "hal/motors.h"
#include "hal/kicker.h"
#include "hal/ext_flash.h"

#include "util/console.h"
#include "util/flash_fs.h"
#include "util/flash.h"
#include "util/network_print.h"
#include "util/terminal.h"
#include "util/boot.h"
#include "util/sys_time.h"
#include "util/log.h"
#include "util/crc.h"
#include "util/trajectory.h"
#include "util/trajectory_orient.h"
#include "util/traj_generator.h"
#include "util/arm_mat_util_f32.h"
#include "util/hermite_spline_cubic.h"
#include "util/ukf.h"
#include "util/angle_math.h"
#include "util/init_hal.h"
#include "robot/network.h"
#include "util/srukf.h"
#include "robot/skill_basics.h"
#include "util/ekf.h"
#include "util/fw_updater.h"

#include "version.h"
#include "commands.h"
#include "intercom_constants.h"
#include "cpu_load.h"
#include "constants.h"
#include "spi4.h"
#include "spi6.h"
#include "power.h"
#include "gfx.h"
#include "fatfs/ff.h"
#include "util/log_file.h"
#include "robot/ctrl_tigga.h"
#include "robot/robot.h"
#include "test.h"
#include "ext.h"
#include "presenter.h"
#include "gui/ids.h"

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>

static void processCommand();

static void consoleTxCallback(void* param)
{
	(void)param;

	while(console.txFifo.numPackets>0)
	{
		uint8_t* pData;
		uint32_t len32;
		int16_t result = FifoLinGet(&console.txFifo, &pData, &len32);
		if(result)
			break;

		TerminalWrite(pData, len32);

		FifoLinDelete(&console.txFifo);
	}
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
	conData.txCallback = &consoleTxCallback;
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
		if(ConsoleIsCmdReady())
		{
			processCommand();

			ConsoleCmdProcessed();
		}

		chThdSleepMilliseconds(5);
	}
}

extern memory_heap_t guiHeap;

static void miscCommands()
{
	if(ConsoleCmpCmd("help misc"))
	{
		ConsolePrint("help misc:\r\n");
		ConsolePrint("\treset: do a reset\r\n");
		ConsolePrint("\tshutdown: poweroff the bot\r\n");

		NetworkPrint("help misc:\r\n");
		NetworkPrint("\treset: do a reset\r\n");
		NetworkPrint("\tshutdown: poweroff the bot\r\n");
	}

	if(ConsoleCmpCmd("reset"))
	{
		BootReset();
	}

	if(ConsoleCmpCmd("fault"))
	{
		volatile uint32_t* pFault = (volatile uint32_t*)0x55555555;
		*pFault = 42;
	}

	if(ConsoleCmpCmd("bootloader"))
	{
		ConsolePrint("Selecting bootloader\r\n");

		BootSetBootloaderSelected(1);

//		IntercomHeader iHeader;
//		iHeader.section = INTERCOM_SECTION_BOOTLOADER;
//		iHeader.address = INTERCOM_ADDRESS_MAIN;
//		if(uart8.extInstalled)
//			iHeader.address |= INTERCOM_ADDRESS_EXT;
//		iHeader.command = BOOTLOADER_SELECT;
//
//		IntercomReliableSend(&iHeader, 0, 0);
	}

	if(ConsoleCmpCmd("shutdown"))
	{
		PowerShutdown();
	}

	if(ConsoleCmpCmd("test"))
	{
		ConsolePrint("Truth: %d\r\n", 42);
	}

	float f = 21.0f;
	if(ConsoleScanCmd("float %f", &f)==1)
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
		ConsolePrint("IRQ stack free: %u\r\n", chThdGetFreeStack(0));

		CPULoadPrintTasks();
	}

	if(ConsoleCmpCmd("irqs"))
	{
		ConsolePrint("Listing active IRQs...\r\n");

		for(int32_t i = -15; i<0; i++)
		{
			ConsolePrint("%d: %u\r\n", i, NVIC_GetPriority(i));
			chThdSleepMilliseconds(10);
		}

		for(int32_t i = 0; i<98; i++)
		{
			uint32_t enabled = NVIC->ISER[(((uint32_t)(int32_t)i)>>5UL)]&((uint32_t)(1UL<<(((uint32_t)(int32_t)i)&0x1FUL)));

			if(enabled)
			{
				ConsolePrint("%d: %u\r\n", i, NVIC_GetPriority(i));
				chThdSleepMilliseconds(10);
			}
		}
	}

	if(ConsoleCmpCmd("integ"))
	{
		chSysLock();
		uint8_t result = chSysIntegrityCheckI(CH_INTEGRITY_RLIST|CH_INTEGRITY_VTLIST|CH_INTEGRITY_REGISTRY|CH_INTEGRITY_PORT);
		chSysUnlock();

		if(result)
			ConsolePrint("Integrity check failed\r\n");
		else
			ConsolePrint("Integrity check OK\r\n");
	}

	if(ConsoleCmpCmd("heap"))
	{
		size_t free;
		size_t fragments = chHeapStatus(&guiHeap, &free);

		ConsolePrint("Fragments: %u, Free: %u, Core: %u\r\n", fragments, free, chCoreGetStatusX());
	}

	if(ConsoleCmpCmd("ttest"))
	{
//		thread_t* pThread = chRegNextThread(chRegNextThread(chRegFirstThread()));
//		thread_t* pThread = chRegNextThread(chRegFirstThread());
		thread_t* pThread = chRegFirstThread();
		uint32_t thdAddr = (uint32_t)pThread;

		uint8_t* pEnd = (uint8_t*)pThread->p_ctx.r13;
		uint8_t* pStack = (uint8_t*)(pThread+1);
		while(pStack<pEnd&&*pStack==CH_DBG_STACK_FILL_VALUE)
			pStack++;

		uint32_t stackFree = (uint32_t)pStack-(uint32_t)pThread;

		uint32_t endThd = ((uint32_t)pThread->p_ctx.r13)+sizeof(struct port_intctx);
		uint32_t thdSize = endThd-(uint32_t)pThread;

		ConsolePrint("%s\r\n", pThread->p_name);
		ConsolePrint("pThd: 0x%08X\r\n", thdAddr);
		ConsolePrint("Stack start: 0x%08X\r\n", thdAddr+sizeof(thread_t));
		ConsolePrint("R13: 0x%08X\r\n", (uint32_t)pThread->p_ctx.r13);
		ConsolePrint("Thd: %u, IntCtx: %u, ExtCtx: %u\r\n", sizeof(thread_t), sizeof(struct port_intctx), sizeof(struct port_extctx)); // +PORT_INT_REQUIRED_STACK
		ConsolePrint("waSize: %u\r\n", thdSize);
		ConsolePrint("free: %u\r\n", stackFree);
	}

	if(ConsoleCmpCmd("time"))
	{
		ConsolePrint("Time: %uus\r\n", SysTimeUSec());

		struct tm* nowtm;
		time_t now = SysTimeUnix();
		nowtm = localtime(&now);

		char tmbuf[64];
		strftime(tmbuf, sizeof tmbuf, "%Y-%m-%d %H:%M:%S", nowtm);
		ConsolePrint("%s\r\n", tmbuf);
	}

	uint16_t u16;
	if(ConsoleScanCmd("log %hu", &u16)==1)
	{
		LogPrintAllRecent(u16);
	}

	if(ConsoleCmpCmd("spi6"))
	{
		ConsolePrint("V_bat: %fV (%huC)\r\n", power.vBat, (uint16_t)power.batCells);
		ConsolePrint("I_cur: %fA\r\n", power.iCur);
		ConsolePrint("V_ir:  %fV\r\n", spi6.vIr);
		ConsolePrint("V_irOn:  %fV\r\n", spi6.vIrOn);
		ConsolePrint("V_irOff:  %fV\r\n", spi6.vIrOff);
		ConsolePrint("Acc: %.4f, %.4f, %.4f\r\n", spi6.imu.acc[0], spi6.imu.acc[1], spi6.imu.acc[2]);
		ConsolePrint("Gyr: %.4f, %.4f, %.4f\r\n", spi6.imu.gyr[0], spi6.imu.gyr[1], spi6.imu.gyr[2]);
		ConsolePrint("Temp: %.2f\r\n", spi6.imu.temp);
		ConsolePrint("t_pwr:  %uus\r\n", spi6.power.sampleTime);
		ConsolePrint("t_kick: %uus\r\n", spi6.kick.sampleTime);
		ConsolePrint("t_imu:  %uus\r\n", spi6.imu.sampleTime);
	}

	if(ConsoleCmpCmd("spi4"))
	{
		ConsolePrint("-- Touch --\r\n");
		ConsolePrint("Pressed: %hu\r\nX: %hu\r\nY: %hu\r\n", (uint16_t)spi4.touch.pressed, spi4.touch.x, spi4.touch.y);
		ConsolePrint("t_touch: %uus\r\n", spi4.touch.sampleTime);
		chThdSleepMilliseconds(20);
		ConsolePrint("Board IDs: %hu / %hu\r\n", spi4.leftBoardType, spi4.rightBoardType);
		ConsolePrint("t_left: %uus\r\n", spi4.motors.sampleTime);
		for(uint16_t ch = 0; ch<4; ch++)
		{
			ConsolePrint("CH%hu: %.5f  %.5f  %.5f\r\n", ch, spi4.motors.raw[ch], spi4.motors.biasCurrent[ch], spi4.motors.current[ch]);
		}
	}

	if(ConsoleCmpCmd("spi4 calib"))
	{
		ConsolePrint("SPI4 motor current bias calibration\r\n");
		SPI4CurrentBiasCalibration();
	}

#ifndef ENV_BOOT
	if(ConsoleScanCmd("rot %hu", &u16)==1)
	{
		switch(u16)
		{
			case 0:
				gdispSetOrientation(GDISP_ROTATE_0);
			break;
			case 1:
				gdispSetOrientation(GDISP_ROTATE_90);
			break;
			case 2:
				gdispSetOrientation(GDISP_ROTATE_180);
			break;
			case 3:
				gdispSetOrientation(GDISP_ROTATE_270);
			break;
		}

		gdispClear(Red);
		gdispFillArea(0, 0, 100, 200, Blue);
		gdispDrawString(10, 10, "Test", gdispOpenFont("fixed_7x14"), White);
	}
#endif

	if(ConsoleCmpCmd("sd"))
	{
		ConsolePrint("STA: 0x%08X\r\n", SDMMC1->STA);
		ConsolePrint("FIFOCNT: %u\r\n", SDMMC1->FIFOCNT);
	}

	if(ConsoleCmpCmd("qspi"))
	{
		ExtFlashPrintPerformance();

		ExtFlashChipErase();

		ConsolePrint("Writing...\r\n");

		for(uint32_t i = 0; i<1*1024*1024; i += 4)
		{
			ExtFlashWrite(i, &i, 4);
		}

		ConsolePrint("Reading...\r\n");

		for(uint32_t i = 0; i<1*1024*1024; i += 4)
		{
			uint32_t data;

			ExtFlashRead(i, &data, 4);

			uint32_t comp = i;

			if(data!=comp)
			{
				ConsolePrint("ExtFlash mismatch at %u: 0x%08X, 0x%08X\r\n", i, i, data);
//				break;
			}
		}

		ConsolePrint("done\r\n");
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

	if(ConsoleCmpCmd("flash test"))
	{
		uint32_t buf[16];
		memset(buf, 0x42, 16*sizeof(uint32_t));
		int16_t result = FlashProgram(flashFS.flashAddr, buf, 8);

		ConsolePrint("FlashProgram: %hd\r\n", result);
	}

	if(ConsoleCmpCmd("prog wifi"))
	{
		FwUpdaterLoad(FW_UPDATER_SOURCE_WIFI);
	}

	if(ConsoleCmpCmd("prog usb"))
	{
		FwUpdaterLoad(FW_UPDATER_SOURCE_USB);
	}

	if(ConsoleCmpCmd("dwt"))
	{
		ConsolePrint("DWT: %u\r\n", *DWT_CYCCNT);
	}

	if(ConsoleCmpCmd("configs"))
	{
		for(uint16_t i = 0; i<config.filesUsed; i++)
		{
			ConsolePrint("%hu: %s\r\n", i, config.files[i].pDesc->pName);
		}
	}

	if(ConsoleCmpCmd("est"))
	{
		arm_matrix_instance_f32 mechState = { 8, 1, ctrlTigga.mechState };

		ConsolePrint("mechState:\r\n");
		arm_mat_print(&mechState);

		ConsolePrint("Vision: %f, %f, %f, %hu\r\n", robot.sensors.vision.pos[0], robot.sensors.vision.pos[1], robot.sensors.vision.pos[2],
				(uint16_t)robot.sensors.vision.updated);

		ConsolePrint("Robot mode: %hu\r\nSystems: 0x%04X\r\n", robot.mode, robot.enabledSystems);
	}

	if(ConsoleCmpCmd("filter stat"))
	{
		NetworkPrint("vision: %hu", ctrlTigga.visionOnline);

		NetworkPrint("ekf: %.3f %.3f %.3f %.3f %.3f %.3f", ctrlTigga.mechState[0], ctrlTigga.mechState[1], ctrlTigga.mechState[2], ctrlTigga.mechState[3],
				ctrlTigga.mechState[4], ctrlTigga.mechState[5]);

		NetworkPrint("enc: %.3f %.3f %.3f %.3f %.3f %.3f", ctrlTigga.encState[0], ctrlTigga.encState[1], ctrlTigga.encState[2], ctrlTigga.encState[3],
				ctrlTigga.encState[4], ctrlTigga.encState[5]);
	}

	if(ConsoleCmpCmd("inject vision"))
	{
		chMBPost(&robot.eventQueue, 1, TIME_IMMEDIATE);
	}

	if(ConsoleCmpCmd("inject reset"))
	{
		chMBPost(&robot.eventQueue, 4, TIME_IMMEDIATE);
	}

	if(ConsoleCmpCmd("inject bat"))
	{
		chMBPost(&robot.eventQueue, 0, TIME_IMMEDIATE);
	}

	if(ConsoleScanCmd("test mode %hu", &u16)==1)
	{
		ConsolePrint("Setting robot test mode: %hu\r\n", u16);

		RobotTestModeEnable(u16);
	}

	uint32_t u32;
	if(ConsoleScanCmd("set systems %X", &u32)==1)
	{
		ConsolePrint("Setting robot systems: 0x%04X\r\n", u32);

		RobotTestModeSetSystems(u32);
	}

	if(ConsoleCmpCmd("bias"))
	{
		ConsolePrint("Gyr: %.5f\r\nAcc: %.5f, %.5f\r\n", ctrlTigga.ekf.x.pData[7], ctrlTigga.ekf.x.pData[5], ctrlTigga.ekf.x.pData[6]);

		NetworkPrint("Gyr: %.5f\nAcc: %.5f, %.5f\n", ctrlTigga.ekf.x.pData[7], ctrlTigga.ekf.x.pData[5], ctrlTigga.ekf.x.pData[6]);
	}

	if(ConsoleCmpCmd("vision delay"))
	{
		ConsolePrint("Late meas: %u, Max delay: %uus\r\n", ctrlTigga.numLateVisionMeasurements, ctrlTigga.maxVisionDelay);

		NetworkPrint("Late meas: %u, Max delay: %uus\n", ctrlTigga.numLateVisionMeasurements, ctrlTigga.maxVisionDelay);

		ctrlTigga.numLateVisionMeasurements = 0;
		ctrlTigga.maxVisionDelay = 0;
	}

	if(ConsoleCmpCmd("watchdog"))
	{
		ConsolePrint("Z: %f\r\n", robot.accZ);
	}

	if(ConsoleCmpCmd("limits"))
	{
		ConsolePrint("XY mode: %u, W mode: %u\r\n", robot.skillOutput.drive.modeXY, robot.skillOutput.drive.modeW);
		ConsolePrint("XY Acc: %f, W Acc: %f\r\n", robot.skillOutput.drive.limits.accMaxXY, robot.skillOutput.drive.limits.accMaxW);
		ConsolePrint("Skill: %hu\r\n", (uint16_t)skills.input.skillId);
	}

	if(ConsoleCmpCmd("em"))
	{
		ConsolePrint("out: %f, %f, %f\r\n", emergency.outVelLocal[0], emergency.outVelLocal[1], emergency.outVelLocal[2]);
		ConsolePrint("dec: %f, %f, %f\r\n", emergency.decrement[0], emergency.decrement[1], emergency.decrement[2]);
	}

	if(ConsoleCmpCmd("nan"))
	{
		ConsolePrint("injecting nan\r\n");
		MAT_ELEMENT(ctrlTigga.ekf.Sigma, 3, 3) = NAN;
	}

	if(ConsoleCmpCmd("ext"))
	{
		if(uart8.extInstalled)
		{
			ConsolePrint("Extension board installed\r\n");

			ConsolePrint("--- UART8 ---\r\n");
			ConsolePrint("RX payload/wire/usage: %u / %u / %3.2f%%\r\n", uart8.uart.stats.rxPayload, uart8.uart.stats.rxWire, uart8.uart.stats.rxUsage*100.0f);
			ConsolePrint("TX payload/wire/usage: %u / %u / %3.2f%%\r\n", uart8.uart.stats.txPayload, uart8.uart.stats.txWire, uart8.uart.stats.txUsage*100.0f);

			ConsolePrint("lastMatchCtrl time: %uus\r\n", robot.ext.lastMatchCtrlTime);
			ConsolePrint("Ext. control interface active: %hu\r\n", robot.ext.active);
		}
		else
		{
			ConsolePrint("Extension board not installed\r\n");
		}
	}
}

static void wifiCommands()
{
	uint16_t u16;

	if(ConsoleCmpCmd("help wifi"))
	{
		ConsolePrint("wifi help:\r\n");
		ConsolePrint("\twifi stats: print wifi configuration\r\n");
		ConsolePrint("\tconfig: print bot configuration\r\n");
		ConsolePrint("\tcolor b: set the color to blue\r\n");
		ConsolePrint("\tcolor y: set the color to yellow\r\n");
		ConsolePrint("\tset botid %%id: set the botid to id\r\n");
		ConsolePrint("\tset channel %%ch: set the wifi channel to ch\r\n");
		ConsolePrint("\tconfig write: persist the current wifi stats\r\n");

		NetworkPrint("wifi help:\r\n");
		NetworkPrint("\twifi stats: print wifi configuration\r\n");
		NetworkPrint("\tconfig: print bot configuration\r\n");
		NetworkPrint("\tcolor b: set the color to blue\r\n");
		NetworkPrint("\tcolor y: set the color to yellow\r\n");
		NetworkPrint("\tset botid %%id: set the botid to id\r\n");
		NetworkPrint("\tset channel %%ch: set the wifi channel to ch\r\n");
		NetworkPrint("\tconfig write: persist the current wifi stats\r\n");
	}

	if(ConsoleCmpCmd("wifi stats"))
	{
		ConsolePrint("Avg: %.3f\r\n", network.avgRxPeriod.value);

		RFQueueStats* pStat = &network.queue.stats;

		ConsolePrint(" %2hu        toBS     fromBS     rxLoss     txLoss\r\n", (uint16_t)network.config.botId);
		ConsolePrint("LL P %10u %10u %10u\r\n", pStat->rfIO.txPackets, pStat->rfIO.rxPackets, pStat->rfFeedback.rxPacketsLost);
		ConsolePrint("   B %10u %10u\r\n", pStat->rfIO.txBytes, pStat->rfIO.rxBytes);
		ConsolePrint("HL P %10u %10u %10u %10u\r\n", pStat->queueIO.txPackets, pStat->queueIO.rxPackets, pStat->queueFeedback.rxPacketsLost,
				pStat->queueFeedback.txPacketsLost);
		ConsolePrint("   B %10u %10u\r\n", pStat->queueIO.txBytes, pStat->queueIO.rxBytes);
	}

	if(ConsoleCmpCmd("color b"))
	{
		if(network.config.botId<CMD_BOT_COUNT_HALF)
		{
			network.config.botId += CMD_BOT_COUNT_HALF;
			NetworkSaveConfig();
			NetworkPrint("Changed BotID to %hu", network.config.botId);
			chThdSleepMilliseconds(100);
			BootReset();
		}
	}

	if(ConsoleCmpCmd("color y"))
	{
		if(network.config.botId>CMD_BOT_HALF_MIN_1)
		{
			network.config.botId -= CMD_BOT_COUNT_HALF;
			NetworkSaveConfig();
			NetworkPrint("Changed BotID to %hu", network.config.botId);
			chThdSleepMilliseconds(100);
			BootReset();
		}
	}

	if(ConsoleScanCmd("set botid %hu", &u16)==1)
	{
		network.config.botId = u16;
		NetworkSaveConfig();

		NetworkPrint("Changed BotID to %hu", u16);
		ConsolePrint("Changed BotID to %hu\r\n", u16);
	}

	if(ConsoleScanCmd("write botid %hu", &u16)==1)
	{
		network.config.botId = u16;
		NetworkSaveConfig();

		NetworkPrint("Changed BotID to %hu", u16);
		ConsolePrint("Changed BotID to %hu\r\n", u16);
	}

	if(ConsoleScanCmd("set channel %hu", &u16)==1)
	{
		network.config.channel = u16;
		NetworkSaveConfig();

		ConsolePrint("Changed channel to %hu\r\n", u16);
	}

	if(ConsoleCmpCmd("config write"))
	{
		NetworkSaveConfig();
		ConsolePrint("Config saved to FLASH\r\n");
	}

	if(ConsoleCmpCmd("config"))
	{
		ConsolePrint("ID: %hu\r\n", network.config.botId);
		ConsolePrint("Channel: %hu\r\n", network.config.channel);
		ConsolePrint("HWID: %hu\r\n", (uint16_t)RobotImplGetHardwareId());
	}
}

static void kickerCommands()
{
	uint16_t u16;
	uint32_t u32;
	float f32;

	if(ConsoleCmpCmd("help kicker"))
	{
		ConsolePrint("kicker help:\r\n");
		ConsolePrint("\tkstat: print kicker status\r\n");
		ConsolePrint("\tvmax %%voltage: set the max voltage to voltage\r\n");
		ConsolePrint("\tchg %%int: enable/disable charging to vmax once, with int <> 0 true\r\n");
		ConsolePrint("\tautochg %%int: enable/disable charging to vmax ever, with int <> 0 true\r\n");
		ConsolePrint("\tautodischg: discharge the bot\r\n");

		NetworkPrint("kicker help:\r\n");
		NetworkPrint("\tkstat: print kicker status\r\n");
		NetworkPrint("\tvmax %%voltage: set the max voltage to voltage\r\n");
		NetworkPrint("\tchg %%int: enable/disable charging to vmax once, with int <> 0 true\r\n");
		NetworkPrint("\tautochg %%int: enable/disable charging to vmax ever, with int <> 0 true\r\n");
		NetworkPrint("\tautodischg: discharge the bot\r\n");
	}

	if(ConsoleScanCmd("vmax %hu", &u16)==1)
	{
		ConsolePrint("vMax: %hu\r\n", u16);

		KickerSetMaxVoltage(u16);
	}

	if(ConsoleScanCmd("chg %hu", &u16)==1)
	{
		if(u16)
		{
			ConsolePrint("Enable charging\r\n");
			KickerEnableCharge(1);
		}
		else
		{
			ConsolePrint("Disable charging\r\n");
			KickerEnableCharge(0);
		}
	}

	if(ConsoleScanCmd("autochg %hu", &u16)==1)
	{
		if(u16)
		{
			ConsolePrint("Enable autocharging\r\n");
			KickerEnableAutoRecharge(1);
		}
		else
		{
			ConsolePrint("Disable autocharging\r\n");
			KickerEnableAutoRecharge(0);
		}
	}

	if(ConsoleCmpCmd("autodischg"))
	{
		ConsolePrint("Enabling auto discharging\r\n");
		KickerAutoDischarge();
	}

	if(ConsoleScanCmd("kstress %f", &f32)==1)
	{
		ConsolePrint("Setting auto-fire stress test: %fms\r\n", f32);

		kicker.stress = f32*0.001f;
	}

	if(ConsoleScanCmd("dis1 %f", &f32)==1)
	{
		ConsolePrint("Fire1: %fms\r\n", f32);

		KickerFire(f32*0.001f, KICKER_DEVICE_STRAIGHT);
	}

	if(ConsoleScanCmd("dis1s %f", &f32)==1)
	{
		ConsolePrint("Fire1: %fm/s\r\n", f32);

		KickerFireSpeed(f32, KICKER_DEVICE_STRAIGHT);
	}

	if(ConsoleScanCmd("arm1 %f", &f32)==1)
	{
		ConsolePrint("Arm1: %fms\r\n", f32);

		KickerArm(f32*0.001f, KICKER_DEVICE_STRAIGHT);
	}

	if(ConsoleScanCmd("dis2 %f", &f32)==1)
	{
		ConsolePrint("Fire2: %fms\r\n", f32);

		KickerFire(f32*0.001f, KICKER_DEVICE_CHIP);
	}

	if(ConsoleScanCmd("dis2s %f", &f32)==1)
	{
		ConsolePrint("Fire2: %fm/s\r\n", f32);

		KickerFireSpeed(f32, KICKER_DEVICE_CHIP);
	}

	if(ConsoleScanCmd("arm2 %f", &f32)==1)
	{
		ConsolePrint("Arm2: %fms\r\n", f32);

		KickerArm(f32*0.001f, KICKER_DEVICE_CHIP);
	}

	if(ConsoleCmpCmd("kstat"))
	{
		ConsolePrint("Installed: %hu, v2017: %hu\r\n", (uint16_t)kicker.installed, (uint16_t)kicker.v2017);
		ConsolePrint("Cap: %.3fV (%.1fV)\r\n", kicker.vCap, kicker.config.maxVoltage);
		ConsolePrint("Chg: %.3fA\r\n", kicker.iChg);
		ConsolePrint("Temp:%.2fC\r\n", kicker.tEnv);
		ConsolePrint("IR: %.3f / %.3f\r\n", kicker.vIrOff, kicker.vIrOn);
		ConsolePrint("State: %hu, Auto: %hu, Cnt: %hu\r\n", (uint16_t)kicker.state, (uint16_t)kicker.autoCharge, (uint16_t)kicker.kickCounter);
		ConsolePrint("IR / Chip / Straight: %hu / %hu / %hu\r\n", kicker.barrierDamaged, kicker.chipDamaged, kicker.straightDamaged);
		ConsolePrint("Last kick usage: %.3fV\r\n", kicker.lastCapUsage);
	}

	if(ConsoleScanCmd("cooldown %u", &u32)==1)
	{
		KickerSetCooldown(u32);

		ConsolePrint("Kicker Cooldown set to %ums\r\n", kicker.config.cooldown);
	}
}

static void motorsCommands()
{
	uint16_t u16[4];
	float f32[4];

	if(ConsoleCmpCmd("help mot"))
	{
		ConsolePrint("mot help:\r\n");
		ConsolePrint("\tmot test n: perform dynamics test on motor n\n\r");
		ConsolePrint("\tmot test all: permorm dynamics tests on all motors\n\r");
	}

	if(ConsoleCmpCmd("ctrl off"))
	{
		CtrlSetController(0);

		ConsolePrint("Controller disabled\r\n");
	}

	if(ConsoleCmpCmd("ctrl motor"))
	{
		CtrlSetController(&ctrlMotorInstance);

		ConsolePrint("Controller: Motors\r\n");
	}

	if(ConsoleCmpCmd("skill off"))
	{
		uint8_t data[SYSTEM_MATCH_CTRL_USER_DATA_SIZE];

		SkillsSetInput(0, data);
	}

	if(ConsoleScanCmd("move %f %f %f", f32, f32+1, f32+2)==3)
	{
//		CtrlMotorSetVelTarget(f32[0], f32[1], f32[2]);
//		CtrlTiggaSetVelTarget(f32[0], f32[1], f32[2]);

		uint8_t data[SYSTEM_MATCH_CTRL_USER_DATA_SIZE];
		int16_t* p16Data = (int16_t*)data;
		p16Data[0] = (int16_t)(f32[0]*1000.0f);
		p16Data[1] = (int16_t)(f32[1]*1000.0f);
		p16Data[2] = (int16_t)(f32[2]*1000.0f);

		SkillsSetInput(1, data);
	}

	if(ConsoleScanCmd("goto %f %f %f", f32, f32+1, f32+2)==3)
	{
		uint8_t data[SYSTEM_MATCH_CTRL_USER_DATA_SIZE];
		int16_t* p16Data = (int16_t*)data;
		p16Data[0] = (int16_t)(f32[0]*1000.0f);
		p16Data[1] = (int16_t)(f32[1]*1000.0f);
		p16Data[2] = (int16_t)(f32[2]*1000.0f);
		p16Data[3] = 0;

		SkillsSetInput(3, data);
	}

	if(ConsoleCmpCmd("mot fault"))
	{
		MotorsDrivePrintFault();
	}

	if(ConsoleCmpCmd("mot off"))
	{
		ConsolePrint("Motors off\r\n");

		MotorsDriveOff();
	}

	if(ConsoleScanCmd("mot vol %hu %f", u16, f32)==2)
	{
		ConsolePrint("Motor %hu: %.3fV\r\n", u16[0], f32[0]);
		MotorsDriveSetVoltage(u16[0], f32[0]);
	}

	if(ConsoleScanCmd("mot vol all %f", f32)==1)
	{
		ConsolePrint("Setting all motor velocities to: %.3fV\r\n", f32[0]);

		for(uint8_t i = 0; i<4; i++)
			MotorsDriveSetVoltage(i, f32[0]);
	}

	if(ConsoleScanCmd("mot vel %hu %f", u16, f32)==2)
	{
		ConsolePrint("Vel %hu: %.3frad/s\r\n", u16[0], f32[0]);

		MotorsDriveSetVelocity(u16[0], f32[0]);
	}

	if(ConsoleScanCmd("mot pid %f %f %f", f32, f32+1, f32+2)==3)
	{
		ConsolePrint("P: %f, I: %f, D: %f\r\n", f32[0], f32[1], f32[2]);

		for(uint8_t i = 0; i<4; i++)
			PIDSetParams(&motors.drive[i].pid, f32[0], f32[1], f32[2]);
	}

	if(ConsoleScanCmd("mot test %hu", u16)==1)
	{
		TestScheduleMotorTest(IC_TEST_MOT_DYN, u16[0]);
		ConsolePrint("Scheduled dynamics test on motor %hu \n\r", u16[0]);

		PresenterShowWindow(GUI_WINDOW_TEST_DRIVE);
	}

	if(ConsoleCmpCmd("mot test all"))
	{
		for(int i = 0; i<5; i++)
		{
			TestScheduleMotorTest(IC_TEST_MOT_DYN, i);
			ConsolePrint("Scheduled dynamics test on motor %hu \n\r", i);
		}

		PresenterShowWindow(GUI_WINDOW_TEST_DRIVE);
	}

	if(ConsoleCmpCmd("mot traction"))
	{
		TestScheduleMotorTest(IC_TEST_MOT_TRACTION, 0);

		ConsolePrint("Scheduled traction test\n\r");
	}

	if(ConsoleCmpCmd("mot enc"))
	{
		MotorsDrivePrintEncoders();
	}

	if(ConsoleCmpCmd("mot sampling"))
	{
		TrajSecOrder1D traj;
		float U;

		for(float v = 1.0f; v<10.0f; v += 1.0f)
		{
			TrajSecOrder1DCreate(&traj, v-1, 0, v, 10, 100);
			float tEnd = TrajSecOrder1DGetTotalTime(&traj);

			for(float t = 0; t<tEnd+0.1f; t += 0.001f)
			{
				TrajSecOrder1DValuesAtTime(&traj, t, &U, 0, 0);

				for(uint8_t i = 0; i<4; i++)
					MotorsDriveSetVoltage(i, U);

				chThdSleepMilliseconds(1);
			}

			chThdSleepMilliseconds(1000);
		}

		for(float v = 9.0f; v>0.0f; v -= 1.0f)
		{
			TrajSecOrder1DCreate(&traj, v+1, 0, v, 10, 100);
			float tEnd = TrajSecOrder1DGetTotalTime(&traj);

			for(float t = 0; t<tEnd+0.1f; t += 0.001f)
			{
				TrajSecOrder1DValuesAtTime(&traj, t, &U, 0, 0);

				for(uint8_t i = 0; i<4; i++)
					MotorsDriveSetVoltage(i, U);

				chThdSleepMilliseconds(1);
			}

			chThdSleepMilliseconds(1000);
		}

		for(uint8_t i = 0; i<4; i++)
			MotorsDriveSetVoltage(i, 0);
	}

	if(ConsoleScanCmd("mot mv %f %f", f32, f32+1)==2)
	{
		ConsolePrint("Move. Angle: %.2f Volt: %.2f\r\n", f32[0], f32[1]);

		float angle = f32[0]*M_PI/180.0f;

		arm_matrix_instance_f32 move = { 3, 1, (float[3] )
				{ } };
		move.pData[0] = arm_cos_f32(angle);
		move.pData[1] = arm_sin_f32(angle);
		move.pData[2] = 0;

		arm_matrix_instance_f32 volt = { 4, 1, (float[4] )
				{ } };

		arm_mat_mult_f32(&ctrl.matXYW2Motor, &move, &volt);

		TrajSecOrder1D traj;

		TrajSecOrder1DCreate(&traj, 0, 0, f32[1], 10, 100);
		float tEnd = TrajSecOrder1DGetTotalTime(&traj);

		float U;
		for(float t = 0; t<tEnd+0.1f; t += 0.001f)
		{
			TrajSecOrder1DValuesAtTime(&traj, t, &U, 0, 0);

			MotorsDriveSetVoltage(0, volt.pData[0]*U);
			MotorsDriveSetVoltage(1, volt.pData[1]*U);
			MotorsDriveSetVoltage(2, volt.pData[2]*U);
			MotorsDriveSetVoltage(3, volt.pData[3]*U);

			chThdSleepMilliseconds(1);
		}

		chThdSleepMilliseconds(1000);

		TrajSecOrder1DCreate(&traj, f32[1], 0, 0, 10, 100);
		tEnd = TrajSecOrder1DGetTotalTime(&traj);

		for(float t = 0; t<tEnd+0.1f; t += 0.001f)
		{
			TrajSecOrder1DValuesAtTime(&traj, t, &U, 0, 0);

			MotorsDriveSetVoltage(0, volt.pData[0]*U);
			MotorsDriveSetVoltage(1, volt.pData[1]*U);
			MotorsDriveSetVoltage(2, volt.pData[2]*U);
			MotorsDriveSetVoltage(3, volt.pData[3]*U);

			chThdSleepMilliseconds(1);
		}

		for(uint8_t i = 0; i<4; i++)
			MotorsDriveSetVoltage(i, 0);
	}

	if(ConsoleCmpCmd("mot cur"))
	{
		for(uint16_t ch = 0; ch<4; ch++)
		{
			ConsolePrint("%hu: %.5f\r\n", ch, robot.sensors.motorCur.cur[ch]);
		}
	}

	if(ConsoleCmpCmd("drib off"))
	{
		ConsolePrint("Dribbler off\r\n");

		MotorsDribblerOff();
	}

	if(ConsoleScanCmd("drib vol %f", f32)==1)
	{
		ConsolePrint("Dribbler voltage: %f\r\n", f32[0]);

		MotorsDribblerSetVoltage(f32[0]);
	}

	if(ConsoleScanCmd("drib vel %f", f32)==1)
	{
		ConsolePrint("Dribbler speed: %fRPM\r\n", f32[0]);

		MotorsDribblerSetVelocity(f32[0]);
	}

	if(ConsoleCmpCmd("drib stat"))
	{
		ConsolePrint("LastUpdate: %u\r\nVelocity: %f\r\n", motors.dribbler.lastUpdateTime, motors.dribbler.velocity*MOTORS_DRIBBLER_MOTOR_RADS_TO_BAR_RPM);
	}

	if(ConsoleScanCmd("drib pid %f %f %f", f32, f32+1, f32+2)==3)
	{
		ConsolePrint("P: %f, I: %f, D: %f\r\n", f32[0], f32[1], f32[2]);

		PIDSetParams(&motors.dribbler.pid, f32[0], f32[1], f32[2]);
	}

	if(ConsoleCmpCmd("mm"))
	{
		ConsolePrint("Mode: %hu\r\n", (uint16_t)motors.driveMode);
	}

	if(ConsoleCmpCmd("mot torque"))
	{
		for(uint16_t i = 0; i<4; i++)
			ConsolePrint("M%hu: %.6fNm\r\n", i, motors.drive[i].setTorque);
	}
}

static void fsCommands()
{
	FRESULT fresult;
	static const uint16_t bufLen = 128;
	static uint8_t buf8[128];

	if(ConsoleScanCmd("chd %127[^\n]", buf8)==1)
	{
		f_chdrive((TCHAR*)buf8);

		f_getcwd((TCHAR*)buf8, bufLen);

		ConsolePrint("%s\r\n", buf8);
	}

	if(ConsoleCmpCmd("pwd"))
	{
		f_getcwd((TCHAR*)buf8, bufLen);

		ConsolePrint("%s\r\n", buf8);
	}

	if(ConsoleScanCmd("stat %127[^\n]", buf8)==1)
	{
		FILINFO info;
		fresult = f_stat((TCHAR*)buf8, &info);

		if(fresult)
		{
			ConsolePrint("Stat error %u\r\n", fresult);
		}
		else
		{
			ConsolePrint("Size: %u\r\n", info.fsize);
		}
	}

	if(ConsoleCmpCmd("ls"))
	{
		DIR dir;
		FILINFO finfo;
		finfo.lfname = (TCHAR*)buf8;
		finfo.lfsize = bufLen-1;

		f_getcwd((TCHAR*)buf8, bufLen);

		ConsolePrint("Content of: %s\r\n", buf8);

		// show folders
		fresult = f_opendir(&dir, "");
		if(fresult!=FR_OK)
			return;

		do
		{
			fresult = f_readdir(&dir, &finfo);

			if(finfo.fname[0]==0)
				break;

			if(finfo.fattrib&AM_DIR)
			{
				ConsolePrint("<DIR>      %s\r\n", *finfo.lfname ? finfo.lfname : finfo.fname);
			}
		}
		while(fresult==FR_OK);

		f_closedir(&dir);

		// show files
		fresult = f_opendir(&dir, "");
		if(fresult!=FR_OK)
			return;

		do
		{
			fresult = f_readdir(&dir, &finfo);

			if(finfo.fname[0]==0)
				break;

			if((finfo.fattrib&AM_DIR)==0)
			{
				if(finfo.fsize>1024*1024*16)	// larger than 16M?
					ConsolePrint("%7uM   %s\r\n", finfo.fsize/(1024*1024), *finfo.lfname ? finfo.lfname : finfo.fname);
				else if(finfo.fsize>1024*16)	// larget than 16k?
					ConsolePrint("%7uk   %s\r\n", finfo.fsize/1024, *finfo.lfname ? finfo.lfname : finfo.fname);
				else
					ConsolePrint("%7u    %s\r\n", finfo.fsize, *finfo.lfname ? finfo.lfname : finfo.fname);
			}
		}
		while(fresult==FR_OK);

		f_closedir(&dir);
	}

	if(ConsoleScanCmd("cd %127[^\n]", buf8)==1)
	{
		f_chdir((TCHAR*)buf8);

		f_getcwd((TCHAR*)buf8, bufLen);

		ConsolePrint("%s\r\n", buf8);
	}

	if(ConsoleScanCmd("touch %127[^\n]", buf8)==1)
	{
		FIL file;
		fresult = f_open(&file, (TCHAR*)buf8, FA_CREATE_NEW);
		if(fresult==FR_EXIST)
			ConsolePrint("File already exists\r\n");
		f_close(&file);
	}

	if(ConsoleScanCmd("mkdir %127[^\n]", buf8)==1)
	{
		fresult = f_mkdir((TCHAR*)buf8);
		if(fresult==FR_OK)
			ConsolePrint("Created dir: %s\r\n", buf8);
	}

	if(ConsoleScanCmd("rm %127[^\n]", buf8)==1)
	{
		fresult = f_unlink((TCHAR*)buf8);
		if(fresult!=FR_OK)
			ConsolePrint("Could not remove object %d\r\n", fresult);
	}

	if(ConsoleScanCmd("logfile %127[^\n]", buf8)==1)
	{
		LogFileClose();

		uint32_t freeCluster;
		FATFS* fs;
		fresult = f_getfree("SD:", &freeCluster, &fs);
		if(fresult)
		{
			NetworkPrint("No media attached\r\n");
			ConsolePrint("No media attached\r\n");
			return;
		}

		uint32_t freeSectors = freeCluster*fs->csize;
		uint32_t freeSizeMB = freeSectors/((1024*1024)/512);

		NetworkPrint("Free Space: %uMB\n", freeSizeMB);

		f_chdrive("SD:");
		LogFileOpen((char*)buf8);
	}

	if(ConsoleCmpCmd("startlog"))
	{
		ConsolePrint("Writing log header\r\n");

		LogFileOpen(0);
	}

	if(ConsoleCmpCmd("stoplog"))
	{
		if(logFile.ready==0)
		{
			NetworkPrint("No log file open!\r\n");
			ConsolePrint("No log file open!\r\n");
		}
		else
		{
			LogFileClose();
		}
	}

	if(ConsoleCmpCmd("sd write"))
	{
		ConsolePrint("SD card write test\r\n");

//		static FIL file __attribute__((section(".dtc")));
		static FIL file;

		fresult = f_open(&file, "SD:/wrtest.txt", FA_CREATE_ALWAYS|FA_WRITE);

//		static uint8_t wrBuf[8192] __attribute__((section(".dtc")));
		static uint8_t wrBuf[8192];
		for(uint32_t i = 0; i<8192; i++)
			wrBuf[i] = i;

		for(uint32_t i = 0; i<64; i++)
		{
			UINT bytesWritten = 0;
			fresult = f_write(&file, wrBuf, 8001, &bytesWritten);
			ConsolePrint("Result: %d, %u bytes written\r\n", fresult, bytesWritten);

			chThdSleepMilliseconds(10);
		}

		f_close(&file);
	}
}

static void buzzerCommands()
{
	uint16_t u16;

	if(ConsoleCmpCmd("help buzzer"))
	{
		ConsolePrint("Help for the buzzer control:\r\n");
		ConsolePrint("\tbuzz off: disable the buzzer\r\n");
		ConsolePrint("\tbuzz %%freq : Enable the buzzer with the given frequency\r\n");
		ConsolePrint("\tbuzz beep: make a beep\r\n");
		ConsolePrint("\tbuzz doublebeep: make a double beep\r\n");
		ConsolePrint("\tbuzz tada\r\n");
		ConsolePrint("\tbuzz final: plays final countdown\r\n");
		ConsolePrint("\tbuzz cant\r\n");

		NetworkPrint("Help for the buzzer control:\r\n");
		NetworkPrint("\tbuzz off: disable the buzzer\r\n");
		NetworkPrint("\tbuzz %%freq : Enable the buzzer with the given frequency\r\n");
		NetworkPrint("\tbuzz beep: make a beep\r\n");
		NetworkPrint("\tbuzz doublebeep: make a double beep\r\n");
		NetworkPrint("\tbuzz tada\r\n");
		NetworkPrint("\tbuzz final: plays final countdown\r\n");
		NetworkPrint("\tbuzz cant\r\n");

		BuzzerPlay(0);
	}

	if(ConsoleCmpCmd("buzz off"))
	{
		ConsolePrint("Disabling buzzer\r\n");

		BuzzerPlay(0);
	}

	if(ConsoleScanCmd("buzz %hu", &u16)==1)
	{
		ConsolePrint("Buzzer freq: %hu\r\n", u16);

		BuzzerTone(u16);
	}

	if(ConsoleCmpCmd("buzz up20"))
	{
		BuzzerPlay(&buzzSeqUp20);
	}

	if(ConsoleCmpCmd("buzz up50"))
	{
		BuzzerPlay(&buzzSeqUp50);
	}

	if(ConsoleCmpCmd("buzz down50"))
	{
		BuzzerPlay(&buzzSeqDown50);
	}

	if(ConsoleCmpCmd("buzz up100"))
	{
		BuzzerPlay(&buzzSeqUp100);
	}

	if(ConsoleCmpCmd("buzz down100"))
	{
		BuzzerPlay(&buzzSeqDown100);
	}

	if(ConsoleCmpCmd("buzz updown20"))
	{
		BuzzerPlay(&buzzSeqUpDown20);
	}

	if(ConsoleCmpCmd("buzz beep"))
	{
		BuzzerPlay(&buzzSeqBeepFast);
	}

	if(ConsoleCmpCmd("buzz doublebeep"))
	{
		BuzzerPlay(&buzzSeqDoubleBeepSlow);
	}

	if(ConsoleCmpCmd("buzz tada"))
	{
		BuzzerPlay(&buzzSeqTada);
	}

	if(ConsoleCmpCmd("buzz final"))
	{
		BuzzerPlay(&buzzSeqFinalShort);
	}

	if(ConsoleCmpCmd("buzz cant"))
	{
		BuzzerPlay(&buzzSeqCant);
	}

	if(ConsoleCmpCmd("buzz elevator"))
	{
		BuzzerPlay(&buzzSeqElevator);
	}

	if(ConsoleCmpCmd("buzz eye1"))
	{
		BuzzerPlay(&buzzSeqEyeLead);
	}

	if(ConsoleCmpCmd("buzz eye2"))
	{
		BuzzerPlay(&buzzSeqEyeFollow);
	}
}

static void processCommand()
{
	buzzerCommands();
#ifndef ENV_BOOT
	miscCommands();
	fsCommands();
	motorsCommands();
	kickerCommands();
	wifiCommands();
#endif

	if(ConsoleCmpCmd("version"))
	{
		ConsolePrint(VersionGetString());
		ConsolePrint("Commit SHA1: %s\r\n", GIT_SHA1);
		ConsolePrint("Commit Date: %s\r\n", GIT_COMMIT_DATE_ISO8601);
		ConsolePrint("Checksum: 0x%08X\r\n", BootGetApplicationCRC32());
	}

	if(ConsoleCmpCmd("help"))
	{
		ConsolePrint("General help menu:\r\n");
		ConsolePrint("\thelp buzzer: an overview over buzzer commands\r\n");
		ConsolePrint("\thelp fs: an overview of filesystem commands\r\n");
		ConsolePrint("\thelp kicker: an overview of kicker commands\r\n");
		ConsolePrint("\thelp mot: an overview of motor and drive commands\r\n");
		ConsolePrint("\thelp wifi: an overview of wifi commands\r\n");
		ConsolePrint("\thelp misc: an overview of other commands\r\n");
		ConsolePrint("\tFor more information and commands see the cli.c file\r\n");

		NetworkPrint("General help menu:\r\n");
		NetworkPrint("\thelp buzzer: an overview over buzzer commands\r\n");
		NetworkPrint("\thelp fs: an overview of filesystem commands\r\n");
		NetworkPrint("\thelp kicker: an overview of kicker commands\r\n");
		NetworkPrint("\thelp mot: an overview of motor and drive command\r\n");
		NetworkPrint("\thelp wifi: an overview of wifi commands\r\n");
		NetworkPrint("\thelp misc: an overview of other commands\r\n");
		NetworkPrint("\tFor more information and commands see the cli.c file\r\n");
	}
}
