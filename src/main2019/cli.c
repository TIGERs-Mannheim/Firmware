/*
 * cli.c
 *
 *  Created on: 01.01.2019
 *      Author: AndreR
 */

#include "cli.h"

#include "robot/ctrl_motor.h"
#include "robot/ctrl_panthera.h"
#include "robot/skills.h"
#include "robot/fusion_ekf.h"

#include "hal/led.h"
#include "hal/buzzer.h"
#include "hal/i2c2.h"
#include "hal/port_ex.h"

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
#include "motors.h"
#include "kicker.h"
#include "spi4.h"
#include "power.h"
#include "commands.h"
#include "intercom_constants.h"
#include "cpu_load.h"
#include "constants.h"
#include "gfx.h"
#include "fatfs/ff.h"
#include "util/log_file.h"
#include "robot/ctrl_tigga.h"
#include "robot/robot.h"
#include "ir.h"
#include "motors.h"
#include "spi2.h"
#include "test/test.h"
#include "test/test_experimental.h"
#include "presenter.h"
#include "gui/ids.h"
#include "ext.h"
#include "pattern_ident.h"
#include "microphone.h"
#include "inventory.h"
#include "struct_ids.h"
#include "robot_pi.h"

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <math.h>

static void processCommand();

static void consoleTxCallback(void *param)
{
	(void) param;

	while(console.txFifo.numPackets > 0)
	{
		uint8_t *pData;
		uint32_t len32;
		int16_t result = FifoLinGet(&console.txFifo, &pData, &len32);
		if(result)
			break;

		TerminalWrite(pData, len32);

		FifoLinDelete(&console.txFifo);
	}
}

static void cmdCallback(char *pBuf, uint16_t length)
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

void CLITask(void *params)
{
	(void) params;

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

static void miscCommands()
{
	float f32[4];

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
		volatile uint32_t *pFault = (volatile uint32_t*) 0x55555555;
		*pFault = 42;
	}

	if(ConsoleCmpCmd("bootloader"))
	{
		ConsolePrint("Selecting bootloader\r\n");

		BootSetBootloaderSelected(1);
	}

	if(ConsoleCmpCmd("boot"))
	{
		ConsolePrint("Magic: 0x%08X\r\n", BootGetBootCode());
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
		ConsolePrint("IRQ stack free: %u\r\n", chThdGetFreeStack(0));

		CPULoadPrintTasks();
	}

	if(ConsoleCmpCmd("irqs"))
	{
		ConsolePrint("Listing active IRQs...\r\n");

		for(int32_t i = -15; i < 0; i++)
		{
			ConsolePrint("%d: %u\r\n", i, NVIC_GetPriority(i));
			chThdSleepMilliseconds(10);
		}

		for(int32_t i = 0; i < 98; i++)
		{
			uint32_t enabled = NVIC->ISER[(((uint32_t) (int32_t) i) >> 5UL)] & ((uint32_t) (1UL << (((uint32_t) (int32_t) i) & 0x1FUL)));

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
		uint8_t result = chSysIntegrityCheckI(CH_INTEGRITY_RLIST | CH_INTEGRITY_VTLIST | CH_INTEGRITY_REGISTRY | CH_INTEGRITY_PORT);
		chSysUnlock();

		if(result)
			ConsolePrint("Integrity check failed\r\n");
		else
			ConsolePrint("Integrity check OK\r\n");
	}

	if(ConsoleCmpCmd("heap"))
	{
		PresenterPrintHeapStatus();
	}

	if(ConsoleCmpCmd("ttest"))
	{
		thread_t *pThread = chRegFirstThread();
		uint32_t thdAddr = (uint32_t) pThread;

		uint8_t *pEnd = (uint8_t*) pThread->p_ctx.r13;
		uint8_t *pStack = (uint8_t*) (pThread + 1);
		while(pStack < pEnd && *pStack == CH_DBG_STACK_FILL_VALUE)
			pStack++;

		uint32_t stackFree = (uint32_t) pStack - (uint32_t) pThread;

		uint32_t endThd = ((uint32_t) pThread->p_ctx.r13) + sizeof(struct port_intctx);
		uint32_t thdSize = endThd - (uint32_t) pThread;

		ConsolePrint("%s\r\n", pThread->p_name);
		ConsolePrint("pThd: 0x%08X\r\n", thdAddr);
		ConsolePrint("Stack start: 0x%08X\r\n", thdAddr + sizeof(thread_t));
		ConsolePrint("R13: 0x%08X\r\n", (uint32_t) pThread->p_ctx.r13);
		ConsolePrint("Thd: %u, IntCtx: %u, ExtCtx: %u\r\n", sizeof(thread_t), sizeof(struct port_intctx), sizeof(struct port_extctx)); // +PORT_INT_REQUIRED_STACK
		ConsolePrint("waSize: %u\r\n", thdSize);
		ConsolePrint("free: %u\r\n", stackFree);
	}

	if(ConsoleScanCmd("led left %f %f %f %f", f32, f32 + 1, f32 + 2, f32 + 3) == 4)
	{
		ConsolePrint("Setting left LED to:\r\n");
		ConsolePrint("R: %.3f, G: %.3f, B: %.3f, W: %.3f\r\n", f32[0], f32[1], f32[2], f32[3]);

		LEDLeftSet(f32[0], f32[1], f32[2], f32[3]);
	}

	if(ConsoleScanCmd("led right %f %f %f %f", f32, f32 + 1, f32 + 2, f32 + 3) == 4)
	{
		ConsolePrint("Setting right LED to:\r\n");
		ConsolePrint("R: %.3f, G: %.3f, B: %.3f, W: %.3f\r\n", f32[0], f32[1], f32[2], f32[3]);

		LEDRightSet(f32[0], f32[1], f32[2], f32[3]);
	}

	if(ConsoleScanCmd("led %f %f %f %f", f32, f32 + 1, f32 + 2, f32 + 3) == 4)
	{
		ConsolePrint("Setting both LEDs to:\r\n");
		ConsolePrint("R: %.3f, G: %.3f, B: %.3f, W: %.3f\r\n", f32[0], f32[1], f32[2], f32[3]);

		LEDLeftSet(f32[0], f32[1], f32[2], f32[3]);
		LEDRightSet(f32[0], f32[1], f32[2], f32[3]);
	}

	if(ConsoleCmpCmd("power"))
	{
		ConsolePrint("V_bat: %fV (%huC)\r\n", power.vBat, (uint16_t) power.batCells);
		ConsolePrint("I_cur: %fA\r\n", power.iCur);
		ConsolePrint("USB powered: %hu\r\n", (uint16_t) power.usbPowered);
		ConsolePrint("Exhausted: %hu\r\n", (uint16_t) power.exhausted);
		ConsolePrint("Empty: %hu\r\n", (uint16_t) power.batEmpty);
	}

	if(ConsoleCmpCmd("i2c2"))
	{
		ConsolePrint("I2C2 test\r\n");

		uint8_t data[3] =
			{ 0x02, 0x00, 0x00 };
		int16_t result = I2CWrite(&i2c2, I2C_ADDR_TCA9539, 1, data);
		ConsolePrint("Result: %hd\r\n", result);

		result = I2CRead(&i2c2, I2C_ADDR_TCA9539, 2, data);
		ConsolePrint("Result: %hd\r\n", result);
		ConsolePrint("Out: 0x%02X 0x%02X\r\n", (uint32_t) data[0], (uint32_t) data[1]);

		data[0] = 0x06;
		result = I2CWrite(&i2c2, I2C_ADDR_TCA9539, 1, data);
		ConsolePrint("Result: %hd\r\n", result);

		result = I2CRead(&i2c2, I2C_ADDR_TCA9539, 2, data);
		ConsolePrint("Result: %hd\r\n", result);
		ConsolePrint("Cfg: 0x%02X 0x%02X\r\n", (uint32_t) data[0], (uint32_t) data[1]);
	}

	uint32_t u32;
	if(ConsoleScanCmd("srst %u", &u32) == 1)
	{
		ConsolePrint("Reset pulse on target %u\r\n", u32);
		int16_t result = PortExResetPulse(u32);
		ConsolePrint("Result: %hd\r\n", result);
	}

	if(ConsoleScanCmd("booten %u", &u32) == 1)
	{
		ConsolePrint("Enable boot pin on target %u\r\n", u32);
		int16_t result = PortExBootSet(u32, 1);
		ConsolePrint("Result: %hd\r\n", result);
	}

	if(ConsoleScanCmd("bootdis %u", &u32) == 1)
	{
		ConsolePrint("Disable boot pin on target %u\r\n", u32);
		int16_t result = PortExBootSet(u32, 0);
		ConsolePrint("Result: %hd\r\n", result);
	}

	if(ConsoleCmpCmd("pat"))
	{
		PatternIdentPrintMeasurements();
	}

	if(ConsoleCmpCmd("spi4"))
	{
		float accNorm = sqrtf(spi4.imu.acc[0]*spi4.imu.acc[0]
						+ spi4.imu.acc[1]*spi4.imu.acc[1]
						+ spi4.imu.acc[2]*spi4.imu.acc[2]);
		ConsolePrint("Acc: %.4f, %.4f, %.4f (%.4f)\r\n", spi4.imu.acc[0], spi4.imu.acc[1], spi4.imu.acc[2], accNorm);
		ConsolePrint("Gyr: %.4f, %.4f, %.4f\r\n", spi4.imu.gyr[0], spi4.imu.gyr[1], spi4.imu.gyr[2]);
		ConsolePrint("Temp: %.2f\r\n", spi4.imu.temp);
		ConsolePrint("t_imu:  %uus\r\n", spi4.imu.sampleTime);
		ConsolePrint("f_imu: %.2f\r\n", spi4.imu.updateRate);

		float magNorm = sqrtf(spi4.mag.strength[0]*spi4.mag.strength[0]
						+ spi4.mag.strength[1]*spi4.mag.strength[1]
						+ spi4.mag.strength[2]*spi4.mag.strength[2]);

		ConsolePrint("Mag: %.4f, %.4f, %.4f (%.4f)\r\n", spi4.mag.strength[0], spi4.mag.strength[1], spi4.mag.strength[2], magNorm);
		ConsolePrint("Temp: %.2f\r\n", spi4.mag.temp);
		ConsolePrint("t_mag:  %uus\r\n", spi4.mag.sampleTime);
		ConsolePrint("f_mag: %.2f\r\n", spi4.mag.updateRate);

		ConsolePrint("-- Touch --\r\n");
		ConsolePrint("Pressed: %hu\r\nX: %hu\r\nY: %hu\r\n", (uint16_t) spi4.touch.pressed, spi4.touch.x, spi4.touch.y);
		ConsolePrint("t_touch: %uus\r\n", spi4.touch.sampleTime);
	}

	if(ConsoleCmpCmd("mag"))
	{
		ConsolePrint("Mag: %.4f, %.4f, %.4f\r\n", robot.sensors.mag.strength[0], robot.sensors.mag.strength[1], robot.sensors.mag.strength[2]);
		ConsolePrint("ZMag: %.4f, ZVis: %.4f", robot.state.magZ, robot.state.pos[2]);
	}
	
	if(ConsoleCmpCmd("mic"))
	{
		ConsolePrint("CH0: % 8u\r\nCH1: % 8u\r\n",
				microphone.totalBytes[0], microphone.totalBytes[1]);
	}

	if(ConsoleCmpCmd("mic record"))
	{
		MicrophoneStartRecoding();
	}

	if(ConsoleCmpCmd("mic stop"))
	{
		MicrophoneStopRecording();
	}

	if(ConsoleCmpCmd("time"))
	{
		ConsolePrint("Time: %uus\r\n", SysTimeUSec());

		struct tm *nowtm;
		time_t now = SysTimeUnix();
		nowtm = localtime(&now);

		char tmbuf[64];
		strftime(tmbuf, sizeof tmbuf, "%Y-%m-%d %H:%M:%S", nowtm);
		ConsolePrint("%s\r\n", tmbuf);
	}

	uint16_t u16;
	if(ConsoleScanCmd("log %hu", &u16) == 1)
	{
		LogPrintAllRecent(u16);
	}

	if(ConsoleCmpCmd("ir update"))
	{
		ConsolePrint("Forcing IR Program Update\r\n");

		IrFlashProgram(1);
	}

	if(ConsoleCmpCmd("irdata"))
	{
		ConsolePrint("CPU Load: %.2f%%\r\n", ir.cpuLoad);
		ConsolePrint("vBarrier: %f\r\n", ir.vBarrier);
		ConsolePrint("vDribblerTemp: %f\r\n", ir.vDribblerTemp);
		ConsolePrint("tDribbler: %f\r\n", ir.dribblerTemperature);

		float vSum[5] = { 0 };

		ConsolePrint("-: %.3f   %.3f   %.3f   %.3f   %.3f\r\n", ir.vIrOff[1],
			ir.vIrOff[2], ir.vIrOff[3], ir.vIrOff[4], ir.vIrOff[5]);

		for(uint16_t i = 0; i < 4; i++)
		{
			ConsolePrint("%hu: %.3f %c %.3f %c %.3f %c %.3f %c %.3f\r\n", i,
				ir.irData.vLateral[i][0], i == 0 ? '*' : ' ',
				ir.irData.vLateral[i][1], i == 1 ? '*' : ' ',
				ir.irData.vLateral[i][2], i == 2 ? '*' : ' ',
				ir.irData.vLateral[i][3], i == 3 ? '*' : ' ',
				ir.irData.vLateral[i][4]);

			for(uint8_t j = 0; j < 5; j++)
				vSum[j] += ir.irData.vLateral[i][j];
		}
		ConsolePrint("S: %.3f * %.3f * %.3f * %.3f * %.3f\r\n", vSum[0],
				vSum[1], vSum[2], vSum[3], vSum[4]);
		ConsolePrint("Estimated Ball Position: [ %07.3f / %07.3f ]\r\n", ir.irData.estimatedBallPosition[0], ir.irData.estimatedBallPosition[1]);
	}

	if(ConsoleCmpCmd("irrec"))
	{
		for(uint16_t i = 0; i < 4; i++)
		{
			ConsolePrint("%.3f %.3f %.3f %.3f %.3f   ",
				ir.irData.vLateral[i][0],
				ir.irData.vLateral[i][1],
				ir.irData.vLateral[i][2],
				ir.irData.vLateral[i][3],
				ir.irData.vLateral[i][4]);
		}

		ConsolePrint("\r\n");
	}

	if(ConsoleCmpCmd("irstate"))
	{
		ConsolePrint("State: %u\r\n", robot.state.ballIrState);
		ConsolePrint("Pos: %.3f, %.3f\r\n", robot.state.ballIrPos[0], robot.state.ballIrPos[1]);
	}

	if(ConsoleCmpCmd("ext"))
	{
		ConsolePrint("--- UART6 ---\r\n");
		ConsolePrint("RX payload/wire/usage: %u / %u / %3.2f%%\r\n", ext.stats.rxPayload, ext.stats.rxWire, ext.stats.rxUsage * 100.0f);
		ConsolePrint("TX payload/wire/usage: %u / %u / %3.2f%%\r\n", ext.stats.txPayload, ext.stats.txWire, ext.stats.txUsage * 100.0f);

		ConsolePrint("lastMatchCtrl time: %uus\r\n", robot.ext.lastMatchCtrlTime);
		ConsolePrint("Ext. control interface active: %hu\r\n", robot.ext.active);
		ConsolePrint("Installed: %hu\r\n", (uint16_t)ext.installed);
	}

	if(ConsoleCmpCmd("flash list"))
	{
		FlashFSListFiles();
	}

	if(ConsoleCmpCmd("flash erase"))
	{
		ConsolePrint("Erasing flash\r\n");

		FlashErase(flashFS.flashAddr, flashFS.flashAddr + flashFS.flashSize);
	}

	if(ConsoleCmpCmd("flash compact"))
	{
		FlashFSCompact();

		ConsolePrint("Flash compacted\r\n");
	}

	if(ConsoleCmpCmd("flash hex"))
	{
		for(uint32_t i = 0; i < 48; i += 8)
		{
			uint8_t *d = (uint8_t*) (flashFS.flashAddr + i);
			ConsolePrint("0x%04X: %02X %02X %02X %02X %02X %02X %02X %02X\r\n", i, d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7]);

			chThdSleepMilliseconds(5);
		}
	}

	if(ConsoleCmpCmd("flash test"))
	{
		uint32_t buf[16];
		memset(buf, 0x42, 16 * sizeof(uint32_t));
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

	if(ConsoleCmpCmd("configs"))
	{
		for(uint16_t i = 0; i < config.filesUsed; i++)
		{
			ConsolePrint("%hu: %s\r\n", i, config.files[i].pDesc->pName);
		}
	}

	if(ConsoleCmpCmd("ref"))
	{
		arm_matrix_instance_f32 refPos =
			{ 3, 1, robot.ctrlRef.trajPos };
		arm_matrix_instance_f32 refVel =
			{ 3, 1, robot.ctrlRef.trajVelLocal };
		arm_matrix_instance_f32 refAcc =
			{ 3, 1, robot.ctrlRef.trajAccLocal };

		ConsolePrint("refPos:\r\n");
		arm_mat_print(&refPos);

		ConsolePrint("refVelLocal:\r\n");
		arm_mat_print(&refVel);

		ConsolePrint("refAccLocal:\r\n");
		arm_mat_print(&refAcc);
	}

	if(ConsoleCmpCmd("est"))
	{
		arm_matrix_instance_f32 mechState =
			{ 8, 1, ctrlTigga.mechState };
		arm_matrix_instance_f32 encState =
			{ 8, 1, ctrlTigga.encState };

		arm_matrix_instance_f32 trajFilter =
			{ 6, 1, ctrlTigga.trajGen.filterGlobalPos };
		arm_matrix_instance_f32 trajGlobal =
			{ 9, 1, ctrlTigga.trajGen.trajGlobalPos };
		arm_matrix_instance_f32 trajLocal =
			{ 9, 1, ctrlTigga.trajGen.trajLocalVel };

		ConsolePrint("mechState:\r\n");
		arm_mat_print(&mechState);

		ConsolePrint("encState:\r\n");
		arm_mat_print(&encState);

		ConsolePrint("trajFilter:\r\n");
		arm_mat_print(&trajFilter);

		ConsolePrint("trajGlobal:\r\n");
		arm_mat_print(&trajGlobal);

		ConsolePrint("trajLocal:\r\n");
		arm_mat_print(&trajLocal);

		ConsolePrint("trajMode: %hu / %hu\r\n", (uint16_t) ctrlTigga.trajGen.modeXY, (uint16_t) ctrlTigga.trajGen.modeW);

		ConsolePrint("Vision: %f, %f, %f, %hu\r\n", robot.sensors.vision.pos[0], robot.sensors.vision.pos[1], robot.sensors.vision.pos[2],
				(uint16_t) robot.sensors.vision.updated);

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

	if(ConsoleCmpCmd("est2"))
	{
		ConsolePrint("x:\r\n");
		arm_mat_print(&fusionEKF.ekf.x);

		ConsolePrint("z:\r\n");
		arm_mat_print(&fusionEKF.ekf.z);

		ConsolePrint("Sigma:\r\n");
		arm_mat_print(&fusionEKF.ekf.Sigma);

		ConsolePrint("Ex:\r\n");
		arm_mat_print(&fusionEKF.ekf.Ex);

		ConsolePrint("timeSlot: %u\r\n", fusionEKF.timeSlotNow);
		ConsolePrint("On vision: %hu\r\n", fusionEKF.vision.online);
		ConsolePrint("tLastValidSample: %u\r\n", fusionEKF.vision.timeLastValidSample);
		ConsolePrint("late meas: %u\r\n", fusionEKF.vision.numLateMeasurements);
		ConsolePrint("No vision in network: %u\r\n", robot.sensors.vision.noVision);

		ConsolePrint("state pos: %.3f, %.3f, %.3f\r\n", robot.state.pos[0], robot.state.pos[1], robot.state.pos[2]);
		ConsolePrint("state vel: %.3f, %.3f, %.3f\r\n", robot.state.vel[0], robot.state.vel[1], robot.state.vel[2]);
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

	if(ConsoleCmpCmd("inject network"))
	{
		RobotEnableNetwork();
	}

	if(ConsoleScanCmd("test mode %hu", &u16) == 1)
	{
		ConsolePrint("Setting robot test mode: %hu\r\n", u16);

		RobotTestModeEnable(u16);
	}

	if(ConsoleScanCmd("set systems %X", &u32) == 1)
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
		ConsolePrint("Skill: %hu\r\n", (uint16_t) skills.input.skillId);
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

	if(ConsoleCmpCmd("calib compass"))
	{
		TestScheduleCompassCalib();
	}

	if(ConsoleCmpCmd("calib imu"))
	{
		TestScheduleImuCalib();
	}

	if(ConsoleCmpCmd("screenshot"))
	{
		PresenterTakeScreenshot();
	}

	if(ConsoleCmpCmd("cpuid"))
	{
		volatile uint32_t* pCPUID = (volatile uint32_t*)0x1FF1E800;
		ConsolePrint("CPUID: 0x%08X 0x%08X 0x%08X\r\n", pCPUID[0], pCPUID[1], pCPUID[2]);

		uint32_t rev = (SCB->CPUID >> 20) & 0x07;
		uint32_t patch = SCB->CPUID & 0x07;
		ConsolePrint("Cortex-M7 version: r%up%u\r\n", rev, patch);

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

		ConsolePrint("Revision: 0x%04hX => %s\r\n", revId, pIdName);
	}

	if(ConsoleCmpCmd("calibration"))
	{
		const InventoryEntry* pEntry = InventoryGetEntry();
		if(pEntry)
		{
			ConsolePrint("HW ID: %u\r\n", pEntry->hwId);

			const InventoryImuCalibration* pCalib = &pEntry->imuCalib;
			if(pCalib->calibrated)
			{
				ConsolePrint("\r\n--- IMU ---\r\n");
				ConsolePrint("Gyro bias: %.6f, %.6f, %.6f\r\n",
					pCalib->gyrBias[0], pCalib->gyrBias[1], pCalib->gyrBias[2]);
				ConsolePrint("Acc bias:  %.6f, %.6f, %.6f\r\n",
					pCalib->accBias[0], pCalib->accBias[1], pCalib->accBias[2]);
				ConsolePrint("MB tilt bias: %.6f, %.6f\r\n",
					pCalib->accTiltBias[0], pCalib->accTiltBias[1]);
				ConsolePrint("Calibrated at %.2f\u00b0C\r\n", pCalib->imuCalibTemp);
				ConsolePrint("\r\n--- MAG ---\r\n");
				ConsolePrint("Bias: %.6f, %.6f, %.6f\r\n",
					pCalib->magBias[0], pCalib->magBias[1], pCalib->magBias[2]);
				ConsolePrint("Calibrated at %.2f\u00b0C\r\n", pCalib->magCalibTemp);
			}
			else
			{
				ConsolePrint("No calibration data present!\r\nPlease run 'calib imu' and update inventory.\r\n");
			}
		}
		else
		{
			ConsolePrint("This robot is not listed in the inventory!\r\n");
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
		ConsolePrint("Avg:  %.5f\r\n", network.avgRxPeriod.value);
		ConsolePrint("RSSI: %.3f\r\n", network.avgRssi.value);

		RFQueueStats *pStat = &network.queue.stats;

		ConsolePrint(" %2hu        toBS     fromBS     rxLoss     txLoss\r\n", (uint16_t) network.config.botId);
		ConsolePrint("LL P %10u %10u %10u\r\n", pStat->rfIO.txPackets, pStat->rfIO.rxPackets, pStat->rfFeedback.rxPacketsLost);
		ConsolePrint("   B %10u %10u\r\n", pStat->rfIO.txBytes, pStat->rfIO.rxBytes);
		ConsolePrint("HL P %10u %10u %10u %10u\r\n", pStat->queueIO.txPackets, pStat->queueIO.rxPackets, pStat->queueFeedback.rxPacketsLost,
				pStat->queueFeedback.txPacketsLost);
		ConsolePrint("   B %10u %10u\r\n", pStat->queueIO.txBytes, pStat->queueIO.rxBytes);
	}

	if(ConsoleCmpCmd("color b"))
	{
		if(network.config.botId < CMD_BOT_COUNT_HALF)
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
		if(network.config.botId > CMD_BOT_HALF_MIN_1)
		{
			network.config.botId -= CMD_BOT_COUNT_HALF;
			NetworkSaveConfig();
			NetworkPrint("Changed BotID to %hu", network.config.botId);
			chThdSleepMilliseconds(100);
			BootReset();
		}
	}

	if(ConsoleScanCmd("set botid %hu", &u16) == 1)
	{
		network.config.botId = u16;
		NetworkSaveConfig();

		NetworkPrint("Changed BotID to %hu", u16);
		ConsolePrint("Changed BotID to %hu\r\n", u16);
	}

	if(ConsoleScanCmd("write botid %hu", &u16) == 1)
	{
		network.config.botId = u16;
		NetworkSaveConfig();

		NetworkPrint("Changed BotID to %hu", u16);
		ConsolePrint("Changed BotID to %hu\r\n", u16);
	}

	if(ConsoleScanCmd("set channel %hu", &u16) == 1)
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
		ConsolePrint("HWID: %hu\r\n", (uint16_t) RobotImplGetHardwareId());
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

	if(ConsoleScanCmd("vmax %hu", &u16) == 1)
	{
		ConsolePrint("vMax: %hu\r\n", u16);

		KickerSetMaxVoltage(u16);
	}

	if(ConsoleScanCmd("chg %hu", &u16) == 1)
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

	if(ConsoleScanCmd("autochg %hu", &u16) == 1)
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

	if(ConsoleScanCmd("kstress %f", &f32) == 1)
	{
		ConsolePrint("Setting auto-fire stress test: %fms\r\n", f32);

		kicker.stress = f32 * 0.001f;
	}

	if(ConsoleScanCmd("dis1 %f", &f32) == 1)
	{
		ConsolePrint("Fire1: %fms\r\n", f32);

		KickerFire(f32 * 0.001f, KICKER_DEVICE_STRAIGHT);
	}

	if(ConsoleScanCmd("dis1s %f", &f32) == 1)
	{
		ConsolePrint("Fire1: %fm/s\r\n", f32);

		KickerFireSpeed(f32, KICKER_DEVICE_STRAIGHT);
	}

	if(ConsoleScanCmd("arm1 %f", &f32) == 1)
	{
		ConsolePrint("Arm1: %fms\r\n", f32);

		KickerArm(f32 * 0.001f, KICKER_DEVICE_STRAIGHT);
	}

	if(ConsoleScanCmd("dis2 %f", &f32) == 1)
	{
		ConsolePrint("Fire2: %fms\r\n", f32);

		KickerFire(f32 * 0.001f, KICKER_DEVICE_CHIP);
	}

	if(ConsoleScanCmd("dis2s %f", &f32) == 1)
	{
		ConsolePrint("Fire2: %fm/s\r\n", f32);

		KickerFireSpeed(f32, KICKER_DEVICE_CHIP);
	}

	if(ConsoleScanCmd("arm2 %f", &f32) == 1)
	{
		ConsolePrint("Arm2: %fms\r\n", f32);

		KickerArm(f32 * 0.001f, KICKER_DEVICE_CHIP);
	}

	if(ConsoleCmpCmd("kstat"))
	{
		ConsolePrint("Installed: %hu, v2017: %hu\r\n", (uint16_t) kicker.installed, (uint16_t) kicker.v2017);
		ConsolePrint("Cap: %.3fV (%.1fV)\r\n", kicker.vCap, kicker.config.maxVoltage);
		ConsolePrint("Temp:%.2fC\r\n", kicker.tEnv);
		ConsolePrint("IR: %.3f / %.3f (%.3f / %.3f)\r\n", kicker.vIrOff, kicker.vIrOn, kicker.emaIrOff.value, kicker.emaIrOn.value);
		ConsolePrint("State: %hu, Auto: %hu, Cnt: %hu\r\n", (uint16_t) kicker.state, (uint16_t) kicker.autoCharge, (uint16_t) kicker.kickCounter);
		ConsolePrint("IR TX / IR RX / Chip / Straight: %hu / %hu / %hu / %hu\r\n",
				kicker.barrierTxDamaged, kicker.barrierRxDamaged, kicker.chipDamaged, kicker.straightDamaged);
		ConsolePrint("Last kick usage: %.3fV\r\n", kicker.lastCapUsage);
		ConsolePrint("tSample: %u\r\n", spi2.kicker.sampleTime);
	}

	if(ConsoleScanCmd("cooldown %u", &u32) == 1)
	{
		KickerSetCooldown(u32);

		ConsolePrint("Kicker Cooldown set to %ums\r\n", kicker.config.cooldown);
	}
}

static void motorsCommands()
{
	uint16_t u16[4];
	float f32[4];

	if(ConsoleScanCmd("mot data %hu", u16) == 1)
	{
		if(u16[0] > 4)
			return;

		MotorsSingle* pMotor = &motors.motor[u16[0]];

		ConsolePrint("CPU: %.3f%%, Temp: %.1fC, vSupply: %.3fV\r\n", pMotor->cpuLoad, pMotor->avgTemperature1ms, pMotor->avgSupplyVoltage1ms);
		ConsolePrint("vDQ: %.3fV / %.3fV\r\n", pMotor->avgVoltageDQ1ms[0], pMotor->avgVoltageDQ1ms[1]);
		ConsolePrint("iUVW: %.3fA / %.3fA / %.3fA\r\n", pMotor->avgCurrentUVW1ms[0], pMotor->avgCurrentUVW1ms[1], pMotor->avgCurrentUVW1ms[2]);
		ConsolePrint("iDQ: %.3fA / %.3fA\r\n", pMotor->avgCurrentDQ1ms[0], pMotor->avgCurrentDQ1ms[1]);
		ConsolePrint("Hall Pos: %hu, comTime: %d, invTran: %hu\r\n", (uint16_t) pMotor->miso.hallPos,
				pMotor->miso.hallComTime, pMotor->miso.hallInvalidTransitions);
		chThdSleepMilliseconds(20);
		ConsolePrint("EncPos: %hu, delta: %hd, cor: %hd, ticks: %hu, psc: %hu\r\n", pMotor->miso.encPos,
				pMotor->miso.encDelta, pMotor->miso.encCorrection, pMotor->miso.encTicks, pMotor->miso.encPrescaler);
		ConsolePrint("Speed: enc: %.4f, hall: %.4f\r\n", pMotor->encoderVelocity, pMotor->hallVelocity);
		ConsolePrint("Flags: 0x%hX\r\n", pMotor->miso.flags);
	}

	if(ConsoleScanCmd("mot hall %hu %hu", u16, u16+1) == 2)
	{
		ConsolePrint("Setting hall only on motor %hu to %hu\r\n", u16[0], u16[1]);

		motors.motor[u16[0]].hallOnly = u16[1];
	}

	if(ConsoleScanCmd("mot record %hu %hu", u16, u16+1) == 2)
	{
		ConsolePrint("Recording 0.5s of data on motor %hu, mode: %hu\r\n", u16[0], u16[1]);
		MotorsDebugRecord(u16[0], u16[1]);
	}

	if(ConsoleScanCmd("mot stepd %hu %f", u16, f32) == 2)
	{
		ConsolePrint("Step on motor %hu to %.3fV\r\n", u16[0], f32[0]);
//		MotorsDebugRecord(u16[0], MOTOR_EXCHANGE_MOSI_RECORD_VOLT_AB_CUR_DQ);
//		MotorsDebugRecord(u16[0], MOTOR_EXCHANGE_MOSI_RECORD_CUR_DQ);
		MotorsDebugRecord(u16[0], MOTOR_EXCHANGE_MOSI_RECORD_CUR_Q_SET_MEAS);

		chThdSleepMilliseconds(50);

//		MotorsTestSetAngle(u16[0], 0 * M_PI / 180.0f, f32[0]);
//		MotorsSetCurrentDQ(u16[0], 0.0f, f32[0]);
		MotorsSetVelocity(u16[0], f32[0], 0.0f, 0.0f);

		chThdSleepMilliseconds(1000);

		MotorsSetOff(u16[0]);
	}

	if(ConsoleCmpCmd("mot print record"))
	{
		MotorsDebugPrintRecording();
	}

	if(ConsoleScanCmd("drv curd pi %f %f", f32, f32 + 1) == 2)
	{
		MotorsSetPIGainsCurrentDDrive(f32[0], f32[1]);
		ConsolePrint("Drive motors current D gains. P: %.3f, I: %.3f\r\n", motors.focConfig.driveDKp, motors.focConfig.driveDKi);
	}

	if(ConsoleScanCmd("drv curq pi %f %f", f32, f32 + 1) == 2)
	{
		MotorsSetPIGainsCurrentQDrive(f32[0], f32[1]);
		ConsolePrint("Drive motors current Q gains. P: %.3f, I: %.3f\r\n", motors.focConfig.driveQKp, motors.focConfig.driveQKi);
	}

	if(ConsoleScanCmd("drv vel pi %f %f", f32, f32 + 1) == 2)
	{
		MotorsSetPIGainsVelocityDrive(f32[0], f32[1]);
		ConsolePrint("Drive motors velocity gains. P: %.3f, I: %.3f\r\n", motors.velConfig.driveKp, motors.velConfig.driveKi);
	}

	if(ConsoleScanCmd("drib curd pi %f %f", f32, f32 + 1) == 2)
	{
		MotorsSetPIGainsCurrentDDribbler(f32[0], f32[1]);
		ConsolePrint("Dribbler current D gains. P: %.3f, I: %.3f\r\n", motors.focConfig.dribblerDKp, motors.focConfig.dribblerDKi);
	}

	if(ConsoleScanCmd("drib curq pi %f %f", f32, f32 + 1) == 2)
	{
		MotorsSetPIGainsCurrentQDribbler(f32[0], f32[1]);
		ConsolePrint("Dribbler current Q gains. P: %.3f, I: %.3f\r\n", motors.focConfig.dribblerQKp, motors.focConfig.dribblerQKi);
	}

	if(ConsoleScanCmd("drib vel pi %f %f", f32, f32 + 1) == 2)
	{
		MotorsSetPIGainsVelocityDribbler(f32[0], f32[1]);
		ConsolePrint("Dribbler motor velocity gains. P: %.3f, I: %.3f\r\n", motors.velConfig.dribblerKp, motors.velConfig.dribblerKi);
	}

	if(ConsoleCmpCmd("mot config"))
	{
		ConsolePrint("=== Drive Motors ===\r\n");
		ConsolePrint("Current D: P: %6.3f, I: %6.3f\r\n", motors.focConfig.driveDKp, motors.focConfig.driveDKi);
		ConsolePrint("Current Q: P: %6.3f, I: %6.3f\r\n", motors.focConfig.driveQKp, motors.focConfig.driveQKi);
		ConsolePrint("Velocity:  P: %6.3f, I: %6.3f\r\n", motors.velConfig.driveKp, motors.velConfig.driveKi);
		ConsolePrint("=== Dribbler ===\r\n");
		ConsolePrint("Current D: P: %6.3f, I: %6.3f\r\n", motors.focConfig.dribblerDKp, motors.focConfig.dribblerDKi);
		ConsolePrint("Current Q: P: %6.3f, I: %6.3f\r\n", motors.focConfig.dribblerQKp, motors.focConfig.dribblerQKi);
		ConsolePrint("Velocity:  P: %6.3f, I: %6.3f\r\n", motors.velConfig.dribblerKp, motors.velConfig.dribblerKi);
	}

	if(ConsoleScanCmd("mot kill %hu", u16) == 1)
	{
		ConsolePrint("Stop comm to motor %hu\r\n", u16[0]);

		motors.commKillMotor = u16[0];
	}

	if(ConsoleCmpCmd("mot off"))
	{
		ConsolePrint("Motors off\r\n");

		for(uint8_t i = 0; i < 5; i++)
			MotorsSetOff(i);
	}

	if(ConsoleScanCmd("mot angle %hu %f %f", u16, f32, f32 + 1) == 3)
	{
		ConsolePrint("Motor %hu angle: %.3fdeg, %.3fV\r\n", u16[0], f32[0], f32[1]);
		MotorsSetElectricalAngle(u16[0], f32[0] * M_PI / 180.0f, f32[1]);
	}

	if(ConsoleScanCmd("mot vol %hu %f", u16, f32) == 2)
	{
		ConsolePrint("Motor %hu: %.3fV\r\n", u16[0], f32[0]);
		MotorsSetVoltageDQ(u16[0], 0.0f, f32[0]);
	}

	if(ConsoleScanCmd("mot vol all %f", f32) == 1)
	{
		ConsolePrint("Setting all motor velocities to: %.3fV\r\n", f32[0]);

		for(uint8_t i = 0; i < 4; i++)
			MotorsSetVoltageDQ(i, 0.0f, f32[0]);
	}

	if(ConsoleScanCmd("mot voldq %hu %f %f", u16, f32, f32+1) == 3)
	{
		ConsolePrint("Motor %hu: D: %.3fV Q: %.3f\r\n", u16[0], f32[0], f32[1]);
		MotorsSetVoltageDQ(u16[0], f32[0], f32[1]);
	}

	if(ConsoleScanCmd("mot vel %hu %f", u16, f32) == 2)
	{
		ConsolePrint("Vel %hu: %.3frad/s\r\n", u16[0], f32[0]);

		MotorsSetVelocity(u16[0], f32[0], 0.0f, 0.0f);
	}

	if(ConsoleScanCmd("mot velq %hu %f %f", u16, f32, f32+1) == 3)
	{
		ConsolePrint("Vel %hu: %.3frad/s, Q: %.3f\r\n", u16[0], f32[0], f32[1]);

		MotorsSetVelocity(u16[0], f32[0], 0.0f, f32[1]);
	}

	if(ConsoleScanCmd("mot cur %hu %f", u16, f32) == 2)
	{
		ConsolePrint("Current %hu: %.3f\r\n", u16[0], f32[0]);

		MotorsSetCurrentDQ(u16[0], 0.0f, f32[0]);
	}

	if(ConsoleScanCmd("mot curdq %hu %f %f", u16, f32) == 3)
	{
		ConsolePrint("Current %hu: D: %.3fA, Q: %.3fA\r\n", u16[0], f32[0], f32[1]);

		MotorsSetCurrentDQ(u16[0], f32[0], f32[1]);
	}

	if(ConsoleScanCmd("mot turn %hu %f", u16, f32) == 2)
	{
		ConsolePrint("Turning motor %hu at %.3fV...\r\n", u16[0], f32[0]);

		uint16_t activeMotor = u16[0];

		uint8_t lastHall = motors.motor[activeMotor].miso.hallPos;
		uint16_t lastEnc = motors.motor[activeMotor].miso.encPos;
		float lastAngle = 0;

		for(uint8_t turn = 0; turn < 8; turn++)
		{
			for(float angle = 0.0f; angle < 360.0f; angle += 0.1f)
			{
				MotorsSetElectricalAngle(activeMotor, angle * M_PI / 180.0f, f32[0]);

				chThdSleep(1);

				if(motors.motor[activeMotor].miso.hallPos != lastHall)
				{
					ConsolePrint("Hall %hu => %hu @ %.2f (%d, %.2f)\r\n", (uint16_t) lastHall, (uint16_t) motors.motor[activeMotor].miso.hallPos, angle,
							(int32_t) motors.motor[activeMotor].miso.encPos - (int32_t) lastEnc, angle - lastAngle);
					lastHall = motors.motor[activeMotor].miso.hallPos;
					lastEnc = motors.motor[activeMotor].miso.encPos;
					lastAngle = angle;
				}
			}
		}

		ConsolePrint("reverse\r\n");

		for(uint8_t turn = 0; turn < 8; turn++)
		{
			for(float angle = 360.0f; angle > 0.0f; angle += -0.1f)
			{
				MotorsSetElectricalAngle(activeMotor, angle * M_PI / 180.0f, f32[0]);

				chThdSleep(1);

				if(motors.motor[activeMotor].miso.hallPos != lastHall)
				{
					ConsolePrint("Hall %hu => %hu @ %.2f (%d, %.2f)\r\n", (uint16_t) lastHall, (uint16_t) motors.motor[activeMotor].miso.hallPos, angle,
							(int32_t) motors.motor[activeMotor].miso.encPos - (int32_t) lastEnc, angle - lastAngle);
					lastHall = motors.motor[activeMotor].miso.hallPos;
					lastEnc = motors.motor[activeMotor].miso.encPos;
					lastAngle = angle;
				}
			}
		}

		ConsolePrint("done\r\n");

		MotorsSetOff(activeMotor);
	}

	if(ConsoleScanCmd("mot test %hu", u16) == 1)
	{
		TestScheduleMotorTest(IC_TEST_MOT_IDENT_ELEC, u16[0]);
		ConsolePrint("Scheduled electrical identification test on motor %hu \n\r", u16[0]);

		TestScheduleMotorTest(IC_TEST_MOT_IDENT_MECH, u16[0]);
		ConsolePrint("Scheduled mechanical identification test on motor %hu \n\r", u16[0]);

		PresenterShowWindow(GUI_WINDOW_TEST_DRIVE);
	}

	if(ConsoleCmpCmd("mot test all"))
	{
		for(int i = 0; i < 5; i++)
		{
			TestScheduleMotorTest(IC_TEST_MOT_IDENT_ELEC, i);
			ConsolePrint("Scheduled electrical identification test on motor %hu \n\r", i);

			TestScheduleMotorTest(IC_TEST_MOT_IDENT_MECH, i);
			ConsolePrint("Scheduled mechanical identification test on motor %hu \n\r", i);
		}

		PresenterShowWindow(GUI_WINDOW_TEST_DRIVE);
	}

	if(ConsoleScanCmd("mot test elec %hu", u16) == 1)
	{
		TestScheduleMotorTest(IC_TEST_MOT_IDENT_ELEC, u16[0]);
		ConsolePrint("Scheduled electrical identification test on motor %hu \n\r", u16[0]);
	}

	if(ConsoleScanCmd("mot test mech %hu", u16) == 1)
	{
		TestScheduleMotorTest(IC_TEST_MOT_IDENT_MECH, u16[0]);
		ConsolePrint("Scheduled mechanical identification test on motor %hu \n\r", u16[0]);
	}

	if(ConsoleCmpCmd("mot test ident all"))
	{
		TestScheduleMotorTest(IC_TEST_MOT_IDENT_ALL, 0);
	}

	if(ConsoleScanCmd("mot test res %hu", u16) == 1)
	{
		TestScheduleMotorTest(IC_TEST_MOT_PHASE_RESISTANCE, u16[0]);
	}

	if(ConsoleCmpCmd("mot traction"))
	{
		TestScheduleMotorTest(IC_TEST_MOT_TRACTION, 0);

		ConsolePrint("Scheduled traction test\n\r");
	}

	if(ConsoleCmpCmd("mot test rot"))
	{
		TestScheduleMotorTest(IC_TEST_ROTATION_IDENT, 0);
	}

	if(ConsoleScanCmd("drib rot %f %f", f32, f32+1) == 2)
	{
		ConsolePrint("Doing dribbling rotation with %.2fA at %.2fRPM\r\n", f32[0], f32[1]);

		TestDribbleRotation(f32[0], f32[1]);
	}

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

	if(ConsoleCmpCmd("ctrl panthera"))
	{
		CtrlSetController(&ctrlPantheraInstance);

		ConsolePrint("Controller: Panthera\r\n");
	}

	if(ConsoleCmpCmd("skill off"))
	{
		uint8_t data[SYSTEM_MATCH_CTRL_USER_DATA_SIZE];

		SkillsSetInput(0, data);
	}

	if(ConsoleScanCmd("move %f %f %f", f32, f32 + 1, f32 + 2) == 3)
	{
		uint8_t data[SYSTEM_MATCH_CTRL_USER_DATA_SIZE];
		int16_t *p16Data = (int16_t*) data;
		p16Data[0] = (int16_t) (f32[0] * 1000.0f);
		p16Data[1] = (int16_t) (f32[1] * 1000.0f);
		p16Data[2] = (int16_t) (f32[2] * 1000.0f);

		SkillsSetInput(1, data);
	}

	if(ConsoleScanCmd("goto %f %f %f", f32, f32 + 1, f32 + 2) == 3)
	{
		uint8_t data[SYSTEM_MATCH_CTRL_USER_DATA_SIZE];
		int16_t *p16Data = (int16_t*) data;
		p16Data[0] = (int16_t) (f32[0] * 1000.0f);
		p16Data[1] = (int16_t) (f32[1] * 1000.0f);
		p16Data[2] = (int16_t) (f32[2] * 1000.0f);
		p16Data[3] = 0;

		SkillsSetInput(3, data);
	}

	if(ConsoleCmpCmd("mot sampling"))
	{
		TrajSecOrder1D traj;
		float U;

		for(float v = 1.0f; v < 10.0f; v += 1.0f)
		{
			TrajSecOrder1DCreate(&traj, v - 1, 0, v, 10, 100);
			float tEnd = TrajSecOrder1DGetTotalTime(&traj);

			for(float t = 0; t < tEnd + 0.1f; t += 0.001f)
			{
				TrajSecOrder1DValuesAtTime(&traj, t, &U, 0, 0);

				for(uint8_t i = 0; i < 4; i++)
					MotorsSetVoltageDQ(i, 0.0f, U);

				chThdSleepMilliseconds(1);
			}

			chThdSleepMilliseconds(1000);
		}

		for(float v = 9.0f; v > 0.0f; v -= 1.0f)
		{
			TrajSecOrder1DCreate(&traj, v + 1, 0, v, 10, 100);
			float tEnd = TrajSecOrder1DGetTotalTime(&traj);

			for(float t = 0; t < tEnd + 0.1f; t += 0.001f)
			{
				TrajSecOrder1DValuesAtTime(&traj, t, &U, 0, 0);

				for(uint8_t i = 0; i < 4; i++)
					MotorsSetVoltageDQ(i, 0.0f, U);

				chThdSleepMilliseconds(1);
			}

			chThdSleepMilliseconds(1000);
		}

		for(uint8_t i = 0; i < 4; i++)
			MotorsSetVoltageDQ(i, 0, 0);
	}

	if(ConsoleScanCmd("mot mv %f %f", f32, f32 + 1) == 2)
	{
		ConsolePrint("Move. Angle: %.2f Volt: %.2f\r\n", f32[0], f32[1]);

		float angle = f32[0] * M_PI / 180.0f;

		arm_matrix_instance_f32 move = { 3, 1, (float[3]){} };
		move.pData[0] = arm_cos_f32(angle);
		move.pData[1] = arm_sin_f32(angle);
		move.pData[2] = 0;

		arm_matrix_instance_f32 volt = { 4, 1, (float[4]){} };

		arm_mat_mult_f32(&ctrl.matXYW2Motor, &move, &volt);

		TrajSecOrder1D traj;

		TrajSecOrder1DCreate(&traj, 0, 0, f32[1], 10, 100);
		float tEnd = TrajSecOrder1DGetTotalTime(&traj);

		float U;
		for(float t = 0; t < tEnd + 0.1f; t += 0.001f)
		{
			TrajSecOrder1DValuesAtTime(&traj, t, &U, 0, 0);

			MotorsSetVoltageDQ(0, 0, volt.pData[0] * U);
			MotorsSetVoltageDQ(1, 0, volt.pData[1] * U);
			MotorsSetVoltageDQ(2, 0, volt.pData[2] * U);
			MotorsSetVoltageDQ(3, 0, volt.pData[3] * U);

			chThdSleepMilliseconds(1);
		}

		chThdSleepMilliseconds(1000);

		TrajSecOrder1DCreate(&traj, f32[1], 0, 0, 10, 100);
		tEnd = TrajSecOrder1DGetTotalTime(&traj);

		for(float t = 0; t < tEnd + 0.1f; t += 0.001f)
		{
			TrajSecOrder1DValuesAtTime(&traj, t, &U, 0, 0);

			MotorsSetVoltageDQ(0, 0, volt.pData[0] * U);
			MotorsSetVoltageDQ(1, 0, volt.pData[1] * U);
			MotorsSetVoltageDQ(2, 0, volt.pData[2] * U);
			MotorsSetVoltageDQ(3, 0, volt.pData[3] * U);

			chThdSleepMilliseconds(1);
		}

		for(uint8_t i = 0; i < 4; i++)
			MotorsSetVoltageDQ(i, 0, 0);
	}
}

static void fsCommands()
{
	FRESULT fresult;
	static const uint16_t bufLen = 128;
	static uint8_t buf8[128];

	if(ConsoleScanCmd("chd %127[^\n]", buf8) == 1)
	{
		f_chdrive((TCHAR*) buf8);

		f_getcwd((TCHAR*) buf8, bufLen);

		ConsolePrint("%s\r\n", buf8);
	}

	if(ConsoleCmpCmd("pwd"))
	{
		f_getcwd((TCHAR*) buf8, bufLen);

		ConsolePrint("%s\r\n", buf8);
	}

	if(ConsoleScanCmd("stat %127[^\n]", buf8) == 1)
	{
		FILINFO info;
		fresult = f_stat((TCHAR*) buf8, &info);

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
		finfo.lfname = (TCHAR*) buf8;
		finfo.lfsize = bufLen - 1;

		f_getcwd((TCHAR*) buf8, bufLen);

		ConsolePrint("Content of: %s\r\n", buf8);

		// show folders
		fresult = f_opendir(&dir, "");
		if(fresult != FR_OK)
			return;

		do
		{
			fresult = f_readdir(&dir, &finfo);

			if(finfo.fname[0] == 0)
				break;

			if(finfo.fattrib & AM_DIR)
			{
				ConsolePrint("<DIR>      %s\r\n", *finfo.lfname ? finfo.lfname : finfo.fname);
			}
		}
		while(fresult == FR_OK);

		f_closedir(&dir);

		// show files
		fresult = f_opendir(&dir, "");
		if(fresult != FR_OK)
			return;

		do
		{
			fresult = f_readdir(&dir, &finfo);

			if(finfo.fname[0] == 0)
				break;

			if((finfo.fattrib & AM_DIR) == 0)
			{
				if(finfo.fsize > 1024 * 1024 * 16)	// larger than 16M?
					ConsolePrint("%7uM   %s\r\n", finfo.fsize / (1024 * 1024), *finfo.lfname ? finfo.lfname : finfo.fname);
				else if(finfo.fsize > 1024 * 16)	// larget than 16k?
					ConsolePrint("%7uk   %s\r\n", finfo.fsize / 1024, *finfo.lfname ? finfo.lfname : finfo.fname);
				else
					ConsolePrint("%7u    %s\r\n", finfo.fsize, *finfo.lfname ? finfo.lfname : finfo.fname);
			}
		}
		while(fresult == FR_OK);

		f_closedir(&dir);
	}

	if(ConsoleScanCmd("cd %127[^\n]", buf8) == 1)
	{
		f_chdir((TCHAR*) buf8);

		f_getcwd((TCHAR*) buf8, bufLen);

		ConsolePrint("%s\r\n", buf8);
	}

	if(ConsoleScanCmd("touch %127[^\n]", buf8) == 1)
	{
		FIL file;
		fresult = f_open(&file, (TCHAR*) buf8, FA_CREATE_NEW);
		if(fresult == FR_EXIST)
			ConsolePrint("File already exists\r\n");
		f_close(&file);
	}

	if(ConsoleScanCmd("mkdir %127[^\n]", buf8) == 1)
	{
		fresult = f_mkdir((TCHAR*) buf8);
		if(fresult == FR_OK)
			ConsolePrint("Created dir: %s\r\n", buf8);
	}

	if(ConsoleScanCmd("rm %127[^\n]", buf8) == 1)
	{
		fresult = f_unlink((TCHAR*) buf8);
		if(fresult != FR_OK)
			ConsolePrint("Could not remove object %d\r\n", fresult);
	}

	if(ConsoleCmpCmd("lf debug"))
	{
		ConsolePrint("file.fs: 0x%08X\r\n", logFile.file.fs);
		ConsolePrint("ready: %hu\r\n", (uint16_t)logFile.ready);
		ConsolePrint("name: %s\r\n", logFile.logFilename);
		ConsolePrint("0x%08X 0x%08X 0x%08X\r\n", logFile.pBufStart, logFile.pBufEnd, logFile.pBufWrite);
	}

	if(ConsoleCmpCmd("lf stress"))
	{
		LogRawI16XYZ raw;
		for(uint32_t i = 0; i < sizeof(LogRawI16XYZ); i++)
		{
			*(((uint8_t*)&raw)+i) = i;
		}

		raw.header.type = SID_GYRO_RAW;
		raw.header.length = sizeof(LogRawI16XYZ);

		for(uint32_t runs = 0; runs < 100; runs++)
		{
			for(uint32_t i = 0; i < 10000; i++)
			{
				LogFileWrite(&raw);
				raw.header.timestamp++;
			}

			chThdSleepMilliseconds(10);
		}
	}

	if(ConsoleScanCmd("logfile %127[^\n]", buf8) == 1)
	{
		LogFileClose();

		uint32_t freeCluster;
		FATFS *fs;
		fresult = f_getfree("SD:", &freeCluster, &fs);
		if(fresult)
		{
			NetworkPrint("No media attached\r\n");
			ConsolePrint("No media attached\r\n");
			return;
		}

		uint32_t freeSectors = freeCluster * fs->csize;
		uint32_t freeSizeMB = freeSectors / ((1024 * 1024) / 512);

		NetworkPrint("Free Space: %uMB\n", freeSizeMB);

		f_chdrive("SD:");
		LogFileOpen((char*) buf8);
	}

	if(ConsoleCmpCmd("startlog"))
	{
		ConsolePrint("Writing log header\r\n");

		LogFileOpen(0);
	}

	if(ConsoleCmpCmd("stoplog"))
	{
		if(logFile.ready == 0)
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

		fresult = f_open(&file, "SD:/wrtest.txt", FA_CREATE_ALWAYS | FA_WRITE);

//		static uint8_t wrBuf[8192] __attribute__((section(".dtc")));
		static uint8_t wrBuf[8192];
		for(uint32_t i = 0; i < 8192; i++)
			wrBuf[i] = i;

		for(uint32_t i = 0; i < 64; i++)
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

	if(ConsoleScanCmd("buzz %hu", &u16) == 1)
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

static void robotPiCommands()
{
	uint16_t u16;
	float f32[2];

	if(ConsoleCmpCmd("camera config"))
	{
		ConsolePrint("Auto exposure:      %s\r\n", robotPi.cameraConfig.expAutoMode ? "On" : "Off");
		ConsolePrint("  Analog gain:      %.2f\r\n  Digital gain:     %.2f\r\n", robotPi.cameraConfig.expAnalogGain, robotPi.cameraConfig.expDigitalGain);
		ConsolePrint("  Exposure time:    %uus\r\n", robotPi.cameraConfig.expTimeUs);
		ConsolePrint("Auto white-balance: %s\r\n", robotPi.cameraConfig.wbAutoMode ? "On" : "Off");
		ConsolePrint("  Red gain:         %.2f\r\n  Blue gain:        %.2f\r\n", robotPi.cameraConfig.wbRedGain, robotPi.cameraConfig.wbBlueGain);
		ConsolePrint("Preview:            %s\r\n", robotPi.previewConfig.enable ? "On" : "Off");
		ConsolePrint("Resolution:         %hu\r\n", robotPi.cameraControl.resolution);
		ConsolePrint("Recording:          %s\r\n", robotPi.cameraControl.recording ? "On" : "Off");
	}

	if(ConsoleCmpCmd("camera stats"))
	{
		ConsolePrint("Processing time used: %.2f%%\r\n", robotPi.cameraStats.rtAvg / robotPi.cameraStats.dtAvg * 100.0f);
		ConsolePrint("Latency: %dus\r\n", robotPi.lastDetectionLatency);
		ConsolePrint("Frame delta times:\r\n");
		ConsolePrint("  min: %.5f\r\n", robotPi.cameraStats.dtMin);
		ConsolePrint("  avg: %.5f (%.2ffps)\r\n", robotPi.cameraStats.dtAvg, 1.0f/robotPi.cameraStats.dtAvg);
		ConsolePrint("  max: %.5f\r\n", robotPi.cameraStats.dtMax);
		ConsolePrint("  dev: %.5f\r\n", robotPi.cameraStats.dtDev);
		ConsolePrint("Algorithm run times:\r\n");
		ConsolePrint("  min: %.5f\r\n", robotPi.cameraStats.rtMin);
		ConsolePrint("  avg: %.5f\r\n", robotPi.cameraStats.rtAvg);
		ConsolePrint("  max: %.5f\r\n", robotPi.cameraStats.rtMax);
		ConsolePrint("  dev: %.5f\r\n", robotPi.cameraStats.rtDev);
		ConsolePrint("Recording: %hu\r\n", (uint16_t)robotPi.cameraStats.recording);
		ConsolePrint("  file: %s\r\n", robotPi.cameraStats.recordFilename);
		ConsolePrint("  size: %.3fMB\r\n", robotPi.cameraStats.recordSize * 1.0f/1024.0f * 1.0f/1024.0f);
		ConsolePrint("  duration: %.1fs\r\n", robotPi.cameraStats.recordDuration);
		ConsolePrint("Images: %u\r\n", robotPi.cameraStats.imagesTaken);
		ConsolePrint("  file: %s\r\n", robotPi.cameraStats.imageFilename);
	}

	if(ConsoleScanCmd("camera exp %hu", &u16) == 1)
	{
		ConsolePrint("Setting camera exposure to %huus\r\n", u16);
		robotPi.cameraConfig.expTimeUs = u16;
		robotPi.cameraConfig.expAutoMode = 0;
		ConfigNotifyUpdate(robotPi.pConfigFileCameraConfig);
	}

	if(ConsoleCmpCmd("camera exp auto"))
	{
		ConsolePrint("Enabling auto exposure mode\r\n");
		robotPi.cameraConfig.expAutoMode = 1;
		ConfigNotifyUpdate(robotPi.pConfigFileCameraConfig);
	}

	if(ConsoleScanCmd("camera gain %f %f", f32, f32+1) == 2)
	{
		ConsolePrint("Setting camera gains. Analog: %.2f, Digital: %.2f\r\n", f32[0], f32[1]);
		robotPi.cameraConfig.expAnalogGain = f32[0];
		robotPi.cameraConfig.expDigitalGain = f32[1];
		ConfigNotifyUpdate(robotPi.pConfigFileCameraConfig);
	}

	if(ConsoleScanCmd("camera awb %hu", &u16) == 1)
	{
		ConsolePrint("Setting AWB to %hu\r\n", u16);
		robotPi.cameraConfig.wbAutoMode = u16;
		ConfigNotifyUpdate(robotPi.pConfigFileCameraConfig);
	}

	if(ConsoleScanCmd("camera wb %f %f", f32, f32+1) == 2)
	{
		ConsolePrint("Setting white balance. Red: %.2f, Blue: %.2f\r\n", f32[0], f32[1]);
		robotPi.cameraConfig.wbRedGain = f32[0];
		robotPi.cameraConfig.wbBlueGain = f32[1];
		ConfigNotifyUpdate(robotPi.pConfigFileCameraConfig);
	}

	if(ConsoleScanCmd("camera preview %hu", &u16) == 1)
	{
		ConsolePrint("Camera preview: %hu\r\n", u16);
		robotPi.previewConfig.enable = u16;
	}

	if(ConsoleScanCmd("camera res %hu", &u16) == 1)
	{
		ConsolePrint("Camera res: %hu\r\n", u16);
		robotPi.cameraControl.resolution = u16;
	}

	if(ConsoleScanCmd("camera rec %hu", &u16) == 1)
	{
		ConsolePrint("Set recording: %hu\r\n", u16);
		robotPi.cameraControl.recording = u16;
	}

	if(ConsoleCmpCmd("camera trigger"))
	{
		RobotPiTriggerImageCapture();
		ConsolePrint("Triggered camera capture\r\n");
	}

	if(ConsoleCmpCmd("balls"))
	{
		float pos[3];
		float robotPos[3];

		memcpy(robotPos, robotPi.ballDetections.robotPos, sizeof(float)*3);

		ConsolePrint("ID      X_map     Y_map     Z_map    X_base    Y_base\r\n");

		for(uint16_t i = 0; i < robotPi.ballDetections.numBalls; i++)
		{
			memcpy(pos, robotPi.ballDetections.balls[i].pos, sizeof(float)*3);

			float posLocal[3] = { 0, 0, pos[2] };
			posLocal[0] = pos[0] - robotPos[0];
			posLocal[1] = pos[1] - robotPos[1];
			CtrlUtilRotate(-robotPos[2], posLocal[0], posLocal[1], posLocal);

			ConsolePrint("%3hu  %8.3f  %8.3f  %8.3f  %8.3f  %8.3f\r\n",
					robotPi.ballDetections.balls[i].trackerId,
					pos[0], pos[1], pos[2],
					posLocal[0], posLocal[1]);
		}
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
	robotPiCommands();
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
