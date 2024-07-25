#include "rtc.h"
#include "cmparams.h"
#include "hal/sys_time.h"
#include <time.h>

RTCGlobal rtc;

static void registerShellCommands(ShellCmdHandler* pHandler);

static void enableInitMode()
{
	PWR->CR1 |= PWR_CR1_DBP;
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;

	RTC->ISR |= RTC_ISR_INIT;

	while((RTC->ISR & RTC_ISR_INITF) == 0)
		asm volatile("nop");
}

static void disableInitMode()
{
	// exit initialization => start RTC
	RTC->ISR &= ~RTC_ISR_INIT;

	// re-enable write-protection
	RTC->WPR = 0;
	PWR->CR1 &= ~PWR_CR1_DBP;
}

void RTCInit()
{
	ShellCmdHandlerInit(&rtc.cmdHandler, 0);
	registerShellCommands(&rtc.cmdHandler);

	// check if RTC is already initialized
	if(RTC->ISR & RTC_ISR_INITS)
		return;

	// disable backup domain write protection
	PWR->CR1 |= PWR_CR1_DBP;

	RCC->BDCR = RCC_BDCR_BDRST;
	RCC->BDCR = 0;

	RCC->BDCR = RCC_BDCR_LSEON;

	while((RCC->BDCR & RCC_BDCR_LSERDY) == 0)
		asm volatile("nop");

	RCC->BDCR |= RCC_BDCR_RTCSEL_0;

	RCC->BDCR |= RCC_BDCR_RTCEN;

	enableInitMode();

	// set divider for 32768Hz LSE clock
	RTC->PRER = (127 << 16) | 255;

	// Set 1.1.2017 as default
	RTC->DR = (1 << 20) | (7 << 16);

	disableInitMode();

	RTCSetCalib(104.0f);
}

void RTCSetTimeBCD(uint32_t time)
{
	enableInitMode();

	RTC->TR = time;

	disableInitMode();
}

void RTCSetDateBCD(uint32_t date)
{
	enableInitMode();

	RTC->DR = date;

	disableInitMode();
}

uint32_t RTCGetUnixTimestamp()
{
	uint32_t tr = RTC->TR;
	uint32_t dr = RTC->DR;

	struct tm rtc;
	rtc.tm_sec = (tr & RTC_TR_SU) + ((tr & RTC_TR_ST) >> 4)*10;
	rtc.tm_min = ((tr & RTC_TR_MNU) >> 8) + ((tr & RTC_TR_MNT) >> 12)*10;
	rtc.tm_hour = ((tr & RTC_TR_HU) >> 16) + ((tr & RTC_TR_HT) >> 20)*10;
	rtc.tm_mday = (dr & RTC_DR_DU) + ((dr & RTC_DR_DT) >> 4)*10;
	rtc.tm_mon = ((dr & RTC_DR_MU) >> 8) + ((dr & RTC_DR_MT) >> 12)*10;
	rtc.tm_year = ((dr & RTC_DR_YU) >> 16) + ((dr & RTC_DR_YT) >> 20)*10 + 100;
	rtc.tm_isdst = -1;

	rtc.tm_mon -= 1;

	time_t now = mktime(&rtc);

	return now;
}

void RTCSetUnixTimestamp(int32_t time)
{
	time_t t = (time_t) time;
	struct tm* nowtm = localtime(&t);
	nowtm->tm_year -= 100;
	nowtm->tm_mon += 1;

	uint32_t tr = 0;
	tr |= (nowtm->tm_sec%10) | ((nowtm->tm_sec/10) << 4);
	tr |= ((nowtm->tm_min%10) << 8) | ((nowtm->tm_min/10) << 12);
	tr |= ((nowtm->tm_hour%10) << 16) | ((nowtm->tm_hour/10) << 20);

	uint32_t dr = 0;
	dr |= (nowtm->tm_mday%10) | ((nowtm->tm_mday/10) << 4);
	dr |= ((nowtm->tm_mon%10) << 8) | ((nowtm->tm_mon/10) << 12);
	dr |= ((nowtm->tm_year%10) << 16) | ((nowtm->tm_year/10) << 20);

	enableInitMode();

	RTC->DR = dr;
	RTC->TR = tr;

	disableInitMode();
}

void RTCSetCalib(float ppm)
{
	if(ppm < -487.0f || ppm > 488.0f)
		return;

	const float res = 0.9537f;
	uint32_t calr = 0;

	if(ppm < 0)
	{
		calr = (uint32_t)((-ppm+res*0.5f)/res);
	}
	else
	{
		calr = (uint32_t)((488.5f-ppm+res*0.5f)/res);
		calr |= RTC_CALR_CALP;
	}

	while(RTC->ISR & RTC_ISR_RECALPF)
		asm volatile("nop");

	PWR->CR1 |= PWR_CR1_DBP;
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;

	RTC->CALR = calr;

	RTC->WPR = 0;
	PWR->CR1 &= ~PWR_CR1_DBP;
}

SHELL_CMD(time, "Show current time and RTC calibration");

SHELL_CMD_IMPL(time)
{
	(void)pUser; (void)argc; (void)argv;

	const float res = 0.9537f;

	printf("SysTime: %uus\r\n", SysTimeUSec());

	struct tm* nowtm;
	time_t now = RTCGetUnixTimestamp();
	nowtm = localtime(&now);

	char tmbuf[64];
	strftime(tmbuf, sizeof tmbuf, "%Y-%m-%d %H:%M:%S", nowtm);
	printf("%s\r\n", tmbuf);

	uint32_t dr = RTC->DR;
	uint32_t tr = RTC->TR;
	uint32_t calr = RTC->CALR;
	float ppm;
	if(calr & RTC_CALR_CALP)
		ppm = 488.5f - ((calr & RTC_CALR_CALM) * res);
	else
		ppm = ((calr & RTC_CALR_CALM) * -res);

	printf("RTC Date: %08X\r\n", dr);
	printf("RTC Time: %08X\r\n", tr);
	printf("RTC Calib: 0x%08X (%.2fppm)\r\n", calr, ppm);
}

SHELL_CMD(set_time, "Set current time in BCD format (HHmmss)",
	SHELL_ARG(time, "New time")
);

SHELL_CMD_IMPL(set_time)
{
	(void)pUser; (void)argc;

	uint32_t time = strtoul(argv[1], 0, 16);

	RTCSetTimeBCD(time);

	chThdSleepMilliseconds(100);

	struct tm* nowtm;
	time_t now = RTCGetUnixTimestamp();
	nowtm = localtime(&now);

	char tmbuf[64];
	strftime(tmbuf, sizeof tmbuf, "%H:%M:%S", nowtm);
	printf("%s\r\n", tmbuf);
}

SHELL_CMD(set_date, "Set current date in BCD format (YYmmdd)",
	SHELL_ARG(date, "New date")
);

SHELL_CMD_IMPL(set_date)
{
	(void)pUser; (void)argc;

	uint32_t date = strtoul(argv[1], 0, 16);

	RTCSetDateBCD(date);

	chThdSleepMilliseconds(100);

	struct tm* nowtm;
	time_t now = RTCGetUnixTimestamp();
	nowtm = localtime(&now);

	char tmbuf[64];
	strftime(tmbuf, sizeof tmbuf, "%Y-%m-%d", nowtm);
	printf("%s\r\n", tmbuf);
}

SHELL_CMD(calib, "Set RTC calibration value",
	SHELL_ARG(cal, "Calibration value (-487.1..488.5ppm)")
);

SHELL_CMD_IMPL(calib)
{
	(void)pUser; (void)argc;

	float ppm = atof(argv[1]);

	if(ppm < -487.0f || ppm > 488.0f)
	{
		fprintf(stderr, "Invalid calibration value\r\n");
		return;
	}

	printf("Setting RTC calibration to %.2fppm\r\n", ppm);

	RTCSetCalib(ppm);
}

static void registerShellCommands(ShellCmdHandler* pHandler)
{
	ShellCmdAdd(pHandler, time_command);
	ShellCmdAdd(pHandler, set_time_command);
	ShellCmdAdd(pHandler, set_date_command);
	ShellCmdAdd(pHandler, calib_command);
}
