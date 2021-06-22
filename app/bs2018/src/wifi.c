/*
 * wifi.c
 *
 *  Created on: 07.07.2014
 *      Author: AndreR
 */

#include "wifi.h"

#include "util/console.h"
#include "util/bits.h"
#include "util/sys_time.h"
#include "util/init_hal.h"
#include "util/log.h"
#include "util/angle_math.h"
#include "util/sx1280.h"
#include "util/sx1280_def.h"
#include "util/map_to_range.h"
#include "util/crc8.h"
#include "constants.h"
#include "commands.h"
#include "errors.h"
#include "network.h"

#include <string.h>
#include <math.h>

WifiGlobal wifi;

static uint8_t tx[SX1280_BUF_SIZE_RXTX] __attribute__((aligned(16), section(".dtc")));
static uint8_t rx[SX1280_BUF_SIZE_RXTX] __attribute__((aligned(16), section(".dtc")));

static const uint8_t antTable[9][10] = {
	{1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
	{1, 0, 0, 0, 1, 0, 0, 0, 0, 0},
	{1, 0, 0, 1, 0, 0, 1, 0, 0, 0},
	{1, 0, 1, 0, 0, 1, 0, 0, 1, 0},
	{1, 0, 1, 0, 1, 0, 1, 0, 1, 0},
	{0, 1, 0, 1, 1, 0, 1, 1, 0, 1},
	{0, 1, 1, 0, 1, 1, 0, 1, 1, 1},
	{0, 1, 1, 1, 0, 1, 1, 1, 1, 1},
	{0, 1, 1, 1, 1, 1, 1, 1, 1, 1},
};

#define WIFI_PAYLOAD_SIZE 32
#define WIFI_HEADER_SIZE 3
#define WIFI_PACKET_SIZE (WIFI_PAYLOAD_SIZE+WIFI_HEADER_SIZE)

// WF Busy IRQ
void EXTI2_IRQHandler()
{
	CH_IRQ_PROLOGUE();

	EXTI->PR = GPIO_PIN_2;

	SX1280BusyIRQ(&wifi.module.radio);

	CH_IRQ_EPILOGUE();
}

// WF Data IRQ
void EXTI3_IRQHandler()
{
	CH_IRQ_PROLOGUE();

	EXTI->PR = GPIO_PIN_3;

	SX1280DataIRQ(&wifi.module.radio);

	CH_IRQ_EPILOGUE();
}

// WF MISO
void DMA1_Stream0_IRQHandler()
{
	CH_IRQ_PROLOGUE();

	SPILowDmaRxInterrupt(&wifi.module.radio.spi);

	CH_IRQ_EPILOGUE();
}

void TIM7_IRQHandler()
{
	CH_IRQ_PROLOGUE();

	TIM7->SR = 0;

	chSysLockFromISR();
	chBSemSignalI(&wifi.timerSem);
	chSysUnlockFromISR();

	CH_IRQ_EPILOGUE();
}

void WifiInit()
{
	GPIOInitData gpioInit;

	wifi.cfg.channel = 100;
	wifi.cfg.speed = 2;
	wifi.cfg.maxBots = 11;
	wifi.cfg.fixedRuntime = 0;
	wifi.cfg.timeout = 1000;

	chBSemObjectInit(&wifi.timerSem, TRUE);

	FlashFSOpenOrCreate("wifi/cfg", 2, &wifi.cfg, sizeof(WifiConfig), &wifi.pCfgFile);

	// SX1280 IRQ lines
	NVICEnableIRQ(EXTI2_IRQn, IRQL_WIFI);
	NVICEnableIRQ(EXTI3_IRQn, IRQL_WIFI);

	// SPI3 - WF
	RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;

	NVICEnableIRQ(DMA1_Stream0_IRQn, IRQL_WIFI);

	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.alternate = 6;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_25MHZ;
	gpioInit.pupd = GPIO_PUPD_DOWN;
	GPIOInit(GPIOC, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12, &gpioInit);  // SCK, MISO, MOSI

	// TIM7 setup
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

	TIM7->CR1 = 0;
	TIM7->CR2 = 0;
	TIM7->SR = 0;
	TIM7->DIER = TIM_DIER_UIE;
	TIM7->PSC = 216; 	// clock is now 1MHz
	TIM7->ARR = 1250;	// = 1.25ms <=> 800Hz
	TIM7->CR1 |= TIM_CR1_CEN;

	NVICEnableIRQ(TIM7_IRQn, IRQL_NRF24_TIMER);

	for(uint8_t i = 0; i <= WIFI_MAX_BOT_ID; i++)
	{
		WifiRobot* pBot = &wifi.bots[i];

		EMAFilterInit(&pBot->emaRssi[0], 0.95f);
		EMAFilterInit(&pBot->emaRssi[1], 0.95f);
		EMAFilterInit(&pBot->botRssi, 0.95f);
		pBot->emaRssi[0].value = -30.0f;
		pBot->emaRssi[1].value = -30.0f;

		RFQueueInit(&pBot->queue, pBot->txData, WIFI_TX_SIZE, pBot->rxData, WIFI_RX_SIZE, WIFI_PAYLOAD_SIZE, 68);

		pBot->lastResponseTime = -2 * MS2ST(wifi.cfg.timeout);
		pBot->online = 0;
		pBot->id = i;
		pBot->antTableRow = 4;
		pBot->antTableColumn = 0;
	}

	// WF setup
	SX1280Data sx = {
		.pTxBuf = tx,
		.pRxBuf = rx,
		.spiData.csPin = GPIO_PIN_6,
		.spiData.pCSPort = GPIOD,
		.spiData.pRegister = SPI3,
		.spiData.pDma = DMA1,
		.spiData.dmaChRx = 0,
		.spiData.dmaChTx = 7,
		.spiData.prescaler = SPI_CR1_BRDIV4,
		.busyPin = GPIO_PIN_2,
		.pBusyPort = GPIOD,
		.irqPin = GPIO_PIN_3,
		.pIrqPort = GPIOD,
	};

	SKY66112 sky = {
		.antPin = { GPIOE, GPIO_PIN_6 },
		.cpsPin = { GPIOE, GPIO_PIN_2 },
		.crxPin = { GPIOE, GPIO_PIN_4 },
		.ctxPin = { GPIOE, GPIO_PIN_3 },
	};

	SX1280SettingsFLRC settings =
	{
		.frequency = 2400000000UL,
		.highSensitivityMode = 0,
		.bitrate = FLRC_BR_1_300_BW_1_2,
		.coderate = FLRC_CR_1_0,
		.syncWordEnable = 1,
		.variableLength = 0,
		.payloadLength = WIFI_PACKET_SIZE,
		.crcSize = CRC_OFF,
		.syncWord = 0,
		.txPower = 16, // = -2dBm, don't go over -2dBm to respect FEM absolute maximum ratings
	};

	WifiModuleData wm = {
		.pRadioInit = &sx,
		.pRadioSettings = &settings,
		.pFEMInit = &sky,
		.pwrPin = { GPIOB, GPIO_PIN_5 },
	};

	WifiModuleInit(&wifi.module, &wm);
}

void WifiSaveConfig()
{
	FlashFSWrite(&wifi.pCfgFile, &wifi.cfg, sizeof(WifiConfig));
}

void WifiSetChannel(uint8_t channel)
{
	SX1280SetChannel(&wifi.module.radio, channel);

	wifi.cfg.channel = channel;
}

void WifiSetMaxBot(uint8_t maxBots)
{
	if(maxBots > WIFI_MAX_BOTS)
		return;

	wifi.cfg.maxBots = maxBots;
}

void WifiSetFixedRuntime(uint8_t enable)
{
	wifi.cfg.fixedRuntime = enable;
}

void WifiSetTimeout(uint32_t ms)
{
	wifi.cfg.timeout = ms;
}

void WifiSetBotDataCallback(BotDataAvailCallback cb)
{
	wifi.cbBotData = cb;
}

void WifiSetHookBotProcess(BotProcessHook hook)
{
	wifi.hookBotProcess = hook;
}

void WifiPrintConfig()
{
	ConsolePrint("--- Wireless Configuration ---\r\n");
	ConsolePrint("Channel:   %hu (2.%03huGHz)\r\n", (uint16_t)wifi.cfg.channel, ((uint16_t)wifi.cfg.channel)+300);
	ConsolePrint("Max. Bots: %hu\r\n", (uint16_t)wifi.cfg.maxBots);
	ConsolePrint("Timeout:   %ums\r\n", wifi.cfg.timeout);

	if(wifi.cfg.fixedRuntime)
		ConsolePrint("Runtime:   fixed\r\n");
	else
		ConsolePrint("Runtime:   variable\r\n");
}

uint8_t WifiGetNumBotsOnline()
{
	uint8_t bots = 0;

	for(uint8_t i = 0; i < WIFI_MAX_BOTS; i++)
	{
		if(wifi.bots[i].online)
			bots++;
	}

	return bots;
}

float WifiGetLinkQuality()
{
	uint8_t bots = 0;
	float link = 0.0f;

	for(uint8_t i = 0; i < WIFI_MAX_BOTS; i++)
	{
		if(wifi.bots[i].online)
		{
			link += fmaxf(wifi.bots[i].emaRssi[0].value, wifi.bots[i].emaRssi[1].value);
			bots++;
		}
	}

	if(bots == 0)
		return 0.0f;

	float dbm = link/(float)bots;

	return MapToRangef32(-90.0f, -12.0f, 0.0f, 99.9f, dbm);
}

static int16_t processBot(WifiRobot* pBot)
{
	int16_t result = 0;

	LogInfoC("Processing bot", pBot->id);

	chBSemWait(&wifi.timerSem);

	if(wifi.hookBotProcess)
		(*wifi.hookBotProcess)(pBot);

	uint16_t rxPkt = pBot->queue.stats.rfIO.rxPackets;

	// select antenna
	uint8_t ant = antTable[pBot->antTableRow][pBot->antTableColumn++];
	if(pBot->antTableColumn >= 10)
	{
		float rssiDiff = pBot->emaRssi[1].value - pBot->emaRssi[0].value;
		if(rssiDiff < -30.0f)
			rssiDiff = -30.0f;
		if(rssiDiff > 30.0f)
			rssiDiff = 30.0f;

		int8_t newRow = (int8_t)((rssiDiff + 30.0f)/6.7f);
		if(newRow < 0)
			newRow = 0;
		if(newRow > 8)
			newRow = 8;

		pBot->antTableRow = newRow;
		pBot->antTableColumn = 0;
	}

	// Assemble data
	/* Layout:
	 * Byte 0: Data length
	 * Byte 1: Own RSSI
	 * Byte 2-33: Data
	 * Byte 34: CRC8 over byte 0 - 33
	 */
	static uint8_t assembly[WIFI_PACKET_SIZE];
	memset(assembly, 0, WIFI_PACKET_SIZE);
	uint32_t packetSize = RFQueueFetchTxPacket(&pBot->queue, assembly+2);
	assembly[0] = packetSize;
	assembly[1] = (uint8_t)(pBot->emaRssi[ant].value*-2.0f);
	assembly[WIFI_PACKET_SIZE-1] = Crc8(assembly, WIFI_PACKET_SIZE-1);

	SX1280RxResult rxResult;
	result = WifiModuleTransmitAndReceive(&wifi.module, ant, pBot->id, WIFI_PACKET_SIZE, assembly, &rxResult);

	if(rxResult.crcError == 0 && rxResult.abortError == 0 && rxResult.packetReceived && rxResult.bytesReceived == WIFI_PACKET_SIZE
		&& Crc8(assembly, WIFI_PACKET_SIZE-1) == assembly[WIFI_PACKET_SIZE-1])
	{
		// successful reception
		float botRssi = assembly[1]*-0.5f;
		EMAFilterUpdate(&pBot->botRssi, botRssi);
		EMAFilterUpdate(&pBot->emaRssi[ant], rxResult.rssiSync*-0.5f);
		RFQueueFeedRxPacket(&pBot->queue, assembly+2, assembly[0]);
	}
	else
	{
		pBot->queue.stats.rfFeedback._deprecated1++;
		EMAFilterUpdate(&pBot->emaRssi[ant], -125.0f);
	}

	if(rxPkt != pBot->queue.stats.rfIO.rxPackets)
		pBot->lastResponseTime = chVTGetSystemTimeX();

	if(pBot->queue.rxFifo.numPackets && wifi.cbBotData)
		(*wifi.cbBotData)(pBot->id);

	// check if bot timed out
	uint32_t elapsedTime = chVTTimeElapsedSinceX(pBot->lastResponseTime);
	if(elapsedTime > MS2ST(wifi.cfg.timeout))
	{
		// no response => timeout
		if(pBot->online)
		{
			pBot->online = 0;
			RFQueueClear(&pBot->queue);

			// TODO: print probably delays processing, move to low prio task?
			ConsolePrint("Bot %hu timed out after %u ms. Elapsed: %u\r\n", (uint16_t)pBot->id, wifi.cfg.timeout, elapsedTime/4);
		}
	}
	else
	{
		if(!pBot->online)
		{
			pBot->online = 1;

			ConsolePrint("Bot %hu online\r\n", (uint16_t)pBot->id);
		}
	}

	return result;
}

void WifiTask(void* params)
{
	(void) params;

	chRegSetThreadName("Wifi");

	ConsolePrint("WifiTask started\r\n");

	uint32_t startTime = SysTimeUSec();
	uint32_t endTime;
	uint8_t checkId = 0;

	SX1280SetChannel(&wifi.module.radio, wifi.cfg.channel);

	uint16_t version = 42;
	SX1280GetFirmwareVersion(&wifi.module.radio, &version);
	ConsolePrint("SX1280 FW Version: 0x%04hX\r\n", version);

	while(1)
	{
		startTime = SysTimeUSec();

		uint8_t numBotsOnline = 0;
		for(uint8_t i = 0; i < WIFI_MAX_BOTS; i++)
		{
			WifiRobot* pBot = &wifi.bots[i];

			if(!pBot->online)
				continue;

			++numBotsOnline;

			processBot(pBot);
		}

		while(numBotsOnline < wifi.cfg.maxBots)
		{
			do
			{
				++checkId;
				checkId %= WIFI_MAX_BOTS;
			}
			while(wifi.bots[checkId].online);

			processBot(&wifi.bots[checkId]);

			++numBotsOnline;

			if(wifi.cfg.fixedRuntime == 0)	// only one check?
				break;
		}

		endTime = SysTimeUSec();

		wifi.runtime = endTime - startTime;

		wifi.lastCall = SysTimeUSec();
	}
}
