/*
 * network_impl.c
 *
 *  Created on: 01.07.2010
 *      Author: AndreR
 */

#include "constants.h"

#include "main/network.h"

#include "util/console.h"
#include "util/init_hal.h"
#include "util/sx1280_def.h"
#include "util/sky66112.h"
#include "util/config.h"

#include <string.h>

#define POWER_ON() GPIOSet(GPIOB, GPIO_PIN_2)
#define POWER_OFF() GPIOReset(GPIOB, GPIO_PIN_2)

static uint8_t sxTx[SX1280_BUF_SIZE_RXTX] __attribute__((aligned(16), section(".sram2")));
static uint8_t sxRx[SX1280_BUF_SIZE_RXTX] __attribute__((aligned(16), section(".sram2")));
static SX1280 sx1280;
static SKY66112 skyFem;

// TODO: add reset option for SX1280 by power-cycling, see wifi_module.c

void DMA2_Stream0_IRQHandler()
{
	CH_IRQ_PROLOGUE();

	SPILowDmaRxInterrupt(&sx1280.spi);

	CH_IRQ_EPILOGUE();
}

void EXTI4_IRQHandler()
{
	CH_IRQ_PROLOGUE();

	EXTI_D1->PR1 = EXTI_PR1_PR4;

	SX1280BusyIRQ(&sx1280);

	CH_IRQ_EPILOGUE();
}

void EXTI9_5_IRQHandler()
{
	CH_IRQ_PROLOGUE();

	EXTI_D1->PR1 = EXTI_PR1_PR5;

	SX1280DataIRQ(&sx1280);

	CH_IRQ_EPILOGUE();
}

void NetworkImplInit()
{
	// SPI1 init
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	__DSB();

	DMAMUX1_Channel8->CCR = 37; // spi1_rx_dma => DMA2-CH0
	DMAMUX1_Channel9->CCR = 38; // spi1_tx_dma => DMA2-CH1

	NVICEnableIRQ(DMA2_Stream0_IRQn, IRQL_WIFI);

	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.alternate = 5;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_25MHZ;
	gpioInit.pupd = GPIO_PUPD_UP;
	GPIOInit(GPIOA, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, &gpioInit); // SPI1: SCK, MISO, MOSI

	// SX1280 busy and irq lines
	NVICEnableIRQ(EXTI4_IRQn, IRQL_WIFI);

	NVICEnableIRQ(EXTI9_5_IRQn, IRQL_WIFI);

	POWER_OFF();

	gpioInit.mode = GPIO_MODE_OUTPUT;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.pupd = GPIO_PUPD_NONE;
	GPIOInit(GPIOB, GPIO_PIN_2, &gpioInit);

	// SKY66112 setup
	skyFem.antPin = (GPIOPin){0, 0 };
	skyFem.cpsPin = (GPIOPin){ GPIOF, GPIO_PIN_3 };
	skyFem.crxPin = (GPIOPin){ GPIOF, GPIO_PIN_4 };
	skyFem.ctxPin = (GPIOPin){ GPIOF, GPIO_PIN_5 };

	SKY66112Init(&skyFem);
	SKY66112SetMode(&skyFem, SKY66112_MODE_OFF);

	POWER_ON();
	chThdSleepMilliseconds(10);

	// SX1280 setup
	SX1280Data data;
	data.pTxBuf = sxTx;
	data.pRxBuf = sxRx;
	data.spiData.csPin = GPIO_PIN_10;
	data.spiData.pCSPort = GPIOF;
	data.spiData.pRegister = SPI1;
	data.spiData.pDma = DMA2;
	data.spiData.dmaChRx = 0;
	data.spiData.dmaChTx = 1;
	data.spiData.prescaler = SPI_CFG1_BRDIV8;
	data.busyPin = GPIO_PIN_4;
	data.pBusyPort = GPIOC;
	data.irqPin = GPIO_PIN_5;
	data.pIrqPort = GPIOC;

	SX1280Init(&sx1280, &data);

	SX1280SettingsFLRC settings =
	{
		.frequency = 2420000000UL,
		.highSensitivityMode = 0,
		.bitrate = FLRC_BR_1_300_BW_1_2,
		.coderate = FLRC_CR_1_0,
		.syncWordEnable = 1,
		.variableLength = 0,
		.payloadLength = WIFI_PACKET_SIZE,
		.crcSize = CRC_OFF,
		.syncWord = 0x54696761, // receiver sync word tolerance: 4-5 bit (no sync with 6 bit difference)
		.txPower = 16, // = -2dBm, don't go over -2dBm to respect FEM absolute maximum ratings
	};

	SX1280Setup(&sx1280, &settings);
}

int16_t NetworkImplReceiveAndTransmit(uint8_t rxBytes, void* pRxData, SX1280RxResult* pResult, uint8_t txBytes, const void* pTxData)
{
	SKY66112SetMode(&skyFem, SKY66112_MODE_RX);

	int16_t rxResult = SX1280FastRxTxReceive(&sx1280, txBytes, pTxData, MS2ST(200));
	if(rxResult != 0)
	{
		SKY66112SetMode(&skyFem, SKY66112_MODE_OFF);
		return rxResult;
	}

	SKY66112SetMode(&skyFem, SKY66112_MODE_TX);

	// TODO: add [us] delay here (previously with autoTx was 130us)

	int16_t txResult = SX1280FastRxTxTransmit(&sx1280, pResult, rxBytes, pRxData);

	SKY66112SetMode(&skyFem, SKY66112_MODE_OFF);

	return txResult;
}

int16_t NetworkImplSetChannel(uint8_t channel)
{
	if(channel != sx1280.channel)
		return SX1280SetChannel(&sx1280, channel);

	return 0;
}

int16_t NetworkImplSetAddress(uint8_t address)
{
	if(address != sx1280.address)
		return SX1280SetAddress(&sx1280, address);

	return 0;
}

// this is the main network processing function
void NetworkImplProcessPacket(PacketHeader* pHeader, uint8_t* pBuf, uint16_t dataLength)
{
	switch(pHeader->section)
	{
		case SECTION_SYSTEM:
		{
			switch(pHeader->cmd)
			{
				case CMD_SYSTEM_CONSOLE_COMMAND:
				{
					// process onboard
					uint8_t* pData;
					uint32_t length = dataLength-sizeof(SystemConsoleCommand);
					int16_t result = FifoLinReserve(&console.rxFifo, length+1, &pData);
					if(result)
						return;

					memcpy(pData, pBuf+sizeof(SystemConsoleCommand), length);
					pData[length] = 0;

					FifoLinCommit(&console.rxFifo, length+1);
				}
				break;
			}
		}
		break;

#ifdef ENV_RUN
		case SECTION_CONFIG:
		{
			ConfigNetworkInput(pHeader, pBuf, dataLength);
		}
		break;
#endif
		default: break;
	}
}
