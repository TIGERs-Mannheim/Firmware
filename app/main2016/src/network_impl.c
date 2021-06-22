/*
 * network_impl.c
 *
 *  Created on: 01.07.2010
 *      Author: AndreR
 */

#include "stm32f746xx.h"
#include "constants.h"

#include "main/network.h"

#include "util/console.h"
#include "util/init_hal.h"
#include "util/sx1280_def.h"
#include "util/config.h"

#include "hal/buzzer.h"
#include "hal/kicker.h"
#include "hal/motors.h"
#include "hal/led.h"

#include <string.h>

static uint8_t sxTx[SX1280_BUF_SIZE_RXTX] __attribute__((aligned(16), section(".dtc")));
static uint8_t sxRx[SX1280_BUF_SIZE_RXTX] __attribute__((aligned(16), section(".dtc")));
static SX1280 sx1280;

void DMA1_Stream3_IRQHandler()
{
	CH_IRQ_PROLOGUE();

	SPILowDmaRxInterrupt(&sx1280.spi);

	CH_IRQ_EPILOGUE();
}

void EXTI1_IRQHandler()
{
	CH_IRQ_PROLOGUE();

	EXTI->PR = (1 << 1);

	SX1280DataIRQ(&sx1280);

	CH_IRQ_EPILOGUE();
}

void EXTI15_10_IRQHandler()
{
	CH_IRQ_PROLOGUE();

	EXTI->PR = (1 << 11);

	SX1280BusyIRQ(&sx1280);

	CH_IRQ_EPILOGUE();
}

void NetworkImplInit()
{
	// SPI2 init
	RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

	NVICEnableIRQ(DMA1_Stream3_IRQn, IRQL_NRF24);

	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.alternate = 5;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_25MHZ;
	gpioInit.pupd = GPIO_PUPD_UP;
	GPIOInit(GPIOC, GPIO_PIN_2 | GPIO_PIN_3, &gpioInit);
	GPIOInit(GPIOD, GPIO_PIN_3, &gpioInit);

	// SX1280 busy and irq lines
	NVICEnableIRQ(EXTI1_IRQn, IRQL_NRF24);

	NVICEnableIRQ(EXTI15_10_IRQn, IRQL_NRF24);

	// SX1280 setup
	SX1280Data data;
	data.pTxBuf = sxTx;
	data.pRxBuf = sxRx;
	data.spiData.csPin = GPIO_PIN_4;
	data.spiData.pCSPort = GPIOA;
	data.spiData.pRegister = SPI2;
	data.spiData.pDma = DMA1;
	data.spiData.dmaChRx = 3;
	data.spiData.dmaChTx = 4;
	data.spiData.prescaler = SPI_CR1_BRDIV8;
	data.busyPin = GPIO_PIN_11;
	data.pBusyPort = GPIOF;
	data.irqPin = GPIO_PIN_1;
	data.pIrqPort = GPIOB;

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
		.txPower = 31,
	};

	SX1280Setup(&sx1280, &settings);
}

int16_t NetworkImplReceiveAndTransmit(uint8_t rxBytes, void* pRxData, SX1280RxResult* pResult, uint8_t txBytes, const void* pTxData)
{
	return SX1280ReceiveAndTransmit(&sx1280, rxBytes, pRxData, pResult, MS2ST(200), 130, txBytes, pTxData);
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
//					SystemConsoleCommand* pCmd = (SystemConsoleCommand*)pBuf;

//					if(pCmd->target == INTERCOM_ADDRESS_MAIN)
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
//					else
//					{
//						if(cli.intercomFwd.inUse == 0)
//						{
//							cli.intercomFwd.bufSize = dataLength-sizeof(SystemConsoleCommand);
//
//							if(cli.intercomFwd.bufSize <= CLI_INTERCOM_BUF_SIZE)
//							{
//								cli.intercomFwd.inUse = 1;
//								cli.intercomFwd.target = pCmd->target;
//
//								memcpy(cli.intercomFwd.buf, pBuf+sizeof(SystemConsoleCommand), cli.intercomFwd.bufSize);
//							}
//						}
//					}
				}
				break;
			}
		}
		break;
		case SECTION_BOOTLOADER:
		{
//			if(pHeader->cmd == CMD_BOOTLOADER_CHECK)
//			{
//				IntercomHeader iHeader;
//				iHeader.address = INTERCOM_ADDRESS_MEDIA;
//				iHeader.section = INTERCOM_SECTION_NETWORK;
//				iHeader.command = 1;
//
//				IntercomReliableSend(&iHeader, 0, 0);
//			}
//			else
//			{
//				IntercomHeader iHeader;
//				iHeader.address = INTERCOM_ADDRESS_MEDIA;
//				iHeader.section = INTERCOM_SECTION_NETWORK;
//				iHeader.command = 0;
//
//				IntercomSend(&iHeader, dataLength+PACKET_HEADER_SIZE, pHeader);
//			}
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
