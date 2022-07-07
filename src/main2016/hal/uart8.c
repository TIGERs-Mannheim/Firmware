/*
 * uart8.c
 *
 *  Created on: 24.10.2015
 *      Author: AndreR
 */

#include "uart8.h"

#include "../constants.h"
#include "util/init_hal.h"

UART8Data uart8 __attribute__((section(".dtc")));

void UART8_IRQHandler()
{
	USARTIRQ(&uart8.uart);
}

void UART8Init()
{
	GPIOInitData gpioInit;
	gpioInit.alternate = 0;
	gpioInit.mode = GPIO_MODE_INPUT;
	gpioInit.pupd = GPIO_PUPD_UP;
	GPIOInit(GPIOE, GPIO_PIN_0, &gpioInit);

	chThdSleepMilliseconds(2);

	if(GPIOE->IDR & GPIO_PIN_0)
		uart8.extInstalled = 0;	// RX pin follows pull-up from main => no board on EXT
	else
		uart8.extInstalled = 1;

	// enable peripheral clock
	RCC->APB1ENR |= RCC_APB1ENR_UART8EN;

	// map data and configure
	USARTData data;
	data.pDataRx = uart8.rxData;
	data.rxSize = UART8_RX_SIZE;
	data.pDataTx = uart8.txData;
	data.txSize = UART8_TX_SIZE;
	data.maxPacketSize = UART8_MAX_PACKET_SIZE;
	data.pDMA = DMA1;
	data.rxStream = 6;
	data.txStream = 0;
	data.dmaChannel = 5;
	data.usartIRQn = UART8_IRQn;
	data.IRQL = IRQL_UART8;
	data.baudrate = 115200;
	data.peripheralClock = systemClockInfo.APB1PeriphClk;

	// Init USART util layer
	USARTInit(&uart8.uart, UART8, data);

	// GPIO setup
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.alternate = 8;	// UART8
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_50MHZ;
	gpioInit.pupd = GPIO_PUPD_UP;
	GPIOInit(GPIOE, GPIO_PIN_0 | GPIO_PIN_1, &gpioInit);
}
