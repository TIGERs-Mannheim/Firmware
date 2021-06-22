/*
 * constants.h
 *
 *  Created on: 24.10.2015
 *      Author: AndreR
 */

#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

/////////////////////////////////////////////////
// Buffer Sizes
/////////////////////////////////////////////////

// NRF24
#define NRF24_TX_SIZE 2048
#define NRF24_RX_SIZE 2048

// USART3 - USB
#define USART3_TX_SIZE 1000
#define USART3_RX_SIZE 300

// UART8 - EXT
#define UART8_TX_SIZE 1000
#define UART8_RX_SIZE 1000
#define UART8_MAX_PACKET_SIZE 64

// CONSOLE
#define CONSOLE_RX_SIZE 1000
#define CONSOLE_TX_SIZE 2000
#define CONSOLE_PRINT_SIZE 400

/////////////////////////////////////////////////
// IRQ Levels
/////////////////////////////////////////////////

// External Interrupts
#define IRQL_EXTI0 15	// KILL
#define IRQL_EXTI5_9 13
#define IRQL_EXTI10_15 10

// USART3
#define IRQL_USART3 14

#define IRQL_UART8 8

#define IRQL_SPI6 9

#define IRQL_SPI4 9

#define IRQL_SDMMC1 10

#define IRQL_USB 10

// External Flash
#define IRQL_QUADSPI 11

#define IRQL_KICK_CTRL 6

#define IRQL_NRF24 7

#endif
