/*
 * constants.h
 *
 *  Created on: 23.10.2017
 *      Author: AndreR
 */

#pragma once

/////////////////////////////////////////////////
// Buffer Sizes
/////////////////////////////////////////////////

// USART1 - USB
#define USART1_TX_SIZE 1000
#define USART1_RX_SIZE 300

// CONSOLE
#define CONSOLE_RX_SIZE 1000
#define CONSOLE_TX_SIZE 2000
#define CONSOLE_PRINT_SIZE 400

// VISION
// protoc allocator size for unpacking
#define VISION_ALLOC_BUF_SIZE 8192

// maximum number of cameras
#define VISION_MAX_CAMS 16

// WIFI
// for each WifiRobot (WIFI_MAX_BOTS)
#define WIFI_TX_SIZE 400
#define WIFI_RX_SIZE 400

/*
// NRF24
#define NRF24_TX_SIZE 2048
#define NRF24_RX_SIZE 2048

// UART8 - EXT
#define UART8_TX_SIZE 1000
#define UART8_RX_SIZE 1000
#define UART8_MAX_PACKET_SIZE 64
*/

/////////////////////////////////////////////////
// IRQ Levels
/////////////////////////////////////////////////

// Console USART
#define IRQL_USART1 14

// MII Data
#define IRQL_ETH_DATA 12

// MII IRQ (link change)
#define IRQL_ETH_LINK 15

// Wifi IRQ line (WF1)
#define IRQL_WIFI 9

// TIM7 - Wifi timer
#define IRQL_NRF24_TIMER 10

// SPI RX for touch sensor
#define IRQL_TOUCH 15

/*
// External Interrupts
#define IRQL_EXTI0 15	// KILL
#define IRQL_EXTI5_9 13
#define IRQL_EXTI10_15 10

#define IRQL_UART8 8

#define IRQL_SPI6 9

#define IRQL_SPI4 9

#define IRQL_SDMMC1 10

#define IRQL_USB 10

// External Flash
#define IRQL_QUADSPI 11

#define IRQL_KICK_CTRL 6

#define IRQL_NRF24 7
*/
