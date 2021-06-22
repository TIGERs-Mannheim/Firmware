/*
 * constants.h
 *
 *  Created on: 24.10.2015
 *      Author: AndreR
 */

#pragma once

/////////////////////////////////////////////////
// Buffer Sizes
/////////////////////////////////////////////////

// USART2 - USB
#define USART2_TX_SIZE 1000
#define USART2_RX_SIZE 300

//// UART8 - EXT
//#define UART8_TX_SIZE 1000
//#define UART8_RX_SIZE 1000
//#define UART8_MAX_PACKET_SIZE 64

// CONSOLE
#define CONSOLE_RX_SIZE 1000
#define CONSOLE_TX_SIZE 2000
#define CONSOLE_PRINT_SIZE 400

/////////////////////////////////////////////////
// IRQ Levels
/////////////////////////////////////////////////

// USART3
#define IRQL_USART2 14

// External Interrupts
#define IRQL_EXTI3 15	// KILL
#define IRQL_EXTI10_15 15 // USB power change
//#define IRQL_EXTI5_9 13

#define IRQL_I2C2 15

#define IRQL_SPI4 9

#define IRQL_SPI2 8

#define IRQL_KICK_CTRL 6

#define IRQL_USB 10

#define IRQL_SDMMC1 10

#define IRQL_WIFI 7

#define IRQL_UART8 8 // IR

#define IRQL_MOTOR_COMMS 8

#define IRQL_EXT_COMM 12

#define IRQL_I2C_SOFT 15

#define IRQL_MICROPHONE 15
