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

/////////////////////////////////////////////////
// Task Priorities
/////////////////////////////////////////////////
#define TASK_PRIO_ROUTER_TX		(NORMALPRIO + 50)
#define TASK_PRIO_ROUTER_RX		(NORMALPRIO + 30)
#define TASK_PRIO_BASE_STATION	(NORMALPRIO + 15)
#define TASK_PRIO_NET_IF		(NORMALPRIO + 10)
#define TASK_PRIO_ETH			(NORMALPRIO +  9)
#define TASK_PRIO_SSL_VISION	(NORMALPRIO +  0)
#define TASK_PRIO_SHELL			(NORMALPRIO -  5)
#define TASK_PRIO_CLI_SERIAL	(NORMALPRIO - 10)
#define TASK_PRIO_TOUCH			(NORMALPRIO - 35)
#define TASK_PRIO_PRESENTER		(NORMALPRIO - 40)

/////////////////////////////////////////////////
// IRQ Levels
/////////////////////////////////////////////////

#define IRQL_WIFI_HIGH_PRIO	0
#define IRQL_WIFI_SPI_DONE	1
#define IRQL_WIFI_PIN		2
#define IRQL_WIFI_TIMEOUT	3

#define IRQL_WIFI_LOW_PRIO	5

// Console USART
#define IRQL_USART1 14

// Network
#define IRQL_ETH 12

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
