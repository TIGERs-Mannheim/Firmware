/*
 * errors.h
 *
 *  Created on: 25.02.2011
 *      Author: AndreR
 */

#pragma once

#define ERROR_NOT_ENOUGH_MEMORY						0x0001
#define ERROR_INVALID_HANDLE						0x0002
#define ERROR_INVALID_PARAMETER						0x0003
#define ERROR_EVENT_QUEUE_FULL						0x0004

#define ERROR_ETH_SEND_DELAYED						0x0100
#define ERROR_ETH_EMPTY								0x0101
#define ERROR_ETH_RX_OVERFLOW						0x0102
#define ERROR_ETH_RX_TOO_LARGE						0x0103
#define ERROR_ETH_NO_LINK							0x0104

#define ERROR_NRF24_TIMEOUT							0x0200
#define ERROR_NRF24_NO_LINK							0x0201
#define ERROR_NRF24_INVALID_LENGTH					0x0202
#define ERROR_NRF24_BAD_ADDRESS						0x0203
#define ERROR_NRF24_INVALID_CHANNEL					0x0204
#define ERROR_NRF24_IRQ_MALFUNCTION					0x0205

#define ERROR_FLASH_WRITE_PROTECT					0x0200
#define ERROR_FLASH_NOT_ERASED						0x0201
#define ERROR_FLASH_INVALID_OPERATION				0x0202

#define ERROR_IGMP_BAD_PACKET						0x0300
#define ERROR_IGMP_BAD_CHECKSUM						0x0301

#define ERROR_ETHERNET_BAD_FRAME					0x0400
#define ERROR_ETHERNET_UNSUPPORTED_PROTOCOL 		0x0401
#define ERROR_ETHERNET_NOT_ADDRESSED				0x0402

#define ERROR_ARP_NOT_SUPPORTED						0x0500
#define ERROR_ARP_BAD_PACKET						0x0501

#define ERROR_IPV4_BAD_PACKET						0x0600
#define ERROR_IPV4_NOT_SUPPORTED					0x0601
#define ERROR_IPV4_BAD_CHECKSUM						0x0602

#define ERROR_UDP_BAD_PACKET						0x0700

#define ERROR_ICMP_BAD_PACKET						0x0800
#define ERROR_ICMP_WRONG_TYPE						0x0801

#define ERROR_SNTP_BAD_PACKET						0x1C00

// 0x0900 reserved for ATISP
#define ERROR_ATMEGA_MODE_FAULT						0x0910
#define ERROR_ATMEGA_OUT_OF_SYNC					0x0911
#define ERROR_ATMEGA_TIMEOUT						0x0912
#define ERROR_ATMEGA_UNKOWN_SIGNATURE				0x0913
#define ERROR_ATMEGA_PROGRAM_TOO_BIG				0x0914
#define ERROR_ATMEGA_VERIFICATION_FAILED			0x0915
#define ERROR_ATMEGA_EXCHANGE_CRC_WRONG				0x0916
#define ERROR_ATMEGA_EXCHANGE_START_FLAG_MISMATCH	0x0917
#define ERROR_ATMEGA_EXCHANGE_END_FLAG_MISMATCH		0x0918

#define ERROR_SPI_NO_FREE_SLOT						0x0A00
#define ERROR_SPI_EXCLUSIVE_LOCK					0x0A01
#define ERROR_SPI_INVALID_LENGTH					0x0A02
#define ERROR_SPI_INVALID_SLAVE						0x0A03
#define ERROR_SPI_BUSY								0x0A04

#define ERROR_FIFO_BLOCK_DELAYED					0x0B00
#define ERROR_FIFO_BLOCK_NOT_FULL					0x0B01

#define ERROR_FIFO_SYNCED_EMPTY						0x0C00
#define ERROR_FIFO_SYNCED_NOT_ENOUGH_MEMORY			0x0C01

#define ERROR_CONSOLE_NO_FREE_SLOT					0x0D00
#define ERROR_CONSOLE_NO_CMD_READY					0x0D01

#define ERROR_INTERCOM_NO_FREE_SLOT					0x0E00
#define ERROR_INTERCOM_DROPPED_PACKET				0x0E01

#define ERROR_TOKEN_MANAGER_NO_FREE_SLOT			0x0F00

#define ERROR_MULTIPROCESSOR_NO_FREE_SLOT			0x1000	// FIXME: double assignment, see ERROR_FIFO_PACKET_SYNCED_NOT_ENOUGH_MEMORY

#define ERROR_FIFO_PACKET_SYNCED_NOT_ENOUGH_MEMORY	0x1000
#define ERROR_FIFO_PACKET_SYNCED_EMPTY				0x1001

#define ERROR_SCCB_WRITE_NACK						0x1100

#define ERROR_SICP_BUSY								0x1200
#define ERROR_SICP_START_TIMEOUT					0x1201
#define ERROR_SICP_NO_HANDLER						0x1202
#define ERROR_SICP_SEND_FAILED						0x1203
#define ERROR_SICP_NO_FREE_SLOT						0x1204
#define ERROR_SICP_UNKNOWN							0x12FF

#define ERROR_AUDIO_INVALID_FILE					0x1300
#define ERROR_AUDIO_INVALID_FORMAT					0x1301

#define ERROR_USART_QUEUE_FULL						0x1400

#define ERROR_USB_URB_TOO_BIG						0x1500
#define ERROR_USB_INVALID_PIPE						0x1501
#define ERROR_USB_PIPE_ACTIVE						0x1502
#define ERROR_USB_CTRL_FAILED						0x1503
#define ERROR_USB_MSC_BOT_ERROR						0x1504
#define ERROR_USB_URB_TIMEOUT						0x1505

#define ERROR_CONFIG_FLASH_ERROR					0x1600

#define ERROR_FIFO_LIN_NOMEM						0x1700
#define ERROR_FIFO_LIN_EXCEED_RESV					0x1701
#define ERROR_FIFO_LIN_EMPTY						0x1702
#define ERROR_FIFO_LIN_NOT_OWNED					0x1703

#define ERROR_FIFO_CHAR_NOT_FOUND					0x1800
#define ERROR_FIFO_EMPTY							0x1801
#define ERROR_FIFO_FULL								0x1802

#define ERROR_NRF24EX_TIMEOUT						0x1900
#define ERROR_NRF24EX_INVALID_CHANNEL				0x1904
#define ERROR_NRF24EX_IRQ_MALFUNCTION				0x1905
#define ERROR_NRF24EX_QUEUE_EMPTY					0x1906
#define ERROR_NRF24EX_BAD_CHECKSUM					0x1907

#define ERROR_VISION_PARSE							0x1A00

#define ERROR_RST_PARSE								0x1B00

#define ERROR_SX1280_TIMEOUT						0x1C00
#define ERROR_SX1280_CMD_PROC						0x1C01
#define ERROR_SX1280_CMD_EXEC						0x1C02
#define ERROR_SX1280_TX_FAIL						0x1C03

#define ERROR_RF_QUEUE_EMPTY						0x1E00

#define ERROR_SPI_SYNC_TIMEOUT						0x2000
#define ERROR_SPI_SYNC_INVALID_TASK					0x2001

#define ERROR_COBS_ZERO_SIZE						0x3000

#define ERROR_I2C_SOFT_ARBITRATION_LOST				0x4000
#define ERROR_I2C_SOFT_NACK							0x4001

#define ERROR_FW_LOADER_TIMEOUT						0x5000
#define ERROR_FW_LOADER_NO_MEDIA					0x5001
#define ERROR_FW_LOADER_TOO_BIG						0x5002
#define ERROR_FW_LOADER_UP_TO_DATE					0x5003
#define ERROR_FW_LOADER_UNKNOWN_SOURCE				0x5004
