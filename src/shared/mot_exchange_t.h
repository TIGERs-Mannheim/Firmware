/*
 * mot_exchange_t.h
 *
 *  Created on: 19.01.2014
 *      Author: AndreR
 */

#ifndef MOT_EXCHANGE_T_H_
#define MOT_EXCHANGE_T_H_

#include <stdint.h>

#define MOT_EXCHANGE_DATA_MSG_SIZE 512

typedef struct __attribute__((packed, aligned(2))) _SPIMot2MainData
{
	uint8_t blockData[MOT_EXCHANGE_DATA_MSG_SIZE];
	uint16_t bytesWritten;
} SPIMot2MainData;

typedef struct __attribute__((packed, aligned(2))) _SPIMain2MotData
{
	uint8_t blockData[MOT_EXCHANGE_DATA_MSG_SIZE];
	uint16_t bytesWritten;
} SPIMain2MotData;

#endif /* MOT_EXCHANGE_T_H_ */
