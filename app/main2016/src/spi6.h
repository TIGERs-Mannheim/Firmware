/*
 * spi6.h
 *
 *  Created on: 26.10.2015
 *      Author: AndreR
 */

#ifndef SPI6_H_
#define SPI6_H_

#include "ch.h"
#include "util/spi_sync.h"
#include "util/lag_element.h"

#define SPI6_PWR_ADC_SAMPLES 16
#define SPI6_KICK_SAMPLES 16

typedef struct _SPI6Global
{
	SPISync bus;

	struct _power
	{
		SPISyncSlave slave;

		uint8_t tx[SPI6_PWR_ADC_SAMPLES*3*2];
		uint16_t rx[SPI6_PWR_ADC_SAMPLES*3];

		uint32_t sampleTime;	// [us]
	} power;

	struct _kick
	{
		SPISyncSlave slave;

		uint8_t tx[4*SPI6_KICK_SAMPLES];
		uint16_t rx[2*SPI6_KICK_SAMPLES];

		uint32_t sampleTime;
	} kick;

	struct _imu
	{
		SPISyncSlave slave;

		uint8_t tx[16];
		uint8_t rx[16];

		uint32_t sampleTime;

		float acc[3];	// [m/s^2]
		float temp; 	// [°C]
		float gyr[3];	// [rad/s]
	} imu;

	float vIr;

	float vIrOn;
	float vIrOff;
} SPI6Global;

extern SPI6Global spi6;

void SPI6Init();
void SPI6Task(void* params);

#endif /* SPI6_H_ */
