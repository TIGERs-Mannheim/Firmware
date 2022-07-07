/*
 * tca9548a.c
 *
 *  Created on: 13.03.2020
 *      Author: AndreR
 */

#include "util/tca9548a.h"

void TCA9548AInit(TCA9548A* pTca, I2CSoft* pI2C)
{
	pTca->pI2C = pI2C;
	pTca->channels = 0;
}

int16_t TCA9548ASetChannels(TCA9548A* pTca, uint8_t ch)
{
	int16_t result = I2CSoftWrite(pTca->pI2C, 0x70, 1, &ch);
	if(result)
		return result;

	pTca->channels = ch;

	return 0;
}
