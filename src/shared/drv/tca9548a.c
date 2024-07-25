#include "tca9548a.h"

void TCA9548AInit(TCA9548A* pTca, I2C* pI2C)
{
	pTca->pI2C = pI2C;
	pTca->channels = 0;
}

int16_t TCA9548ASetChannels(TCA9548A* pTca, uint8_t ch)
{
	uint8_t* pTx;

	I2CAcquire(pTca->pI2C, &pTx, 0);

	pTx[0] = ch;

	int16_t result = I2CWrite(pTca->pI2C, 0x70, 1);
	I2CRelease(pTca->pI2C);

	if(result)
		return result;

	pTca->channels = ch;

	return 0;
}
