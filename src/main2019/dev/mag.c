#include "mag.h"

MagLIS3 devMag;

void DevMagInit(SPI* pSPI, tprio_t taskPrio)
{
	MagLIS3Init(&devMag, pSPI, (GPIOPin){ GPIOC, GPIO_PIN_13 }, SPI_CFG1_BRDIV16, taskPrio);
}
