#include "imu.h"

ImuICM20689 devImu;

void DevImuInit(SPI* pSPI, tprio_t taskPrio)
{
	ImuICM20689Init(&devImu, pSPI, (GPIOPin){ GPIOB, GPIO_PIN_7 }, SPI_CFG1_BRDIV16, taskPrio);
}
