#pragma once

#include "drv/imu_icm20689.h"

extern ImuICM20689 devImu;

void DevImuInit(SPI* pSPI, tprio_t taskPrio);
