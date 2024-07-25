#include "power_mon.h"

PwrINA226 devPowerMon;

void DevPowerMonInit(I2C* pI2C, tprio_t taskPrio)
{
	PwrINA226Init(&devPowerMon, pI2C, taskPrio);
}
