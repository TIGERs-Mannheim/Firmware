#include "raspberry_pi.h"

MpuExt devRaspberryPi;

void DevRaspberryPiInit(UartDma* pUart, tprio_t taskPrio)
{
	MpuExtInit(&devRaspberryPi, pUart, taskPrio);
}
