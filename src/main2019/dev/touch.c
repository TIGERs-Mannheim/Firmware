#include "touch.h"

TouchAD7843 devTouch;

void DevTouchInit(SPI* pSPI, tprio_t taskPrio)
{
	TouchAD7843Init(&devTouch, pSPI, (GPIOPin){GPIOG, GPIO_PIN_0}, (GPIOPin){GPIOA, GPIO_PIN_4}, SPI_CFG1_BRDIV128, taskPrio);
}
