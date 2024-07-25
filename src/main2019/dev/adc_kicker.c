#include "adc_kicker.h"

ADCKicker devAdcKicker;

void DevAdcKickerInit(SPI* pSPI, tprio_t taskPrio)
{
	ADCKickerInit(&devAdcKicker, pSPI, (GPIOPin){ GPIOG, GPIO_PIN_12 }, SPI_CFG1_BRDIV16, taskPrio);
}
