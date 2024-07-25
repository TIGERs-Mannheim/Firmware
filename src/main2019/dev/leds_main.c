#include "leds_main.h"

LEDMono devLedMainGreen;
LEDMono devLedMainRed;

void DevLedsMainInit()
{
	LEDMonoInit(&devLedMainGreen, (GPIOPin){ GPIOC, GPIO_PIN_0 }, 0);
	LEDMonoInit(&devLedMainRed, (GPIOPin){ GPIOC, GPIO_PIN_3 }, 0);
}
