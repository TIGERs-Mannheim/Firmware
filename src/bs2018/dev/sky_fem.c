#include "sky_fem.h"
#include "ch.h"

SKY66112 devSkyFem;

void DevSkyFemInit()
{
	// SKY66112 setup
	devSkyFem.antPin = (GPIOPin){ GPIOE, GPIO_PIN_6 };
	devSkyFem.cpsPin = (GPIOPin){ GPIOE, GPIO_PIN_2 };
	devSkyFem.crxPin = (GPIOPin){ GPIOE, GPIO_PIN_4 };
	devSkyFem.ctxPin = (GPIOPin){ GPIOE, GPIO_PIN_3 };

	SKY66112Init(&devSkyFem);
	SKY66112SetMode(&devSkyFem, SKY66112_MODE_OFF);
	SKY66112UseAntenna(&devSkyFem, 0);
}
