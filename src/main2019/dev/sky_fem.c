#include "sky_fem.h"
#include "ch.h"

SKY66112 devSkyFem;

void DevSkyFemInit()
{
	// SKY66112 setup
	devSkyFem.antPin = (GPIOPin){0, 0 };
	devSkyFem.cpsPin = (GPIOPin){ GPIOF, GPIO_PIN_3 };
	devSkyFem.crxPin = (GPIOPin){ GPIOF, GPIO_PIN_4 };
	devSkyFem.ctxPin = (GPIOPin){ GPIOF, GPIO_PIN_5 };

	SKY66112Init(&devSkyFem);
	SKY66112SetMode(&devSkyFem, SKY66112_MODE_OFF);
}
