#include "rng.h"
#include "hal/init_hal.h"

void RNGInit()
{
	RCC->AHB2ENR |= RCC_AHB2ENR_RNGEN;

	RNG->CR = RNG_CR_RNGEN;
}

uint32_t RNGGetRandomU32()
{
	while((RNG->SR & RNG_SR_DRDY) == 0);

	return RNG->DR;
}
