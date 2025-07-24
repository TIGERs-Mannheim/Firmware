#include "hal/cpuid.h"

volatile uint32_t* CPUIDGet()
{
	return (volatile uint32_t*)0x1FF0F420;
}
