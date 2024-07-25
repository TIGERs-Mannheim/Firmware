#include "gpio_pin_st.h"

static void write(GPIOPinInterface* pGPIO, uint8_t level)
{
	GPIOPinSt* pPin = (GPIOPinSt*)pGPIO;

	if(level)
		GPIOSet(pPin->pPort, pPin->pinMask);
	else
		GPIOReset(pPin->pPort, pPin->pinMask);
}

static uint8_t read(GPIOPinInterface* pGPIO)
{
	GPIOPinSt* pPin = (GPIOPinSt*)pGPIO;

	if(pPin->pPort->IDR & pPin->pinMask)
		return 1;

	return 0;
}

static void configure(GPIOPinInterface* pGPIO, uint8_t isOutput)
{
	GPIOPinSt* pPin = (GPIOPinSt*)pGPIO;

	uint8_t pinNr = __CLZ(__RBIT(pPin->pinMask));

	pPin->pPort->MODER &= ~(GPIO_MODER_MODE0 << (pinNr*2));
	if(isOutput)
		pPin->pPort->MODER |= (1 << (pinNr*2));
}

void GPIOPinStInit(GPIOPinSt* pPin, GPIO_TypeDef* pPort, uint16_t pinMask)
{
	pPin->pPort = pPort;
	pPin->pinMask = pinMask;
	pPin->pin.write = &write;
	pPin->pin.read = &read;
	pPin->pin.configure = &configure;
}
