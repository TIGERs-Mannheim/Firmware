#include "sx1280.h"
#include "sx1280_def.h"
#include "ch.h"

SX1280LLD devSX1280;

CH_FAST_IRQ_HANDLER(Vector178) // DCMI, used as SX1280 high prio IRQ
{
	SX1280LLDHighPrioIRQ(&devSX1280);
}

CH_FAST_IRQ_HANDLER(Vector68) // EXTI4
{
	if(EXTI_D1->PR1 & EXTI_PR1_PR4)
	{
		EXTI_D1->PR1 = EXTI_PR1_PR4;
		SX1280LLDBusyIRQ(&devSX1280);
	}
}

CH_FAST_IRQ_HANDLER(Vector9C) // EXTI9_5
{
	if(EXTI_D1->PR1 & EXTI_PR1_PR5)
	{
		EXTI_D1->PR1 = EXTI_PR1_PR5;
		SX1280LLDDioIRQ(&devSX1280);
	}
}

void DevSX1280Init(SPILLD* pSPI, TimerSimpleLLD* pTimer1us, uint32_t irqLevelHighPrio, uint32_t irqLevelPin)
{
	SX1280LLDData sxDrvInit;
	sxDrvInit.pSpi = pSPI;
	sxDrvInit.csPin = (GPIOPin){ GPIOF, GPIO_PIN_10 };
	sxDrvInit.prescaler = SPI_CFG1_BRDIV4;
	sxDrvInit.busyPin = (GPIOPin){ GPIOC, GPIO_PIN_4 };
	sxDrvInit.dioPins[0] = (GPIOPin){ GPIOC, GPIO_PIN_5 };
	sxDrvInit.dioPins[1] = (GPIOPin){ 0, 0 };
	sxDrvInit.dioPins[2] = (GPIOPin){ 0, 0 };
	sxDrvInit.pTimer1us = pTimer1us;
	sxDrvInit.highPrioIRQn = DCMI_IRQn;

	SX1280LLDInit(&devSX1280, &sxDrvInit);

	NVICEnableIRQ(DCMI_IRQn, irqLevelHighPrio);
	NVICEnableIRQ(EXTI4_IRQn, irqLevelPin);
	NVICEnableIRQ(EXTI9_5_IRQn, irqLevelPin);
}
