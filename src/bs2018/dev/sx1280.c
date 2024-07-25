#include "sx1280.h"
#include "ch.h"
#include "sx1280_def.h"

SX1280LLD devSX1280;

CH_FAST_IRQ_HANDLER(Vector178) // DCMI, used as SX1280 high prio IRQ
{
	SX1280LLDHighPrioIRQ(&devSX1280);
}

CH_FAST_IRQ_HANDLER(Vector60) // EXTI2
{
	if(EXTI->PR & EXTI_PR_PR2)
	{
		EXTI->PR = EXTI_PR_PR2;
		SX1280LLDBusyIRQ(&devSX1280);
	}
}

CH_FAST_IRQ_HANDLER(Vector64) // EXTI3
{
	if(EXTI->PR & EXTI_PR_PR3)
	{
		EXTI->PR = EXTI_PR_PR3;
		SX1280LLDDioIRQ(&devSX1280);
	}
}

void DevSX1280Init(SPILLD* pSPI, TimerSimpleLLD* pTimer1us, uint32_t irqLevelHighPrio, uint32_t irqLevelPin)
{
	SX1280LLDData sxDrvInit;
	sxDrvInit.pSpi = pSPI;
	sxDrvInit.csPin = (GPIOPin){ GPIOD, GPIO_PIN_6 };
	sxDrvInit.prescaler = SPI_CR1_BRDIV4;
	sxDrvInit.busyPin = (GPIOPin){ GPIOD, GPIO_PIN_2 };
	sxDrvInit.dioPins[0] = (GPIOPin){ GPIOD, GPIO_PIN_3 };
	sxDrvInit.dioPins[1] = (GPIOPin){ 0, 0 };
	sxDrvInit.dioPins[2] = (GPIOPin){ 0, 0 };
	sxDrvInit.pTimer1us = pTimer1us;
	sxDrvInit.highPrioIRQn = DCMI_IRQn;

	SX1280LLDInit(&devSX1280, &sxDrvInit);

	NVICEnableIRQ(DCMI_IRQn, irqLevelHighPrio);
	NVICEnableIRQ(EXTI2_IRQn, irqLevelPin);
	NVICEnableIRQ(EXTI3_IRQn, irqLevelPin);
}
