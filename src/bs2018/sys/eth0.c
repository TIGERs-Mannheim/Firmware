#include "hal/eth.h"
#include "../constants.h"
#include "hal/init_hal.h"
#include "errors.h"
#include <stdio.h>
#include <string.h>

#define DP83848_PHY_ADDRESS 0x01

Eth eth0 __attribute__((aligned(16), section(".eth")));

CH_IRQ_HANDLER(Vector134) // ETH
{
	CH_IRQ_PROLOGUE();
	EthIRQ(&eth0);
	CH_IRQ_EPILOGUE();
}

CH_IRQ_HANDLER(Vector9C) // EXTI9_5
{
	CH_IRQ_PROLOGUE();

	if(EXTI->PR & EXTI_PR_PR5)
	{
		EXTI->PR = EXTI_PR_PR5;
		EthMiiIRQ(&eth0);
	}

	CH_IRQ_EPILOGUE();
}

void Eth0Init(tprio_t prio, uint32_t irqLevelMii)
{
	GPIOInitData gpioInit;

	// select MII mode
	SYSCFG->PMC &= ~SYSCFG_PMC_MII_RMII_SEL;

	// configure pins
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.alternate = 11;	// ETH
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_50MHZ;
	gpioInit.pupd = GPIO_PUPD_NONE;
	GPIOInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_7, &gpioInit);
	GPIOInit(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13, &gpioInit);
	GPIOInit(GPIOC, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, &gpioInit);

	gpioInit.mode = GPIO_MODE_EXTI;
	gpioInit.pupd = GPIO_PUPD_NONE;
	gpioInit.extiTrigger = GPIO_EXTI_TRIG_FALLING;
	GPIOInit(GPIOE, GPIO_PIN_5, &gpioInit);

	RCC->AHB1ENR |= RCC_AHB1ENR_ETHMACEN | RCC_AHB1ENR_ETHMACTXEN | RCC_AHB1ENR_ETHMACRXEN;

	EthData data;
	data.pReg = ETH;
	data.phyAddr = DP83848_PHY_ADDRESS;

	EthInit(&eth0, &data, prio);

	NVICEnableIRQ(EXTI9_5_IRQn, irqLevelMii);
	NVICEnableIRQ(ETH_IRQn, irqLevelMii);
}
