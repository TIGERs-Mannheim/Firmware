#include "i2c5.h"
#include "hal/i2c_lld_soft.h"

I2C i2c5;

static I2CLLDSoft i2c5Driver;
static uint8_t i2c5Tx[64];
static uint8_t i2c5Rx[64];

CH_IRQ_HANDLER(VectorEC) // TIM8_BRK_TIM12, TIM12 => I2CSoft tick
{
	CH_IRQ_PROLOGUE();
	TIM12->SR = 0;
	I2CLLDSoftTick(&i2c5Driver);
	CH_IRQ_EPILOGUE();
}

void I2C5Init(uint32_t irqLevel)
{
	I2CLLDSoftData i2cSoftInit;
	i2cSoftInit.pTimer = TIM12;
	i2cSoftInit.sclPin = (GPIOPin){GPIOG, GPIO_PIN_4};
	i2cSoftInit.sdaPin = (GPIOPin){GPIOG, GPIO_PIN_3};
	i2cSoftInit.pBufTx = i2c5Tx;
	i2cSoftInit.pBufRx = i2c5Rx;
	i2cSoftInit.bufSize = sizeof(i2c5Tx);

	I2CLLDSoftInit(&i2c5Driver, &i2cSoftInit);
	I2CInit(&i2c5, &i2c5Driver.base);

	RCC->APB1LENR |= RCC_APB1LENR_TIM12EN;
	__DSB();

	/// 100MHz base clk
	TIM12->CR1 = 0;
	TIM12->CR2 = 0;
	TIM12->CNT = 0;
	TIM12->PSC = 0; // => 100MHz
	TIM12->ARR = 399; // => 250kHz => 83.333kHz I2C frequency
	TIM12->SR = 0;
	TIM12->DIER = TIM_DIER_UIE;
	TIM12->CR1 |= TIM_CR1_OPM;

	NVICEnableIRQ(TIM8_BRK_TIM12_IRQn, irqLevel);
}
