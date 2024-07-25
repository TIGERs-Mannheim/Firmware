#include "i2c2.h"
#include "hal/i2c_lld_hw.h"

I2C i2c2;

static I2CLLDHW i2c2Driver;
static uint8_t i2c2Tx[64];
static uint8_t i2c2Rx[64];

CH_IRQ_HANDLER(VectorC4) // I2C2_EV
{
	CH_IRQ_PROLOGUE();
	I2CLLDHWEventIRQ(&i2c2Driver);
	CH_IRQ_EPILOGUE();
}

CH_IRQ_HANDLER(VectorC8) // I2C2_ER
{
	CH_IRQ_PROLOGUE();
	I2CLLDHWErrorIRQ(&i2c2Driver);
	CH_IRQ_EPILOGUE();
}

void I2C2Init(uint32_t irqLevel)
{
	RCC->APB1LENR |= RCC_APB1LENR_I2C2EN;
	__DSB();

	I2CLLDHWData i2c2Init;
	i2c2Init.pRegister = I2C2;
	i2c2Init.sdaPin = (GPIOPinAlt){ GPIOF, GPIO_PIN_0, 4 };
	i2c2Init.sclPin = (GPIOPinAlt){ GPIOF, GPIO_PIN_1, 4 };
	i2c2Init.pBufTx = i2c2Tx;
	i2c2Init.pBufRx = i2c2Rx;
	i2c2Init.bufSize = sizeof(i2c2Tx);

	I2CLLDHWInit(&i2c2Driver, &i2c2Init);
	I2CInit(&i2c2, &i2c2Driver.base);

	NVICEnableIRQ(I2C2_EV_IRQn, irqLevel);
	NVICEnableIRQ(I2C2_ER_IRQn, irqLevel);
}
