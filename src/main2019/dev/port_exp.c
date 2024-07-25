#include "port_exp.h"

DevPortExp devPortExp;

void DevPortExpInit(I2C* pI2C, tprio_t taskPrio)
{
	// Port expander on power board
	IoTCA9539Init(&devPortExp.tca, pI2C, (GPIOPin){ GPIOG, GPIO_PIN_13 }, taskPrio);

	for(uint8_t i = 0; i < 16; i++)
		IoTCA9539GPIOPinInit(&devPortExp.pins[i], &devPortExp.tca, 1 << i);

	/** TC9539 pinout
	+-------+-------+-------+-------+-------+-------+-----+----+
	| P0    | P1    | P2    | P3    | P4    | P5    | P6  | P7 |
	| SRST5 | BOOT5 | SRST4 | BOOT4 | SRST1 | BOOT1 | -   | -  |
	|      IR       |     Drib      |      M2       |     |    |
	+-------+-------+-------+-------+-------+-------+-----+----+
	| P8    | P9    | P10   | P11   | P12   | P13   | P14 | P15|
	| SRST2 | BOOT2 | SRST3 | BOOT3 | SRST0 | BOOT0 | -   | -  |
	|      M3       |      M4       |      M1       |     |    |
	+---------------+---------------+---------------+-----+----+
	*/
}
