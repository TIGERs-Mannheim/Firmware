#include "drib_ir.h"
#include "generated/mcu_dribbler_code.h"

McuDribbler devDribIr;

void DevDribIrInit(UARTFifo* pUart, GPIOPinInterface* pRstPin, GPIOPinInterface* pBootPin, tprio_t taskPrio)
{
	McuDribblerInit(&devDribIr, pUart, pRstPin, pBootPin, mcu_dribbler_code_data, mcu_dribbler_code_size, taskPrio);
}
