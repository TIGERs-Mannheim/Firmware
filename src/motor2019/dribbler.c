/*
 * dribbler.c
 *
 *  Created on: 20.08.2020
 *      Author: AndreR
 */

#include "main.h"
#include "system_init.h"
#include "usart1.h"
#include "adc.h"
#include "dribbler.h"
#include "dribbler_motor.h"
#include "dribbler_ctrl.h"

void DribblerMain()
{
	USART1Init();
	ADCInit(&DribblerCtrlPerPWMCycle, 4, 2);
	DribblerMotorInit();

	while(1)
	{
		ADCWaitFor1MsTrigger();

		USART1ReceiveCommand();

		DribblerCtrlUpdate();

		USART1SendFeedback();

		ADCUpdate();

		if(data.timeMs % 1024 == 0)
			TOGGLE_PRIMARY_LED();
	}
}
