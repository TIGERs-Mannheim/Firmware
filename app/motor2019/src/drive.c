/*
 * drive.c
 *
 *  Created on: 20.08.2020
 *      Author: AndreR
 */

#include "main.h"
#include "system_init.h"
#include "usart1.h"
#include "adc.h"
#include "drive_motor.h"
#include "drive_ctrl.h"

#if ADC_NUM_SAMPLES_PER_MS != MOTOR_EXCHANGE_MISO_MAX_MEAS
#error Number of ADC Samples and motor exchange measurements must match
#endif

void DriveMain()
{
	USART1Init();
	ADCInit(&DriveCtrlUpdatePerPWMCycle, 7, 1);
	DriveMotorInit();
	DriveCtrlInit();

	// configure independent watchdog
	IWDG->KR = 0xCCCC;
	IWDG->KR = 0x5555;
	IWDG->PR = 0; // div4 => 10kHz
	IWDG->RLR = 100; // => 10ms timeout
	while(IWDG->SR != 0);
	IWDG->KR = 0xAAAA;

	while(1)
	{
		ADCWaitFor1MsTrigger();

		USART1ReceiveCommand();

		DriveCtrlUpdate();
		DriveMotorUpdate();

		USART1SendFeedback();

		ADCUpdate();

		IWDG->KR = 0xAAAA; // reload IWDG counter value to avoid reset

		if(data.timeMs % 512 == 0)
			TOGGLE_PRIMARY_LED();
	}
}
