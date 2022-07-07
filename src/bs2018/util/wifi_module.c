/*
 * wifi_module.c
 *
 *  Created on: 24.10.2017
 *      Author: AndreR
 */

#include "wifi_module.h"
#include "errors.h"
#include "util/log.h"

#define POWER_ON() GPIOSet(pModule->pwrPin.pPort, pModule->pwrPin.pin)
#define POWER_OFF() GPIOReset(pModule->pwrPin.pPort, pModule->pwrPin.pin)

static void resetSX1280(WifiModule* pModule)
{
	POWER_OFF();
	chThdSleepMilliseconds(50);

	POWER_ON();
	chThdSleepMilliseconds(10);

	uint16_t status;
	SX1280GetStatus(&pModule->radio, &status);

	SX1280Setup(&pModule->radio, &pModule->radioSettings);
}

void WifiModuleInit(WifiModule* pModule, WifiModuleData* pData)
{
	// set up power pin, controls low-noise LDO for radio and PA/LNA chips
	pModule->pwrPin = pData->pwrPin;

	POWER_OFF();

	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_OUTPUT;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.pupd = GPIO_PUPD_NONE;
	GPIOInit(pModule->pwrPin.pPort, pModule->pwrPin.pin, &gpioInit);

	// turn off and deplete capacitors to force reset on SX1280
	chThdSleepMilliseconds(50);

	// Set up Front End Module (FEM)
	pModule->fem = *pData->pFEMInit;
	SKY66112Init(&pModule->fem);
	SKY66112SetMode(&pModule->fem, SKY66112_MODE_OFF);
	SKY66112UseAntenna(&pModule->fem, 0);

	POWER_ON();
	chThdSleepMilliseconds(10);

	// store radio settings
	pModule->radioSettings = *pData->pRadioSettings;

	// Set up radio module
	SX1280Init(&pModule->radio, pData->pRadioInit);

	SX1280Setup(&pModule->radio, &pModule->radioSettings);
}

int16_t WifiModuleTransmitAndReceive(WifiModule* pModule,  uint8_t antenna, uint8_t address,
		uint8_t numBytes, void* pTxRxData, SX1280RxResult* pRxResult)
{
	SKY66112UseAntenna(&pModule->fem, antenna);

	SX1280SetAddress(&pModule->radio, address);

	SKY66112SetMode(&pModule->fem, SKY66112_MODE_TX);

	int16_t result = SX1280Transmit(&pModule->radio, numBytes, pTxRxData);
	if(result == ERROR_SX1280_TIMEOUT)
	{
		LogError("SX1280Transmit time out => reset");
		resetSX1280(pModule);
	}

	SKY66112SetMode(&pModule->fem, SKY66112_MODE_RX);

	SX1280Receive(&pModule->radio, numBytes, pTxRxData, pRxResult, US2ST(750));

	SKY66112SetMode(&pModule->fem, SKY66112_MODE_OFF);

	return result;
}
