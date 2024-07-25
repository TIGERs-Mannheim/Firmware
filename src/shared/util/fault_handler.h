#pragma once

#include "hal/init_hal.h"

typedef void(*FaultHandlerEmergencyStopFunc)();

typedef struct _FaultHandler
{
	USART_TypeDef* pPrintUart;
	FaultHandlerEmergencyStopFunc pEmergencyStopFunc;
} FaultHandler;

extern FaultHandler faultHandlerObj;

void FaultHandlerInit(USART_TypeDef* pPrintUsart, FaultHandlerEmergencyStopFunc emergencyStopFunc);
