#pragma once

#include "ch.h"
#include "hal/init_hal.h"
#include "util/songs.h"
#include "util/shell_cmd.h"

typedef struct _BuzzerData
{
	GPIOPinAlt outputPin;

	TIM_TypeDef* pTim;
	uint32_t psc1MHz;
	volatile uint32_t* pCCR;
	volatile uint32_t* pCCMR;
	uint32_t ccmrBuzzerOn;
	uint32_t ccmrBuzzerOff;
} BuzzerData;

typedef struct _Buzzer
{
	TIM_TypeDef* pTim;
	volatile uint32_t* pCCR;
	volatile uint32_t* pCCMR;
	uint32_t ccmrBuzzerOn;
	uint32_t ccmrBuzzerOff;

	virtual_timer_t timer;

	const BuzzerSequence* pSeq;
	uint16_t curTone;

	ShellCmdHandler cmdHandler;
} Buzzer;

void BuzzerInit(Buzzer* pBuzz, BuzzerData* pInit);
void BuzzerTone(Buzzer* pBuzz, uint16_t freq);
void BuzzerPlay(Buzzer* pBuzz, const BuzzerSequence* pSeq);
void BuzzerPlayId(Buzzer* pBuzz, uint8_t id);
