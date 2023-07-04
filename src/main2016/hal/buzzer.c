/*
 * buzzer.c
 *
 *  Created on: 23.12.2010
 *      Author: AndreR
 */

#include "buzzer.h"

#include "stm32f746xx.h"
#include "util/init_hal.h"
#include "commands.h"

Buzzer buzzer;

#define BUZZER_ON() (TIM10->CCMR1 = TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2)
#define BUZZER_OFF() (TIM10->CCMR1 = TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_2)

void BuzzerInit()
{
	RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;	// enable TIM10 clock

	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.alternate = 3;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.pupd = GPIO_PUPD_NONE;
	GPIOInit(GPIOB, GPIO_PIN_8, &gpioInit);

	TIM10->CR1 = TIM_CR1_ARPE;	// enable ARR preload
	BUZZER_OFF();
	TIM10->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P;	// enable CC1 output and set polarity to active low
	TIM10->PSC = 215;		// prescaler
	TIM10->ARR = 320;	// period
	TIM10->CCR1 = 103;	// pulse
	TIM10->EGR = TIM_EGR_UG;	// generate update event to take new ARR value
	TIM10->CR1 |= TIM_CR1_CEN;	// enable timer

	SongsLoad();
}

void BuzzerTone(uint16_t freq)
{
	if(freq == 1)
		freq = 3150;

	if(freq < 16 || freq > 20000)
	{
		BUZZER_OFF();
		return;
	}

	BUZZER_ON();

	uint16_t arr = 1000000/freq;

	TIM10->ARR = arr;
	TIM10->CCR1 = arr/2;
	TIM10->EGR = TIM_EGR_UG;
}

static void buzzTimer(void* param)
{
	(void)param;

	if(!buzzer.pSeq)
		return;

	if(buzzer.curTone == 0 && buzzer.pSeq->once)
	{
		buzzer.pSeq = 0;
		BuzzerTone(0);
		return;
	}

	BuzzerTone(buzzer.pSeq->pSong[buzzer.curTone]);

	chSysLockFromISR();
	chVTSetI(&buzzer.timer, MS2ST(buzzer.pSeq->pSong[buzzer.curTone+1]), &buzzTimer, 0);
	chSysUnlockFromISR();

	buzzer.curTone += 2;
	buzzer.curTone %= buzzer.pSeq->numTones;
}

void BuzzerPlay(const BuzzerSequence* pSeq)
{
	if(pSeq == buzzer.pSeq && pSeq != 0)
		return;

	chVTReset(&buzzer.timer);
	buzzer.pSeq = 0;
	BuzzerTone(0);

	if(pSeq == 0)
		return;

	if(pSeq->numTones % 2 == 1 || pSeq->numTones == 0)
		return;

	buzzer.curTone = 2;
	buzzer.pSeq = pSeq;

	BuzzerTone(buzzer.pSeq->pSong[0]);
	chVTSet(&buzzer.timer, MS2ST(buzzer.pSeq->pSong[1]), &buzzTimer, 0);
}

void BuzzerPlayId(uint8_t id)
{
	switch(id)
	{
		case BUZZ_UP20:	BuzzerPlay(&buzzSeqUp20); break;
		case BUZZ_BEEP_FAST: BuzzerPlay(&buzzSeqBeepFast); break;
		case BUZZ_UP50: BuzzerPlay(&buzzSeqUp50); break;
		case BUZZ_DOWN50: BuzzerPlay(&buzzSeqDown50); break;
		case BUZZ_UPDOWN20: BuzzerPlay(&buzzSeqUpDown20); break;
		case BUZZ_DOUBLE_SLOW: BuzzerPlay(&buzzSeqDoubleBeepSlow); break;
		case BUZZ_UP100: BuzzerPlay(&buzzSeqUp100); break;
		case BUZZ_DOWN100: BuzzerPlay(&buzzSeqDown100); break;
		case BUZZ_TADA: BuzzerPlay(&buzzSeqTada); break;
		case BUZZ_SONG_FINAL: BuzzerPlay(&buzzSeqFinalShort); break;
		case BUZZ_SONG_CANTINA: BuzzerPlay(&buzzSeqCant); break;
		case BUZZ_SONG_EYE_LEAD: BuzzerPlay(&buzzSeqEyeLead); break;
		case BUZZ_SONG_EYE_FOLLOW: BuzzerPlay(&buzzSeqEyeFollow); break;
		case BUZZ_SONG_ELEVATOR: BuzzerPlay(&buzzSeqElevator); break;
		case BUZZ_SONG_MACARENA: BuzzerPlay(&buzzSeqMacarena); break;
		default: BuzzerPlay(0); break;
	}
}
