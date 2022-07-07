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

const BuzzerSequence buzzSeqBeepFast = {
		(uint16_t[]){1, 100, 0, 100},
		4, 0,
};

const BuzzerSequence buzzSeqDoubleBeepSlow = {
		(uint16_t[]){1, 100, 0, 100, 1, 100, 0, 2000},
		8, 0,
};

const BuzzerSequence buzzSeqUp100 = {
		(uint16_t[]){2800, 100, 3000, 100, 3200, 100},
		6, 1,
};

const BuzzerSequence buzzSeqTada = {
		(uint16_t[]){3200, 100, 2900, 100, 2600, 100, 3200, 300, 2600, 100, 3200, 200},
		12, 1,
};

const BuzzerSequence buzzSeqDown100 = {
		(uint16_t[]){3200, 100, 3000, 100, 2800, 100},
		6, 1,
};

static uint16_t buzzSeqUp50Data[32];

const BuzzerSequence buzzSeqUp50 = {
		buzzSeqUp50Data, 32, 1,
};

static uint16_t buzzSeqDown50Data[32];

const BuzzerSequence buzzSeqDown50 = {
		buzzSeqDown50Data, 32, 1,
};

static uint16_t buzzSeqUp20Data[32];

const BuzzerSequence buzzSeqUp20 = {
		buzzSeqUp20Data, 32, 1,
};

static uint16_t buzzSeqUpDown20Data[64];

const BuzzerSequence buzzSeqUpDown20 = {
		buzzSeqUpDown20Data, 64, 0,
};

const BuzzerSequence buzzSeqFinalShort = {(uint16_t[]){
	1976, 803,
	2489, 133,
	2217, 133,
	2489, 535,
	1661, 535,
	1976, 803,
	2637, 133,
	2489, 133,
	2637, 267,
	2489, 267,
	2217, 535,
	2217, 803,
	2637, 133,
	2489, 133,
	2637, 535,
	1661, 535,
	1865, 803,
	2217, 133,
	1976, 133,
	2217, 267,
	1976, 267,
	1865, 267,
	2217, 267,
	1976, 803,
	2489, 133,
	2217, 133,
	2489, 535,
	1661, 535,
	1976, 803,
	2637, 133,
	2489, 133,
	2637, 267,
	2489, 267,
	2217, 535,
	2217, 803,
	2637, 133,
	2489, 133,
	2637, 535,
	1661, 535,
	1865, 803,
	2217, 133,
	1976, 133,
	2217, 267,
	1976, 267,
	1865, 267,
	2217, 267,
}, 92, 1,};

const BuzzerSequence buzzSeqCant = {(uint16_t[]){
	3951, 230,
	5274, 230,
	3951, 230,
	5274, 230,
	3951, 115,
	5274, 230,
	3951, 115,
	988, 115,
	3729, 115,
	3951, 230,
	3951, 115,
	3729, 115,
	3951, 115,
	3520, 115,
	988, 115,
	3322, 115,
	3520, 115,
	3322, 115,
	3136, 346,
	2637, 576,
	3951, 230,
	5274, 230,
	3951, 230,
	5274, 230,
	3951, 115,
	5274, 230,
	3951, 115,
	988, 115,
	3729, 115,
	3951, 230,
	3520, 230,
	3520, 230,
	1976, 115,
	3322, 115,
	3520, 230,
	4699, 115,
	4186, 230,
	3951, 230,
	3520, 346,
	3951, 230,
	5274, 230,
	3951, 230,
	5274, 230,
	3951, 115,
	5274, 230,
	3951, 115,
	988, 115,
	3729, 115,
	3951, 230,
	4699, 230,
	4699, 230,
	1976, 115,
	3951, 115,
	3520, 230,
	3136, 346,
	2637, 576,
	2637, 461,
	3136, 461,
	3951, 461,
	4699, 461,
	5588, 230,
	5274, 230,
	3729, 115,
	3951, 230,
	3136, 115,
	1047, 230,
	988, 230,
	880, 230,
	784, 230,
	659, 115,
	7902, 230,
	6272, 115,
	7902, 461,
	659, 115,
	7902, 230,
	6272, 115,
	7902, 461,
	659, 115,
	7902, 230,
	6272, 115,
	7459, 115,
	7902, 230,
	6272, 115,
	659, 346,
	5274, 576,
	659, 115,
	7902, 230,
	6272, 115,
	7902, 461,
	659, 115,
	7902, 230,
	6272, 115,
	7902, 461,
	659, 115,
	7902, 230,
	6272, 115,
	7459, 115,
	7902, 230,
	7040, 576,
	4699, 461,
	659, 115,
	7902, 230,
	6272, 115,
	7902, 461,
	659, 115,
	7902, 230,
	6272, 115,
	7902, 461,
	659, 115,
	7902, 230,
	6272, 115,
	7459, 115,
	7902, 230,
	6272, 115,
	659, 346,
	5274, 576,
	4186, 115,
	5274, 115,
	6272, 230,
	4435, 115,
	5274, 115,
	6272, 230,
	7459, 115,
	7902, 230,
	5274, 576,
	2637, 115,
	3136, 115,
	4186, 115,
	5274, 115,
	3729, 115,
	3951, 230,
	3136, 807,
}, 264, 1,};

const BuzzerSequence buzzSeqElevator = {(uint16_t[]){
	4186, 249,
	1245, 249,
	1175, 249,
	1245, 249,
	1175, 249,
	1245, 249,
	1175, 249,
	1245, 249,
	1175, 249,
	1245, 249,
	932, 499,
	4186, 249,
	4699, 249,
	932, 499,
	5920, 499,
	8372, 499,
	988, 249,
	7902, 249,
	8372, 499,
	5920, 499,
	932, 499,
	4186, 249,
	4699, 249,
	932, 499,
	5274, 499,
	7040, 499,
	988, 249,
	6645, 249,
	7040, 499,
	5274, 499,
	4699, 499,
	988, 249,
	4435, 249,
	4699, 499,
	3520, 499,
	3951, 1999,
	932, 499,
	4186, 249,
	4699, 249,
	932, 499,
	5920, 499,
	8372, 499,
	988, 249,
	7902, 249,
	8372, 499,
	5920, 499,
	932, 499,
	4186, 249,
	4699, 249,
	932, 499,
	5274, 499,
	7040, 499,
	988, 249,
	6645, 249,
	7040, 499,
	5274, 499,
	4699, 499,
	988, 249,
	4435, 249,
	4699, 499,
	3520, 499,
	3951, 499,
	932, 499,
	4186, 249,
	4699, 249,
	932, 499,
	5920, 499,
	8372, 499,
	988, 249,
	7902, 249,
	8372, 499,
	5920, 499,
	932, 499,
	4186, 249,
	4699, 249,
	932, 499,
	5274, 499,
	7040, 499,
	988, 249,
	6645, 249,
	7040, 499,
	5274, 499,
	4699, 499,
	988, 249,
	4435, 249,
	4699, 499,
	3520, 499,
	3951, 1999,
	932, 499,
	4186, 249,
	4699, 249,
	932, 499,
	5920, 499,
	8372, 499,
	988, 249,
	7902, 249,
	8372, 499,
	5920, 499,
	932, 499,
	4186, 249,
	4699, 249,
	932, 499,
	5274, 499,
	7040, 499,
	988, 249,
	6645, 249,
	7040, 499,
	5274, 499,
	4699, 499,
	988, 249,
	4435, 249,
	4699, 499,
	3520, 499,
	3951, 165,
}, 228, 1,};

//EyeOfTheTiger - Obere Stimme:
const BuzzerSequence buzzSeqEyeLead = {(uint16_t[]){
	3520, 550,
	0, 550,
	3520, 250,
	0, 160,
	3136, 250,
	0, 160,
	3520, 830,
	0, 550,
	3520, 250,
	0, 160,
	3136, 250,
	0, 160,
	3520, 830,
	0, 550,
	3520, 250,
	0, 160,
	3136, 250,
	0, 160,
	2794, 110,
}, 28, 1,};

//EyeOfTheTiger - Untere Stimme:
const BuzzerSequence buzzSeqEyeFollow = {(uint16_t[]){
	2349, 550,
	0, 550,
	2349, 250,
	0, 160,
	2093, 250,
	0, 160,
	2349, 830,
	0, 550,
	2349, 250,
	0, 160,
	2093, 250,
	0, 160,
	2349, 830,
	0, 550,
	2349, 250,
	0, 160,
	2093, 250,
	0, 160,
	1865, 110,
}, 28, 1,};

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

	// initalize buzzer sequences
	for(uint16_t i = 0; i < 32; i += 2)
	{
		buzzSeqUp50Data[i] = 2600+i*25;
		buzzSeqUp50Data[i+1] = 50;

		buzzSeqUp20Data[i] = 2600+i*25;
		buzzSeqUp20Data[i+1] = 20;

		buzzSeqUpDown20Data[i] = 2600+i*25;
		buzzSeqUpDown20Data[i+1] = 20;
	}

	for(uint16_t i = 32; i < 64; i += 2)
	{
		buzzSeqUpDown20Data[i] = 3400-(i-32)*25;
		buzzSeqUpDown20Data[i+1] = 20;

		buzzSeqDown50Data[i-32] = 3400-(i-32)*25;
		buzzSeqDown50Data[i-31] = 50;
	}
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
		default: BuzzerPlay(0); break;
	}
}
