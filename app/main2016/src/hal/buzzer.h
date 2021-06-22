/*
 * buzzer.h
 *
 *  Created on: 23.12.2010
 *      Author: AndreR
 */

#ifndef __BUZZER_H__
#define __BUZZER_H__

#include "ch.h"

typedef struct _BuzzerSequence
{
	uint16_t* pSong;
	uint16_t numTones;
	uint8_t once;
} BuzzerSequence;

typedef struct _Buzzer
{
	virtual_timer_t timer;

	const BuzzerSequence* pSeq;

	uint16_t curTone;
} Buzzer;

extern Buzzer buzzer;

extern const BuzzerSequence buzzSeqBeepFast;
extern const BuzzerSequence buzzSeqUp50;
extern const BuzzerSequence buzzSeqUp20;
extern const BuzzerSequence buzzSeqUpDown20;
extern const BuzzerSequence buzzSeqDoubleBeepSlow;
extern const BuzzerSequence buzzSeqUp100;
extern const BuzzerSequence buzzSeqDown100;
extern const BuzzerSequence buzzSeqDown50;
extern const BuzzerSequence buzzSeqTada;
extern const BuzzerSequence buzzSeqCant;
extern const BuzzerSequence buzzSeqFinalShort;
extern const BuzzerSequence buzzSeqElevator;
extern const BuzzerSequence buzzSeqEyeLead;
extern const BuzzerSequence buzzSeqEyeFollow;

void BuzzerInit();
void BuzzerTone(uint16_t freq);
void BuzzerPlay(const BuzzerSequence* pSeq);
void BuzzerPlayId(uint8_t id);

#endif /* BUZZER_H_ */
