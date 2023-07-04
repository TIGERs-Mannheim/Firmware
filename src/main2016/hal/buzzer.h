/*
 * buzzer.h
 *
 *  Created on: 23.12.2010
 *      Author: AndreR
 */

#ifndef __BUZZER_H__
#define __BUZZER_H__

#include "ch.h"
#include "util/songs.h"

typedef struct _Buzzer
{
	virtual_timer_t timer;

	const BuzzerSequence* pSeq;

	uint16_t curTone;
} Buzzer;

extern Buzzer buzzer;

void BuzzerInit();
void BuzzerTone(uint16_t freq);
void BuzzerPlay(const BuzzerSequence* pSeq);
void BuzzerPlayId(uint8_t id);

#endif /* BUZZER_H_ */
