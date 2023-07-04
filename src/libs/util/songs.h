#pragma once

#include <stdint.h>

typedef struct _BuzzerSequence
{
	uint16_t* pSong;
	uint16_t numTones;
	uint8_t once;
} BuzzerSequence;

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
extern const BuzzerSequence buzzSeqMacarena;

void SongsLoad();
