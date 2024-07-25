#pragma once

#include <stdint.h>
#include <stddef.h>

typedef struct _BuzzerSequence
{
	uint16_t* pSong;
	uint16_t numTones;
	uint8_t once;
} BuzzerSequence;

typedef struct _Song
{
	uint32_t id;
	const char* pName;
	const BuzzerSequence* pSequence;
} Song;

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
extern const BuzzerSequence buzzSeqTetris;
extern const BuzzerSequence buzzSeqEyeLead;
extern const BuzzerSequence buzzSeqEyeFollow;
extern const BuzzerSequence buzzSeqMacarena;

void SongsLoad();
void SongsGetList(const Song** ppSongs, size_t* pNumSongs);
