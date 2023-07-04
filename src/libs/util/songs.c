#include "songs.h"

const BuzzerSequence buzzSeqBeepFast = {
		(uint16_t[]){1, 100, 0, 100},
		4, 0,
};

const BuzzerSequence buzzSeqDoubleBeepSlow = {
		(uint16_t[]){1, 100, 0, 100, 1, 100, 0, 2000},
		8, 0,
};

const BuzzerSequence buzzSeqUp100 = {
		(uint16_t[]){2300, 100, 2500, 100, 2700, 100},
		6, 1,
};

const BuzzerSequence buzzSeqTada = {
		(uint16_t[]){2700, 100, 2400, 100, 2100, 100, 1950, 200, 2100, 100, 2700, 200},
		12, 1,
};

const BuzzerSequence buzzSeqDown100 = {
		(uint16_t[]){2700, 100, 2500, 100, 2300, 100},
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
	1760, 225,
	0, 75,
	2349, 225,
	0, 75,
	1760, 225,
	0, 75,
	2349, 225,
	0, 75,
	1760, 112,
	0, 38,
	2349, 262,
	0, 38,
	1760, 112,
	0, 188,
	1661, 112,
	0, 38,
	1760, 262,
	0, 38,
	1760, 150,
	1661, 150,
	1760, 150,
	0, 150,
	1568, 150,
	1480, 150,
	1568, 150,
	0, 150,
	1397, 412,
	0, 38,
	1175, 412,
	0, 338,
	1760, 225,
	0, 75,
	2349, 225,
	0, 75,
	1760, 225,
	0, 75,
	2349, 225,
	0, 75,
	1760, 112,
	0, 38,
	2349, 262,
	0, 38,
	1760, 112,
	0, 188,
	1661, 112,
	0, 38,
	1760, 262,
	0, 38,
	1568, 262,
	0, 38,
	1568, 262,
	0, 38,
	1568, 112,
	0, 38,
	1480, 112,
	0, 38,
	1568, 262,
	0, 38,
	2093, 262,
	0, 38,
	1864, 262,
	0, 38,
	1760, 262,
	0, 38,
	1568, 262,
	0, 38,
	1760, 225,
	0, 75,
	2349, 225,
	0, 75,
	1760, 225,
	0, 75,
	2349, 225,
	0, 75,
	1760, 112,
	0, 38,
	2349, 262,
	0, 38,
	1760, 112,
	0, 188,
	1661, 112,
	0, 38,
	1760, 262,
	0, 38,
	2093, 112,
	0, 188,
	2093, 262,
	0, 188,
	1760, 112,
	0, 38,
	1568, 225,
	0, 75,
	1397, 412,
	0, 38,
	1175, 412,
	0, 338,
	1175, 600,
	1397, 600,
	1760, 600,
	2093, 562,
	0, 38,
	2489, 262,
	0, 38,
	2349, 262,
	0, 38,
	1661, 112,
	0, 38,
	1760, 262,
	0, 38,
	1397, 1012,
	0, 488,
	3520, 262,
	0, 38,
	2794, 112,
	0, 38,
	3520, 262,
	0, 488,
	3520, 262,
	0, 38,
	2794, 112,
	0, 38,
	3520, 262,
	0, 488,
	3520, 262,
	0, 38,
	2794, 112,
	0, 38,
	3322, 112,
	0, 38,
	3520, 262,
	0, 38,
	2794, 562,
	0, 38,
	2349, 412,
	0, 488,
	3520, 262,
	0, 38,
	2794, 112,
	0, 38,
	3520, 262,
	0, 488,
	3520, 262,
	0, 38,
	2794, 112,
	0, 38,
	3520, 262,
	0, 488,
	3136, 262,
	0, 38,
	2349, 112,
	0, 38,
	3136, 262,
	0, 338,
	4186, 262,
	0, 38,
	3729, 262,
	0, 38,
	3520, 262,
	0, 38,
	3136, 262,
	0, 188,
	3520, 262,
	0, 38,
	2794, 112,
	0, 38,
	3520, 262,
	0, 488,
	3520, 262,
	0, 38,
	2794, 112,
	0, 38,
	3520, 262,
	0, 488,
	3520, 262,
	0, 38,
	2794, 112,
	0, 38,
	3322, 112,
	0, 38,
	3520, 262,
	0, 38,
	2794, 562,
	0, 38,
	2349, 412,
	0, 338,
	1175, 600,
	1397, 600,
	1760, 600,
	2093, 562,
	0, 38,
	2489, 262,
	0, 38,
	2349, 262,
	0, 38,
	1661, 112,
	0, 38,
	1760, 262,
	0, 38,
	1397, 1312,
	0, 338,
	1760, 225,
	0, 75,
	2349, 225,
	0, 75,
	1760, 225,
	0, 75,
	2349, 225,
	0, 75,
	1760, 112,
	0, 38,
	2349, 262,
	0, 38,
	1760, 112,
	0, 188,
	1661, 112,
	0, 38,
	1760, 262,
	0, 38,
	1760, 150,
	1661, 150,
	1760, 150,
	0, 150,
	1568, 150,
	1480, 150,
	1568, 150,
	0, 150,
	1397, 412,
	0, 38,
	1175, 412,
	0, 338,
	1760, 225,
	0, 75,
	2349, 225,
	0, 75,
	1760, 225,
	0, 75,
	2349, 225,
	0, 75,
	1760, 112,
	0, 38,
	2349, 262,
	0, 38,
	1760, 112,
	0, 188,
	1661, 112,
	0, 38,
	1760, 262,
	0, 38,
	1568, 262,
	0, 38,
	1568, 262,
	0, 38,
	1568, 112,
	0, 38,
	1480, 112,
	0, 38,
	1568, 262,
	0, 38,
	2093, 262,
	0, 38,
	1864, 262,
	0, 38,
	1760, 262,
	0, 38,
	1568, 262,
	0, 38,
	1760, 225,
	0, 75,
	2349, 225,
	0, 75,
	1760, 225,
	0, 75,
	2349, 225,
	0, 75,
	1760, 112,
	0, 38,
	2349, 262,
	0, 38,
	1760, 112,
	0, 188,
	1661, 112,
	0, 38,
	1760, 262,
	0, 38,
	2093, 112,
	0, 188,
	2093, 262,
	0, 188,
	1760, 112,
	0, 38,
	1568, 225,
	0, 75,
	1397, 412,
	0, 38,
	1175, 412,
	0, 338,
	1175, 600,
	1397, 600,
	1760, 600,
	2093, 562,
	0, 38,
	2489, 262,
	0, 38,
	2349, 262,
	0, 38,
	1661, 112,
	0, 38,
	1760, 262,
	0, 38,
	1397, 1012,
	0, 338,
}, 622, 1,};

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
	2093, 330,
	0, 780,
	2093, 330,
	0, 86,
	1864, 330,
	0, 86,
	2093, 330,
	0, 1058,
	2093, 330,
	0, 86,
	1864, 330,
	0, 86,
	2093, 330,
	0, 1058,
	2093, 416,
	1568, 416,
	1661, 1440,
	0, 1058,
}, 36, 1,};

//EyeOfTheTiger - Untere Stimme:
const BuzzerSequence buzzSeqEyeFollow = {(uint16_t[]){
	1568, 330,
	0, 780,
	1568, 330,
	0, 86,
	1397, 330,
	0, 86,
	1568, 330,
	0, 1058,
	1568, 330,
	0, 86,
	1397, 330,
	0, 86,
	1568, 330,
	0, 1058,
	1568, 416,
	1174, 416,
	1244, 1440,
	0, 1058,
}, 36, 1,};

// Macarena
const BuzzerSequence buzzSeqMacarena = {(uint16_t[]){
	1568, 225,
	0, 75,
	1568, 75,
	0, 75,
	1568, 75,
	0, 75,
	1568, 225,
	0, 75,
	1568, 75,
	0, 75,
	1568, 75,
	0, 75,
	1568, 75,
	0, 75,
	1568, 75,
	0, 75,
	1568, 75,
	0, 75,
	1568, 75,
	0, 75,
	1568, 75,
	0, 75,
	1864, 75,
	0, 75,
	1175, 75,
	0, 75,
	1175, 75,
	0, 75,
	1568, 225,
	0, 75,
	1568, 75,
	0, 75,
	1568, 75,
	0, 75,
	1568, 225,
	0, 75,
	1568, 75,
	0, 75,
	1568, 75,
	0, 75,
	1568, 75,
	0, 75,
	1568, 75,
	0, 75,
	1568, 75,
	0, 75,
	1568, 75,
	0, 75,
	1318, 75,
	0, 75,
	1175, 75,
	0, 375,
	1568, 225,
	0, 75,
	1568, 75,
	0, 75,
	1568, 75,
	0, 75,
	1568, 225,
	0, 75,
	1568, 75,
	0, 75,
	1568, 75,
	0, 75,
	1568, 75,
	0, 75,
	1568, 75,
	0, 75,
	1568, 75,
	0, 75,
	1568, 75,
	0, 75,
	1568, 75,
	0, 75,
	1864, 75,
	0, 375,
	2349, 675,
	0, 75,
	2093, 225,
	0, 75,
	2349, 75,
	0, 75,
	1864, 75,
	0, 75,
	1568, 75,
	0, 975,
}, 172, 1,};

void SongsLoad()
{
	for(uint16_t i = 0; i < 32; i += 2)
	{
		buzzSeqUp50Data[i] = 2000+i*25;
		buzzSeqUp50Data[i+1] = 50;

		buzzSeqUp20Data[i] = 2000+i*25;
		buzzSeqUp20Data[i+1] = 20;

		buzzSeqUpDown20Data[i] = 2000+i*25;
		buzzSeqUpDown20Data[i+1] = 20;
	}

	for(uint16_t i = 32; i < 64; i += 2)
	{
		buzzSeqUpDown20Data[i] = 2800-(i-32)*25;
		buzzSeqUpDown20Data[i+1] = 20;

		buzzSeqDown50Data[i-32] = 2800-(i-32)*25;
		buzzSeqDown50Data[i-31] = 50;
	}
}
