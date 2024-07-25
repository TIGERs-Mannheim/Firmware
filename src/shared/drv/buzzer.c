#include "buzzer.h"
#include "commands.h"

static void registerShellCommands(ShellCmdHandler* pHandler);

static void buzzerOn(Buzzer* pBuzz)
{
	*pBuzz->pCCMR = pBuzz->ccmrBuzzerOn;
}

static void buzzerOff(Buzzer* pBuzz)
{
	*pBuzz->pCCMR = pBuzz->ccmrBuzzerOff;
}

void BuzzerInit(Buzzer* pBuzz, BuzzerData* pInit)
{
	pBuzz->pTim = pInit->pTim;
	pBuzz->pCCR = pInit->pCCR;
	pBuzz->pCCMR = pInit->pCCMR;
	pBuzz->ccmrBuzzerOn = pInit->ccmrBuzzerOn;
	pBuzz->ccmrBuzzerOff = pInit->ccmrBuzzerOff;

	TIM_TypeDef* pTim = pInit->pTim;

	pTim->CR1 = TIM_CR1_ARPE;	// enable ARR preload
	pTim->BDTR = TIM_BDTR_MOE;
	buzzerOff(pBuzz);
	pTim->CCER = TIM_CCER_CC1E | TIM_CCER_CC1P;	// enable CC1 output and set polarity to active low
	pTim->PSC = pInit->psc1MHz; // => 1MHz
	pTim->ARR = 320;	// period
	pTim->CCR1 = 103;	// pulse
	pTim->EGR = TIM_EGR_UG;	// generate update event to take new ARR value
	pTim->CR1 |= TIM_CR1_CEN;	// enable timer

	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.pupd = GPIO_PUPD_NONE;
	gpioInit.alternate = pInit->outputPin.alternate;
	GPIOInit(pInit->outputPin.pPort, pInit->outputPin.pin, &gpioInit);

	SongsLoad();

	ShellCmdHandlerInit(&pBuzz->cmdHandler, pBuzz);
	registerShellCommands(&pBuzz->cmdHandler);
}

void BuzzerTone(Buzzer* pBuzz, uint16_t freq)
{
	if(freq == 1)
		freq = 2430;

	if(freq < 16 || freq > 20000)
	{
		buzzerOff(pBuzz);
		return;
	}

	buzzerOn(pBuzz);

	uint16_t arr = 1000000/freq;

	pBuzz->pTim->ARR = arr;
	*pBuzz->pCCR = arr/2;
	pBuzz->pTim->EGR = TIM_EGR_UG;
}

static void buzzTimer(virtual_timer_t *pTimer, void* param)
{
	(void)pTimer;

	Buzzer* pBuzz = (Buzzer*)param;

	if(!pBuzz->pSeq)
		return;

	if(pBuzz->curTone == 0 && pBuzz->pSeq->once)
	{
		pBuzz->pSeq = 0;
		BuzzerTone(pBuzz, 0);
		return;
	}

	BuzzerTone(pBuzz, pBuzz->pSeq->pSong[pBuzz->curTone]);

	chSysLockFromISR();
	chVTSetI(&pBuzz->timer, TIME_MS2I(pBuzz->pSeq->pSong[pBuzz->curTone+1]), &buzzTimer, pBuzz);
	chSysUnlockFromISR();

	pBuzz->curTone += 2;
	pBuzz->curTone %= pBuzz->pSeq->numTones;
}

void BuzzerPlay(Buzzer* pBuzz, const BuzzerSequence* pSeq)
{
	if(pSeq == pBuzz->pSeq && pSeq != 0)
		return;

	chVTReset(&pBuzz->timer);
	pBuzz->pSeq = 0;
	BuzzerTone(pBuzz, 0);

	if(pSeq == 0)
		return;

	if(pSeq->numTones % 2 == 1 || pSeq->numTones == 0)
		return;

	pBuzz->curTone = 2;
	pBuzz->pSeq = pSeq;

	BuzzerTone(pBuzz, pBuzz->pSeq->pSong[0]);
	chVTSet(&pBuzz->timer, TIME_MS2I(pBuzz->pSeq->pSong[1]), &buzzTimer, pBuzz);
}

void BuzzerPlayId(Buzzer* pBuzz, uint8_t id)
{
	switch(id)
	{
		case BUZZ_UP20:	BuzzerPlay(pBuzz, &buzzSeqUp20); break;
		case BUZZ_BEEP_FAST: BuzzerPlay(pBuzz, &buzzSeqBeepFast); break;
		case BUZZ_UP50: BuzzerPlay(pBuzz, &buzzSeqUp50); break;
		case BUZZ_DOWN50: BuzzerPlay(pBuzz, &buzzSeqDown50); break;
		case BUZZ_UPDOWN20: BuzzerPlay(pBuzz, &buzzSeqUpDown20); break;
		case BUZZ_DOUBLE_SLOW: BuzzerPlay(pBuzz, &buzzSeqDoubleBeepSlow); break;
		case BUZZ_UP100: BuzzerPlay(pBuzz, &buzzSeqUp100); break;
		case BUZZ_DOWN100: BuzzerPlay(pBuzz, &buzzSeqDown100); break;
		case BUZZ_TADA: BuzzerPlay(pBuzz, &buzzSeqTada); break;
		case BUZZ_SONG_FINAL: BuzzerPlay(pBuzz, &buzzSeqFinalShort); break;
		case BUZZ_SONG_CANTINA: BuzzerPlay(pBuzz, &buzzSeqCant); break;
		case BUZZ_SONG_EYE_LEAD: BuzzerPlay(pBuzz, &buzzSeqEyeLead); break;
		case BUZZ_SONG_EYE_FOLLOW: BuzzerPlay(pBuzz, &buzzSeqEyeFollow); break;
		case BUZZ_SONG_TETRIS: BuzzerPlay(pBuzz, &buzzSeqTetris); break;
		case BUZZ_SONG_MACARENA: BuzzerPlay(pBuzz, &buzzSeqMacarena); break;
		default: BuzzerPlay(pBuzz, 0); break;
	}
}

SHELL_CMD(off, "Turn buzzer off");

SHELL_CMD_IMPL(off)
{
	(void)argc; (void)argv;
	Buzzer* pBuzz = (Buzzer*)pUser;

	printf("Disabling buzzer\r\n");

	BuzzerPlay(pBuzz, 0);
}

SHELL_CMD(tone, "Sound at specified frequency",
	SHELL_ARG(freq, "Frequency [Hz]")
);

SHELL_CMD_IMPL(tone)
{
	(void)argc;
	Buzzer* pBuzz = (Buzzer*)pUser;
	int freq = atoi(argv[1]);

	printf("Buzzer freq: %hu\r\n", freq);

	BuzzerTone(pBuzz, freq);
}

SHELL_CMD(list, "List available songs");

SHELL_CMD_IMPL(list)
{
	(void)pUser; (void)argc; (void)argv;

	const Song* pSongs;
	size_t numSongs;

	SongsGetList(&pSongs, &numSongs);

	printf("Available songs:\r\n");

	for(size_t i = 0; i < numSongs; i++)
	{
		printf("  %s\r\n", pSongs[i].pName);
	}
}

SHELL_CMD(play, "Play a song",
	SHELL_ARG(song, "Song name")
);

SHELL_CMD_IMPL(play)
{
	(void)argc;
	Buzzer* pBuzz = (Buzzer*)pUser;
	const char* pSongname = argv[1];

	const Song* pSongs;
	size_t numSongs;

	SongsGetList(&pSongs, &numSongs);

	for(size_t i = 0; i < numSongs; i++)
	{
		if(strcmp(pSongs[i].pName, pSongname) == 0)
		{
			printf("Playing song: %s\r\n", pSongname);
			BuzzerPlay(pBuzz, pSongs[i].pSequence);
			return;
		}
	}

	fprintf(stderr, "Unknown song: %s\r\n", pSongname);
}

static void registerShellCommands(ShellCmdHandler* pHandler)
{
	ShellCmdAdd(pHandler, off_command);
	ShellCmdAdd(pHandler, tone_command);
	ShellCmdAdd(pHandler, list_command);
	ShellCmdAdd(pHandler, play_command);
}
