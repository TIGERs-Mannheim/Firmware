#include "microphone.h"
#include "constants.h"
#include "util/console.h"
#include "util/init_hal.h"

Microphone microphone;

#define DMA_BUFFER_SIZE 1600

static int16_t dmaDataCh0[2][DMA_BUFFER_SIZE] __attribute__((aligned(1024), section(".sram3")));
static int16_t dmaDataCh1[2][DMA_BUFFER_SIZE] __attribute__((aligned(1024), section(".sram3")));

// DFSDM Filter 0 => Channel 0
void DMA1_Stream4_IRQHandler()
{
	CH_IRQ_PROLOGUE();

	DMA1->HIFCR = DMA_HIFCR_CTCIF4;

	msg_t event = 0x0000;
	if((DMA1_Stream4->CR & DMA_SxCR_CT) == 0)
		event |= 0x01;

	chSysLockFromISR();
	chMBPostI(&microphone.eventQueue, event);
	chSysUnlockFromISR();

	CH_IRQ_EPILOGUE();
}

// DFSDM Filter 1 => Channel 7
void DMA1_Stream5_IRQHandler()
{
	CH_IRQ_PROLOGUE();

	DMA1->HIFCR = DMA_HIFCR_CTCIF5;

	msg_t event = 0x0100;
	if((DMA1_Stream5->CR & DMA_SxCR_CT) == 0)
		event |= 0x01;

	chSysLockFromISR();
	chMBPostI(&microphone.eventQueue, event);
	chSysUnlockFromISR();

	CH_IRQ_EPILOGUE();
}

void MicrophoneInit()
{
	chMBObjectInit(&microphone.eventQueue, microphone.eventQueueData, MICROPHONE_EVENT_QUEUE_SIZE);

	// configure DFSDM
	RCC->APB2ENR |= RCC_APB2ENR_DFSDM1EN;
	__DSB();

	// Audio clock is PLL3P @ 30.72Mhz
	// Possible audio configurations:
	// ADCLK    CKOUTDIV      =SCK  FOSR  FORD  DTRBS  AudioFreq
	// 30.72MHz       10  3.072MHz    64     4      0      48kHz
	// 30.72MHz       15  2.048MHz   128     4      4      16kHz   <= selected

	// select IN0 as input, output clock source is internal audio clock, enable channel, sample on rising clock edge
	DFSDM1_Channel0->CHCFGR2 = (4 << DFSDM_CHCFGR2_DTRBS_Pos);
	DFSDM1_Channel0->CHCFGR1 = DFSDM_CHCFGR1_CKOUTSRC | (14 << DFSDM_CHCFGR1_CKOUTDIV_Pos) | DFSDM_CHCFGR1_CHEN | DFSDM_CHCFGR1_SPICKSEL_0;
	
	// select IN0 as input, enable chanel, sample on falling clock edge
	DFSDM1_Channel7->CHCFGR2 = (4 << DFSDM_CHCFGR2_DTRBS_Pos);
	DFSDM1_Channel7->CHCFGR1 = DFSDM_CHCFGR1_CHINSEL | DFSDM_CHCFGR1_CHEN | DFSDM_CHCFGR1_SPICKSEL_0 | DFSDM_CHCFGR1_SITP_0;
	
	DFSDM1_Filter0->FLTFCR = (127 << DFSDM_FLTFCR_FOSR_Pos) | (4 << DFSDM_FLTFCR_FORD_Pos);
	DFSDM1_Filter1->FLTFCR = (127 << DFSDM_FLTFCR_FOSR_Pos) | (4 << DFSDM_FLTFCR_FORD_Pos);
	
	// continuous fast mode, regular conversions, enable DMA, sync channel 1 to channel 0, enable filters
	DFSDM1_Filter0->FLTCR1 = DFSDM_FLTCR1_FAST | DFSDM_FLTCR1_RCONT | DFSDM_FLTCR1_RDMAEN | DFSDM_FLTCR1_DFEN;
	DFSDM1_Filter1->FLTCR1 = DFSDM_FLTCR1_FAST | (7 << DFSDM_FLTCR1_RCH_Pos) | DFSDM_FLTCR1_RSYNC | DFSDM_FLTCR1_RCONT | DFSDM_FLTCR1_RDMAEN | DFSDM_FLTCR1_DFEN;
	
	// Globally enable DFSDM interface
	DFSDM1_Channel0->CHCFGR1 |= DFSDM_CHCFGR1_DFSDMEN;
	
	// configure DMA
	DMAMUX1_Channel4->CCR = 101; // dfsdm1_dma0
	DMAMUX1_Channel5->CCR = 102; // dfsdm1_dma1
	
	DMA1->HIFCR = DMA_HIFCR_CTCIF4 | DMA_HIFCR_CTCIF5;
	
	// DMA in double-buffered mode, low priority, transfer complete IRQ enabled
	DMA1_Stream4->CR = DMA_PL_LOW | DMA_SxCR_DBM | DMA_MSIZE_2BYTE | DMA_PSIZE_2BYTE | DMA_SxCR_MINC | DMA_DIR_PERIPH2MEM | DMA_SxCR_TCIE;
	DMA1_Stream4->FCR = 0;
	DMA1_Stream4->NDTR = DMA_BUFFER_SIZE;
	DMA1_Stream4->PAR = (uint32_t)&(DFSDM1_Filter0->FLTRDATAR) + 2;
	DMA1_Stream4->M0AR = (uint32_t)&dmaDataCh0[0];
	DMA1_Stream4->M1AR = (uint32_t)&dmaDataCh0[1];

	// DMA in double-buffered mode, low priority, transfer complete IRQ enabled
	DMA1_Stream5->CR = DMA_PL_LOW | DMA_SxCR_DBM | DMA_MSIZE_2BYTE | DMA_PSIZE_2BYTE | DMA_SxCR_MINC | DMA_DIR_PERIPH2MEM | DMA_SxCR_TCIE;
	DMA1_Stream5->FCR = 0;
	DMA1_Stream5->NDTR = DMA_BUFFER_SIZE;
	DMA1_Stream5->PAR = (uint32_t)&(DFSDM1_Filter1->FLTRDATAR) + 2;
	DMA1_Stream5->M0AR = (uint32_t)&dmaDataCh1[0];
	DMA1_Stream5->M1AR = (uint32_t)&dmaDataCh1[1];
	
	DMA1_Stream4->CR |= DMA_SxCR_EN;
	DMA1_Stream5->CR |= DMA_SxCR_EN;

	// init GPIO
	GPIOInitData gpioInit;
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.alternate = 3;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_25MHZ;
	gpioInit.pupd = GPIO_PUPD_NONE;
	GPIOInit(GPIOC, GPIO_PIN_1, &gpioInit);
	
	gpioInit.alternate = 6;
	GPIOInit(GPIOC, GPIO_PIN_2, &gpioInit);

	NVICEnableIRQ(DMA1_Stream4_IRQn, IRQL_MICROPHONE);
	NVICEnableIRQ(DMA1_Stream5_IRQn, IRQL_MICROPHONE);

	// Start regular conversions
	DFSDM1_Filter0->FLTCR1 |= DFSDM_FLTCR1_RSWSTART;
}

void MicrophoneTask(void *params)
{
	(void)params;
	
	chRegSetThreadName("Microphone");
	
	msg_t event;

	while(1)
	{
		if(chMBFetch(&microphone.eventQueue, &event, MS2ST(20)) != MSG_OK)
			continue;

		uint32_t channelId = (event & 0x0100) >> 8;
		uint32_t bufferId = event & 0x01;

		microphone.totalBytes[channelId] += DMA_BUFFER_SIZE;

		if(microphone.files[0].fs && channelId == 0)
		{
			UINT bytesWritten;
			f_write(&microphone.files[0], dmaDataCh0[bufferId], DMA_BUFFER_SIZE*2, &bytesWritten);
		}

		if(microphone.files[1].fs && channelId == 1)
		{
			UINT bytesWritten;
			f_write(&microphone.files[1], dmaDataCh1[bufferId], DMA_BUFFER_SIZE*2, &bytesWritten);
		}
	}
}

void MicrophoneStartRecoding()
{
	if(f_open(&microphone.files[0], "channel0.rawsound", FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
	{
		ConsolePrint("Could not open file!");
		return;
	}

	f_open(&microphone.files[1], "channel1.rawsound", FA_WRITE | FA_CREATE_ALWAYS);

	ConsolePrint("Started recording.\r\n");
}

void MicrophoneStopRecording()
{
	f_close(&microphone.files[0]);
	f_close(&microphone.files[1]);

	ConsolePrint("Recording stopped.\r\n");
}
