#include "hal/uart_dma.h"
#include "hal/sys_time.h"
#include "util/log.h"
#include <string.h>

#define EVENT_MASK_IDLE_LINE		EVENT_MASK(0)
#define EVENT_MASK_BREAK_DETECTED	EVENT_MASK(1)
#define EVENT_MASK_TX_DONE			EVENT_MASK(2)
#define EVENT_MASK_START_TX			EVENT_MASK(3)

typedef struct _DmaChannel
{
	uint32_t stream;
	USART_TypeDef* pUSART;
	uint32_t channel;
} DmaChannel;

static const DmaChannel dma1Channels[] = {
	{ 0, UART5, 4 },
	{ 0, UART8, 5 },
	{ 1, USART3, 4 },
	{ 1, UART7, 5 },
	{ 2, UART4, 4 },
	{ 3, USART3, 4 },
	{ 3, UART7, 5 },
	{ 4, UART4, 4 },
	{ 4, USART3, 7 },
	{ 5, USART2, 4 },
	{ 6, USART2, 4 },
	{ 7, UART5, 4 },
};

static const DmaChannel dma2Channels[] = {
	{ 1, USART6, 5 },
	{ 2, USART1, 4 },
	{ 2, USART6, 5 },
	{ 5, USART1, 4 },
	{ 6, USART6, 5 },
	{ 7, USART1, 4 },
	{ 7, USART6, 5 },
};

static uint32_t lookupDmaChannel(DMA_TypeDef* pDMA, USART_TypeDef* pUSART, uint32_t streamNumber)
{
	const DmaChannel* pList;
	uint32_t numOptions;

	if(pDMA == DMA1)
	{
		pList = dma1Channels;
		numOptions = sizeof(dma1Channels)/sizeof(dma1Channels[0]);
	}
	else
	{
		pList = dma2Channels;
		numOptions = sizeof(dma2Channels)/sizeof(dma2Channels[0]);
	}

	for(uint32_t i = 0; i < numOptions; i++)
	{
		if(pList[i].pUSART == pUSART && pList[i].stream == streamNumber)
			return pList[i].channel;
	}

	return 0;
}

static void uartDmaTask(void* params);

void UartDmaIrq(UartDma* pUart)
{
	USART_TypeDef* pRegister = pUart->data.pRegister;

	const uint32_t sr = pRegister->ISR;
	pRegister->ICR = sr;

	eventmask_t events = 0;

	if(sr & USART_ISR_IDLE)
		events |= EVENT_MASK_IDLE_LINE;

	if(sr & USART_ISR_FE)
		events |= EVENT_MASK_BREAK_DETECTED;

	if(events)
	{
		chSysLockFromISR();
		chEvtSignalI(pUart->pTask, events);
		chSysUnlockFromISR();
	}
}

void UartDmaTxDmaIrq(UartDma* pUart)
{
	const uint32_t sr = *pUart->pDMATxISR;
	*pUart->pDMATxIFCR = pUart->IFCRTxMask;

	if(sr & pUart->ISRTxTCMask)
	{
		chSysLockFromISR();
		chEvtSignalI(pUart->pTask, EVENT_MASK_TX_DONE);
		chSysUnlockFromISR();
	}
}

void UartDmaInit(UartDma* pUart, UartDmaData* pInit, tprio_t prio)
{
	chMtxObjectInit(&pUart->rxDataMutex);
	chMtxObjectInit(&pUart->txFifoMutex);
	chEvtObjectInit(&pUart->eventSource);

	pUart->data = *pInit;

	FifoInit(&pUart->txFifo, pInit->pDmaBufTx, pInit->dmaBufSizeTx);
	pUart->pRxRead = pInit->pDmaBufRx;
	pUart->pRxWriteLast = pInit->pDmaBufRx;

	pUart->rxMinEmptySize = (float)(pInit->baudrate/10) * 0.002f * 2.0f;

	if(pInit->dmaBufSizeRx < pUart->rxMinEmptySize*2)
	{
		chSysHalt("UART DMA buffer size too small for selected baudrate");
	}

	// Configure UART
	USART_TypeDef* pRegister = pInit->pRegister;

	pRegister->CR1 = USART_CR1_TE | USART_CR1_RE;
	pRegister->CR2 = pInit->swapRxTx ? USART_CR2_SWAP : 0;
	pRegister->CR3 = USART_CR3_DMAR | USART_CR3_DMAT;
	pRegister->BRR = USARTCalcBRR(pInit->baudrate, pInit->perClk, 0);
	pRegister->CR1 |= USART_CR1_UE;
	pRegister->ICR = 0xFFFFFFFF;

	pRegister->CR1 |= USART_CR1_IDLEIE;
	pRegister->CR3 |= USART_CR3_EIE;

	// Configure DMA
	pUart->pDmaRx = DMAGenerateStreamTypeDef(pUart->data.pDma, pUart->data.dmaStreamRx);
	pUart->pDmaTx = DMAGenerateStreamTypeDef(pUart->data.pDma, pUart->data.dmaStreamTx);
	pUart->pDMARxIFCR = DMAGenerateIFCR(pUart->data.pDma, pUart->data.dmaStreamRx);
	pUart->pDMATxIFCR = DMAGenerateIFCR(pUart->data.pDma, pUart->data.dmaStreamTx);
	pUart->pDMATxISR = DMAGenerateISR(pUart->data.pDma, pUart->data.dmaStreamTx);
	pUart->IFCRRxMask = DMAGenerateMaskAll(pUart->data.dmaStreamRx);
	pUart->IFCRTxMask = DMAGenerateMaskAll(pUart->data.dmaStreamTx);
	pUart->ISRTxTCMask = DMAGenerateMaskTCIF(pUart->data.dmaStreamTx);

	uint32_t dmaChRx = lookupDmaChannel(pUart->data.pDma, pRegister, pUart->data.dmaStreamRx);
	uint32_t dmaChTx = lookupDmaChannel(pUart->data.pDma, pRegister, pUart->data.dmaStreamTx);

	pUart->pDmaRx->CR = DMA_DIR_PERIPH2MEM | pInit->dmaPrio | DMA_SxCR_MINC | DMA_SxCR_CIRC | (dmaChRx << DMA_SxCR_CHSEL_Pos);
	pUart->pDmaRx->FCR = 0;
	pUart->pDmaRx->M0AR = (uint32_t)pInit->pDmaBufRx;
	pUart->pDmaRx->PAR = (uint32_t)&pRegister->RDR;
	pUart->pDmaRx->NDTR = pInit->dmaBufSizeRx;

	pUart->pDmaTx->CR = DMA_DIR_MEM2PERIPH | pInit->dmaPrio | DMA_SxCR_MINC | DMA_SxCR_TCIE | (dmaChTx << DMA_SxCR_CHSEL_Pos);
	pUart->pDmaTx->FCR = 0;
	pUart->pDmaTx->M0AR = (uint32_t)pUart->txFifo.pRead;
	pUart->pDmaTx->PAR = (uint32_t)&pRegister->TDR;
	pUart->pDmaTx->NDTR = 0;

	*pUart->pDMARxIFCR = pUart->IFCRRxMask; // clear rx flags
	*pUart->pDMATxIFCR = pUart->IFCRTxMask; // clear tx flags

	pUart->pDmaRx->CR |= DMA_SxCR_EN;

	pUart->pTask = chThdCreateStatic(pUart->waTask, sizeof(pUart->waTask), prio, &uartDmaTask, pUart);
}

static void uartDmaTask(void* params)
{
	UartDma* pUart = (UartDma*)params;

	chRegSetThreadName(pUart->data.pTaskName);

	uint32_t last1sTime = SysTimeUSec();

	while(1)
	{
		eventmask_t events = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(2));

		eventflags_t broadcastFlags = 0;

		// Event handling
		if(events & EVENT_MASK_BREAK_DETECTED)
		{
			broadcastFlags |= UART_DMA_EVENT_BREAK_DETECTED;
		}

		// TX handling
		if((pUart->pDmaTx->CR & DMA_SxCR_EN) == 0)
		{
			if(pUart->txTransferSize)
			{
				chMtxLock(&pUart->txFifoMutex);
				FifoDrain(&pUart->txFifo, pUart->txTransferSize);
				chMtxUnlock(&pUart->txFifoMutex);

				*pUart->pDMATxIFCR = pUart->IFCRTxMask;
				pUart->liveStats.txBytes += pUart->txTransferSize;

				pUart->txTransferSize = 0;

				broadcastFlags |= UART_DMA_EVENT_DATA_SENT;
			}

			if(FifoGetFullSpace(&pUart->txFifo) > 0)
			{
				// Initiate a new transfer
				chMtxLock(&pUart->txFifoMutex);
				pUart->txTransferSize = FifoGetLinearFullSpace(&pUart->txFifo);
				pUart->pDmaTx->M0AR = (uint32_t)pUart->txFifo.pRead;
				chMtxUnlock(&pUart->txFifoMutex);
				pUart->pDmaTx->NDTR = pUart->txTransferSize;
				pUart->pDmaTx->CR |= DMA_SxCR_EN;
			}
		}

		if((events & (EVENT_MASK_BREAK_DETECTED | EVENT_MASK_IDLE_LINE)) || events == 0)
		{
			// RX handling
			chMtxLock(&pUart->rxDataMutex);

			uint8_t* pBegin = pUart->data.pDmaBufRx;
			uint8_t* pEnd = pBegin + pUart->data.dmaBufSizeRx;

			uint8_t* pWrite = pUart->data.pDmaBufRx + (pUart->data.dmaBufSizeRx - pUart->pDmaRx->NDTR);

			size_t bytesAvail;
			if(pUart->pRxRead <= pWrite)
				bytesAvail = pWrite - pUart->pRxRead;
			else
				bytesAvail = (pEnd - pUart->pRxRead) + (pWrite - pBegin);

			size_t bytesEmpty = pUart->data.dmaBufSizeRx - bytesAvail;

			if(bytesEmpty < pUart->rxMinEmptySize)
			{
				// Read pointer is approaching write pointer: it will be pushed to ensure space, bytes will be lost
				size_t bytesAdvance = pUart->rxMinEmptySize - bytesEmpty;

				LogWarnC("UART DMA bytes lost", bytesAdvance);

				if(pUart->pRxRead <= pWrite)
				{
					pUart->pRxRead += bytesAdvance;
				}
				else
				{
					size_t bytesToEnd = pEnd - pUart->pRxRead;
					if(bytesToEnd < bytesAdvance)
						pUart->pRxRead += bytesAdvance;
					else
						pUart->pRxRead = pBegin + (bytesAdvance - bytesToEnd);
				}

				bytesAvail -= bytesAdvance;
				pUart->liveStats.rxLost += bytesAdvance;
			}

			chMtxUnlock(&pUart->rxDataMutex);

			// Count received bytes
			size_t newBytes;
			if(pUart->pRxWriteLast <= pWrite)
				newBytes = pWrite - pUart->pRxWriteLast;
			else
				newBytes = (pEnd - pUart->pRxWriteLast) + (pWrite - pBegin);

			pUart->liveStats.rxBytes += newBytes;

			if(newBytes)
				broadcastFlags |= UART_DMA_EVENT_DATA_RECEIVED;

			pUart->pRxWriteLast = pWrite;
		}

		if(broadcastFlags)
			chEvtBroadcastFlags(&pUart->eventSource, broadcastFlags);

		// Update statistics
		uint32_t tNow_us = SysTimeUSec();
		uint32_t statsTDelta_us = tNow_us - last1sTime;

		if(statsTDelta_us > 1000000)
		{
			last1sTime = tNow_us;

			float dt_s = statsTDelta_us * 1e-6f;

			pUart->stats.rxBytes = pUart->liveStats.rxBytes / dt_s;
			pUart->stats.txBytes = pUart->liveStats.txBytes / dt_s;
			pUart->stats.rxUsage = ((float)pUart->stats.rxBytes)/(pUart->data.baudrate*0.1f);
			pUart->stats.txUsage = ((float)pUart->stats.txBytes)/(pUart->data.baudrate*0.1f);
			memset(&pUart->liveStats, 0, sizeof(UartDmaStats));
		}
	}
}

size_t UartDmaWrite(UartDma* pUart, const void* pData, size_t dataSize)
{
	chMtxLock(&pUart->txFifoMutex);
	size_t bytesWritten = FifoWrite(&pUart->txFifo, pData, dataSize);
	chMtxUnlock(&pUart->txFifoMutex);

	if(bytesWritten)
		chEvtSignal(pUart->pTask, EVENT_MASK_START_TX);

	return bytesWritten;
}

size_t UartDmaRead(UartDma* pUart, void* pTarget, size_t targetSize)
{
	uint8_t* pBegin = pUart->data.pDmaBufRx;
	uint8_t* pEnd = pBegin + pUart->data.dmaBufSizeRx;

	chMtxLock(&pUart->rxDataMutex);

	uint8_t* pWrite = pUart->data.pDmaBufRx + (pUart->data.dmaBufSizeRx - pUart->pDmaRx->NDTR);

    if(pUart->pRxRead <= pWrite)
    {
    	size_t bytesAvail = pWrite - pUart->pRxRead;

    	if(bytesAvail < targetSize)
    		targetSize = bytesAvail;

    	memcpy(pTarget, pUart->pRxRead, targetSize);
    	pUart->pRxRead += targetSize;
    }
    else
    {
    	size_t bytesToEnd = pEnd - pUart->pRxRead;
    	if(bytesToEnd > targetSize)
    	{
    		memcpy(pTarget, pUart->pRxRead, targetSize);
    		pUart->pRxRead += targetSize;
    	}
    	else
    	{
    		size_t bytesAvail = bytesToEnd + (pWrite - pBegin);

        	if(bytesAvail < targetSize)
        		targetSize = bytesAvail;

    		memcpy(pTarget, pUart->pRxRead, bytesToEnd);
    		memcpy((uint8_t*)pTarget + bytesToEnd, pBegin, targetSize - bytesToEnd);
    		pUart->pRxRead = pBegin + targetSize - bytesToEnd;
    	}
    }

	chMtxUnlock(&pUart->rxDataMutex);

	return targetSize;
}
