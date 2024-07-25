#include "hal/eth.h"
#include "hal/init_hal.h"
#include "hal/sys_time.h"
#include "util/log.h"
#include "errors.h"
#include <stdio.h>
#include <string.h>

#define EVENT_LINK_CHANGED	EVENT_MASK(0)

/* Section 3: Common PHY Registers */
#define PHY_BCR                         ((uint16_t)0x00)    /*!< Transceiver Basic Control Register   */
#define PHY_BSR                         ((uint16_t)0x01)    /*!< Transceiver Basic Status Register    */

#define PHY_RESET                       ((uint16_t)0x8000)  /*!< PHY Reset */
#define PHY_LOOPBACK                    ((uint16_t)0x4000)  /*!< Select loop-back mode */
#define PHY_FULLDUPLEX_100M             ((uint16_t)0x2100)  /*!< Set the full-duplex mode at 100 Mb/s */
#define PHY_HALFDUPLEX_100M             ((uint16_t)0x2000)  /*!< Set the half-duplex mode at 100 Mb/s */
#define PHY_FULLDUPLEX_10M              ((uint16_t)0x0100)  /*!< Set the full-duplex mode at 10 Mb/s  */
#define PHY_HALFDUPLEX_10M              ((uint16_t)0x0000)  /*!< Set the half-duplex mode at 10 Mb/s  */
#define PHY_AUTONEGOTIATION             ((uint16_t)0x1000)  /*!< Enable auto-negotiation function     */
#define PHY_RESTART_AUTONEGOTIATION     ((uint16_t)0x0200)  /*!< Restart auto-negotiation function    */
#define PHY_POWERDOWN                   ((uint16_t)0x0800)  /*!< Select the power down mode           */
#define PHY_ISOLATE                     ((uint16_t)0x0400)  /*!< Isolate PHY from MII                 */

#define PHY_AUTONEGO_COMPLETE           ((uint16_t)0x0020)  /*!< Auto-Negotiation process completed   */
#define PHY_LINKED_STATUS               ((uint16_t)0x0004)  /*!< Valid link established               */
#define PHY_JABBER_DETECTION            ((uint16_t)0x0002)  /*!< Jabber condition detected            */

/* Section 4: Extended PHY Registers */
#define PHY_SR                          ((uint16_t)0x10)    /*!< PHY status register Offset                      */
#define PHY_MICR                        ((uint16_t)0x11)    /*!< MII Interrupt Control Register                  */
#define PHY_MISR                        ((uint16_t)0x12)    /*!< MII Interrupt Status and Misc. Control Register */

#define PHY_LINK_STATUS                 ((uint16_t)0x0001)  /*!< PHY Link mask                                   */
#define PHY_SPEED_STATUS                ((uint16_t)0x0002)  /*!< PHY Speed mask                                  */
#define PHY_DUPLEX_STATUS               ((uint16_t)0x0004)  /*!< PHY Duplex mask                                 */
#define PHY_MDI_STATUS					((uint16_t)0x4000)  /*!< PHY MDI mode mask                               */

#define PHY_MICR_INT_EN                 ((uint16_t)0x0002)  /*!< PHY Enable interrupts                           */
#define PHY_MICR_INT_OE                 ((uint16_t)0x0001)  /*!< PHY Enable output interrupt events              */

#define PHY_MISR_LINK_INT_EN            ((uint16_t)0x0020)  /*!< Enable Interrupt on change of link status       */
#define PHY_MISR_ANC_INT_EN				0x0004
#define PHY_LINK_INTERRUPT              ((uint16_t)0x2000)  /*!< PHY link status interrupt mask                  */

static void ethTask(void* pParam);
static uint8_t linkUpdate(Eth* pEth);
static uint16_t phyRead(Eth* pEth, uint32_t phyReg);
static void phyWrite(Eth* pEth, uint32_t phyReg, uint16_t value);
static void dmaTxDescListInit(Eth* pEth, ETHDMADesc *DMATxDescTab, uint8_t *TxBuff, uint32_t TxBuffCount);
static void dmaRxDescListInit(Eth* pEth, ETHDMADesc *DMARxDescTab, uint8_t *RxBuff, uint32_t RxBuffCount);

void EthIRQ(Eth* pEth)
{
	uint32_t dmasr = pEth->pReg->DMASR;
	pEth->pReg->DMASR = ETH_DMASR_NIS | ETH_DMASR_RS; // clear flags

	if(dmasr & ETH_DMASR_RS)
	{
		chSysLockFromISR();
		chEvtBroadcastFlagsI(&pEth->eventSource, ETH_EVENT_DATA_RECEIVED);
		chSysUnlockFromISR();
	}
}

void EthMiiIRQ(Eth* pEth)
{
	chSysLockFromISR();
	chEvtSignalI(pEth->pTask, EVENT_LINK_CHANGED);
	chSysUnlockFromISR();
}

void EthInit(Eth* pEth, EthData* pInit, tprio_t prio)
{
	memset(pEth, 0, sizeof(Eth)); // .eth memory section is not zero initialized

	pEth->data = *pInit;
	pEth->pReg = pInit->pReg;

	chMtxObjectInit(&pEth->txMutex);
	chEvtObjectInit(&pEth->eventSource);

	pEth->pCurTxDesc = pEth->txDescTab;
	pEth->pCurRxDesc = pEth->rxDescTab;

	// perform soft-reset of the ETH core
	pEth->pReg->DMABMR |= ETH_DMABMR_SR;
	while(pEth->pReg->DMABMR & ETH_DMABMR_SR)
		asm volatile("nop");

	// configure MAC MII speed for 168MHz HCLK
	pEth->pReg->MACMIIAR = ETH_MACMIIAR_CR_Div102;

	// reset PHY
	phyWrite(pEth, PHY_BCR, PHY_RESET);
	chThdSleepMilliseconds(5);

	// MACCR
	// enable checksum offloading
	pEth->pReg->MACCR = ETH_MACCR_IPCO | ETH_MACCR_RD;

	// MACFFR
	pEth->pReg->MACFFR = ETH_MACFFR_PAM;

	// MACFCR
	// disable zero-quanta pause frames
	pEth->pReg->MACFCR = ETH_MACFCR_ZQPD;

	// DMAOMR
	// enable store&forward on transmit and receive, enable operate on second frame
	pEth->pReg->DMAOMR = ETH_DMAOMR_RSF | ETH_DMAOMR_TSF | ETH_DMAOMR_OSF;

	// DMABMR
	// address-aligned beats of 32 beats for RX and TX, extended descriptor mode enabled
	pEth->pReg->DMABMR = ETH_DMABMR_AAB | ETH_DMABMR_FB | ETH_DMABMR_RDP_32Beat | ETH_DMABMR_PBL_32Beat	| ETH_DMABMR_USP | ETH_DMABMR_EDE;

	// enable receive interrupt
	pEth->pReg->DMAIER = ETH_DMAIER_NISE | ETH_DMAIER_RIE;

	dmaRxDescListInit(pEth, pEth->rxDescTab, pEth->rxBuffer, ETH_RXBUFNB);
	dmaTxDescListInit(pEth, pEth->txDescTab, pEth->txBuffer, ETH_TXBUFNB);

	phyWrite(pEth, PHY_MICR, PHY_MICR_INT_EN | PHY_MICR_INT_OE);

	uint16_t misr = phyRead(pEth, PHY_MISR);
	misr |= PHY_MISR_LINK_INT_EN;
	phyWrite(pEth, PHY_MISR, misr);

//	phyWrite(0x04, 0x0061);	// 10MBit only

	pEth->pTask = chThdCreateStatic(pEth->waTask, sizeof(pEth->waTask), prio, &ethTask, pEth);
}

static void ethTask(void* pParam)
{
	Eth* pEth = (Eth*)pParam;

	chRegSetThreadName("ETH");

	uint32_t tLastRateUpdate_us = SysTimeUSec();
	EthStats lastStats = pEth->stats;

	while(1)
	{
		eventmask_t events = chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(100));

		if(events & EVENT_LINK_CHANGED)
		{
			linkUpdate(pEth);

			if(pEth->linkStatus.up)
				chEvtBroadcastFlags(&pEth->eventSource, ETH_EVENT_LINK_UP);
			else
				chEvtBroadcastFlags(&pEth->eventSource, ETH_EVENT_LINK_DOWN);
		}

		EthGetStats(pEth);

		uint32_t tNow_us = SysTimeUSec();
		float dt_s = (tNow_us - tLastRateUpdate_us) * 1e-6f;
		if(dt_s > 1.0f)
		{
			tLastRateUpdate_us = tNow_us;

			size_t statFields = sizeof(EthStats)/sizeof(uint32_t);
			for(size_t i = 0; i < statFields; i++)
			{
				((uint32_t*)&pEth->rates)[i] = (((uint32_t*)&pEth->stats)[i] - ((uint32_t*)&lastStats)[i]) / dt_s;
			}

			lastStats = pEth->stats;
		}
	}
}

void EthAllowMAC(Eth* pEth, uint8_t slot, const uint8_t* pMac, uint8_t enable, uint8_t saFiltering)
{
	if(slot > 3)
		return;

	volatile uint32_t* pMacHigh = (volatile uint32_t *)((uint32_t)pEth->pReg + 0x40 + slot*0x08);
	volatile uint32_t* pMacLow = (volatile uint32_t *)((uint32_t)pEth->pReg + 0x44 + slot*0x08);

	uint32_t hReg = ((uint32_t)pMac[5] << 8) | (uint32_t)pMac[4];
	uint32_t lReg = ((uint32_t)pMac[3] << 24) | ((uint32_t)pMac[2] << 16) | ((uint32_t)pMac[1] << 8) | pMac[0];

	if(slot == 0)
	{
		hReg |= ETH_MACA1HR_AE;	// slot 0 is the own MAC address, must always be on
	}
	else
	{
		if(enable)
			hReg |= ETH_MACA1HR_AE;

		if(saFiltering)
			hReg |= ETH_MACA1HR_SA;
	}

	*pMacHigh = hReg;
	*pMacLow = lReg;
}

int16_t EthSendEthernetFrame(Eth* pEth, const uint8_t* pData, size_t length)
{
	if(!pEth->linkStatus.up)
		return ERROR_ETH_NO_LINK;

	chMtxLock(&pEth->txMutex);

	// Check available TX space
	ETHDMADesc* pTxDesc = pEth->pCurTxDesc;
	size_t bytesFree = 0;
	while(bytesFree < length)
	{
		if(pTxDesc->Status & ETH_DMATXDESC_OWN)
		{
			chMtxUnlock(&pEth->txMutex);
			return ERROR_NOT_ENOUGH_MEMORY;
		}

		bytesFree += ETH_TX_BUF_SIZE;
		pTxDesc = (ETHDMADesc*)pTxDesc->Buffer2NextDescAddr;
	}

	int16_t result = 0;

	for(uint32_t bytesSent = 0; bytesSent < length; bytesSent += ETH_TX_BUF_SIZE)
	{
		// how much data to send?
		uint32_t sendLen = length - bytesSent;
		if(sendLen > ETH_TX_BUF_SIZE)
			sendLen = ETH_TX_BUF_SIZE;

		// copy data
		memcpy((void*)pEth->pCurTxDesc->Buffer1Addr, pData, sendLen);

		pData += sendLen;

		// clear first and last flags
		pEth->pCurTxDesc->Status &= ~(ETH_DMATXDESC_FS | ETH_DMATXDESC_LS);

		if(bytesSent == 0)	// first segment
			pEth->pCurTxDesc->Status |= ETH_DMATXDESC_FS;

		if(bytesSent + sendLen >= length)	// last segment
			pEth->pCurTxDesc->Status |= ETH_DMATXDESC_LS;

		// set data size
		pEth->pCurTxDesc->ControlBufferSize = (sendLen & ETH_DMATXDESC_TBS1);

		// "let fly..."
		pEth->pCurTxDesc->Status |= ETH_DMATXDESC_OWN;

		// point to next descriptor
		pEth->pCurTxDesc = (ETHDMADesc*)pEth->pCurTxDesc->Buffer2NextDescAddr;
	}

	// When Tx Buffer unavailable flag is set: clear it and resume transmission
	if(pEth->pReg->DMASR & ETH_DMASR_TBUS)
	{
		// Clear TBUS ETHERNET DMA flag
		pEth->pReg->DMASR = ETH_DMASR_TBUS;

		// Resume DMA transmission
		pEth->pReg->DMATPDR = 0;
	}

	pEth->stats.txFramesProcessed++;
	pEth->stats.txBytesProcessed += length;

	chMtxUnlock(&pEth->txMutex);

	return result;
}

void EthPrintDebugOutput(Eth* pEth)
{
	uint16_t bmcr = phyRead(pEth, 0x00);
	uint16_t bmsr = phyRead(pEth, 0x01);
	uint16_t physts = phyRead(pEth, 0x10);
	uint16_t micr = phyRead(pEth, 0x11);
	uint16_t misr = phyRead(pEth, 0x12);

	printf("BMCR:   0x%04X\r\n", bmcr);
	printf("BMSR:   0x%04X\r\n", bmsr);
	printf("PHYSTS: 0x%04X\r\n", physts);
	printf("MICR:   0x%04X\r\n", micr);
	printf("MISR:   0x%04X\r\n", misr);

	printf("MACDBG:  0x%08X\r\n", pEth->pReg->MACDBGR);
	printf("MACA0HR: 0x%08X\r\n", pEth->pReg->MACA0HR);
	printf("MACA0LR: 0x%08X\r\n", pEth->pReg->MACA0LR);
	printf("DMASR:   0x%08X\r\n", pEth->pReg->DMASR);

	printf("txDesc:     0x%08X\r\n", pEth->txDescTab);
	printf("pCurTxDesc: 0x%08X\r\n", pEth->pCurTxDesc);
	printf("DMACHTDR:   0x%08X\r\n", pEth->pReg->DMACHTDR);

	ETHDMADesc* pTxDesc = (ETHDMADesc*)pEth->pReg->DMACHTDR;
	printf("TDES0:      0x%08X\r\n", pTxDesc->Status);
	printf("TDES1:      0x%08X\r\n", pTxDesc->ControlBufferSize);
}

int16_t EthGetEthernetFrame(Eth* pEth, uint8_t* pData, size_t dataSize, size_t* pBytesRead)
{
	int16_t result = ERROR_ETH_EMPTY;
	uint16_t frameSize = 0;
	uint16_t frameSegments = 0;
	uint8_t badFrame = 0;
	if(pBytesRead)
		*pBytesRead = 0;

	if(!pEth->linkStatus.up)
		return result;

	// run through descriptors to find out if there is a complete frame
	for(ETHDMADesc* pDesc = pEth->pCurRxDesc; 1; pDesc = (ETHDMADesc*)pDesc->Buffer2NextDescAddr)
	{
		if(pDesc->Status & ETH_DMARXDESC_OWN)
			return result;

		++frameSegments;

		if(pDesc->Status & ETH_DMARXDESC_LS)
		{
			if(pDesc->Status & (1 << 15)) // Frame error?
			{
				LogErrorC("Frame error", pDesc->Status);
				badFrame = 1;
			}

			frameSize = ((pDesc->Status & ETH_DMARXDESC_FL) >> 16) - 4;
			break;
		}
	}

	if(!badFrame)
	{
		if(frameSize <= dataSize)
		{
			uint32_t bytesCopied = 0;
			ETHDMADesc* pRxDMADesc = pEth->pCurRxDesc;

			for(uint32_t i = 0; i < frameSegments; i++)
			{
				uint32_t bytesToCopy = frameSize-bytesCopied;
				if(bytesToCopy > ETH_RX_BUF_SIZE)
					bytesToCopy = ETH_RX_BUF_SIZE;

				memcpy(pData+bytesCopied, (void*)pRxDMADesc->Buffer1Addr, bytesToCopy);
				bytesCopied += bytesToCopy;

				// move on to next descriptor
				pRxDMADesc = (ETHDMADesc*)pRxDMADesc->Buffer2NextDescAddr;
			}

			pEth->stats.rxFramesProcessed++;
			pEth->stats.rxBytesProcessed += frameSize;

			if(pBytesRead)
				*pBytesRead = frameSize;

			result = 0;
		}
		else
		{
			result = ERROR_ETH_RX_TOO_LARGE;
		}
	}
	else
	{
		result = ERROR_ETH_BAD_FRAME;
	}

	// Set Own bit in Rx descriptors: gives the buffers back to DMA
	for(uint32_t i = 0; i < frameSegments; i++)
	{
		pEth->pCurRxDesc->Status = ETH_DMARXDESC_OWN;
		pEth->pCurRxDesc = (ETHDMADesc*)pEth->pCurRxDesc->Buffer2NextDescAddr;
	}

	if(pEth->pReg->DMASR & ETH_DMASR_RBUS)
	{
		pEth->pReg->DMASR = ETH_DMASR_RBUS;
		pEth->pReg->DMARPDR = 0;
	}

	return result;
}

EthStats* EthGetStats(Eth* pEth)
{
	pEth->stats.txFrames = pEth->pReg->MMCTGFCR;
	pEth->stats.rxFrames = pEth->pReg->MMCRGUFCR;

	uint32_t dmamfbocr = pEth->pReg->DMAMFBOCR;
	pEth->stats.rxMissedApp = (dmamfbocr & 0xFFE0000) >> 17;
	pEth->stats.rxMissedCtrl = dmamfbocr & 0xFFFF;

	return &pEth->stats;
}

static void startMAC(Eth* pEth)
{
	// enable transmitter and receiver
	pEth->pReg->MACCR |= ETH_MACCR_TE | ETH_MACCR_RE;

	// flush transmit FIFO and wait for operation to finish
	pEth->pReg->DMAOMR |= ETH_DMAOMR_FTF;
	while(pEth->pReg->DMAOMR & ETH_DMAOMR_FTF)
		asm volatile("nop");

	// enable DMA transmission and reception
	pEth->pReg->DMAOMR |= ETH_DMAOMR_ST | ETH_DMAOMR_SR;
}

static void stopMAC(Eth* pEth)
{
	// disable DMA transmission and reception
	pEth->pReg->DMAOMR &= ~(ETH_DMAOMR_ST | ETH_DMAOMR_SR);

	// disable receiver
	pEth->pReg->MACCR &= ~ETH_MACCR_RE;

	// flush transmit FIFO and wait for operation to finish
	pEth->pReg->DMAOMR |= ETH_DMAOMR_FTF;
	while(pEth->pReg->DMAOMR & ETH_DMAOMR_FTF)
		asm volatile("nop");

	// disable transmitter
	pEth->pReg->MACCR &= ~ETH_MACCR_TE;
}

static uint8_t linkUpdate(Eth* pEth)
{
	// Read PHY_MISR
	phyRead(pEth, PHY_MISR);

	// Read PHY_SR
	uint16_t sr = phyRead(pEth, PHY_SR);

	// Check whether the link is up or down
	if(sr & PHY_LINK_STATUS)
	{
		if(!pEth->linkStatus.up)
		{
			pEth->linkStatus.up = 1;

			// Configure the MAC with the Duplex Mode fixed by the auto-negotiation process
			if(sr & PHY_DUPLEX_STATUS)
				pEth->linkStatus.fullDuplex = 1;
			else
				pEth->linkStatus.fullDuplex = 0;

			// Configure the MAC with the speed fixed by the auto-negotiation process
			if(sr & PHY_SPEED_STATUS)
				pEth->linkStatus.speed100M = 0;
			else
				pEth->linkStatus.speed100M = 1;

			if(sr & PHY_MDI_STATUS)
				pEth->linkStatus.MDI = 0;	// normal MDI mode
			else
				pEth->linkStatus.MDI = 1; 	// MDI-X mode, TX/RX automatically swapped

			// ETHERNET MAC Re-Configuration
			pEth->pReg->MACCR &= ~(ETH_MACCR_FES | ETH_MACCR_DM);
			if(pEth->linkStatus.speed100M)
				pEth->pReg->MACCR |= ETH_MACCR_FES;
			if(pEth->linkStatus.fullDuplex)
				pEth->pReg->MACCR |= ETH_MACCR_DM;

			// Restart MAC interface
			startMAC(pEth);
		}
	}
	else
	{
		if(pEth->linkStatus.up)
		{
			// Stop MAC interface
			stopMAC(pEth);
		}

		pEth->linkStatus.up = 0;
	}

	if(pEth->linkStatus.up)
	{
		printf("Link UP\r\n");

		if(pEth->linkStatus.speed100M)
			printf("Speed: 100M\r\n");
		else
			printf("Speed: 10M\r\n");

		if(pEth->linkStatus.fullDuplex)
			printf("Full Duplex\r\n");
		else
			printf("Half Duplex\r\n");

		if(pEth->linkStatus.MDI)
			printf("MDI mode\r\n");
		else
			printf("MDI-X mode\r\n");
	}
	else
	{
		printf("Link DOWN\r\n");
	}

	return pEth->linkStatus.up;
}

static uint16_t phyRead(Eth* pEth, uint32_t phyReg)
{
	while(pEth->pReg->MACMIIAR & ETH_MACMIIAR_MB)
		chThdSleepMilliseconds(1);

	uint32_t ar = pEth->pReg->MACMIIAR & ETH_MACMIIAR_CR;
	ar |= (phyReg << 6) | (pEth->data.phyAddr << 11) | ETH_MACMIIAR_MB;

	pEth->pReg->MACMIIAR = ar;

	while(pEth->pReg->MACMIIAR & ETH_MACMIIAR_MB)
		chThdSleepMilliseconds(1);

	return (uint16_t)pEth->pReg->MACMIIDR;
}

static void phyWrite(Eth* pEth, uint32_t phyReg, uint16_t value)
{
	while(pEth->pReg->MACMIIAR & ETH_MACMIIAR_MB)
		chThdSleepMilliseconds(1);

	uint32_t ar = pEth->pReg->MACMIIAR & ETH_MACMIIAR_CR;
	ar |= (phyReg << 6) | (pEth->data.phyAddr << 11) | ETH_MACMIIAR_MB | ETH_MACMIIAR_MW;

	pEth->pReg->MACMIIDR = value;
	pEth->pReg->MACMIIAR = ar;

	while(pEth->pReg->MACMIIAR & ETH_MACMIIAR_MB)
		chThdSleepMilliseconds(1);
}

// Initializes the DMA Tx descriptors in chain mode.
static void dmaTxDescListInit(Eth* pEth, ETHDMADesc *DMATxDescTab, uint8_t *TxBuff, uint32_t TxBuffCount)
{
	uint32_t i = 0;
	ETHDMADesc *dmatxdesc;

	/* Fill each DMATxDesc descriptor with the right values */
	for(i = 0; i < TxBuffCount; i++)
	{
		/* Get the pointer on the ith member of the Tx Desc list */
		dmatxdesc = DMATxDescTab + i;

		/* Set Second Address Chained bit */
		dmatxdesc->Status = ETH_DMATXDESC_TCH;

		/* Set Buffer1 address pointer */
		dmatxdesc->Buffer1Addr = (uint32_t) (&TxBuff[i * ETH_TX_BUF_SIZE]);

		/* Set the DMA Tx descriptors checksum insertion */
		dmatxdesc->Status |= ETH_DMATXDESC_CHECKSUMTCPUDPICMPFULL;

		/* Initialize the next descriptor with the Next Descriptor Polling Enable */
		if(i < (TxBuffCount - 1))
		{
			/* Set next descriptor address register with next descriptor base address */
			dmatxdesc->Buffer2NextDescAddr = (uint32_t) (DMATxDescTab + i + 1);
		}
		else
		{
			/* For last descriptor, set next descriptor address register equal to the first descriptor base address */
			dmatxdesc->Buffer2NextDescAddr = (uint32_t) DMATxDescTab;
		}
	}

	/* Set Transmit Descriptor List Address Register */
	pEth->pReg->DMATDLAR = (uint32_t) DMATxDescTab;
}

 // Initializes the DMA Rx descriptors in chain mode.
static void dmaRxDescListInit(Eth* pEth, ETHDMADesc *DMARxDescTab, uint8_t *RxBuff, uint32_t RxBuffCount)
{
	uint32_t i = 0;
	ETHDMADesc* DMARxDesc;

	/* Fill each DMARxDesc descriptor with the right values */
	for(i = 0; i < RxBuffCount; i++)
	{
		/* Get the pointer on the ith member of the Rx Desc list */
		DMARxDesc = DMARxDescTab + i;

		/* Set Own bit of the Rx descriptor Status */
		DMARxDesc->Status = ETH_DMARXDESC_OWN;

		/* Set Buffer1 size and Second Address Chained bit */
		DMARxDesc->ControlBufferSize = ETH_DMARXDESC_RCH | ETH_RX_BUF_SIZE;

		/* Set Buffer1 address pointer */
		DMARxDesc->Buffer1Addr = (uint32_t) (&RxBuff[i * ETH_RX_BUF_SIZE]);

		/* Enable Ethernet DMA Rx Descriptor interrupt */
		DMARxDesc->ControlBufferSize &= ~ETH_DMARXDESC_DIC;

		/* Initialize the next descriptor with the Next Descriptor Polling Enable */
		if(i < (RxBuffCount - 1))
		{
			/* Set next descriptor address register with next descriptor base address */
			DMARxDesc->Buffer2NextDescAddr = (uint32_t) (DMARxDescTab + i + 1);
		}
		else
		{
			/* For last descriptor, set next descriptor address register equal to the first descriptor base address */
			DMARxDesc->Buffer2NextDescAddr = (uint32_t) (DMARxDescTab);
		}
	}

	/* Set Receive Descriptor List Address Register */
	pEth->pReg->DMARDLAR = (uint32_t) DMARxDescTab;
}
