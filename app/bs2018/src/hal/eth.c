/*
 * eth.c
 *
 *  Created on: 23.10.2017
 *      Author: AndreR
 */

#include "eth.h"
#include "../constants.h"
#include "util/console.h"
#include "util/init_hal.h"
#include "errors.h"
#include <string.h>

#define ETH_EVENT_QUEUE_SIZE	4

#define ETH_MAX_PACKET_SIZE		1524	// ETH_HEADER + ETH_EXTRA + VLAN_TAG + MAX_ETH_PAYLOAD + ETH_CRC

#define ETH_RX_BUF_SIZE			128
#define ETH_TX_BUF_SIZE			128
#define ETH_RXBUFNB				64
#define ETH_TXBUFNB				64

/* Section 2: PHY configuration section */
/* DP83848 PHY Address*/
#define DP83848_PHY_ADDRESS             0x01

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

typedef struct __attribute__((packed)) _ETHDMADesc
{
	volatile uint32_t Status;
	uint32_t ControlBufferSize;		// Control and Buffer1, Buffer2 lengths
	uint32_t Buffer1Addr;			// Buffer1 address pointer
	uint32_t Buffer2NextDescAddr;	// Buffer2 or next descriptor address pointer
	uint32_t ExtendedStatus;		// Extended status for PTP receive descriptor
	uint32_t _reserved;
	uint32_t TimeStampLow;			// Time Stamp Low value for transmit and receive
	uint32_t TimeStampHigh;			// Time Stamp High value for transmit and receive
} ETHDMADesc;

typedef struct _Eth
{
	uint8_t rxBuffer[ETH_RXBUFNB*ETH_RX_BUF_SIZE] __attribute__((aligned(16)));
	uint8_t txBuffer[ETH_TXBUFNB*ETH_TX_BUF_SIZE] __attribute__((aligned(16)));
	ETHDMADesc rxDescTab[ETH_RXBUFNB] __attribute__((aligned(16)));
	ETHDMADesc txDescTab[ETH_TXBUFNB] __attribute__((aligned(16)));

	ETHDMADesc* pCurTxDesc;
	ETHDMADesc* pCurRxDesc;

	mailbox_t eventQueue;
	msg_t eventQueueData[ETH_EVENT_QUEUE_SIZE];

	EthLinkCallback cbLink;
	EthRxCallback cbRx;

	EthStats stats;

	LinkStatus linkStatus;

	uint32_t phyAddr;
} Eth;

static Eth eth __attribute__((aligned(16), section(".dtc")));

void ETH_IRQHandler(void)
{
	CH_IRQ_PROLOGUE();

	uint32_t dmasr = ETH->DMASR;
	ETH->DMASR = ETH_DMASR_NIS | ETH_DMASR_RS; // clear flags

	if((dmasr & ETH_DMASR_RS) && eth.cbRx)
		(*eth.cbRx)();

	CH_IRQ_EPILOGUE();
}

void EXTI9_5_IRQHandler(void)
{
	CH_IRQ_PROLOGUE();

	EXTI->PR = GPIO_PIN_5;

	if(eth.cbLink)
		(*eth.cbLink)();

	CH_IRQ_EPILOGUE();
}

 // Initializes the DMA Tx descriptors in chain mode.
static void dmaTxDescListInit(ETHDMADesc *DMATxDescTab, uint8_t *TxBuff, uint32_t TxBuffCount)
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
	ETH->DMATDLAR = (uint32_t) DMATxDescTab;
}

 // Initializes the DMA Rx descriptors in chain mode.
static void dmaRxDescListInit(ETHDMADesc *DMARxDescTab, uint8_t *RxBuff, uint32_t RxBuffCount)
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
	ETH->DMARDLAR = (uint32_t) DMARxDescTab;
}

uint16_t ETHPhyRead(uint32_t phyReg)
{
	while(ETH->MACMIIAR & ETH_MACMIIAR_MB)
		chThdSleepMilliseconds(1);

	uint32_t ar = ETH->MACMIIAR & ETH_MACMIIAR_CR;
	ar |= (phyReg << 6) | (eth.phyAddr << 11) | ETH_MACMIIAR_MB;

	ETH->MACMIIAR = ar;

	while(ETH->MACMIIAR & ETH_MACMIIAR_MB)
		chThdSleepMilliseconds(1);

	return (uint16_t)ETH->MACMIIDR;
}

void ETHPhyWrite(uint32_t phyReg, uint16_t value)
{
	while(ETH->MACMIIAR & ETH_MACMIIAR_MB)
		chThdSleepMilliseconds(1);

	uint32_t ar = ETH->MACMIIAR & ETH_MACMIIAR_CR;
	ar |= (phyReg << 6) | (eth.phyAddr << 11) | ETH_MACMIIAR_MB | ETH_MACMIIAR_MW;

	ETH->MACMIIDR = value;
	ETH->MACMIIAR = ar;

	while(ETH->MACMIIAR & ETH_MACMIIAR_MB)
		chThdSleepMilliseconds(1);
}

static void startMAC()
{
	// enable transmitter and receiver
	ETH->MACCR |= ETH_MACCR_TE | ETH_MACCR_RE;

	// flush transmit FIFO and wait for operation to finish
	ETH->DMAOMR |= ETH_DMAOMR_FTF;
	while(ETH->DMAOMR & ETH_DMAOMR_FTF)
		asm volatile("nop");

	// enable DMA transmission and reception
	ETH->DMAOMR |= ETH_DMAOMR_ST | ETH_DMAOMR_SR;
}

static void stopMAC()
{
	// disable DMA transmission and reception
	ETH->DMAOMR &= ~(ETH_DMAOMR_ST | ETH_DMAOMR_SR);

	// disable receiver
	ETH->MACCR &= ~ETH_MACCR_RE;

	// flush transmit FIFO and wait for operation to finish
	ETH->DMAOMR |= ETH_DMAOMR_FTF;
	while(ETH->DMAOMR & ETH_DMAOMR_FTF)
		asm volatile("nop");

	// disable transmitter
	ETH->MACCR &= ~ETH_MACCR_TE;
}

void EthInit()
{
	GPIOInitData gpioInit;
	const uint8_t mac[6] = {2, 84, 73, 71, 69, 82}; // ASCII: 02-T-I-G-E-R

	chMBObjectInit(&eth.eventQueue, eth.eventQueueData, ETH_EVENT_QUEUE_SIZE);

	eth.phyAddr = DP83848_PHY_ADDRESS;
	eth.pCurTxDesc = eth.txDescTab;
	eth.pCurRxDesc = eth.rxDescTab;

	// select MII mode
	SYSCFG->PMC &= ~SYSCFG_PMC_MII_RMII_SEL;

	// configure pins
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.alternate = 11;	// ETH
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_50MHZ;
	gpioInit.pupd = GPIO_PUPD_NONE;
	GPIOInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_7, &gpioInit);
	GPIOInit(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13, &gpioInit);
	GPIOInit(GPIOC, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5, &gpioInit);

	gpioInit.mode = GPIO_MODE_EXTI;
	gpioInit.pupd = GPIO_PUPD_NONE;
	gpioInit.extiTrigger = GPIO_EXTI_TRIG_FALLING;
	GPIOInit(GPIOE, GPIO_PIN_5, &gpioInit);

	RCC->AHB1ENR |= RCC_AHB1ENR_ETHMACEN | RCC_AHB1ENR_ETHMACTXEN | RCC_AHB1ENR_ETHMACRXEN;

	// perform soft-reset of the ETH core
	ETH->DMABMR |= ETH_DMABMR_SR;
	while(ETH->DMABMR & ETH_DMABMR_SR)
		asm volatile("nop");

	// configure MAC MII speed for 168MHz HCLK
	ETH->MACMIIAR = ETH_MACMIIAR_CR_Div102;

	// reset PHY
	ETHPhyWrite(PHY_BCR, PHY_RESET);
	chThdSleepMilliseconds(5);

	// MACCR
	// enable checksum offloading
	ETH->MACCR = ETH_MACCR_IPCO | ETH_MACCR_RD;

	// MACFFR
	ETH->MACFFR = 0;

	// MACFCR
	// disable zero-quanta pause frames
	ETH->MACFCR = ETH_MACFCR_ZQPD;

	// DMAOMR
	// enable store&forward on transmit and receive, enable operate on second frame
	ETH->DMAOMR = ETH_DMAOMR_RSF | ETH_DMAOMR_TSF | ETH_DMAOMR_OSF;

	// DMABMR
	// address-aligned beats of 32 beats for RX and TX, extended descriptor mode enabled
	ETH->DMABMR = ETH_DMABMR_AAB | ETH_DMABMR_FB | ETH_DMABMR_RDP_32Beat | ETH_DMABMR_PBL_32Beat
			| ETH_DMABMR_USP | ETH_DMABMR_EDE;

	// enable receive interrupt
	ETH->DMAIER = ETH_DMAIER_NISE | ETH_DMAIER_RIE;

	EthAllowMAC(0, mac, 1, 0);	// configure our MAC address

	dmaRxDescListInit(eth.rxDescTab, eth.rxBuffer, ETH_RXBUFNB);
	dmaTxDescListInit(eth.txDescTab, eth.txBuffer, ETH_TXBUFNB);

	ETHPhyWrite(PHY_MICR, PHY_MICR_INT_EN | PHY_MICR_INT_OE);

	uint16_t misr = ETHPhyRead(PHY_MISR);
	misr |= PHY_MISR_LINK_INT_EN;
	ETHPhyWrite(PHY_MISR, misr);

//	phyWrite(0x04, 0x0061);	// 10MBit only

	NVICEnableIRQ(EXTI9_5_IRQn, IRQL_ETH_LINK);
	NVICEnableIRQ(ETH_IRQn, IRQL_ETH_DATA);
}

void EthSetCallbacks(EthLinkCallback cbLink, EthRxCallback cbRx)
{
	eth.cbLink = cbLink;
	eth.cbRx = cbRx;
}

void EthAllowMAC(uint8_t slot, const uint8_t* pMac, uint8_t enable, uint8_t saFiltering)
{
	if(slot > 3)
		return;

	volatile uint32_t* pMacHigh = (volatile uint32_t *)((uint32_t)(ETH_MAC_ADDR_HBASE + slot*0x08));
	volatile uint32_t* pMacLow = (volatile uint32_t *)((uint32_t)(ETH_MAC_ADDR_LBASE + slot*0x08));

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

int16_t EthSendEthernetFrame(uint8_t* pData, uint32_t length)
{
	if(!eth.linkStatus.up)
		return ERROR_ETH_NO_LINK;

	int16_t result = 0;

	for(uint32_t bytesSent = 0; bytesSent < length; bytesSent += ETH_TX_BUF_SIZE)
	{
		if(eth.pCurTxDesc->Status & ETH_DMATXDESC_OWN)
		{
			chThdSleepMilliseconds(1);
			bytesSent -= ETH_TX_BUF_SIZE;
			result = ERROR_ETH_SEND_DELAYED;
			continue;
		}

		// how much data to send?
		uint32_t sendLen = length-bytesSent;
		if(sendLen > ETH_TX_BUF_SIZE)
			sendLen = ETH_TX_BUF_SIZE;

		// copy data
		memcpy((void*)eth.pCurTxDesc->Buffer1Addr, pData, sendLen);

		pData += sendLen;

		// clear first and last flags
		eth.pCurTxDesc->Status &= ~(ETH_DMATXDESC_FS | ETH_DMATXDESC_LS);

		if(bytesSent == 0)	// first segment
			eth.pCurTxDesc->Status |= ETH_DMATXDESC_FS;

		if(bytesSent + sendLen >= length)	// last segment
			eth.pCurTxDesc->Status |= ETH_DMATXDESC_LS;

		// set data size
		eth.pCurTxDesc->ControlBufferSize = (sendLen & ETH_DMATXDESC_TBS1);

		// "let fly..."
		eth.pCurTxDesc->Status |= ETH_DMATXDESC_OWN;

		// point to next descriptor
		eth.pCurTxDesc = (ETHDMADesc*)eth.pCurTxDesc->Buffer2NextDescAddr;
	}

	// When Tx Buffer unavailable flag is set: clear it and resume transmission
	if(ETH->DMASR & ETH_DMASR_TBUS)
	{
		// Clear TBUS ETHERNET DMA flag
		ETH->DMASR = ETH_DMASR_TBUS;

		// Resume DMA transmission
		ETH->DMATPDR = 0;
	}

	eth.stats.txFramesProcessed++;
	eth.stats.txBytesProcessed += length;

	return result;
}

const LinkStatus* EthGetLinkStatus()
{
	return &eth.linkStatus;
}

void EthDebug()
{
	ConsolePrint("MACDBG:  0x%08X\r\n", ETH->MACDBGR);
	ConsolePrint("MACA0HR: 0x%08X\r\n", ETH->MACA0HR);
	ConsolePrint("MACA0LR: 0x%08X\r\n", ETH->MACA0LR);
	ConsolePrint("DMASR:   0x%08X\r\n", ETH->DMASR);

	ConsolePrint("txDesc:     0x%08X\r\n", eth.txDescTab);
	ConsolePrint("pCurTxDesc: 0x%08X\r\n", eth.pCurTxDesc);
	ConsolePrint("DMACHTDR:   0x%08X\r\n", ETH->DMACHTDR);

	ETHDMADesc* pTxDesc = (ETHDMADesc*)ETH->DMACHTDR;
	ConsolePrint("TDES0:      0x%08X\r\n", pTxDesc->Status);
	ConsolePrint("TDES1:      0x%08X\r\n", pTxDesc->ControlBufferSize);

}

uint8_t EthLinkUpdate()
{
	// Read PHY_MISR
	ETHPhyRead(PHY_MISR);

	// Read PHY_SR
	uint16_t sr = ETHPhyRead(PHY_SR);

	// Check whether the link is up or down
	if(sr & PHY_LINK_STATUS)
	{
		if(!eth.linkStatus.up)
		{
			eth.linkStatus.up = 1;

			// Configure the MAC with the Duplex Mode fixed by the auto-negotiation process
			if(sr & PHY_DUPLEX_STATUS)
				eth.linkStatus.fullDuplex = 1;
			else
				eth.linkStatus.fullDuplex = 0;

			// Configure the MAC with the speed fixed by the auto-negotiation process
			if(sr & PHY_SPEED_STATUS)
				eth.linkStatus.speed100M = 0;
			else
				eth.linkStatus.speed100M = 1;

			if(sr & PHY_MDI_STATUS)
				eth.linkStatus.MDI = 0;	// normal MDI mode
			else
				eth.linkStatus.MDI = 1; 	// MDI-X mode, TX/RX automatically swapped

			// ETHERNET MAC Re-Configuration
			ETH->MACCR &= ~(ETH_MACCR_FES | ETH_MACCR_DM);
			if(eth.linkStatus.speed100M)
				ETH->MACCR |= ETH_MACCR_FES;
			if(eth.linkStatus.fullDuplex)
				ETH->MACCR |= ETH_MACCR_DM;

			// Restart MAC interface
			startMAC();
		}
	}
	else
	{
		if(eth.linkStatus.up)
		{
			// Stop MAC interface
			stopMAC();
		}

		eth.linkStatus.up = 0;
	}

	if(eth.linkStatus.up)
	{
		ConsolePrint("Link UP\r\n");

		if(eth.linkStatus.speed100M)
			ConsolePrint("Speed: 100M\r\n");
		else
			ConsolePrint("Speed: 10M\r\n");

		if(eth.linkStatus.fullDuplex)
			ConsolePrint("Full Duplex\r\n");
		else
			ConsolePrint("Half Duplex\r\n");

		if(eth.linkStatus.MDI)
			ConsolePrint("MDI mode\r\n");
		else
			ConsolePrint("MDI-X mode\r\n");
	}
	else
	{
		ConsolePrint("Link DOWN\r\n");
	}

	return eth.linkStatus.up;
}

int16_t EthGetEthernetFrame(uint8_t* pData, uint32_t dataSize, uint32_t* pBytesRead)
{
	int16_t result = ERROR_ETH_EMPTY;
	uint16_t frameSize = 0;
	uint16_t frameSegments = 0;
	if(pBytesRead)
		*pBytesRead = 0;

	if(!eth.linkStatus.up)
		return result;

	// run through descriptors to find out if there is a complete frame
	for(ETHDMADesc* pDesc = eth.pCurRxDesc; 1; pDesc = (ETHDMADesc*)pDesc->Buffer2NextDescAddr)
	{
		if(pDesc->Status & ETH_DMARXDESC_OWN)
			return result;

		++frameSegments;

		if(pDesc->Status & ETH_DMARXDESC_LS)
		{
			frameSize = ((pDesc->Status & ETH_DMARXDESC_FL) >> 16) - 4;
			break;
		}
	}

	if(frameSize <= dataSize)
	{
		uint32_t bytesCopied = 0;
		ETHDMADesc* pRxDMADesc = eth.pCurRxDesc;

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

		eth.stats.rxFramesProcessed++;
		eth.stats.rxBytesProcessed += frameSize;

		if(pBytesRead)
			*pBytesRead = frameSize;
		result = 0;
	}
	else
	{
		result = ERROR_ETH_RX_TOO_LARGE;
	}

	// Set Own bit in Rx descriptors: gives the buffers back to DMA
	for(uint32_t i = 0; i < frameSegments; i++)
	{
		eth.pCurRxDesc->Status |= ETH_DMARXDESC_OWN;
		eth.pCurRxDesc = (ETHDMADesc*)eth.pCurRxDesc->Buffer2NextDescAddr;
	}

	return result;
}

EthStats* EthGetStats()
{
	eth.stats.txFrames = ETH->MMCTGFCR;
	eth.stats.rxFrames = ETH->MMCRGUFCR;

	uint32_t dmamfbocr = ETH->DMAMFBOCR;
	eth.stats.rxMissedApp = (dmamfbocr & 0xFFE0000) >> 17;
	eth.stats.rxMissedCtrl = dmamfbocr & 0xFFFF;

	return &eth.stats;
}
