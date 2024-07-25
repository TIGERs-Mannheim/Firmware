#include "usb.h"
#include "hal/init_hal.h"
#include "hal/sys_time.h"
#include "util/log.h"
#include <stdio.h>

#define USB_OTG_FS_HOST		((USB_OTG_HostTypeDef*) (USB_OTG_FS_PERIPH_BASE + USB_OTG_HOST_BASE))
#define USB_OTG_FS_HPRT		((volatile uint32_t*) (USB_OTG_FS_PERIPH_BASE + USB_OTG_HOST_PORT_BASE))
#define USB_OTG_FS_PCGCCTL	((volatile uint32_t*) (USB_OTG_FS_PERIPH_BASE + USB_OTG_PCGCCTL_BASE))
#define USB_HC(i)	      	((USB_OTG_HostChannelTypeDef*) (USB_OTG_FS_PERIPH_BASE + USB_OTG_HOST_CHANNEL_BASE + (i)*USB_OTG_HOST_CHANNEL_SIZE))
#define USB_DFIFO(i)	   *(volatile uint32_t *) (USB_OTG_FS_PERIPH_BASE + USB_OTG_FIFO_BASE + (i) * USB_OTG_FIFO_SIZE)

USBGlobal usb;

#define EVENT_MODE_MISMATCH		0x0001
#define EVENT_CONNECT_DETECT	0x0002
#define EVENT_PORT_ENABLED		0x0004
#define EVENT_PORT_DISABLED		0x0008
#define EVENT_DISCONNECT		0x0010
#define EVENT_RX				0x0020
#define EVENT_SOF				0x0040
#define EVENT_PORT_OVERCURRENT	0x0080
#define EVENT_CHANNEL_IRQ		0x1000	// | (channel number) << 8
#define EVENT_BEGIN_IRP			0x2000	// | (channel number) << 8

static void eventRx();
static void eventChIRQ(uint32_t ch);
//static void restartTransfer(USBEndpoint* pEP, USBIRP* pIRP);
static void abortAllIRPs(USBEndpoint* pEP, USBIRPState state);

CH_IRQ_HANDLER(Vector1D4) // OTG_FS
{
	CH_IRQ_PROLOGUE();

	msg_t event = 0;

	uint32_t gintsts = USB_OTG_FS->GINTSTS;
	USB_OTG_FS->GINTSTS = gintsts;

	if(gintsts & USB_OTG_GINTSTS_MMIS)
		event |= EVENT_MODE_MISMATCH;

	if(gintsts & USB_OTG_GINTSTS_DISCINT)
		event |= EVENT_DISCONNECT;

	if(gintsts & USB_OTG_GINTSTS_SOF)
		event |= EVENT_SOF;

	// host port interrupt
	if(gintsts & USB_OTG_GINTSTS_HPRTINT)
	{
		uint32_t hprt = *USB_OTG_FS_HPRT;
		uint32_t hprtClr = *USB_OTG_FS_HPRT;

		hprtClr &= ~(USB_OTG_HPRT_PENA | USB_OTG_HPRT_PCDET | USB_OTG_HPRT_PENCHNG | USB_OTG_HPRT_POCCHNG);

		if(hprt & USB_OTG_HPRT_PCDET)
		{
			hprtClr |= USB_OTG_HPRT_PCDET;

			event |= EVENT_CONNECT_DETECT;
		}

		if(hprt & USB_OTG_HPRT_PENCHNG)
		{
			hprtClr |= USB_OTG_HPRT_PENCHNG;

			if(hprt & USB_OTG_HPRT_PENA)
			{
				event |= EVENT_PORT_ENABLED;
			}
			else
			{
				event |= EVENT_PORT_DISABLED;
			}
		}

		if(hprt & USB_OTG_HPRT_POCCHNG)
		{
			hprtClr |= USB_OTG_HPRT_POCCHNG;

			event |= EVENT_PORT_OVERCURRENT;
		}

		*USB_OTG_FS_HPRT = hprtClr;
	}

	// host channel interrupt
	if(gintsts & USB_OTG_GINTSTS_HCINT)
	{
		uint32_t haint = USB_OTG_FS_HOST->HAINT;

		for(uint32_t ch = 0; ch < 8; ch++)
		{
			if(haint & (1 << ch))
			{
				event |= EVENT_CHANNEL_IRQ | (ch << 8);

				USB_OTG_HostChannelTypeDef* pCh = USB_HC(ch);

				uint32_t hcint = pCh->HCINT;

				chSysLockFromISR();
				if(chMBPostI(&usb.endpoints[ch].eventQueue, hcint) != MSG_OK)
					LogError("MB full!\r\n");
				chSysUnlockFromISR();

				pCh->HCINT = hcint;	// clear IRQs

				break;
			}
		}
	}

	if(gintsts & USB_OTG_GINTSTS_RXFLVL)
	{
		USB_OTG_FS->GINTMSK &= ~USB_OTG_GINTMSK_RXFLVLM;

		event |= EVENT_RX;
	}

	if(event != 0)
	{
		chSysLockFromISR();
		if(chMBPostI(&usb.eventQueue, event) != MSG_OK)
			LogError("Master MB full\r\n");
		chSysUnlockFromISR();
	}

	CH_IRQ_EPILOGUE();
}

void USBFlushTxFifo(USB_OTG_GlobalTypeDef *USBx, uint32_t num)
{
  USBx->GRSTCTL = (USB_OTG_GRSTCTL_TXFFLSH | (num << 6));

  while((USBx->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) == USB_OTG_GRSTCTL_TXFFLSH)
	  asm volatile("nop");
}

void USBFlushRxFifo(USB_OTG_GlobalTypeDef *USBx)
{
  USBx->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;

  while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) == USB_OTG_GRSTCTL_RXFFLSH)
	  asm volatile("nop");
}

void USBInit(USBEnableVBusFunc enableVBus, uint32_t usbIRQLevel)
{
	usb.enableVBusFunc = enableVBus;

	chMBObjectInit(&usb.eventQueue, usb.eventQueueData, USB_EVENT_QUEUE_SIZE);
	usb.lastActiveNPCh = 0;
	usb.lastActivePCh = 0;
	usb.conCallback = 0;

	for(uint8_t ep = 0; ep < USB_MAX_ENDPOINTS; ep++)
	{
		USBEndpoint* pEp = &usb.endpoints[ep];

		chMBObjectInit(&pEp->eventQueue, pEp->eventQueueData, USB_EP_EVENT_QUEUE_SIZE);
		pEp->chNum = ep;
		pEp->pChannel = USB_HC(ep);
		pEp->state = USB_EP_STATE_ACK;
	}

	USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;

	// Wait for AHB master IDLE state
	chThdSleepMilliseconds(1);
	USB_OTG_FS->GRSTCTL = USB_OTG_GRSTCTL_CSRST;	// reset core
	chThdSleepMilliseconds(5);

	USB_OTG_FS->GCCFG = USB_OTG_GCCFG_PWRDWN;

	USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_FHMOD;	// force host mode
	chThdSleepMilliseconds(100);	// change to host mode requires at least 25ms

//	if(USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_CMOD)
//		printf("USB Host mode\r\n");
//	else
//		printf("USB Client mode\r\n");

//	USB_OTG_FS_HOST->HCFG |= USB_OTG_HCFG_FSLSPCS_0;	// 48 MHz PHY clock

	// Restart the Phy Clock
	*USB_OTG_FS_PCGCCTL = 0;

	// no VBUS sensing
	USB_OTG_FS->GCCFG &= ~(USB_OTG_GCCFG_VBDEN);

	USB_OTG_FS_HOST->HCFG &= ~(USB_OTG_HCFG_FSLSS);

	// Make sure the FIFOs are flushed
	USBFlushTxFifo(USB_OTG_FS, 0x10); // all Tx FIFOs
	USBFlushRxFifo(USB_OTG_FS);

	// Clear all pending HC Interrupts
	for(uint32_t ch = 0; ch < 8; ch++)
	{
		USB_HC(ch)->HCINT = 0xFFFFFFFF;
		USB_HC(ch)->HCINTMSK = 0;
	}

	// Enable VBUS driving
	*USB_OTG_FS_HPRT = USB_OTG_HPRT_PPWR;

	chThdSleepMilliseconds(200);

	// Disable all interrupts
	USB_OTG_FS->GINTMSK = 0;

	// Clear any pending interrupts
	USB_OTG_FS->GINTSTS = 0xFFFFFFFF;

	// set Rx FIFO size
	USB_OTG_FS->GRXFSIZ  = (uint32_t )0x80;
	USB_OTG_FS->DIEPTXF0_HNPTXFSIZ = (uint32_t )(((0x60 << 16)& USB_OTG_NPTXFD) | 0x80);
	USB_OTG_FS->HPTXFSIZ = (uint32_t )(((0x40 << 16)& USB_OTG_HPTXFSIZ_PTXFD) | 0xE0);

	// Enable interrupts matching to the Host mode ONLY
	USB_OTG_FS->GINTMSK |= (USB_OTG_GINTMSK_PRTIM | USB_OTG_GINTMSK_HCIM | USB_OTG_GINTSTS_DISCINT);
//	USB_OTG_FS->GINTMSK |= (USB_OTG_GINTMSK_PRTIM | USB_OTG_GINTMSK_HCIM |
//			/*USB_OTG_GINTMSK_SOFM | */USB_OTG_GINTSTS_DISCINT |
//			USB_OTG_GINTMSK_PXFRM_IISOOXFRM  | USB_OTG_GINTMSK_WUIM);

	USB_OTG_FS->GAHBCFG |= USB_OTG_GAHBCFG_GINT;

	// Peripheral interrupt init
	NVICEnableIRQ(OTG_FS_IRQn, usbIRQLevel);
}

USBEndpoint* USBEndpointCreate(USBEndpointCfg* pCfg)
{
	uint8_t ep;

	for(ep = 0; ep < USB_MAX_ENDPOINTS && usb.endpoints[ep].used; ep++);

	if(ep == USB_MAX_ENDPOINTS)
		return 0;

	USBEndpoint* pEP = &usb.endpoints[ep];

	pEP->used = 1;
	chMtxObjectInit(&pEP->irpListMutex);
	pEP->pIRPList = 0;
	pEP->cfg = *pCfg;
	pEP->txErrCnt = 0;
	pEP->state = USB_EP_STATE_ACK;
	pEP->nakCnt = 0;

	pEP->pChannel->HCCHAR = pCfg->maxPktSize;
	if(pCfg->epIn)
		pEP->pChannel->HCCHAR |= USB_OTG_HCCHAR_EPDIR;
	pEP->pChannel->HCCHAR |= (pCfg->type << 18) | (usb.deviceAddress << 22) | (pCfg->epNumber << 11);

	pEP->pChannel->HCINT = 0xFFFF;		// clear all IRQs
	pEP->pChannel->HCINTMSK = 0x7FB;	// enable all IRQs

	USB_OTG_FS_HOST->HAINTMSK |= (1 << ep);	// enable IRQ for channel

	return pEP;
}

void USBEndpointDelete(USBEndpoint* pEP)
{
	if(pEP->used == 0)
		return;

	pEP->pChannel->HCCHAR |= USB_OTG_HCCHAR_CHENA | USB_OTG_HCCHAR_CHDIS;

	chThdSleepMilliseconds(5);
}

void USBChangeDeviceAddress(uint32_t newAddress)
{
	usb.deviceAddress = newAddress;

	for(uint8_t i = 0; i < USB_MAX_ENDPOINTS; i++)
	{
		if(usb.endpoints[i].used == 0)
			continue;

		usb.endpoints[i].pChannel->HCCHAR &= ~(0x7F << 22);
		usb.endpoints[i].pChannel->HCCHAR |= (usb.deviceAddress << 22);
	}
}

USBDescHeader* USBFindDescriptor(uint8_t type, USBDescHeader* pRead, uint8_t* pEnd)
{
	USBDescHeader* pHeader;
	uint8_t* pRead8 = (uint8_t*)pRead;

	while(pRead8 < pEnd)
	{
		pHeader = (USBDescHeader*)pRead8;

		if(pHeader->bDescriptorType != type)
		{
			pRead8 += pHeader->bLength;
			continue;
		}

		return pHeader;
	}

	return 0;
}

int16_t USBIRPSubmit(USBEndpoint* pEP, USBIRP* pIRP)
{
	pIRP->state = USB_IRP_STATE_NEW;
	pIRP->pNext = 0;
	pIRP->numPackets = 1;
	chBSemObjectInit(&pIRP->semDone, 1);

	if(pIRP->dataLength > USB_MAX_TRANSFER_SIZE || pIRP->dataLength >= USB_MAX_NUM_PACKETS*pEP->cfg.maxPktSize)
		return ERROR_USB_IRP_TOO_BIG;

	if(pIRP->dataLength > 0)
	{
		pIRP->numPackets = (pIRP->dataLength + pEP->cfg.maxPktSize - 1) / pEP->cfg.maxPktSize;
	}

	pIRP->packetsDone = 0;
	pIRP->bytesDone = 0;

	// enqueue in endpoint IRP list
	chMtxLock(&pEP->irpListMutex);
	if(pEP->pIRPList == 0)
	{
		pEP->pIRPList = pIRP;
	}
	else
	{
		USBIRP* pEnd;
		for(pEnd = pEP->pIRPList; pEnd->pNext; pEnd = pEnd->pNext);	// go to end of list
		pEnd->pNext = pIRP;
	}
	pIRP->state = USB_IRP_STATE_ENQUEUED;
	chMtxUnlock(&pEP->irpListMutex);

	chMBPostTimeout(&usb.eventQueue, EVENT_BEGIN_IRP | (pEP->chNum << 8), TIME_INFINITE);

	return 0;
}

void USBIRPWait(USBIRP* pIRP)
{
	chBSemWait(&pIRP->semDone);
}

int16_t USBIRPTransfer(USBEndpoint* pEP, USBIRP* pIRP)
{
	int16_t result = USBIRPSubmit(pEP, pIRP);
	if(result == 0)
		USBIRPWait(pIRP);

	return pIRP->state;
}

static void startTx(USBEndpoint* pEP, uint8_t enableCh)
{
	if(pEP->pIRPList == 0)
		return;

//	LogInfoC("startTx", pEP->chNum);

	USBIRP* pIRP = pEP->pIRPList;

	uint32_t pktLeft = pIRP->numPackets-pIRP->packetsDone;

	if(enableCh)
	{
		pEP->pChannel->HCTSIZ &= ~0x1FFFFFFF;

		if(pEP->cfg.epIn)
			pEP->pChannel->HCTSIZ |= (pktLeft*pEP->cfg.maxPktSize) | (pktLeft << 19);
		else
			pEP->pChannel->HCTSIZ |= (pIRP->dataLength-pIRP->bytesDone) | (pktLeft << 19);

		pEP->pChannel->HCCHAR &= ~USB_OTG_HCCHAR_CHDIS;
		pEP->pChannel->HCCHAR |= USB_OTG_HCCHAR_CHENA;
	}

	volatile uint32_t* pTXSTS;
	if(pEP->cfg.type == USB_EPTYPE_BULK || pEP->cfg.type == USB_EPTYPE_CONTROL)
		pTXSTS = &USB_OTG_FS->HNPTXSTS;
	else
		pTXSTS = &USB_OTG_FS_HOST->HPTXSTS;

	if(pIRP->state != USB_IRP_STATE_ACTIVE)
		return;

	if(pIRP->bytesDone >= pIRP->dataLength)
		return;

	if(pEP->cfg.epIn)
		return;

	uint32_t pktLength = pIRP->dataLength - pIRP->bytesDone;
	if(pktLength > pEP->cfg.maxPktSize)
		pktLength = pEP->cfg.maxPktSize;

	uint32_t wordLength = (pktLength + 3) / 4;

	uint32_t hnptxsts = *pTXSTS;
	uint32_t wordsFree = hnptxsts & 0xFFFF;
	uint32_t locsFree = (hnptxsts & 0xFF0000) >> 16;

	if(locsFree == 0 || wordsFree < wordLength)
	{
		LogErrorC("TX full", pEP->chNum);
//		pIRP->state = USB_IRP_STATE_TX_STOPPED;
		return;	// stop processing here, this channel will be checked first next call
	}

	uint32_t* pData = (uint32_t*)(pIRP->pData+pIRP->bytesDone);

	if(((USB_OTG_FS_HOST->HFNUM & USB_OTG_HFNUM_FTREM) >> 16) < 8192)
	{
		pEP->state = USB_EP_STATE_SOF_CONT;
		LogWarnC("DELAYED", pEP->chNum);
		return;
	}

	LogInfoC("HCTSIZ", pEP->pChannel->HCTSIZ);
	LogInfoC("HFIR", USB_OTG_FS_HOST->HFIR);
	LogInfoC("TXSTS", *pTXSTS);
	LogInfoC("HFNUM1", USB_OTG_FS_HOST->HFNUM);

	// write one packet
	for(uint16_t word = 0; word < wordLength; word++)
		USB_DFIFO(pEP->chNum) = pData[word];

	LogInfoC("HFNUM2", USB_OTG_FS_HOST->HFNUM);
	LogInfoC("HCTSIZ", pEP->pChannel->HCTSIZ);
	LogInfoC("TXSTS", *pTXSTS);
	LogInfoC("HCCHAR", pEP->pChannel->HCCHAR);
}

static void abortAllIRPs(USBEndpoint* pEP, USBIRPState state)
{
	chMtxLock(&pEP->irpListMutex);
	for(USBIRP* pIRP = pEP->pIRPList; pIRP; pIRP = pIRP->pNext)
	{
		pIRP->state = state;
		chBSemSignal(&pIRP->semDone);
	}

	pEP->pIRPList = 0;
	chMtxUnlock(&pEP->irpListMutex);

	pEP->state = USB_EP_STATE_NONE;
}

void USBTask(void* params)
{
	(void)params;
	msg_t event;

	chRegSetThreadName("USB-D");

	chThdSleepMilliseconds(1000);

	(*usb.enableVBusFunc)(1);

	while(1)
	{
		if(chMBFetchTimeout(&usb.eventQueue, &event, TIME_INFINITE) != MSG_OK)
			continue;

//		printf("\r\nUSB-IRQ: 0x%08X, %uus\r\n", event, SysTimeUSec());
//		chThdSleepMilliseconds(1);

		if(event & EVENT_CONNECT_DETECT)
		{
			printf("Device connected\r\n");

			chThdSleepMilliseconds(200);

			// start reset sequence
			*USB_OTG_FS_HPRT |= USB_OTG_HPRT_PRST;
			chThdSleepMilliseconds(12);
			*USB_OTG_FS_HPRT &= ~USB_OTG_HPRT_PRST;
		}

		if(event & EVENT_PORT_ENABLED)
		{
			uint32_t speed = ((*USB_OTG_FS_HPRT) & USB_OTG_HPRT_PSPD) >> 17;

			chThdSleepMilliseconds(100);

			if(speed == 2)
			{
				USB_OTG_FS_HOST->HCFG &= ~USB_OTG_HCFG_FSLSPCS;
				USB_OTG_FS_HOST->HCFG |= USB_OTG_HCFG_FSLSPCS_1;
				USB_OTG_FS_HOST->HFIR = 6000;

				chThdSleepMilliseconds(100);
			}
			else
			{
				USB_OTG_FS_HOST->HCFG &= ~USB_OTG_HCFG_FSLSPCS;
				USB_OTG_FS_HOST->HCFG |= USB_OTG_HCFG_FSLSPCS_0;
//				USB_OTG_FS_HOST->HFIR = 48000;
				USB_OTG_FS_HOST->HFIR = 47999;

				chThdSleepMilliseconds(100);
			}

			printf("Port enabled\r\n");

			switch(speed)
			{
				case 1: printf("Full Speed\r\n"); break;
				case 2: printf("Low Speed\r\n"); break;
				default: printf("Unkown Speed\r\n"); break;
			}

			USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM | USB_OTG_GINTMSK_SOFM;	// enable RX IRQ and SOFs

			if(usb.conCallback)
				(*usb.conCallback)(1);

			chThdSleepMilliseconds(200);
		}

		if(event & EVENT_PORT_DISABLED)
		{
			printf("Port disabled\r\n");

			// TODO: debug IRP output
			for(uint8_t ep = 0; ep < USB_MAX_ENDPOINTS; ep++)
			{
				USBEndpoint* pEP = &usb.endpoints[ep];

				if(pEP->used == 0)
					continue;

				printf("\r\nEP%hu IRPs\r\n", (uint16_t)pEP->chNum);
				for(USBIRP* pIRP = pEP->pIRPList; pIRP; pIRP = pIRP->pNext)
				{
					printf("-> %hu", (uint16_t)pIRP->state);
				}
			}

/*
			for(uint8_t ep = 0; ep < USB_MAX_ENDPOINTS; ep++)
			{
				USBEndpoint* pEP = &usb.endpoints[ep];
				if(pEP->used == 0)
					continue;

				pEP->pChannel->HCCHAR |= USB_OTG_HCCHAR_CHENA;
				pEP->pChannel->HCCHAR |= USB_OTG_HCCHAR_CHDIS;
			}*/
		}

		if(event & EVENT_PORT_OVERCURRENT)
		{
			printf("Port overcurrent\r\n");
		}

		if(event & EVENT_DISCONNECT)
		{
			printf("Device disconnected\r\n");

			if(usb.conCallback)
				(*usb.conCallback)(0);

			*USB_OTG_FS_HPRT &= ~(USB_OTG_HPRT_PENA | USB_OTG_HPRT_PCDET | USB_OTG_HPRT_PENCHNG | USB_OTG_HPRT_POCCHNG);

			chThdSleepMilliseconds(10);

			USB_OTG_FS->GINTMSK = 0;

			// Make sure the FIFOs are flushed
			USBFlushTxFifo(USB_OTG_FS, 0x00); // non-periodic Tx FIFOs
			USBFlushTxFifo(USB_OTG_FS, 0x01); // periodic Tx FIFOs
			USBFlushRxFifo(USB_OTG_FS);

//			printf("FIFOs flushed\r\n");

			// Halt all channels to put them into a known state.
			for(uint8_t ep = 0; ep < USB_MAX_ENDPOINTS; ep++)
			{
				USBEndpoint* pEP = &usb.endpoints[ep];

				if(pEP->used == 0)
					continue;

//				printf("HCCHAR: 0x%08X\r\n", pEP->pChannel->HCCHAR);

//				pEP->pChannel->HCCHAR |= USB_OTG_HCCHAR_CHDIS;
				chThdSleepMilliseconds(10);

//				printf("CH %hu flushed\r\n", (uint16_t)ep);

				chThdSleepMilliseconds(10);

//				pEP->pChannel->HCCHAR |= USB_OTG_HCCHAR_CHDIS | USB_OTG_HCCHAR_CHENA;
				chThdSleepMilliseconds(10);

//				printf("HCCHAR: 0x%08X\r\n", pEP->pChannel->HCCHAR);

				pEP->pChannel->HCTSIZ = 0;
				pEP->pChannel->HCCHAR = 0;
				pEP->pChannel->HCINTMSK = 0;
				pEP->pChannel->HCINT = 0xFFFF;

				USB_OTG_FS_HOST->HAINTMSK &= ~(1 << pEP->chNum);	// disable IRQ for channel

				abortAllIRPs(pEP, USB_IRP_STATE_ABORTED);

				pEP->used = 0;

				chMBReset(&pEP->eventQueue);
				chMBResumeX(&pEP->eventQueue);

				printf("CH %hu halted\r\n", (uint16_t)ep);
			}

			chThdSleepMilliseconds(50);

			// Clear any pending Host interrupts
			USB_OTG_FS_HOST->HAINT = 0xFFFFFFFF;
			USB_OTG_FS->GINTSTS = 0xFFFFFFFF;

			chMBReset(&usb.eventQueue);
			chMBResumeX(&usb.eventQueue);

			event = 0;

			usb.deviceAddress = 0;

			USB_OTG_FS->GINTMSK = (USB_OTG_GINTMSK_PRTIM | USB_OTG_GINTMSK_HCIM | USB_OTG_GINTSTS_DISCINT);
		}

		if(event & EVENT_RX)
		{
			eventRx();
		}

		if(event & EVENT_CHANNEL_IRQ)
		{
			uint32_t ch = (event & 0xF00) >> 8;

			eventChIRQ(ch);
		}

		if(event & EVENT_SOF)
		{
			LogInfo("SOF");
			for(uint8_t ep = 0; ep < USB_MAX_ENDPOINTS; ep++)
			{
				USBEndpoint* pEP = &usb.endpoints[ep];

				switch(pEP->state)
				{
					case USB_EP_STATE_SOF_RESTART:
					{
						pEP->state = USB_EP_STATE_NONE;
						startTx(pEP, 1);
						LogInfoC("SOF_REST->NONE", pEP->chNum);
					}
					break;
					case USB_EP_STATE_SOF_CONT:
					{
						pEP->state = USB_EP_STATE_NONE;
						startTx(pEP, 0);
						LogInfoC("SOF_CONT->ACK", pEP->chNum);
					}
					break;
					case USB_EP_STATE_ACK:
					{
//						if(pEP->used && pEP->cfg.epIn == 0)
//							pEP->state = USB_EP_STATE_SOF_CONT;
					}
					break;
					default:
						break;
				}
			}
		}

		if(event & EVENT_BEGIN_IRP)
		{
			uint32_t ch = (event & 0xF00) >> 8;

			USBEndpoint* pEP = &usb.endpoints[ch];

			USBIRP* pIRP = pEP->pIRPList;

			if(pIRP && pIRP->state == USB_IRP_STATE_ENQUEUED)
			{
				uint32_t pid;
				if(pIRP->pid == USB_PID_HOST_CONTROLLED)
					pid = (pEP->pChannel->HCTSIZ & USB_OTG_HCTSIZ_DPID);
				else
					pid = pIRP->pid << 29;

				pEP->nakCnt = 0;
				pEP->pChannel->HCTSIZ = pid;
				pIRP->state = USB_IRP_STATE_ACTIVE;
				LogInfoC("Begin IRP", pEP->chNum);
				startTx(pEP, 1);
			}
		}
	}
}

static void eventRx()
{
	if((USB_OTG_FS->GINTSTS & USB_OTG_GINTSTS_RXFLVL) == 0)
		return;

	uint32_t grxstsp = USB_OTG_FS->GRXSTSP;
	uint32_t bytes = (grxstsp & USB_OTG_GRXSTSP_BCNT) >> 4;
	uint32_t ch = grxstsp & USB_OTG_GRXSTSP_EPNUM;
	uint32_t pktsts = (grxstsp & USB_OTG_GRXSTSP_PKTSTS) >> 17;

	LogInfoC("ERX", ch);

//	printf("RX. CH: %u, Bytes: %u, pktsts: %02X\r\n", ch, bytes, pktsts);

	if(pktsts != 0x02)
	{
		USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
		return;
	}

	USBEndpoint* pEP = &usb.endpoints[ch];

	chMtxLock(&pEP->irpListMutex);
	USBIRP* pIRP = pEP->pIRPList;
	chMtxUnlock(&pEP->irpListMutex);

	if(pIRP == 0)
	{
		uint32_t dummy;
		for(uint32_t i = 0; i < (bytes+3)/4; i++)
			dummy = USB_DFIFO(ch);

		(void)dummy;

		printf("No IRP pending for CH %u!\r\n", ch);
	}
	else
	{
		if(pIRP->packetsDone >= pIRP->numPackets)
			return;

		LogInfoC("RX", pEP->chNum);

		uint32_t* pRx = (uint32_t*)(pIRP->pData+pIRP->bytesDone);
		for(uint32_t i = 0; i < (bytes+3)/4; i++)
			pRx[i] = USB_DFIFO(ch);

		pIRP->bytesDone += bytes;
		++pIRP->packetsDone;

//		printf("IRP: %hu/%hu, HCTSIZ: 0x%08X\r\n", pIRP->packetsDone, pIRP->numPackets, pEP->pChannel->HCTSIZ);

		if(pEP->pChannel->HCTSIZ & USB_OTG_HCTSIZ_PKTCNT)
		{
			pEP->pChannel->HCCHAR &= ~USB_OTG_HCCHAR_CHDIS;
			pEP->pChannel->HCCHAR |= USB_OTG_HCCHAR_CHENA;
		}
	}

	USB_OTG_FS->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
}

static void eventChIRQ(uint32_t ch)
{
	msg_t chEvent;

	USBEndpoint* pEP = &usb.endpoints[ch];

	if(chMBFetchTimeout(&pEP->eventQueue, &chEvent, TIME_IMMEDIATE) != MSG_OK)
		return;

	LogInfoC("CH-IRQ", chEvent);

	if(chEvent & USB_OTG_HCINT_CHH)
	{
		LogInfoC("CHH", pEP->chNum);

		switch(pEP->state)
		{
			case USB_EP_STATE_NAK:
			{
				if(pEP->nakCnt > pEP->cfg.timeout && pEP->cfg.timeout > 0)
					abortAllIRPs(pEP, USB_IRP_STATE_TIMEOUT);
				else
					pEP->state = USB_EP_STATE_SOF_RESTART;
			}
			break;
			case USB_EP_STATE_TXERR:
			{
				if(pEP->txErrCnt < 3)
					pEP->state = USB_EP_STATE_SOF_RESTART;
				else
					abortAllIRPs(pEP, USB_IRP_STATE_TXERR);
			}
			break;
			case USB_EP_STATE_STALL:
			{
				abortAllIRPs(pEP, USB_IRP_STATE_STALL);
			}
			break;
			default:
			{
				LogInfoC("CHH???", pEP->state);
				abortAllIRPs(pEP, USB_IRP_STATE_ABORTED);

				printf("Halt CH%hu, state: %u\r\n", (uint16_t)pEP->chNum, (uint32_t)pEP->state);

				pEP->pChannel->HCINTMSK = 0;	// disable all IRQs
				pEP->pChannel->HCINT = 0xFFFF;
				USB_OTG_FS_HOST->HAINTMSK &= ~(1 << pEP->chNum);	// disable IRQ for channel

				pEP->pIRPList = 0;
				pEP->used = 0;
			}
			break;
		}
	}

	if(chEvent & USB_OTG_HCINT_STALL)
	{
		printf("EP%hu stalled\r\n", (uint16_t)pEP->chNum);

		pEP->state = USB_EP_STATE_STALL;
		pEP->pChannel->HCCHAR |= USB_OTG_HCCHAR_CHDIS;
	}

	if(chEvent & USB_OTG_HCINT_DTERR)
	{
		printf("EP%hu data toggle error\r\n", (uint16_t)pEP->chNum);

		pEP->state = USB_EP_STATE_DTERR;
		pEP->pChannel->HCCHAR |= USB_OTG_HCCHAR_CHDIS;
	}

	if(chEvent & USB_OTG_HCINT_FRMOR)
	{
		printf("EP%hu frame overrun\r\n", (uint16_t)pEP->chNum);
	}

	if(chEvent & USB_OTG_HCINT_BBERR)
	{
		printf("EP%hu babble error\r\n", (uint16_t)pEP->chNum);

		pEP->state = USB_EP_STATE_BBERR;
		pEP->pChannel->HCCHAR |= USB_OTG_HCCHAR_CHDIS;
	}

	if(chEvent & USB_OTG_HCINT_TXERR)
	{
		pEP->state = USB_EP_STATE_TXERR;
		++pEP->txErrCnt;
		pEP->pChannel->HCCHAR |= USB_OTG_HCCHAR_CHDIS;
	}

	if(chEvent & USB_OTG_HCINT_NAK)
	{
		LogInfoC("NAK", pEP->chNum);
		if(pEP->state != USB_EP_STATE_NAK)
		{
			pEP->state = USB_EP_STATE_NAK;
			++pEP->nakCnt;
			pEP->pChannel->HCCHAR |= USB_OTG_HCCHAR_CHDIS;
		}
	}

	if(chEvent & USB_OTG_HCINT_ACK)
	{
		LogInfoC("ACK", pEP->chNum);
		pEP->state = USB_EP_STATE_ACK;

		if(pEP->cfg.epIn == 0)
		{
			chMtxLock(&pEP->irpListMutex);
			if(pEP->pIRPList)
			{
				++pEP->pIRPList->packetsDone;
				pEP->pIRPList->bytesDone += pEP->cfg.maxPktSize;
			}
			chMtxUnlock(&pEP->irpListMutex);

			startTx(pEP, 0);
		}
	}

	if(chEvent & USB_OTG_HCINT_XFRC)
	{
		LogInfoC("XFRC", pEP->chNum);
		pEP->state = USB_EP_STATE_XFRC;
		USBIRP* pIRP = pEP->pIRPList;
		if(pIRP == 0)
			return;

		LogErrorC("XFRC", pEP->pChannel->HCCHAR);

		// IRP complete!
		pIRP->state = USB_IRP_STATE_DONE;
		pEP->txErrCnt = 0;
		chBSemSignal(&pIRP->semDone);

		// advance IRP list
		chMtxLock(&pEP->irpListMutex);
		pEP->pIRPList = pIRP->pNext;
		chMtxUnlock(&pEP->irpListMutex);

		// start a new IRP
		chMBPostTimeout(&usb.eventQueue, EVENT_BEGIN_IRP | (pEP->chNum << 8), TIME_INFINITE);
	}
}
