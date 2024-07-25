#include "usb_msc.h"
#include "util/log.h"
#include <string.h>
#include <stdio.h>

USBMSCGlobal usbMsc;

static USBSetupPkt setupMSCReset = {
		.bmRequestType = USB_H2D | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_INTERFACE,
		.bRequest = 0xFF,
		.wValue = 0,
		.wIndex = 1,
		.wLength = 0,
};

static USBSetupPkt setupGetMaxLUN = {
		.bmRequestType = USB_D2H | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_INTERFACE,
		.bRequest = 0xFE,
		.wValue = 0,
		.wIndex = 1,
		.wLength = 1,
};

static int16_t mscReset()
{
	setupMSCReset.wIndex = usbMsc.interfaceNumber;

	return USBCtrlWrite(&setupMSCReset, 0);
}

static int16_t mscGetMaxLUN(uint8_t* pMaxLUN)
{
	setupGetMaxLUN.wIndex = usbMsc.interfaceNumber;

	return USBCtrlRead(&setupGetMaxLUN, pMaxLUN, 0);
}

static int16_t botTransfer(uint8_t isInTransfer, uint8_t* pData, uint32_t dataLength, uint32_t* pBytesRead)
{
	usbMsc.cbw.dataTransferLength = dataLength;
	if(isInTransfer)
		usbMsc.cbw.flags = 0x80;	// direction IN (device to host)
	else
		usbMsc.cbw.flags = 0;

	usbMsc.irpData.pData = pData;
	usbMsc.irpData.dataLength = dataLength;

	int16_t result = USBIRPTransfer(usbMsc.pEPOut, &usbMsc.irpCbw);
	if(result)
	{
		printf("Transfer CBW failed: 0x%04X\r\n", result);
		return result;
	}

	if(isInTransfer)
	{
		result = USBIRPTransfer(usbMsc.pEPIn, &usbMsc.irpData);
	}
	else
	{
		if(dataLength > 0)
			result = USBIRPTransfer(usbMsc.pEPOut, &usbMsc.irpData);
	}
	if(result)
	{
		printf("Transfer data failed: 0x%04X\r\n", result);
		return result;
	}

	result = USBIRPTransfer(usbMsc.pEPIn, &usbMsc.irpCsw);
	if(result)
	{
		printf("Transfer CSW failed: 0x%04X\r\n", result);
		return result;
	}

	if(pBytesRead)
		*pBytesRead = usbMsc.irpData.bytesDone;

	if(usbMsc.csw.signature != 0x53425355)
		return 1;

	if(usbMsc.csw.tag != usbMsc.cbw.tag)
		return 2;

	if(usbMsc.csw.status != 0x00)
	{
		return 3;
	}

	return 0;
}

static int16_t botRead(uint8_t* pData, uint32_t dataLength, uint32_t* pBytesRead)
{
	return botTransfer(1, pData, dataLength, pBytesRead);
}

static int16_t botWrite(uint8_t* pData, uint32_t dataLength)
{
	return botTransfer(0, pData, dataLength, 0);
}

int16_t UsbMscScsiTestUnitReady()
{
	chMtxLock(&usbMsc.scsiMtx);

	usbMsc.cbw.commandBlockLength = 10;
	memset(usbMsc.cbw.commandBlock, 0, 16);

	int16_t result = botWrite(0, 0);
	chMtxUnlock(&usbMsc.scsiMtx);

	return result;
}

int16_t UsbMscScsiInquiry()
{
	chMtxLock(&usbMsc.scsiMtx);

	usbMsc.cbw.commandBlockLength = 6;
	usbMsc.cbw.commandBlock[0] = 0x12;
	usbMsc.cbw.commandBlock[1] = 0;
	usbMsc.cbw.commandBlock[2] = 0;
	usbMsc.cbw.commandBlock[3] = 0;
	usbMsc.cbw.commandBlock[4] = 36;
	usbMsc.cbw.commandBlock[5] = 0;

	int16_t result = botRead(usbMsc.procBuf, 36, 0);
	chMtxUnlock(&usbMsc.scsiMtx);

	return result;
}

int16_t UsbMscScsiRequestSense(SCSIRequestSenseResult* pResult)
{
	chMtxLock(&usbMsc.scsiMtx);

	// issue a fixed-format request sense
	usbMsc.cbw.commandBlockLength = 6;
	usbMsc.cbw.commandBlock[0] = 0x03;
	usbMsc.cbw.commandBlock[1] = 0;
	usbMsc.cbw.commandBlock[2] = 0;
	usbMsc.cbw.commandBlock[3] = 0;
	usbMsc.cbw.commandBlock[4] = 18;
	usbMsc.cbw.commandBlock[5] = 0;

	int16_t result = botRead(usbMsc.procBuf, 252, 0);

	pResult->responseCode = usbMsc.procBuf[0] & 0x7F;
	pResult->sense = usbMsc.procBuf[2] & 0x0F;
	pResult->ASC = usbMsc.procBuf[12];
	pResult->ASCQ = usbMsc.procBuf[13];

	chMtxUnlock(&usbMsc.scsiMtx);

	return result;
}

int16_t UsbMscScsiReadCapacity()
{
	chMtxLock(&usbMsc.scsiMtx);

	usbMsc.cbw.commandBlockLength = 10;
	usbMsc.cbw.commandBlock[0] = 0x25;	// op code
	usbMsc.cbw.commandBlock[1] = 0;		// reserved
	usbMsc.cbw.commandBlock[2] = 0;		// LBA
	usbMsc.cbw.commandBlock[3] = 0;
	usbMsc.cbw.commandBlock[4] = 0;
	usbMsc.cbw.commandBlock[5] = 0;
	usbMsc.cbw.commandBlock[6] = 0;		// reserved
	usbMsc.cbw.commandBlock[7] = 0;
	usbMsc.cbw.commandBlock[8] = 0;		// PMI
	usbMsc.cbw.commandBlock[9] = 0;		// control

	int16_t result = botRead(usbMsc.procBuf, 8, 0);
	if(result)
	{
		chMtxUnlock(&usbMsc.scsiMtx);
		return result;
	}

	usbMsc.numBlocks = usbMsc.procBuf[3] | (usbMsc.procBuf[2] << 8) | (usbMsc.procBuf[1] << 16) | (usbMsc.procBuf[0] << 24);
	usbMsc.blockSize = usbMsc.procBuf[7] | (usbMsc.procBuf[6] << 8) | (usbMsc.procBuf[5] << 16) | (usbMsc.procBuf[4] << 24);

	chMtxUnlock(&usbMsc.scsiMtx);

	return 0;
}

/**
 * @param lba: logical block address
 * @param numBlocks: number of blocks to read
 * @param pData: read target buffer
 */
int16_t UsbMscScsiRead(uint32_t lba, uint16_t numBlocks, uint8_t* pData)
{
	if(numBlocks == 0)
		return 0;

	chMtxLock(&usbMsc.scsiMtx);

	usbMsc.cbw.commandBlockLength = 10;
	usbMsc.cbw.commandBlock[0] = 0x28;	// op code
	usbMsc.cbw.commandBlock[1] = 0;		// options
	usbMsc.cbw.commandBlock[2] = ((uint8_t*)&lba)[3];		// LBA
	usbMsc.cbw.commandBlock[3] = ((uint8_t*)&lba)[2];
	usbMsc.cbw.commandBlock[4] = ((uint8_t*)&lba)[1];
	usbMsc.cbw.commandBlock[5] = ((uint8_t*)&lba)[0];
	usbMsc.cbw.commandBlock[6] = 0;		// group number
	usbMsc.cbw.commandBlock[7] = ((uint8_t*)&numBlocks)[1];		// transfer length, number of blocks
	usbMsc.cbw.commandBlock[8] = ((uint8_t*)&numBlocks)[0];
	usbMsc.cbw.commandBlock[9] = 0;		// control

	int16_t result = botRead(pData, numBlocks*usbMsc.blockSize, 0);
	chMtxUnlock(&usbMsc.scsiMtx);

	return result;
}

int16_t UsbMscScsiWrite(uint32_t lba, uint16_t numBlocks, uint8_t* pData)
{
	chMtxLock(&usbMsc.scsiMtx);

	usbMsc.cbw.commandBlockLength = 10;
	usbMsc.cbw.commandBlock[0] = 0x2A;	// op code
	usbMsc.cbw.commandBlock[1] = 0;		// options
	usbMsc.cbw.commandBlock[2] = ((uint8_t*)&lba)[3];		// LBA
	usbMsc.cbw.commandBlock[3] = ((uint8_t*)&lba)[2];
	usbMsc.cbw.commandBlock[4] = ((uint8_t*)&lba)[1];
	usbMsc.cbw.commandBlock[5] = ((uint8_t*)&lba)[0];
	usbMsc.cbw.commandBlock[6] = 0;		// group number
	usbMsc.cbw.commandBlock[7] = ((uint8_t*)&numBlocks)[1];		// transfer length, number of blocks
	usbMsc.cbw.commandBlock[8] = ((uint8_t*)&numBlocks)[0];
	usbMsc.cbw.commandBlock[9] = 0;		// control

	int16_t result = botWrite(pData, numBlocks*usbMsc.blockSize);
	chMtxUnlock(&usbMsc.scsiMtx);

	return result;
}

static void init(USBDeviceDesc* pDeviceDesc, uint8_t* pConfigDesc, uint16_t cfgDescSize)
{
	(void)pDeviceDesc;

	uint8_t* pConfigEnd = pConfigDesc+cfgDescSize;
	USBDescHeader* pRead = (USBDescHeader*)pConfigDesc;

	pRead = USBFindDescriptor(USB_DESC_TYPE_INTERFACE, pRead, pConfigEnd);
	USBInterfaceDesc* pInterfaceDesc = (USBInterfaceDesc*)pRead;
	if(pInterfaceDesc == 0)
		return;

	// Is this SCSI transparent command set?
	if(pInterfaceDesc->bInterfaceSubClass != 0x06)
	{
		printf("Unsupported command set\r\n");
		return;
	}

	// Is this Bulk-only transport protocol?
	if(pInterfaceDesc->bInterfaceProtocol != 0x50)
	{
		printf("Unsupported protocol\r\n");
	}

	usbMsc.interfaceNumber = pInterfaceDesc->bInterfaceNumber;

	// endpoint configuration
	USBEndpointDesc* pBulkIn = 0;
	USBEndpointDesc* pBulkOut = 0;

	pRead = USBFindDescriptor(USB_DESC_TYPE_ENDPOINT, pRead, pConfigEnd);
	USBEndpointDesc* pBulkEP = (USBEndpointDesc*)pRead;
	if(pBulkEP == 0)
		return;

	if(pBulkEP->bEndpointAddress & 0x80)
		pBulkIn = pBulkEP;
	else
		pBulkOut = pBulkEP;

	USB_SKIP_DESCRIPTOR(pRead);

	pRead = USBFindDescriptor(USB_DESC_TYPE_ENDPOINT, pRead, pConfigEnd);
	pBulkEP = (USBEndpointDesc*)pRead;
	if(pBulkEP == 0)
		return;

	if(pBulkEP->bEndpointAddress & 0x80)
		pBulkIn = pBulkEP;
	else
		pBulkOut = pBulkEP;

	if(pBulkIn == 0 || pBulkOut == 0)
	{
		printf("Missing bulk endpoint\r\n");
		return;
	}

	USBEndpointCfg epCfgIn;
	epCfgIn.epIn = 1;
	epCfgIn.epNumber = pBulkIn->bEndpointAddress & 0x0F;
	epCfgIn.maxPktSize = pBulkIn->wMaxPacketSize;
	epCfgIn.type = USB_EPTYPE_BULK;
	epCfgIn.timeout = 5000;

	USBEndpointCfg epCfgOut;
	epCfgOut.epIn = 0;
	epCfgOut.epNumber = pBulkOut->bEndpointAddress & 0x0F;
	epCfgOut.maxPktSize = pBulkOut->wMaxPacketSize;
	epCfgOut.type = USB_EPTYPE_BULK;
	epCfgOut.timeout = 5000;

	usbMsc.pEPIn = USBEndpointCreate(&epCfgIn);
	if(usbMsc.pEPIn == 0)
	{
		printf("Creating IN endpoint failed\r\n");
		return;
	}

	usbMsc.pEPOut = USBEndpointCreate(&epCfgOut);
	if(usbMsc.pEPOut == 0)
	{
		printf("Creating OUT endpoint failed\r\n");
		return;
	}

	printf("Issuing MSC reset...");
	int16_t result = mscReset();
	if(result)
		printf("failed\r\n");
	else
		printf("OK\r\n");

	uint32_t LUNs;
	result = mscGetMaxLUN((uint8_t*)&LUNs);
	usbMsc.numLogicalUnits = (LUNs  & 0xFF);
	if(result)
		printf("GetMaxLUN failed\r\n");
	else
		printf("Max LUNs: %u\r\n", usbMsc.numLogicalUnits);

//	chThdSleepMilliseconds(1000);

	result = UsbMscScsiInquiry();
	if(result)
	{
		printf("Inquiry failed: 0x%04X\r\n", result);
		return;
	}

//	printf("--- INQ ---\r\nDevice Type: 0x%02hX\r\nPeriphQual: 0x%02hX\r\n", (uint16_t)(usbMsc.procBuf[0] & 0x1F), (uint16_t)(usbMsc.procBuf[0] >> 5));
//	if(usbMsc.procBuf[1] & 0x80)
//		printf("Removable\r\n");
//	else
//		printf("NOT Removable\r\n");

	SCSIRequestSenseResult requestSense;
	result = UsbMscScsiRequestSense(&requestSense);
	if(result)
	{
		printf("RequestSense failed: 0x%04X\r\n", result);
		return;
	}

//	printf("Request-Sense\r\nCode: 0x%02hX\r\nKey: %hX\r\nASC: 0x%02hX\r\nASCQ: 0x%02hX\r\n",
//			(uint16_t)requestSense.responseCode, (uint16_t)requestSense.sense,
//			(uint16_t)requestSense.ASC, (uint16_t)requestSense.ASCQ);

	for(uint8_t i = 0; i < 10; i++)
	{
		result = UsbMscScsiTestUnitReady();
		if(result == 0)
			break;

		chThdSleepMilliseconds(10);
	}

	result = UsbMscScsiReadCapacity();
	if(result)
	{
		printf("ReadCapacity failed: 0x%04X\r\n", result);
		return;
	}

	printf("Disk capacity: %uMB\r\nBlock Size: %u\r\n", usbMsc.numBlocks/((1024*1024)/usbMsc.blockSize), usbMsc.blockSize);

	FRESULT fresult = f_mount(&usbMsc.fatFs, "USB:", 1);
	if(fresult != FR_OK)
	{
		printf("FatFS mount error: %u\r\n", (uint32_t)fresult);
		return;
	}

	// Get volume information and free clusters of drive 1
	uint32_t freeCluster;
	FATFS* fs;
	fresult = f_getfree("USB:", &freeCluster, &fs);
	if(fresult)
		return;

	// Get total sectors and free sectors
	uint32_t totalSectors = (fs->n_fatent - 2) * fs->csize;
	uint32_t freeSectors = freeCluster * fs->csize;

	uint32_t totalSizeMB = totalSectors/((1024*1024)/usbMsc.blockSize);
	uint32_t freeSizeMB = freeSectors/((1024*1024)/usbMsc.blockSize);

	printf("--- FatFS ---\r\nSize: %uMB\r\nFree: %uMB\r\n", totalSizeMB, freeSizeMB);
	printf("Cluster size: %ukB\r\n", (fs->csize*usbMsc.blockSize)/1024);

	usbMsc.event.connect = 1;
	usbMsc.event.fatSize = totalSizeMB;
	usbMsc.event.freeSize = freeSizeMB;
	usbMsc.event.clusterSize = (fs->csize*usbMsc.blockSize)/1024;

	if(usbMsc.eventCb)
		(*usbMsc.eventCb)(&usbMsc.event);
}

static void stop()
{
	usbMsc.event.connect = 0;
	usbMsc.event.fatSize = 0;
	usbMsc.event.freeSize = 0;
	usbMsc.event.clusterSize = 0;

	if(usbMsc.eventCb)
		(*usbMsc.eventCb)(&usbMsc.event);

	FRESULT fresult = f_mount(0, "USB:", 1);
	if(fresult != FR_OK)
	{
		printf("FatFS umount error: %u\r\n", (uint32_t)fresult);
		return;
	}

	usbMsc.errored = 0;
}

static void process(msg_t event)
{
	(void)event;
}

void USBMSCInit()
{
	chMtxObjectInit(&usbMsc.scsiMtx);

	usbMsc.cbw.signature = 0x43425355;
	usbMsc.cbw.tag = 0x42434445;
	usbMsc.cbw.dataTransferLength = 0;
	usbMsc.cbw.flags = 0;
	usbMsc.cbw.LUN = 0;
	usbMsc.cbw.commandBlockLength = 0;

	usbMsc.irpCbw.pData = (uint8_t*)&usbMsc.cbw;
	usbMsc.irpCbw.dataLength = USB_MSC_CBW_SIZE;
	usbMsc.irpCbw.pid = USB_PID_HOST_CONTROLLED;

	usbMsc.irpCsw.pData = (uint8_t*)&usbMsc.csw;
	usbMsc.irpCsw.dataLength = USB_MSC_CSW_SIZE;
	usbMsc.irpCsw.pid = USB_PID_HOST_CONTROLLED;

	usbMsc.irpData.pData = 0;
	usbMsc.irpData.dataLength = 0;
	usbMsc.irpData.pid = USB_PID_HOST_CONTROLLED;

	usbMsc.class.isDeviceClass = 0;
	usbMsc.class.classCode = 0x08;
	usbMsc.class.pName = "Mass Storage";
	usbMsc.class.Init = &init;
	usbMsc.class.Stop = &stop;
	usbMsc.class.process = &process;

	USBHCDAddClass(&usbMsc.class);
}
