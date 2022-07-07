/*
 * usb_msc.h
 *
 *  Created on: 10.08.2014
 *      Author: AndreR
 */

#pragma once

#include "usb_hcd.h"
#include "../fatfs/ff.h"

#define USB_MSC_PROC_BUF_SIZE	256

#define USB_MSC_CBW_SIZE 31
#define USB_MSC_CSW_SIZE 13

#define SCSI_SENSE_KEY_NO_SENSE			0
#define SCSI_SENSE_KEY_RECOVERED_ERROR	1
#define SCSI_SENSE_KEY_NOT_READY		2
#define SCSI_SENSE_KEY_MEDIUM_ERROR		3
#define SCSI_SENSE_KEY_HARDWARE_ERROR	4
#define SCSI_SENSE_KEY_ILLEGAL_REQUEST	5
#define SCSI_SENSE_KEY_UNIT_ATTENTION	6
#define SCSI_SENSE_KEY_DATA_PROTECT		7
#define SCSI_SENSE_KEY_BLANK_CHECK		8
#define SCSI_SENSE_KEY_VENDOR_SPECIFIC	9
#define SCSI_SENSE_KEY_COPY_ABORTED		10
#define SCSI_SENSE_KEY_ABORTED_COMMAND	11
#define SCSI_SENSE_KEY_VOLUME_OVERFLOW	13
#define SCSI_SENSE_KEY_MISCOMPARE		14

#define SCSI_RESPONSE_CODE_CURRENT_FIXED	0x70
#define SCSI_RESPONSE_CODE_DEFERRED_FIXED	0x71

typedef struct __attribute__((packed)) _SCSIRequestSenseResult
{
	uint8_t responseCode;
	uint8_t sense;
	uint8_t ASC;
	uint8_t ASCQ;
} SCSIRequestSenseResult;

typedef struct __attribute__((packed)) _USBMSCCBW
{
	uint32_t signature;
	uint32_t tag;
	uint32_t dataTransferLength;
	uint8_t flags;
	uint8_t LUN;
	uint8_t commandBlockLength;
	uint8_t commandBlock[16];
	uint8_t _pad[1];
} USBMSCCBW;

typedef struct __attribute__((packed)) _USBMSCCSW
{
	uint32_t signature;
	uint32_t tag;
	uint32_t dataResidue;
	uint8_t status;
	uint8_t _pad[3];
} USBMSCCSW;

typedef void(*USBMSCCb)(USBMSCEvent*);

typedef struct _USBMSCGlobal
{
	USBHCDClass class;

	USBEndpoint* pEPIn;
	USBEndpoint* pEPOut;

	uint8_t interfaceNumber;
	uint8_t numLogicalUnits;

	USBIRP irpCbw;
	USBIRP irpCsw;
	USBIRP irpData;
	USBMSCCBW cbw;
	USBMSCCSW csw;

	uint8_t procBuf[USB_MSC_PROC_BUF_SIZE];

	uint32_t numBlocks;
	uint32_t blockSize;

	mutex_t scsiMtx;

	FATFS fatFs;

	uint8_t errored;

	USBMSCCb eventCb;
	USBMSCEvent event;
} USBMSCGlobal;

extern USBMSCGlobal usbMsc;

void	USBMSCInit();

int16_t	UsbMscScsiTestUnitReady();
int16_t UsbMscScsiInquiry();
int16_t UsbMscScsiRequestSense(SCSIRequestSenseResult* pResult);
int16_t UsbMscScsiReadCapacity();
int16_t UsbMscScsiRead(uint32_t lba, uint16_t numBlocks, uint8_t* pData);
int16_t UsbMscScsiWrite(uint32_t lba, uint16_t numBlocks, uint8_t* pData);

void	UsbMscScsiWriteBuf(uint32_t lba, uint16_t numBlocks, uint8_t* pData);
void	UsbMscScsiReadBuf(uint32_t lba, uint16_t numBlocks, uint8_t* pData);
void	UsbMscScsiWriteFlush();
