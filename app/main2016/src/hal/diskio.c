/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2014        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "fatfs/diskio.h"		/* FatFs lower layer API */
#include "../sdcard.h"
#include "../usb/usb_msc.h"
#include "util/console.h"
#include "util/log.h"
#include <string.h>

/* Definitions of physical drive number for each drive */
#define SD		1
#define USB		0

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	DRESULT res;
	SCSIRequestSenseResult reqSense;

	switch (pdrv)
	{
		case SD:
		{
			if(sdCard.ready)
			{
				if(sdCard.writeProtected)
					return STA_PROTECT;
				else
					return 0;
			}
			else
			{
				return STA_NOINIT;
			}
		}
		case USB:
		{
			if(UsbMscScsiRequestSense(&reqSense) != 0)
			{
				return STA_NOINIT;
			}
			else
			{
				if(reqSense.sense != SCSI_SENSE_KEY_NO_SENSE)
					return STA_NOINIT;
			}

			if(UsbMscScsiTestUnitReady() == 0)
			{
				res = 0;
			}
			else
			{
				res = STA_NOINIT;
			}

			return res;
		}
	}

	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	switch (pdrv)
	{
		case SD:
		{
			if(sdCard.ready)
			{
				if(sdCard.writeProtected)
					return STA_PROTECT;
				else
					return 0;
			}
			else
			{
				return STA_NOINIT;
			}
		}
		case USB:
		{
			return 0;
		}
	}

	return STA_NOINIT;
}

#define DMA_NUM_BLOCKS 16
#define SD_BLOCK_SIZE 512
#define DMA_BUF_SIZE (DMA_NUM_BLOCKS*SD_BLOCK_SIZE)
static uint8_t dmaBuf[DMA_BUF_SIZE] __attribute__((section(".dtc")));

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address in LBA */
	UINT count		/* Number of sectors to read */
)
{
	switch (pdrv)
	{
		case SD:
		{
			if(!sdCard.ready)
				return STA_NOINIT;

			// copy & split is required as buff might be unaligned
			while(count > 0)
			{
				UINT toRead = count;
				if(toRead > DMA_NUM_BLOCKS)
					toRead = DMA_NUM_BLOCKS;

				if(SDCardRead(sector, dmaBuf, toRead) != 0)
				{
					LogError("SDRead error");
					return RES_ERROR;
				}

				memcpy(buff, dmaBuf, toRead*SD_BLOCK_SIZE);

				count -= toRead;
				sector += toRead;
				buff += toRead*SD_BLOCK_SIZE;
			}

			return RES_OK;
		}
		case USB:
		{
			int16_t result = UsbMscScsiRead(sector, count, buff);
			if(result)
				return RES_ERROR;

			return RES_OK;
		}
	}

	return RES_PARERR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE
DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address in LBA */
	UINT count			/* Number of sectors to write */
)
{
	switch (pdrv)
	{
		case SD:
		{
			if(!sdCard.ready)
				return STA_NOINIT;

			// copy & split is required as buff might be unaligned
			while(count > 0)
			{
				UINT toWrite = count;
				if(toWrite > DMA_NUM_BLOCKS)
					toWrite = DMA_NUM_BLOCKS;

				memcpy(dmaBuf, buff, toWrite*SD_BLOCK_SIZE);

				if(SDCardWrite(sector, dmaBuf, toWrite) != 0)
				{
					LogError("SDWrite error");
					return RES_ERROR;
				}

				count -= toWrite;
				sector += toWrite;
				buff += toWrite*SD_BLOCK_SIZE;
			}

			return RES_OK;
		}
		case USB:
		{
			int16_t result = UsbMscScsiWrite(sector, count, (uint8_t*) buff);
			if(result)
				return RES_ERROR;

			return RES_OK;
		}
	}

	return RES_PARERR;
}
#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	switch (pdrv)
	{
		case SD:
		{
			if(!sdCard.ready)
				return STA_NOINIT;

			switch(cmd)
			{
				case CTRL_SYNC:
				{
					SDCardWaitUntilReady();
					return RES_OK;
				}
				case GET_SECTOR_COUNT:
					*((uint32_t*) buff) = sdCard.csd.capacity*2;
					return RES_OK;
				case GET_SECTOR_SIZE:
					*((uint32_t*) buff) = 512;
					return RES_OK;
				case GET_BLOCK_SIZE:
					*((uint32_t*) buff) = 512;
					return RES_OK;
				default:
					return RES_PARERR;
			}

			return RES_OK;
		}
		case USB:
		{
			switch(cmd)
			{
				case CTRL_SYNC:
					return RES_OK;
				case GET_SECTOR_COUNT:
					*((uint32_t*) buff) = usbMsc.numBlocks;
					return RES_OK;
				case GET_SECTOR_SIZE:
					*((uint32_t*) buff) = usbMsc.blockSize;
					return RES_OK;
				case GET_BLOCK_SIZE:
					*((uint32_t*) buff) = usbMsc.blockSize;
					return RES_OK;
				default:
					return RES_PARERR;
			}
			break;
		}
	}

	return RES_PARERR;
}
#endif
