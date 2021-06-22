/*
 * sdcard.c
 *
 *  Created on: 05.11.2015
 *      Author: AndreR
 */

#include "sdcard.h"
#include "hal/led.h"
#include "util/init_hal.h"
#include "util/console.h"
#include "constants.h"
#include "util/log_file.h"
#include "util/log.h"
#include <string.h>

#define SD_PIN_WP GPIO_PIN_5
#define SD_PIN_CD GPIO_PIN_6
#define SD_PIN_PWR GPIO_PIN_10

#define SD_PWR_ON() GPIOSet(GPIOA, SD_PIN_PWR)
#define SD_PWR_OFF() GPIOReset(GPIOA, SD_PIN_PWR)

#define SD_CLEAR_ALL_IRQ() (SDMMC1->ICR = 0x4005FF)

#define SD_DMA DMA2_Stream3

SDCard sdCard;

static uint8_t dmaBuf[64] __attribute__((section(".dtc")));

void SDMMC1_IRQHandler()
{
	CH_IRQ_PROLOGUE();

	SDMMC1->MASK = 0;

	chSysLockFromISR();
	chBSemSignalI(&sdCard.irqSem);
	chSysUnlockFromISR();

	CH_IRQ_EPILOGUE();
}

void SDCardInit()
{
	GPIOInitData gpioInit;

	chBSemObjectInit(&sdCard.irqSem, 1);
	chMtxObjectInit(&sdCard.transferMtx);

	// SD_PWR
	GPIOReset(GPIOA, GPIO_PIN_10); // pull low to disable SD card

	gpioInit.mode = GPIO_MODE_OUTPUT;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_2MHZ;
	gpioInit.pupd = GPIO_PUPD_NONE;
	GPIOInit(GPIOA, GPIO_PIN_10, &gpioInit);

	// WP and CD
	gpioInit.mode = GPIO_MODE_INPUT;
	gpioInit.pupd = GPIO_PUPD_UP;
	GPIOInit(GPIOG, GPIO_PIN_5 | GPIO_PIN_6, &gpioInit);

	// D0-D1, CK, CMD
	gpioInit.mode = GPIO_MODE_AF;
	gpioInit.pupd = GPIO_PUPD_UP;
	gpioInit.otype = GPIO_OTYPE_PUSH_PULL;
	gpioInit.ospeed = GPIO_OSPEED_100MHZ;
	gpioInit.alternate = 12;
	GPIOInit(GPIOC, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12, &gpioInit);
	GPIOInit(GPIOD, GPIO_PIN_2, &gpioInit);

	RCC->APB2ENR |= RCC_APB2ENR_SDMMC1EN;

	SDMMC1->POWER = 0;
	SDMMC1->CLKCR = 0;
	SDMMC1->MASK = 0;

	NVICEnableIRQ(SDMMC1_IRQn, IRQL_SDMMC1);

	// Configure to channel 4, high prio, data size: 4byte, periph flow ctrl
	SD_DMA->FCR = DMA_FTH_FULL | DMA_SxFCR_DMDIS;
	SD_DMA->CR = (4 << 25) | DMA_PL_HIGH | DMA_PSIZE_4BYTE | DMA_SxCR_PBURST_0 |
			DMA_MSIZE_4BYTE | DMA_DIR_MEM2PERIPH | DMA_SxCR_MINC | DMA_SxCR_PFCTRL;
	SD_DMA->M0AR = (uint32_t)dmaBuf;
	SD_DMA->PAR = (uint32_t)&SDMMC1->FIFO;
}

#define SD_WAITRESP_SENT	0
#define SD_WAITRESP_SHORT	1
#define SD_WAITRESP_LONG	3

static uint32_t sendCmd(uint32_t cmd, uint32_t arg, uint32_t waitResponse)
{
	uint32_t staMask = SDMMC_STA_CMDSENT;
	if(waitResponse > 0)
		staMask = SDMMC_STA_CCRCFAIL | SDMMC_STA_CMDREND | SDMMC_STA_CTIMEOUT;

	SD_CLEAR_ALL_IRQ();
	chBSemReset(&sdCard.irqSem, 1);
	SDMMC1->MASK = staMask;
	SDMMC1->ARG = arg;
	SDMMC1->CMD = cmd | (waitResponse << 6) | SDMMC_CMD_CPSMEN;

	chBSemWait(&sdCard.irqSem);

	uint32_t sta = SDMMC1->STA;

	return sta;
}

static int16_t readData(uint8_t* pBuffer, uint32_t numBytes, uint32_t dBlockSize, uint32_t cmd, uint32_t arg)
{
	SDMMC1->DCTRL = 0;

//	uint32_t buffAddr = (uint32_t)pBuffer;
//	if(buffAddr%4 != 0)
//	{
//		LogErrorC("Unaligned buffer", numBytes);
//	}
//	buffAddr &= 0xFFFFFFE0;	// align to 32 byte boundary

	SD_CLEAR_ALL_IRQ();
	chBSemReset(&sdCard.irqSem, 1);

	// flush cache lines that will be updated in a moment by DMA
//	SCB_CleanInvalidateDCache_by_Addr((uint32_t*)buffAddr, numBytes+32);

	DMA2->LIFCR = 0x3D << 22;
	SD_DMA->M0AR = (uint32_t)pBuffer;
	SD_DMA->CR = (4 << 25) | DMA_PL_HIGH | DMA_PSIZE_4BYTE | DMA_SxCR_PBURST_0 |
			DMA_MSIZE_4BYTE | DMA_DIR_PERIPH2MEM | DMA_SxCR_MINC | DMA_SxCR_PFCTRL;
	SD_DMA->CR |= DMA_SxCR_EN;

	SDMMC1->DTIMER = 24000000; // 1s
	SDMMC1->DLEN = numBytes;
	SDMMC1->DCTRL = SDMMC_DCTRL_DTEN | SDMMC_DCTRL_DTDIR | SDMMC_DCTRL_DMAEN | (dBlockSize << 4);
	SDMMC1->ARG = arg;
	SDMMC1->MASK = SDMMC_STA_CCRCFAIL | SDMMC_STA_CMDREND | SDMMC_STA_CTIMEOUT;
	SDMMC1->CMD = cmd | (SD_WAITRESP_SHORT << 6) | SDMMC_CMD_CPSMEN;

	chBSemWait(&sdCard.irqSem);
//	ConsolePrint("CMD done\r\nSTA: 0x%08X\r\n", SDMMC1->STA);

	if((SDMMC1->STA & SDMMC_STA_CMDREND) == 0)
		return 1;	// CMD failed

	SDMMC1->MASK = SDMMC_STA_DATAEND | SDMMC_STA_DTIMEOUT | SDMMC_STA_DCRCFAIL;
	chBSemWait(&sdCard.irqSem);

//	ConsolePrint("Data done\r\nSTA: 0x%08X\r\n", SDMMC1->STA);

	if(SDMMC1->STA & (SDMMC_STA_DTIMEOUT | SDMMC_STA_DCRCFAIL))
	{
		return 2; // DATA transfer failed
	}

	return 0;
}

static int16_t writeData(const uint8_t* pBuffer, uint32_t numBytes, uint32_t dBlockSize, uint32_t cmd, uint32_t arg)
{
	SDMMC1->DCTRL = 0;

//	uint32_t buffAddr = (uint32_t)pBuffer;
//	if(buffAddr%4 != 0)
//	{
//		LogErrorC("Unaligned buffer", numBytes);
//	}
//	buffAddr &= 0xFFFFFFE0;	// align to 32 byte boundary

	// flush cache lines that will be updated in a moment by DMA
//	SCB_CleanInvalidateDCache_by_Addr((uint32_t*)buffAddr, numBytes+32);

	uint32_t sta = sendCmd(cmd, arg, SD_WAITRESP_SHORT);
	if((sta & SDMMC_STA_CMDREND) == 0)
	{
		return 1; // CMD failed
	}

//	ConsolePrint("CMD done\r\nSTA: 0x%08X\r\n", sta);

	chBSemReset(&sdCard.irqSem, 1);

	DMA2->LIFCR = 0x3D << 22;
	SD_DMA->M0AR = (uint32_t)pBuffer;
	SD_DMA->CR = (4 << 25) | DMA_PL_HIGH | DMA_PSIZE_4BYTE | DMA_SxCR_PBURST_0 |
			DMA_MSIZE_4BYTE | DMA_DIR_MEM2PERIPH | DMA_SxCR_MINC | DMA_SxCR_PFCTRL;
	SD_DMA->CR |= DMA_SxCR_EN;

	SDMMC1->DTIMER = 24000000; // 1s
	SDMMC1->DLEN = numBytes;
	SDMMC1->DCTRL = SDMMC_DCTRL_DTEN | SDMMC_DCTRL_DMAEN | (dBlockSize << 4);
	SDMMC1->MASK = SDMMC_STA_DTIMEOUT | SDMMC_STA_DCRCFAIL | SDMMC_STA_DATAEND;

	chBSemWait(&sdCard.irqSem);
//	ConsolePrint("Data done\r\nSTA: 0x%08X\r\n", SDMMC1->STA);

	SD_DMA->CR &= ~(DMA_SxCR_EN);

	if(SDMMC1->STA & (SDMMC_STA_DTIMEOUT | SDMMC_STA_DCRCFAIL))
	{
		return 2; // DATA failed
	}

	return 0;
}

static void parseCID(SDCID* pCID)
{
	memset(pCID, 0, sizeof(SDCID));

	uint32_t R2[4];
	R2[0] = SDMMC1->RESP1;
	R2[1] = SDMMC1->RESP2;
	R2[2] = SDMMC1->RESP3;
	R2[3] = SDMMC1->RESP4;

	pCID->MDTMonth = (R2[3] & 0xF00) >> 8;
	pCID->MDTYear = 2000 + ((R2[3] & 0xF000) >> 12);

	pCID->PSN = ((R2[3] & 0xFF000000) >> 24) | (R2[2] & 0x00FFFFFF);

	pCID->PRV[0] = 48 + ((R2[2] & 0xF0000000) >> 28);
	pCID->PRV[1] = '.';
	pCID->PRV[2] = 48 + ((R2[2] & 0x0F000000) >> 24);

	pCID->PNM[0] = (R2[0] & 0x000000FF) >> 0;
	pCID->PNM[1] = (R2[1] & 0xFF000000) >> 24;
	pCID->PNM[2] = (R2[1] & 0x00FF0000) >> 16;
	pCID->PNM[3] = (R2[1] & 0x0000FF00) >> 8;
	pCID->PNM[4] = (R2[1] & 0x000000FF) >> 0;

	pCID->OID[0] = (R2[0] & 0x00FF0000) >> 16;
	pCID->OID[1] = (R2[0] & 0x0000FF00) >> 8;

	pCID->MID = ((R2[0] & 0xFF000000) >> 24);
}

static void printCID(SDCID* pCID)
{
//	ConsolePrint("MID: 0x%02hX\r\n", (uint16_t)pCID->MID);
//	ConsolePrint("OID: %s\r\n", pCID->OID);
	ConsolePrint("PNM: %s\r\n", pCID->PNM);
	ConsolePrint("PRV: %s\r\n", pCID->PRV);
	ConsolePrint("PSN: 0x%08X\r\n", pCID->PSN);
	ConsolePrint("MDT: %02hu.%hu\r\n", (uint16_t)pCID->MDTMonth, pCID->MDTYear);
}

static void parseCSD(SDCSD* pCSD)
{
	memset(pCSD, 0, sizeof(SDCSD));

	uint32_t R2[4];
	R2[0] = SDMMC1->RESP1;
	R2[1] = SDMMC1->RESP2;
	R2[2] = SDMMC1->RESP3;
	R2[3] = SDMMC1->RESP4;

//	ConsolePrint("R2: 0x%08X, 0x%08X, 0x%08X, 0x%08X\r\n", R2[0], R2[1], R2[2], R2[3]);

	pCSD->highCap = (R2[0] & 0xC0000000) >> 30;
	pCSD->dsrImp = (R2[1] & 0x1000) >> 12;

	if(pCSD->highCap == 0)
	{
		// CSD Version 1.0, Standard Capacity (SD)

		uint32_t readBlLen = (R2[1] & 0xF0000) >> 16;
		uint32_t cSize = ((R2[2] & 0xC0000000) >> 30) | ((R2[1] & 0x3FF) << 2);
		uint32_t cSizeMult = (R2[2] & 0x38000) >> 15;

		pCSD->capacity = ((cSize+1) * (1 << (cSizeMult+2)) * (1 << readBlLen))/1024;
	}

	if(pCSD->highCap == 1)
	{
		// CSD Version 2.0, SDHC or SDXC

		uint32_t cSize = ((R2[2] & 0xFFFF0000) >> 16) | ((R2[1] & 0x3F) << 16);

		pCSD->capacity = (cSize+1) * 512;
	}
}

static int16_t initCard()
{
	uint32_t sta;

	SDMMC1->CLKCR = 118;	// CLKDIV for 400kHz (during init)
	SDMMC1->POWER = SDMMC_POWER_PWRCTRL;

	chThdSleepMilliseconds(10);

	SDMMC1->CLKCR |= SDMMC_CLKCR_CLKEN;	// enable clock

	// CMD0
	sendCmd(0, 0, SD_WAITRESP_SENT);
//	ConsolePrint("CMD0 sent\r\n");

	// CMD8
	sta = sendCmd(8, 0x1AA, SD_WAITRESP_SHORT);  // check pattern | 2.7-3.6V range
//	ConsolePrint("CMD8 finished.\r\nSTA: 0x%08X\r\n", sta);

	if(sta & SDMMC_STA_CMDREND)
	{
		// Good response
//		uint32_t R7 = SDMMC1->RESP1;

//		uint32_t voltageAccepted = (R7 >> 8) & 1;
//		uint32_t checkPattern = R7 & 0xFF;

//		ConsolePrint("Voltage: %u, check: %02X\r\n", voltageAccepted, checkPattern);
	}
	else if(sta & SDMMC_STA_CTIMEOUT)
	{
		// No Response
	}
	else
	{
		// Error response
		return 1;
	}

	// ACMD41, Arg=S18R, HCS, WV
	systime_t startTime = chVTGetSystemTimeX();
	uint32_t acmd41Runs = 0;

	while(chVTTimeElapsedSinceX(startTime) < MS2ST(1000))
	{
		acmd41Runs++;

		// CMD55, APP_CMD, next cmd is interpreted as application specific
		sta = sendCmd(55, 0, SD_WAITRESP_SHORT);
		if((sta & SDMMC_STA_CMDREND) == 0)
		{
			ConsolePrint("CMD55 failed 0x%08X, %u\r\n", sta, acmd41Runs);
			return 1;
		}

		// ACMD41, doesn't have CRC!
		sta = sendCmd(41, 0xC0100000, SD_WAITRESP_SHORT);

		uint32_t R3 = SDMMC1->RESP1;
		if(R3 & 0x80000000) // busy bit set?
		{
			// init complete!
//			ConsolePrint("ACMD41\r\nR3: 0x%08X\r\nTime: %u, STA: 0x%08X\r\n", R3, chVTGetSystemTimeX()-startTime, sta);
			break;
		}
	}

	if(chVTTimeElapsedSinceX(startTime) >= MS2ST(1000))
	{
		ConsolePrint("ACMD41 timed out\r\n");
		return 1;
	}

	// CMD2, Arg = 0 (ALL_SEND_CID), expect R2
	sta = sendCmd(2, 0, SD_WAITRESP_LONG);

	if(sta & SDMMC_STA_CMDREND)
	{
		parseCID(&sdCard.cid);
		printCID(&sdCard.cid);
	}
	else
	{
		ConsolePrint("CMD2: no response, 0x%08X\r\n", sta);
		return 1;
	}

	// CMD3, Arg = 0 (SEND_RELATIVE_ ADDR), expect R6
	sta = sendCmd(3, 0, SD_WAITRESP_SHORT);

	if(sta & SDMMC_STA_CMDREND)
	{
		SDMMC1->RESP1;
//		uint32_t R6 = SDMMC1->RESP1;
//		ConsolePrint("CMD3 finished.\r\nSTA: 0x%08X\r\nR6: 0x%08X\r\n", sta, R6);
	}
	else
	{
		ConsolePrint("CMD3: no response, 0x%08X\r\n", sta);
		return 1;
	}

	sdCard.RCA = SDMMC1->RESP1 & 0xFFFF0000;

	// CMD9, Arg = RCA (SEND_CSD), expect R6
	sta = sendCmd(9, sdCard.RCA, SD_WAITRESP_LONG);
	if(sta & SDMMC_STA_CMDREND)
	{
		parseCSD(&sdCard.csd);
		ConsolePrint("Capacity: %uMB\r\n", sdCard.csd.capacity/1024);
	}
	else
	{
		ConsolePrint("CMD9: no response, 0x%08X\r\n", sta);
		return 1;
	}

	// CMD13, ARG = RCA (SEND_STATUS), expect R1
	sta = sendCmd(13, sdCard.RCA, SD_WAITRESP_SHORT);
	if(sta & SDMMC_STA_CMDREND)
	{
//		uint32_t status = SDMMC1->RESP1;
//		uint32_t appCmd = (status & 0x20) >> 5;
//		uint32_t readyForData = (status & 0x100) >> 8;
//		uint32_t state = (status & 0x1E00) >> 9;

//		ConsolePrint("CMD13 finished.\r\nState: %u\r\nReady: %u\r\nApp: %u\r\n",
//				state, readyForData, appCmd);
	}
	else
	{
		ConsolePrint("CMD13: no response, 0x%08X\r\n", sta);
		return 1;
	}

	// CMD7, Arg = RCA (SELECT_CARD), go to transfer
	sta = sendCmd(7, sdCard.RCA, SD_WAITRESP_SHORT);
	if((sta & SDMMC_STA_CMDREND) == 0)
	{
		ConsolePrint("CMD7 failed 0x%08X\r\n", sta);
		return 1;
	}

	// CMD13, ARG = RCA (SEND_STATUS), expect R1
	sta = sendCmd(13, sdCard.RCA, SD_WAITRESP_SHORT);
	if(sta & SDMMC_STA_CMDREND)
	{
//		uint32_t status = SDMMC1->RESP1;
//		uint32_t appCmd = (status & 0x20) >> 5;
//		uint32_t readyForData = (status & 0x100) >> 8;
//		uint32_t state = (status & 0x1E00) >> 9;
//
//		ConsolePrint("CMD13 finished.\r\nState: %u\r\nReady: %u\r\nApp: %u\r\n",
//				state, readyForData, appCmd);
	}
	else
	{
		ConsolePrint("CMD13: no response, 0x%08X\r\n", sta);
		return 1;
	}

	// CMD42, unlock

	// CMD55, APP_CMD, next cmd is interpreted as application specific
	sta = sendCmd(55, sdCard.RCA, SD_WAITRESP_SHORT);
	if((sta & SDMMC_STA_CMDREND) == 0)
	{
		ConsolePrint("CMD55 failed 0x%08X\r\n", sta);
		return 1;
	}

	// ACMD6, ARG = 2 (SET_BUS_WIDTH), expect R1
	sta = sendCmd(6, 2, SD_WAITRESP_SHORT);
	if(sta & SDMMC_STA_CMDREND)
	{
//		ConsolePrint("ACMD6 finished. Bus-Width: 4 bit\r\n");
	}
	else
	{
		ConsolePrint("ACMD6: no response, 0x%08X\r\n", sta);
		return 1;
	}

	SDMMC1->CLKCR = 0 | SDMMC_CLKCR_CLKEN | SDMMC_CLKCR_WIDBUS_0 | SDMMC_CLKCR_HWFC_EN;	// CLKDIV for 24MHz

	// CMD13, ARG = RCA (SEND_STATUS), expect R1
	sta = sendCmd(13, sdCard.RCA, SD_WAITRESP_SHORT);
	if(sta & SDMMC_STA_CMDREND)
	{
//		uint32_t status = SDMMC1->RESP1;
//		uint32_t appCmd = (status & 0x20) >> 5;
//		uint32_t readyForData = (status & 0x100) >> 8;
//		uint32_t state = (status & 0x1E00) >> 9;
//
//		ConsolePrint("CMD13 finished.\r\nState: %u\r\nReady: %u\r\nApp: %u\r\n",
//				state, readyForData, appCmd);
	}
	else
	{
		ConsolePrint("CMD13: no response, 0x%08X\r\n", sta);
		return 1;
	}

	// CMD55, APP_CMD, next cmd is interpreted as application specific
	sta = sendCmd(55, sdCard.RCA, SD_WAITRESP_SHORT);
	if((sta & SDMMC_STA_CMDREND) == 0)
	{
		ConsolePrint("CMD55 failed 0x%08X\r\n", sta);
		return 1;
	}

	// ACMD13, ARG = 0 (SD_STATUS), expect R1 + 64B of data
	if(readData(dmaBuf, 64, 6, 13, 0) == 0)
	{
//		ConsolePrint("ACMD13 complete\r\n");

		uint8_t speedClass = dmaBuf[8]*2;
		if(speedClass == 8)
			speedClass = 10;
		uint8_t uhsClass = dmaBuf[14] >> 4;

		sdCard.speedClass = speedClass;
		sdCard.uhsClass = uhsClass;

		ConsolePrint("Speed class: %hu\r\nUHS: %hu\r\n", (uint16_t)speedClass, (uint16_t)uhsClass);

		sdCard.ready = 1;
	}

//	SDCardRead(0, dmaBuf, 4);
//	SDCardWrite(0, dmaBuf, 4);

	return 0;
}

int16_t SDCardRead(uint32_t blockAddr, uint8_t* pBuffer, uint32_t numBlocks)
{
	int16_t result = 0;

	uint32_t addr = blockAddr;
	if(sdCard.csd.highCap == 0)
		addr = blockAddr*512;

	if(numBlocks == 0)
		return 1;

	SDCardWaitUntilReady();

	chMtxLock(&sdCard.transferMtx);

	if(numBlocks == 1)
	{
		// CMD17, ARG = addr (READ_SINGLE_BLOCK), expect R1 + 512B of data
		result = readData(pBuffer, 512, 9, 17, addr);
//		ConsolePrint("CMD17 complete: %hd\r\n", result);
	}
	else
	{
		// TODO: could use CMD23, check SCR with ACMD51 first

		// CMD18, ARG = addr (READ_MULTIPLE_BLOCK), expect R1b + data
		result = readData(pBuffer, numBlocks*512, 9, 18, addr);
//		ConsolePrint("CMD18 complete: %hd\r\n", result);

		// CMD12, ARG = 0 (STOP_TRANSMISSION)
		sendCmd(12, 0, SD_WAITRESP_SHORT);
//		uint32_t sta = sendCmd(12, 0, SD_WAITRESP_SHORT);
//		ConsolePrint("CMD12 complete: 0x%08X\r\n", sta);
	}

	chMtxUnlock(&sdCard.transferMtx);

	return result;
}

int16_t SDCardWrite(uint32_t blockAddr, const uint8_t* pBuffer, uint32_t numBlocks)
{
	int16_t result = 0;

	uint32_t addr = blockAddr;
	if(sdCard.csd.highCap == 0)
		addr = blockAddr*512;

	if(numBlocks == 0)
		return 1;

	SDCardWaitUntilReady();

	chMtxLock(&sdCard.transferMtx);

	if(numBlocks == 1)
	{
		// CMD24, ARG = addr (WRITE_BLOCK), expect R1
		result = writeData(pBuffer, 512, 9, 24, addr);
//		ConsolePrint("CMD24 complete: %hd\r\n", result);
	}
	else
	{
		// CMD55, APP_CMD, next cmd is interpreted as application specific
		uint32_t sta = sendCmd(55, sdCard.RCA, SD_WAITRESP_SHORT);
		if((sta & SDMMC_STA_CMDREND) == 0)
		{
			chMtxUnlock(&sdCard.transferMtx);
			return 1;
		}

		// ACMD23, ARG = numBlocks (SET_WR_BLK_ERASE_COUNT), expect R1
		sta = sendCmd(23, numBlocks, SD_WAITRESP_SHORT);
		if((sta & SDMMC_STA_CMDREND) == 0)
		{
			chMtxUnlock(&sdCard.transferMtx);
			return 1;
		}

		// TODO: could use CMD23, check SCR with ACMD51 first

		// CMD25, ARG = addr (WRITE_MULTIPLE_BLOCK), expect R1
		result = writeData(pBuffer, numBlocks*512, 9, 25, addr);
//		ConsolePrint("CMD25 complete: %hd\r\n", result);

		// CMD12, ARG = 0 (STOP_TRANSMISSION)
		sta = sendCmd(12, 0, SD_WAITRESP_SHORT);
//		ConsolePrint("CMD12 complete: 0x%08X\r\n", sta);
	}

	chMtxUnlock(&sdCard.transferMtx);

	return result;
}

void SDCardWaitUntilReady()
{
	uint8_t ready;
	uint8_t state;

	SDCardGetStatus(&ready, &state);

	while(ready == 0)
	{
		chThdSleepMilliseconds(1);
		SDCardGetStatus(&ready, &state);
	}
}

int16_t SDCardGetStatus(uint8_t* pReady, uint8_t* pState)
{
	int16_t result = 0;

	chMtxLock(&sdCard.transferMtx);

	// CMD13, ARG = RCA (SEND_STATUS), expect R1
	uint32_t sta = sendCmd(13, sdCard.RCA, SD_WAITRESP_SHORT);
	if(sta & SDMMC_STA_CMDREND)
	{
		uint32_t status = SDMMC1->RESP1;
//		uint32_t appCmd = (status & 0x20) >> 5;
		*pReady = (status & 0x100) >> 8;
		*pState = (status & 0x1E00) >> 9;

//		ConsolePrint("CMD13 finished.\r\nState: %hu\r\nReady: %hu\r\n",
//				(uint16_t)*pState, (uint16_t)*pReady);
	}
	else
	{
//		ConsolePrint("CMD13: no response, 0x%08X\r\n", sta);
		result = 1;
	}

	chMtxUnlock(&sdCard.transferMtx);

	return result;
}

void SDCardTask(void* params)
{
	(void)params;

	chRegSetThreadName("SDCard");

	chThdSleepMilliseconds(200);

	while(1)
	{
		chThdSleepMilliseconds(10);

		if(sdCard.present == 0 && (GPIOG->IDR & SD_PIN_CD) == 0)
		{
			// card detected
			sdCard.writeProtected = ((GPIOG->IDR & SD_PIN_WP) == 0) ? 0 : 1;

			ConsolePrint("SD Card detected.\r\nWP: %hu\r\n", (uint16_t)sdCard.writeProtected);

//			LEDMustOn(LED_RIGHT_RED);

			SD_PWR_ON();
			if(initCard())
			{
				continue;
//				LEDMustOff(LED_RIGHT_RED);
			}

			FRESULT fresult = f_mount(&sdCard.fatFs, "SD:", 1);
			if(fresult != FR_OK)
			{
				ConsolePrint("FatFS mount error: %u\r\n", (uint32_t)fresult);
				return;
			}

			// Get volume information and free clusters of drive 1
			uint32_t freeCluster;
			FATFS* fs;
			fresult = f_getfree("SD:", &freeCluster, &fs);
			if(fresult)
			{
//				LEDMustOff(LED_RIGHT_RED);
				return;
			}

			// Get total sectors and free sectors
			uint32_t totalSectors = (fs->n_fatent - 2) * fs->csize;
			uint32_t freeSectors = freeCluster * fs->csize;

			uint32_t totalSizeMB = totalSectors/((1024*1024)/512);
			uint32_t freeSizeMB = freeSectors/((1024*1024)/512);

			ConsolePrint("--- FatFS ---\r\nSize: %uMB\r\nFree: %uMB\r\n", totalSizeMB, freeSizeMB);
			ConsolePrint("Cluster size: %ukB\r\n", (fs->csize*512)/1024);

			sdCard.present = 1;

//			LEDMustOff(LED_RIGHT_RED);
//			LEDMustOn(LED_LEFT_GREEN);

			f_chdrive("SD:");
			LogFileOpen(0);
		}

		if(sdCard.present == 1 && (GPIOG->IDR & SD_PIN_CD) != 0)
		{
			ConsolePrint("SD Card removed\r\n");

			SDMMC1->POWER = 0;
			SD_PWR_OFF();

			sdCard.ready = 0;
			sdCard.present = 0;
			sdCard.writeProtected = 0;
			sdCard.speedClass = 0;
			sdCard.uhsClass = 0;
			memset(&sdCard.csd, 0, sizeof(SDCSD));
			memset(&sdCard.cid, 0, sizeof(SDCID));

			LogFileClose();

//			LEDMustOff(LED_LEFT_GREEN);
		}
	}
}
