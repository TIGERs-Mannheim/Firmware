#include "bootloader.h"
#include "util/crc.h"
#include "hal/flash.h"
#include "errors.h"
#include <string.h>

static void bootloaderTask(void* params);
static int16_t transferNewProgram(Bootloader* pBoot);
static void handleMessage(Bootloader* pBoot);
static void sendAck(Bootloader* pBoot);
static void sendNack(Bootloader* pBoot);
static uint8_t isProgramValid(Bootloader* pBoot, uint32_t programAddr, uint8_t crcRequired);
static void jumpToApplication(Bootloader* pBoot);

void BootloaderInit(Bootloader* pBoot, BootloaderData* pInit, tprio_t prio)
{
	pBoot->data = *pInit;

	if(pBoot->data.flashWriteGranularity > sizeof(pBoot->tmpBuf))
	{
		chSysHalt("Bootloader temporary buffer too small");
	}

	pBoot->pTask = chThdCreateStatic(pBoot->waTask, sizeof(pBoot->waTask), prio, &bootloaderTask, pBoot);
}

void BootloaderAddInterface(Bootloader* pBoot, BootloaderInterface* pInterface)
{
	if(pBoot->pInterfaceList == 0)
	{
		pBoot->pInterfaceList = pInterface;
	}
	else
	{
		BootloaderInterface* pLast = pBoot->pInterfaceList;
		while(pLast->pNext)
			pLast = pLast->pNext;

		pLast->pNext = pInterface;
	}

	pInterface->pNext = 0;
}

static void bootloaderTask(void *params)
{
	Bootloader *pBoot = (Bootloader*) params;

	chRegSetThreadName("Bootloader");

	uint8_t cmd;

	const uint8_t isActiveProgramValid = isProgramValid(pBoot, pBoot->data.activeProgramAddr, 0);

	// check if there is a new program waiting to be transferred to active program area
	int16_t transferResult = transferNewProgram(pBoot);
	if(!transferResult)
	{
		// If update was successful jump to application right away
		jumpToApplication(pBoot);
	}

	systime_t tStart = chVTGetSystemTime();

	while(1)
	{
		if(chVTTimeElapsedSinceX(tStart) > TIME_MS2I(pBoot->data.bootloaderTimeout_ms) && isActiveProgramValid && pBoot->pSelectedInterface == 0)
		{
			jumpToApplication(pBoot);
		}

		if(pBoot->pSelectedInterface)
		{
			while((*pBoot->pSelectedInterface->pReadByte)(pBoot->pSelectedInterface->pUser, &cmd))
			{
				if(BootloaderParserStream(&pBoot->pSelectedInterface->parser, cmd))
				{
					handleMessage(pBoot);
				}
			}
		}
		else
		{
			BootloaderInterface *pInterface = pBoot->pInterfaceList;

			while(pInterface)
			{
				while((*pInterface->pReadByte)(pInterface->pUser, &cmd))
				{
					if(BootloaderParserStream(&pInterface->parser, cmd))
					{
						pBoot->pSelectedInterface = pInterface;
						handleMessage(pBoot);
					}
				}

				pInterface = pInterface->pNext;
			}
		}

		if(pBoot->initiateExitTime != 0 && chVTTimeElapsedSinceX(pBoot->initiateExitTime) > TIME_MS2I(100))
		{
			jumpToApplication(pBoot);
		}

		chThdSleep(1);
	}
}

static int16_t transferNewProgram(Bootloader *pBoot)
{
	const uint8_t isNewProgramValid = isProgramValid(pBoot, pBoot->data.newProgramAddr, 1);

	if(!isNewProgramValid)
		return ERROR_INVALID_PARAMETER;

	pBoot->currentOperation.state = BOOTLOADER_OPERATION_STATE_ACTIVE;
	pBoot->currentOperation.type = BOOTLOADER_MSG_TYPE_ERASE;

	// Erase active program
	int16_t result = FlashErase(pBoot->data.activeProgramAddr, pBoot->data.activeProgramAddr + pBoot->data.maxProgramSize);
	if(result)
		return result;

	// Program CRC first
	memset(pBoot->tmpBuf, 0xFF, sizeof(pBoot->tmpBuf));

	uint32_t writeSize = ((6 + pBoot->data.flashWriteGranularity - 1) / pBoot->data.flashWriteGranularity) * pBoot->data.flashWriteGranularity;

	uint16_t *pCrcPresent = (uint16_t*) &pBoot->tmpBuf[writeSize - 6];
	uint32_t *pCrc = (uint32_t*) &pBoot->tmpBuf[writeSize - 4];

	volatile uint32_t *pNewCrc = (volatile uint32_t*) (pBoot->data.newProgramAddr + pBoot->data.maxProgramSize - 4);
	*pCrc = *pNewCrc;
	*pCrcPresent = 0;

	pBoot->currentOperation.type = BOOTLOADER_MSG_TYPE_WRITE;

	result = FlashProgram(pBoot->data.activeProgramAddr + pBoot->data.maxProgramSize - writeSize, (uint32_t*) pBoot->tmpBuf, writeSize / 4);
	if(result)
		return result;

	// Check size of new program
	uint32_t newProgramSize = 0;
	for(uint32_t addr = pBoot->data.newProgramAddr + pBoot->data.maxProgramSize - 8; addr > pBoot->data.newProgramAddr; addr -= 4)
	{
		if(*((volatile uint32_t*) addr) != 0xFFFFFFFF)
		{
			newProgramSize = addr - pBoot->data.newProgramAddr + 4;
			newProgramSize = ((newProgramSize + pBoot->data.flashWriteGranularity - 1) / pBoot->data.flashWriteGranularity) * pBoot->data.flashWriteGranularity;
			break;
		}
	}

	// Copy program data
	result = FlashProgram(pBoot->data.activeProgramAddr, (uint32_t*) pBoot->data.newProgramAddr, newProgramSize / 4);
	if(result)
		return result;

	pBoot->currentOperation.type = BOOTLOADER_MSG_TYPE_ERASE;

	// Clean new program area on success
	result = FlashErase(pBoot->data.newProgramAddr, pBoot->data.newProgramAddr + pBoot->data.maxProgramSize);

	return result;
}

static void archiveOperation(Bootloader* pBoot, uint8_t ack)
{
	pBoot->lastOperation = pBoot->currentOperation;
	pBoot->lastOperation.state = ack ? BOOTLOADER_OPERATION_STATE_ACK : BOOTLOADER_OPERATION_STATE_NACK;
	pBoot->currentOperation.type = 0;
	pBoot->currentOperation.state = BOOTLOADER_OPERATION_STATE_NONE;
}

static void handleMessage(Bootloader *pBoot)
{
	BootloaderInterface *pInterface = pBoot->pSelectedInterface;
	const Bootloader_Header_t *pHeader = &pInterface->parser.header;
	const void *pData = pInterface->parser.buf;

	const uint32_t bootloaderStartAddr = pBoot->data.bootloaderProgramAddr;
	const uint32_t bootloaderEndAddr = pBoot->data.bootloaderProgramAddr + pBoot->data.bootloaderMaxProgramSize;
	const uint32_t flashStartAddr = pBoot->data.flashAddr;
	const uint32_t flashEndAddr = pBoot->data.flashAddr + pBoot->data.flashSize;

	pBoot->currentOperation.type = pHeader->type;
	pBoot->currentOperation.state = BOOTLOADER_OPERATION_STATE_ACTIVE;

	switch(pHeader->type)
	{
		case BOOTLOADER_MSG_TYPE_GET_INFO:
		{
			Bootloader_Info_t info;
			info.header.magic[0] = 'B';
			info.header.magic[1] = 'L';
			info.header.type = BOOTLOADER_MSG_TYPE_GET_INFO;
			info.header.length = sizeof(info);
			info.version = 0x0001;
			info.writeGranularity = pBoot->data.flashWriteGranularity;
			info.writeTime_us = pBoot->data.flashWriteTime_us;
			info.eraseTime_us = pBoot->data.flashEraseTime_us;
			strncpy(info.deviceName, pBoot->data.pDeviceName, sizeof(info.deviceName) - 1);
			info.deviceName[sizeof(info.deviceName) - 1] = 0;

			BootloaderInsertChecksums(&info);

			(*pInterface->pWriteData)(pInterface->pUser, &info, sizeof(info));

			archiveOperation(pBoot, 1);
		}
			break;
		case BOOTLOADER_MSG_TYPE_EXIT:
		{
			if(isProgramValid(pBoot, pBoot->data.activeProgramAddr, 0))
			{
				sendAck(pBoot);

				pBoot->initiateExitTime = chVTGetSystemTime();
			}
			else
			{
				sendNack(pBoot);
			}
		}
			break;
		case BOOTLOADER_MSG_TYPE_CHECK_CRC:
		{
			const Bootloader_CRC_t *pCrc = (const Bootloader_CRC_t*) pData;

			pBoot->currentOperation.addr = pCrc->addr;
			pBoot->currentOperation.length = pCrc->length;

			uint32_t calcCrc = CRC32CalcChecksum((void*) pCrc->addr, pCrc->length);
			if(calcCrc == pCrc->crc32)
				sendAck(pBoot);
			else
				sendNack(pBoot);
		}
			break;
		case BOOTLOADER_MSG_TYPE_ERASE:
		{
			const Bootloader_Erase_t *pErase = (const Bootloader_Erase_t*) pData;

			pBoot->currentOperation.addr = pErase->addr;
			pBoot->currentOperation.length = pErase->length;

			const uint32_t eraseStartAddr = pErase->addr;
			const uint32_t eraseEndAddr = pErase->addr + pErase->length;

			uint8_t isTargetOutsideBootloader = eraseEndAddr < bootloaderStartAddr || eraseStartAddr >= bootloaderEndAddr;
			uint8_t isTargetInFlash = eraseStartAddr >= flashStartAddr && eraseEndAddr <= flashEndAddr;

			// bootloader cannot be erased, check that range is actually in flash
			if(!isTargetOutsideBootloader || !isTargetInFlash)
			{
				sendNack(pBoot);
				return;
			}

			if(FlashErase(pErase->addr, pErase->addr + pErase->length))
			{
				sendNack(pBoot);
				return;
			}

			sendAck(pBoot);
		}
			break;
		case BOOTLOADER_MSG_TYPE_WRITE:
		{
			const Bootloader_Write_t *pWrite = (const Bootloader_Write_t*) pData;

			const uint32_t *pWriteData = (const uint32_t*) (((const uint8_t*) pData) + sizeof(Bootloader_Write_t));
			uint32_t writeSize = pHeader->length - sizeof(Bootloader_Write_t);

			pBoot->currentOperation.addr = pWrite->addr;
			pBoot->currentOperation.length = writeSize;

			const uint32_t writeStartAddr = pWrite->addr;
			const uint32_t writeEndAddr = pWrite->addr + writeSize;

			uint8_t isTargetOutsideBootloader = writeEndAddr < bootloaderStartAddr || writeStartAddr >= bootloaderEndAddr;
			uint8_t isTargetInFlash = writeStartAddr >= flashStartAddr && writeEndAddr <= flashEndAddr;

			if((writeSize % pBoot->data.flashWriteGranularity) != 0)
			{
				// write size not a multiple of write granularity
				sendNack(pBoot);
				return;
			}

			// bootloader cannot be erased, check that range is actually in flash
			if(!isTargetOutsideBootloader || !isTargetInFlash)
			{
				sendNack(pBoot);
				return;
			}

			if(FlashProgram(pWrite->addr, pWriteData, writeSize / 4))
			{
				sendNack(pBoot);
				return;
			}

			sendAck(pBoot);
		}
			break;
	}
}

static void sendAck(Bootloader *pBoot)
{
	Bootloader_Ack_t ack;
	ack.header.magic[0] = 'B';
	ack.header.magic[1] = 'L';
	ack.header.type = BOOTLOADER_MSG_TYPE_ACK;
	ack.header.length = sizeof(ack);
	ack.ack = 0x79;
	BootloaderInsertChecksums(&ack);

	(*pBoot->pSelectedInterface->pWriteData)(pBoot->pSelectedInterface->pUser, &ack, sizeof(ack));

	archiveOperation(pBoot, 1);
}

static void sendNack(Bootloader *pBoot)
{
	Bootloader_Ack_t ack;
	ack.header.magic[0] = 'B';
	ack.header.magic[1] = 'L';
	ack.header.type = BOOTLOADER_MSG_TYPE_ACK;
	ack.header.length = sizeof(ack);
	ack.ack = 0x1F;
	BootloaderInsertChecksums(&ack);

	(*pBoot->pSelectedInterface->pWriteData)(pBoot->pSelectedInterface->pUser, &ack, sizeof(ack));

	archiveOperation(pBoot, 0);
}

static uint8_t isProgramValid(Bootloader *pBoot, uint32_t programAddr, uint8_t crcRequired)
{
	uint32_t crcAddr = programAddr + pBoot->data.maxProgramSize - 4;
	uint32_t crcPresentAddr = crcAddr - 2;

	volatile uint32_t *pCrc = (volatile uint32_t*) crcAddr;
	volatile uint16_t *pCrcPresent = (volatile uint16_t*) crcPresentAddr;

	uint32_t programSize = 0;

	for(volatile uint32_t *pRead = pCrc - 4; (uint32_t) pRead > programAddr; pRead--)
	{
		if(*pRead != 0xFFFFFFFF)
		{
			programSize = (uint32_t) pRead - programAddr + 4;
			break;
		}
	}

	if(programSize == 0)
		return 0; // no data found

	if(*pCrcPresent == 0xFFFF) // no CRC present
	{
		if(crcRequired)
			return 0;
		else
			return 1; // probably programmed by debug adapter, assume it's valid
	}

	uint32_t calcCrc = CRC32CalcChecksum((void*) programAddr, programSize);

	if(*pCrc != calcCrc)
		return 0; // CRC mismatch in program code

	return 1;
}

static void jumpToApplication(Bootloader *pBoot)
{
	void (*jumpTarget)(void);
	volatile uint32_t BootAddr = pBoot->data.activeProgramAddr;

	if(pBoot->data.pSystemDeinit)
		(*pBoot->data.pSystemDeinit)();

	// Set up the jump to address + 4 */
	jumpTarget = (void (*)(void)) (*((uint32_t*) ((BootAddr + 4))));

	// Set the main stack pointer to the application stack */
	__set_MSP(*(uint32_t*) BootAddr);

	// Call the function to jump to application
	jumpTarget();

	while(1)
		;
}
