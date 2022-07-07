/*
 * config.c
 *
 *  Created on: 16.05.2015
 *      Author: AndreR
 */

#include "config.h"
#include "commands.h"
#include "log.h"
#include "log_file.h"
#include "robot/network.h"
#include <string.h>
#include <stdio.h>

Config config;

void ConfigInit(ConfigOutputFunc outFunc)
{
	chMBObjectInit(&config.eventQueue, config.eventQueueData, CONFIG_EVENT_QUEUE_SIZE);

	config.filesUsed = 0;
	config.outputFunction = outFunc;
}

ConfigFile* ConfigOpenOrCreate(const ConfigFileDesc* pConfigDesc, void* pData, uint16_t size, ConfigUpdateCallback updateCb, uint8_t internal)
{
	for(uint16_t i = 0; i < config.filesUsed; i++)
	{
		if(config.files[i].pDesc->cfgId == pConfigDesc->cfgId)
			return &config.files[i];
	}

	if(config.filesUsed == CONFIG_MAX_FILES)
		return 0;

	ConfigFile* pCfgFile = &config.files[config.filesUsed++];

	if(FlashFSOpenOrCreate(pConfigDesc->pName, pConfigDesc->version, pData, size, &pCfgFile->pFlashFile) == FLASH_FS_NOMEM)
	{
		--config.filesUsed;
		return 0;
	}

	pCfgFile->pDesc = pConfigDesc;
	pCfgFile->pData = pData;
	pCfgFile->size = size;
	pCfgFile->updateCb = updateCb;
	pCfgFile->internal = internal;

	return pCfgFile;
}

void ConfigNotifyUpdate(ConfigFile* pFile)
{
	FlashFSWrite(&pFile->pFlashFile, pFile->pData, pFile->size);

	if(pFile->updateCb)
		(*pFile->updateCb)(pFile->pDesc->cfgId);
}

void ConfigInternalWriteFile(uint16_t id, const void* pData, uint16_t size)
{
	ConfigFile* pCfgFile = 0;

	for(uint16_t i = 0; i < config.filesUsed; i++)
	{
		if(config.files[i].pDesc->cfgId == id)
		{
			pCfgFile = &config.files[i];
			break;
		}
	}

	if(pCfgFile == 0)
	{
		LogErrorC("File does not exist", id);
		return;
	}

	if(pCfgFile->size != size)
	{
		LogErrorC("Size mismatch", (pCfgFile->size << 16 | size));
		return;
	}

	memcpy(pCfgFile->pData, pData, size);

	ConfigNotifyUpdate(pCfgFile);
}

void ConfigInternalInjectEvent(msg_t event)
{
	chMBPost(&config.eventQueue, event, TIME_IMMEDIATE);
}

void ConfigTask(void* params)
{
	(void)params;

	msg_t event;

	chRegSetThreadName("Config");
	systime_t lastCfg2LogTime = chVTGetSystemTimeX();
	uint16_t cfgToSend = 0;

	while(1)
	{
		msg_t result = chMBFetch(&config.eventQueue, &event, MS2ST(100));

		if(logFile.newLogFileStarted)
		{
			logFile.newLogFileStarted = 0;

			// write all configs once
			for(uint16_t cfgId = 0; cfgId < config.filesUsed; cfgId++)
			{
				uint32_t bytesWritten;

				if(LogFilePackConfigDesc(config.files[cfgId].pDesc, config.procData, CONFIG_PROC_SIZE, &bytesWritten) == 0)
				{
					LogFileWriteMultiple(config.procData, bytesWritten);

					if(LogFilePackConfigFile(&config.files[cfgId], config.procData, CONFIG_PROC_SIZE, &bytesWritten) == 0)
					{
						LogFileWriteMultiple(config.procData, bytesWritten);
					}
				}

				chThdSleepMilliseconds(5);
			}
		}

		if(chVTTimeElapsedSinceX(lastCfg2LogTime) > MS2ST(1000))
		{
			lastCfg2LogTime = chVTGetSystemTimeX();

			if(cfgToSend < config.filesUsed)
			{
				uint32_t bytesWritten;

				if(LogFilePackConfigDesc(config.files[cfgToSend].pDesc, config.procData, CONFIG_PROC_SIZE, &bytesWritten) == 0)
				{
					LogFileWriteMultiple(config.procData, bytesWritten);

					if(LogFilePackConfigFile(&config.files[cfgToSend], config.procData, CONFIG_PROC_SIZE, &bytesWritten) == 0)
					{
						LogFileWriteMultiple(config.procData, bytesWritten);
					}
				}
			}

			++cfgToSend;
			if(cfgToSend >= CONFIG_MAX_FILES)
				cfgToSend = 0;
		}

		if(result != MSG_OK)
			continue;

		msg_t internalEvent = (event & 0xF0000000);

		switch(internalEvent)
		{
			case CONFIG_EVENT_SEND_FILE_LIST:
			{
				for(uint16_t i = 0; i < config.filesUsed; i++)
				{
					ConfigFile* pFile = &config.files[i];

					if(pFile->internal)
						continue;

					PacketHeader* pHeader = (PacketHeader*)config.procData;
					pHeader->section = SECTION_CONFIG;
					pHeader->cmd = CMD_CONFIG_FILE_STRUCTURE;

					ConfigFileStructure* pStruct = (ConfigFileStructure*)(config.procData+sizeof(PacketHeader));
					pStruct->id = pFile->pDesc->cfgId;
					pStruct->version = pFile->pDesc->version;

					uint8_t* pElements = config.procData+sizeof(ConfigFileStructure)+sizeof(PacketHeader);

					for(uint16_t ele = 0; ele < pFile->pDesc->numElements; ele++)
					{
						*pElements = pFile->pDesc->elements[ele].type;
						++pElements;
					}

					(*config.outputFunction)(config.procData, sizeof(PacketHeader)+sizeof(ConfigFileStructure)+pFile->pDesc->numElements);

					chThdSleepMilliseconds(50);
				}
			}
			break;
			case CONFIG_EVENT_SEND_FILE:
			{
				uint16_t cfgId = event & 0xFFFF;

				ConfigFile* pFile = 0;

				for(uint16_t i = 0; i < config.filesUsed; i++)
				{
					if(config.files[i].pDesc->cfgId == cfgId)
					{
						pFile = &config.files[i];
						break;
					}
				}

				if(pFile == 0)
				{
					LogErrorC("Unknown config", cfgId);
					continue;
				}

				PacketHeader* pHeader = (PacketHeader*)config.procData;
				pHeader->section = SECTION_CONFIG;
				pHeader->cmd = CMD_CONFIG_READ;

				ConfigReadWrite* pRead = (ConfigReadWrite*)(config.procData+sizeof(PacketHeader));
				pRead->id = cfgId;

				memcpy(config.procData+sizeof(PacketHeader)+sizeof(ConfigReadWrite), pFile->pData, pFile->size);

				(*config.outputFunction)(config.procData, sizeof(PacketHeader)+sizeof(ConfigReadWrite)+pFile->size);
				chThdSleepMilliseconds(20);
			}
			break;
			case CONFIG_EVENT_SEND_ITEM_DESC:
			{
				uint16_t cfgId = event & 0xFFFF;
				uint16_t element = (event & 0xFF0000) >> 16;

				const ConfigFileDesc* pFileDesc = 0;

				for(uint16_t i = 0; i < config.filesUsed; i++)
				{
					if(config.files[i].pDesc->cfgId == cfgId)
					{
						pFileDesc = config.files[i].pDesc;
						break;
					}
				}

				if(pFileDesc == 0)
				{
					LogErrorC("Unknown config", cfgId);
					continue;
				}

				PacketHeader* pHeader = (PacketHeader*)config.procData;
				pHeader->section = SECTION_CONFIG;
				pHeader->cmd = CMD_CONFIG_ITEM_DESC;

				ConfigItemDesc* pDesc = (ConfigItemDesc*)(config.procData+sizeof(PacketHeader));
				pDesc->id = cfgId;
				pDesc->element = element;

				if(element == 0xFF)
				{
					// file name
					size_t nameLength = strlen(pFileDesc->pName);
					memcpy(config.procData+sizeof(PacketHeader)+sizeof(ConfigItemDesc), pFileDesc->pName, nameLength);

					(*config.outputFunction)(config.procData, sizeof(PacketHeader)+sizeof(ConfigItemDesc)+nameLength);
					chThdSleepMilliseconds(20);

					continue;
				}

				if(element >= pFileDesc->numElements)
				{
					LogErrorC("Unknown element", (element << 8) | pFileDesc->numElements);
					continue;
				}

				// element name
				size_t nameLength = 0;
				if(strlen(pFileDesc->elements[element].pUnit) > 0)
				{
					nameLength = snprintf((char*)(config.procData+sizeof(PacketHeader)+sizeof(ConfigItemDesc)), 60, "%s [%s]",
							pFileDesc->elements[element].pDesc, pFileDesc->elements[element].pUnit);
				}
				else
				{
					nameLength = snprintf((char*)(config.procData+sizeof(PacketHeader)+sizeof(ConfigItemDesc)), 60, "%s",
							pFileDesc->elements[element].pDesc);
				}

				(*config.outputFunction)(config.procData, sizeof(PacketHeader)+sizeof(ConfigItemDesc)+nameLength);
				chThdSleepMilliseconds(20);
			}
			break;
		}
	}
}

void ConfigNetworkOutput(void* pData, uint16_t dataLength)
{
	NetworkSendPacketReliable((PacketHeader*)pData, pData+PACKET_HEADER_SIZE, dataLength-PACKET_HEADER_SIZE);
}

void ConfigNetworkInput(PacketHeader* pHeader, uint8_t* pData, uint16_t dataLength)
{
	if(pHeader->section != SECTION_CONFIG)
		return;

	switch(pHeader->cmd)
	{
		case CMD_CONFIG_QUERY_FILE_LIST:
		{
			ConfigInternalInjectEvent(CONFIG_EVENT_SEND_FILE_LIST);
		}
		break;
		case CMD_CONFIG_ITEM_DESC:
		{
			ConfigItemDesc* pDesc = (ConfigItemDesc*)pData;
			pDesc->id &= 0x0FFF;	// remove (deprecated) processor bits

			ConfigInternalInjectEvent(CONFIG_EVENT_SEND_ITEM_DESC | pDesc->id | (((msg_t)pDesc->element) << 16));
		}
		break;
		case CMD_CONFIG_WRITE:
		{
			ConfigReadWrite* pWrite = (ConfigReadWrite*)pData;

			if(dataLength < sizeof(ConfigReadWrite))
			{
				LogErrorC("Invalid datalength", dataLength);
				return;
			}

			pWrite->id &= 0x0FFF;	// remove (deprecated) processor bits

			uint16_t size = dataLength - sizeof(ConfigReadWrite);
			pData += sizeof(ConfigReadWrite);

			ConfigInternalWriteFile(pWrite->id, pData, size);
		}
		break;
		case CMD_CONFIG_READ:
		{
			ConfigReadWrite* pRead = (ConfigReadWrite*)pData;
			pRead->id &= 0x0FFF;	// remove (deprecated) processor bits

			ConfigInternalInjectEvent(CONFIG_EVENT_SEND_FILE | pRead->id);
		}
		break;
	}
}
