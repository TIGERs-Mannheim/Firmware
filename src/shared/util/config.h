/*
 * config.h
 *
 *  Created on: 16.05.2015
 *      Author: AndreR
 */

#pragma once

#include "flash_fs.h"
#include "log_msgs.h"
#include "commands.h"

#define CONFIG_MAX_FILES 30
#define CONFIG_EVENT_QUEUE_SIZE 20
#define CONFIG_PROC_SIZE 480

#define CONFIG_EVENT_SEND_FILE_LIST		0x00000000
#define CONFIG_EVENT_SEND_FILE			0x10000000 // | id
#define CONFIG_EVENT_SEND_ITEM_DESC		0x20000000 // | (element << 16) | id

#define CONFIG_FILE_FLAG_INTERNAL		0x01 // internal configs are not transmitted to Sumatra
#define CONFIG_FILE_FLAG_VOLATILE		0x02 // effectively not a config anymore, used for logging of quasi-static data

typedef struct _ConfigHeader
{
	uint16_t configId;
	uint16_t configVersion;
	const char* pConfigName;
} ConfigHeader;

typedef struct _ConfigFileDesc
{
	uint16_t cfgId;
	uint16_t version;
	const char* pName; // 60 characters max
	uint16_t numElements;
	const ElementDesc* elements;
} ConfigFileDesc;

typedef void(*ConfigUpdateCallback)(uint16_t);

typedef struct _ConfigFile
{
	FlashFile* pFlashFile;
	ConfigUpdateCallback updateCb;
	const ConfigFileDesc* pDesc;
	void* pData;
	uint16_t size;
	uint8_t flags;
} ConfigFile;

typedef void(*ConfigOutputFunc)(void* pData, uint16_t dataLength);

typedef struct _Config
{
	ConfigFile files[CONFIG_MAX_FILES];
	uint16_t filesUsed;

	mailbox_t eventQueue;
	msg_t eventQueueData[CONFIG_EVENT_QUEUE_SIZE];

	ConfigOutputFunc outputFunction;

	uint8_t procData[CONFIG_PROC_SIZE];
} Config;

extern Config config;

void ConfigInit(ConfigOutputFunc outFunc);
ConfigFile* ConfigOpenOrCreate(const ConfigFileDesc* pConfigDesc, void* pData, uint16_t size, ConfigUpdateCallback updateCb, uint8_t flags);
void ConfigNotifyUpdate(ConfigFile* pFile);
void ConfigTask(void* params);

void ConfigNetworkInput(PacketHeader* pHeader, uint8_t* pData, uint16_t dataLength);
void ConfigNetworkOutput(void* pData, uint16_t dataLength);

void ConfigInternalInjectEvent(msg_t event);
void ConfigInternalWriteFile(uint16_t id, const void* pData, uint16_t size);
