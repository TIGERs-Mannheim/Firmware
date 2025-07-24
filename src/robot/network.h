#pragma once

#include "util/flash_fs.h"
#include "util/fifo_lin.h"
#include "util/config.h"
#include "math/ema_filter.h"
#include "util/shell_cmd.h"
#include "module/radio/radio_module.h"

#include "commands.h"

#include "ch.h"

// Configuration
#define NETWORK_TX_PROCESSING_SIZE	1000
#define NETWORK_RX_PROCESSING_SIZE	1000
#define NETWORK_PRINT_SIZE 100
#define NETWORK_RELIABLE_BUF_SIZE 1000

#define WIFI_PACKET_SIZE 35

#define WIFI_TX_SIZE 2048
#define WIFI_RX_SIZE 2048

typedef struct PACKED _ConfigNetwork
{
	uint8_t botId;
} ConfigNetwork;

typedef struct _NetworkImplState
{
	uint8_t isSendBufferEmpty;
	uint8_t isBaseOnline;
	RadioMode radioMode;
	uint8_t channel;
	float lastBaseStationPacketRssi_dBm;
} NetworkImplState;

typedef enum NetworkMode
{
	NETWORK_MODE_DISABLED = 0,
	NETWORK_MODE_IDLE,
	NETWORK_MODE_SCANNING,
	NETWORK_MODE_VALIDATE_SCAN,
	NETWORK_MODE_VALIDATE_CHANNEL,
	NETWORK_MODE_PAIRED,
} NetworkMode;

typedef struct
{
	uint8_t isUsed;
	uint8_t channel;
	uint8_t baseStationId;
	uint32_t allocatedBotIds;
	float rssi_dBm;
} NetworkScanEntry;

typedef struct
{
	NetworkScanEntry baseStations[16];
} NetworkScanResult;

typedef struct _Network
{
	uint8_t txProcessingBuf[NETWORK_TX_PROCESSING_SIZE];
	uint8_t rxProcessingBuf[NETWORK_RX_PROCESSING_SIZE];

	uint8_t netPrint[NETWORK_PRINT_SIZE];

	uint8_t reliableBuf[NETWORK_RELIABLE_BUF_SIZE];
	FifoLin reliableCmds;
	uint16_t sendSeq;
	int32_t lastRxSeq;

	mutex_t sendPacketMutex;
	mutex_t printMutex;

	SystemMatchCtrl lastMatchCtrlCmd;
	uint32_t matchCtrlTime;
	uint8_t matchCtrlUpdated;
	mutex_t matchCtrlMutex;

	ConfigNetwork config;
	ConfigFile* pNetworkConfig;

	NetworkMode mode;
	uint32_t modeEntryTime;

	BaseStationBroadcast lastBaseStationBroadcast;
	uint32_t baseStationBroadcastTime;
	uint8_t baseStationBroadcastUpdated;
	mutex_t baseStationBroadcastMutex;

	THD_WORKING_AREA(waNetworkTask, 4096);
	thread_t* pNetworkTask;

	NetworkScanResult lastScanResult;
	NetworkScanResult activeScanResult;
	uint8_t scanLastChannel;

	ShellCmdHandler cmdHandler;
} Network;

extern Network network;

void	NetworkInit(tprio_t taskPrio);
int16_t NetworkSendPacket(const PacketHeader* pHeader, const void* _pData, uint16_t dataLength);
void	NetworkSendPacketRaw(void* _pData, uint16_t dataLength);
int16_t NetworkSendPacketReliable(const PacketHeader* pHeader, const void* _pData, uint16_t dataLength);
void	NetworkSaveConfig();
void	NetworkEnableLink(uint8_t enable);
void	NetworkStartScan();
const char* NetworkGetModeString(NetworkMode mode);

NetworkImplState NetworkImplGetState();
void	NetworkImplSetMode(RadioMode mode);
int16_t NetworkImplSendPacket(const uint8_t* pData, uint32_t dataSize);
int16_t NetworkImplGetPacketFromBase(uint8_t dstId, uint8_t* pDst, uint32_t dstSize, uint32_t* pPktSize);
void	NetworkImplProcessPacket(PacketHeader* pHeader, uint8_t* pBuf, uint16_t dataLength);
int16_t NetworkImplSetBotId(uint8_t botId);
void	NetworkImplStartScan(uint8_t lastChannel);
void	NetworkImplSetChannel(uint8_t channel);
event_source_t* NetworkImplGetRadioEventSource();