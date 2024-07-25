#pragma once

#include "module/radio/radio_base.h"
#include "net/udp_socket.h"
#include "util/ssl_vision.h"
#include "commands.h"

typedef struct PACKED _BaseStationConfig
{
	struct
	{
		IPv4Address ip;
		uint16_t port;
	} vision;

	struct
	{
		IPv4Address ip;
		uint16_t port;
	} base;
} BaseStationConfig;

typedef struct _BaseStationStats
{
	uint32_t rxGood;
	uint32_t rxInvalid;
	uint32_t rxAuthFailed;
	uint32_t rxUnauthorized;

	uint32_t txGood;
	uint32_t txFailed;
} BaseStationStats;

typedef struct _BaseStation
{
	NetIf networkInterface;

    FEMInterface femInterface;
	RadioPhy radioPhy;
	RadioModule radioModule;
	RadioBase radioBase;

	SslVision vision;

	BaseStationConfig config;
	FlashFile* pCfgFile;

	UDPSocket socket;

	struct
	{
		IPv4Address ip;
		uint16_t port;

		systime_t tLastCommandReception;
		uint16_t isOnline;

		systime_t tLastStatsSend;
	} sumatra;

	BaseStationStats stats;

	THD_WORKING_AREA(waTask, 2048);
	thread_t* pTask;
} BaseStation;

extern BaseStation baseStation;

void BaseStationInit();
void BaseStationSetIp(IPv4Address ip);
void BaseStationSetPort(uint16_t port);
void BaseStationSetVisionIp(IPv4Address ip);
void BaseStationSetVisionPort(uint16_t port);
void BaseStationSendSumatraPacket(const PacketHeader* pHeader, const void* pData, size_t dataSize);
