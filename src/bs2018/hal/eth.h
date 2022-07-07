/*
 * eth.h
 *
 *  Created on: 23.10.2017
 *      Author: AndreR
 */

#pragma once

#include "ch.h"

typedef void(*EthLinkCallback)();
typedef void(*EthRxCallback)();

typedef struct _EthStats
{
	// only successful rx/tx frames/bytes
	uint32_t txFramesProcessed;
	uint32_t rxFramesProcessed;
	uint32_t txBytesProcessed;
	uint32_t rxBytesProcessed;

	uint32_t txFrames;
	uint32_t rxFrames;

	uint32_t rxMissedApp;
	uint32_t rxMissedCtrl;
} EthStats;

typedef struct _LinkStatus
{
	uint8_t up;
	uint8_t fullDuplex;
	uint8_t speed100M;
	uint8_t MDI;
} LinkStatus;

void		EthInit();
int16_t		EthSendEthernetFrame(uint8_t* pData, uint32_t length);
int16_t		EthGetEthernetFrame(uint8_t* pData, uint32_t dataSize, uint32_t* pBytesRead);
void		EthSetCallbacks(EthLinkCallback cbLink, EthRxCallback cbRx);
uint8_t		EthLinkUpdate();
const LinkStatus* EthGetLinkStatus();
EthStats*	EthGetStats();
void		EthAllowMAC(uint8_t slot, const uint8_t* pMac, uint8_t enable, uint8_t saFiltering);
uint16_t	ETHPhyRead(uint32_t phyReg);
void		ETHPhyWrite(uint32_t phyReg, uint16_t value);
void 		EthDebug();
