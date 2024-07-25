#pragma once

#include <stdint.h>
#include "net/inet.h"

typedef struct _InventoryEthernet
{
	MAC mac;
} InventoryEthernet;

typedef struct _InventoryEntry
{
	uint32_t cpuId[3];
	uint32_t hwId;

	InventoryEthernet ethernet;
} InventoryEntry;

void InventoryInit();
const InventoryEntry* InventoryGetEntry();
const InventoryEthernet* InventoryGetEthernet();
