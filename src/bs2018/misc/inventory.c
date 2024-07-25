#include "inventory.h"

static const InventoryEntry* pThisBase = 0;

static const InventoryEntry inventory[] = {
	{	.cpuId = { 0x00290036, 0x3536510E, 0x34333832 },
		.hwId = 0,
		.ethernet = {
			.mac.u8 = { 2, 84, 73, 71, 65, 0 },
		},
	},
	{	.cpuId = { 0x00360043, 0x3338510D, 0x35303636 },
		.hwId = 1,
		.ethernet = {
			.mac.u8 = { 2, 84, 73, 71, 65, 1 },
		},
	},
	{	.cpuId = { 0x002A0042, 0x3338510D, 0x35303636 },
		.hwId = 2,
		.ethernet = {
			.mac.u8 = { 2, 84, 73, 71, 65, 2 },
		},
	},
	{	.cpuId = { 0x0036002D, 0x3436510D, 0x35383236 },
		.hwId = 3,
		.ethernet = {
			.mac.u8 = { 2, 84, 73, 71, 65, 3 },
		},
	},
	{	.cpuId = { 0x00300043, 0x3338510D, 0x35303636 },
		.hwId = 4,
		.ethernet = {
			.mac.u8 = { 2, 84, 73, 71, 65, 4 },
		},
	},
	{	.cpuId = { 0x0038002D, 0x3436510D, 0x35383236 },
		.hwId = 5,
		.ethernet = {
			.mac.u8 = { 2, 84, 73, 71, 65, 5 },
		},
	},
	{	.cpuId = { 0x002E0016, 0x3338510C, 0x35303636 },
		.hwId = 6,
		.ethernet = {
			.mac.u8 = { 2, 84, 73, 71, 65, 6 },
		},
	},
	{	.cpuId = { 0x00250042, 0x3338510D, 0x35303636 },
		.hwId = 7,
		.ethernet = {
			.mac.u8 = { 2, 84, 73, 71, 65, 7 },
		},
	},
	{	.cpuId = { 0x002C002B, 0x3436510D, 0x35383236 },
		.hwId = 8,
		.ethernet = {
			.mac.u8 = { 2, 84, 73, 71, 65, 8 },
		},
	},
};

static const InventoryEthernet nullEthernet = { 0 };

void InventoryInit()
{
	volatile uint32_t* pCPUID = (volatile uint32_t*)0x1FF0F420;

	uint16_t numEntries = sizeof(inventory)/sizeof(inventory[0]);

	for(uint16_t i = 0; i < numEntries; i++)
	{
		const InventoryEntry* pEntry = &inventory[i];

		if(pCPUID[0] == pEntry->cpuId[0] && pCPUID[1] == pEntry->cpuId[1] && pCPUID[2] == pEntry->cpuId[2])
		{
			pThisBase = pEntry;
			break;
		}
	}
}

const InventoryEntry* InventoryGetEntry()
{
	return pThisBase;
}

const InventoryEthernet* InventoryGetEthernet()
{
	if(pThisBase)
		return &pThisBase->ethernet;

	return &nullEthernet;
}
