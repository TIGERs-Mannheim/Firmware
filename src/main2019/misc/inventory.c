/*
 * inventory.c
 *
 *  Created on: 26.07.2020
 *      Author: Andre
 */

#include "inventory.h"

static const InventoryEntry* pThisRobot = 0;

static const InventoryEntry inventory[] = {
	{	.cpuId = { 0x0023002A, 0x34375105, 0x31353431 },
		.hwId = 0,
		.imuCalib = {
			.gyrBias = { 0.00529844f, 0.02483885f, 0.00016892f },
			.accBias = { 0.11485922f, -0.08464728f, -0.10796544f },
			.accTiltBias = { -0.00659563f, 0.04441931f },
			.imuCalibTemp =  31.117f,
			.magBias = { -2.62649417f, 1.97503197f, -94.05852509f },
			.magCalibTemp = 26.859f,
			.calibrated = 1
		}
	},
	{	.cpuId = { 0x0025002A, 0x34375105, 0x31353431 },
		.hwId = 1,
		.imuCalib = {
			.gyrBias = { -0.00645477f, 0.02491779f, -0.00584029f },
			.accBias = { 0.09634035f, -0.09793357f, -0.16281694f },
			.accTiltBias = { 0.01068949f, 0.01352560f },
			.imuCalibTemp = 32.834f,
			.magBias = { -38.30636597f, 22.50460625f, -102.14398193f },
			.magCalibTemp = 29.462f,
			.calibrated = 1
		}
	},
	{	.cpuId = { 0x0029002A, 0x34375105, 0x31353431 },
		.hwId = 2,
		.imuCalib = {
			.gyrBias = { 0.00118126f, 0.01374175f, -0.00304806f },
			.accBias = { 0.06722380f, -0.11235823f, -0.18080422f },
			.accTiltBias = { 0.00858817f, 0.04342986f },
			.imuCalibTemp =  35.928f,
			.magBias = { -23.26978874f, -4.34136915f, -67.90779877f },
			.magCalibTemp = 27.953f,
			.calibrated = 1
		}
	},
	{	.cpuId = { 0x0045002A, 0x34375105, 0x31353431 },
		.hwId = 3,
		.imuCalib = {
			.gyrBias = { 0.01837233f, 0.01648540f, 0.00722797f },
			.accBias = { 0.17317596f, -0.10905096f, -0.06486055f },
			.accTiltBias = { 0.00362956f, -0.03758654f },
			.imuCalibTemp =  37.807f,
			.magBias = { -27.22990799f, 11.26732922f, -55.25374603f },
			.magCalibTemp = 31.606f,
			.calibrated = 1
		}
	},
	{	.cpuId = { 0x003C0024, 0x34375105, 0x31353431 },
		.hwId = 4,
		.imuCalib = {
			.gyrBias = { -0.02302191f, 0.03436193f, -0.00097401f },
			.accBias = { 0.12824175f, -0.10173103f, -0.23006058f },
			.accTiltBias = { 0.03925346f, 0.08456449f },
			.imuCalibTemp =  33.666f,
			.magBias = { -54.04345703f, 28.10938072f, -84.00755310f },
			.magCalibTemp = 28.166f,
			.calibrated = 1
		}
	},
	{	.cpuId = { 0x0022002A, 0x34375105, 0x31353431 },
		.hwId = 5,
		.imuCalib = {
			.gyrBias = { -0.00946225f, 0.02651322f, -0.00789276f },
			.accBias = { 0.10444760f, -0.05339740f, 0.06159136f },
			.accTiltBias = { -0.01545200f, 0.00220478f },
			.imuCalibTemp =  32.734f,
			.magBias = { -29.93328476f, 1.26602995f, -41.22175598f },
			.magCalibTemp = 28.496f,
			.calibrated = 1
		}
	},
	{	.cpuId = { 0x0028002A, 0x34375105, 0x31353431 },
		.hwId = 6,
		.imuCalib = {
			.gyrBias = { 0.02267014f, -0.02512348f, 0.00155254f },
			.accBias = { 0.15452705f, -0.14301853f, -0.11701967f },
			.accTiltBias = { 0.02050043f, 0.01525961f },
			.imuCalibTemp =  32.768f,
			.magBias = { -25.28042030f, 19.42914009f, -45.88422012f },
			.magCalibTemp = 28.597f,
			.calibrated = 1
		}
	},
	{	.cpuId = { 0x0027002A, 0x34375105, 0x31353431 },
		.hwId = 7,
		.imuCalib = {
			.gyrBias = { -0.00262421f, 0.04243909f, -0.00267630f },
			.accBias = { 0.10988265f, -0.10147861f, -0.18264373f },
			.accTiltBias = { -0.06608940f, -0.03359748f },
			.imuCalibTemp =  34.697f,
			.magBias = { -16.03496552f, 0.68570298f, -55.28553391f },
			.magCalibTemp = 28.321f,
			.calibrated = 1
		}
	},
	{	.cpuId = { 0x0024002A, 0x34375105, 0x31353431 },
		.hwId = 8,
		.imuCalib = {
			.gyrBias = { -0.00383622f, 0.01065741f, -0.00418252f },
			.accBias = { 0.09184030f, -0.05489811f, -0.13536549f },
			.accTiltBias = { 0.03178179f, -0.00095696f },
			.imuCalibTemp =  29.944f,
			.magBias = { -35.23295212f, 37.63081360f, -82.44959259f },
			.magCalibTemp = 24.454f,
			.calibrated = 1
		}
	},
	{	.cpuId = { 0x0021002A, 0x34375105, 0x31353431 },
		.hwId = 9,
		.imuCalib = {
			.gyrBias = { 0.00747560f, 0.01456644f, -0.00173870f },
			.accBias = { 0.06114066f, -0.08400209f, -0.12048047f },
			.accTiltBias = { -0.02826187f, 0.01985603f },
			.imuCalibTemp =  36.039f,
			.magBias = { -50.86127853f, 43.30810928f, -53.72252655f },
			.magCalibTemp = 28.295f,
			.calibrated = 1
		}
	},
	{	.cpuId = { 0x002A002A, 0x34375105, 0x31353431 },
		.hwId = 10,
		.imuCalib = {
			.gyrBias = { 0.00433419f, 0.04290955f, -0.00417307f },
			.accBias = { 0.09776002f, -0.12081793f, -0.11749662f },
			.accTiltBias = { 0.01228756f, 0.01950775f },
			.imuCalibTemp = 37.688f,
			.magBias = { -45.56551743f, 22.24262238f, -75.82747650f  },
			.magCalibTemp = 33.621f,
			.calibrated = 1
		}
	},
	{	.cpuId = { 0x001E002A, 0x34375105, 0x31353431 },
		.hwId = 11,
		.imuCalib = {
			.gyrBias = { -0.00389303f, 0.00655195f, 0.00333785f },
			.accBias = { 0.19915815f, -0.18425615f, -0.14344570f },
			.accTiltBias = { 0.03822032f, 0.07121012f },
			.imuCalibTemp =  34.629f,
			.magBias = { -24.16687775f, -8.01976585f, -85.74913788f },
			.magCalibTemp = 28.643f,
			.calibrated = 1
		}
	},
	{	.cpuId = { 0x003C003F, 0x30395116, 0x30333533 },
		.hwId = 12,
		.imuCalib = {
			.gyrBias = { 0.00030174f, 0.01026006f, -0.00907751f },
			.accBias = { 0.17868166f, -0.11054666f, -0.08504882f },
			.accTiltBias = { -0.04061417f, 0.02215915f },
			.imuCalibTemp =  32.921f,
			.magBias = { -34.37339783f, 16.99103546f, -47.07408142f },
			.magCalibTemp = 28.113f,
			.calibrated = 1
		}
	},
	{	.cpuId = { 0x003C003D, 0x30395116, 0x30333533 },
		.hwId = 13,
		.imuCalib = {
			.gyrBias = { -0.00470334f, 0.02278174f, 0.01251264f },
			.accBias = { 0.08046112f, -0.09094212f, -0.21573003f },
			.accTiltBias = { 0.00656233f, -0.02242057f },
			.imuCalibTemp =  32.368f,
			.magBias = { -32.26955414f, 42.31258774f, -55.83109665f },
			.magCalibTemp = 29.313f,
			.calibrated = 1
		}
	},
	{	.cpuId = { 0x003C003A, 0x30395116, 0x30333533 },
		.hwId = 14,
		.imuCalib = {
			.gyrBias = { -0.01399980f, 0.02519042f, -0.00120646f },
			.accBias = { 0.04912570f, -0.09720903f, 0.03753800f },
			.accTiltBias = { -0.05677072f, 0.03435843f },
			.imuCalibTemp =  32.752f,
			.magBias = { -43.20172119f, -7.62934637f, -81.61791992f },
			.magCalibTemp = 28.492f,
			.calibrated = 1
		}
	},
	{	.cpuId = { 0x003C0031, 0x30395116, 0x30333533 },
		.hwId = 15,
		.imuCalib = {
			.gyrBias = { 0.02766650f, -0.00243578f, -0.00899176f },
			.accBias = { 0.13125616f, -0.08256469f, -0.16900109f },
			.accTiltBias = { -0.03781428f, 0.08913865f },
			.imuCalibTemp = 34.603f,
			.magBias = { -17.51597595f, 46.75159454f, -58.91084671f },
			.magCalibTemp = 31.651f,
			.calibrated = 1
		}
	},
	{	.cpuId = { 0x001B002D, 0x34375106, 0x31353431 },
		.hwId = 16,
		.imuCalib = {
			.gyrBias = { -0.00800435f, 0.00776062f, -0.00474618f },
			.accBias = { 0.11434338f, -0.16763853f, 0.09906094f },
			.accTiltBias = { -0.05443999f, 0.05426705f },
			.imuCalibTemp =  33.005f,
			.magBias = { -26.59038353f, 32.81006241f, -63.33076477f },
			.magCalibTemp = 28.506f,
			.calibrated = 1
		}
	}
};

static const InventoryImuCalibration nullImuCalibration = { 0 };

void InventoryInit()
{
	volatile uint32_t* pCPUID = (volatile uint32_t*)0x1FF1E800;

	uint16_t numEntries = sizeof(inventory)/sizeof(inventory[0]);

	for(uint16_t i = 0; i < numEntries; i++)
	{
		const InventoryEntry* pEntry = &inventory[i];

		if(pCPUID[0] == pEntry->cpuId[0] && pCPUID[1] == pEntry->cpuId[1] && pCPUID[2] == pEntry->cpuId[2])
		{
			pThisRobot = pEntry;
			break;
		}
	}
}

const InventoryEntry* InventoryGetEntry()
{
	return pThisRobot;
}

const InventoryImuCalibration* InventoryGetImuCalibration()
{
	if(pThisRobot)
		return &pThisRobot->imuCalib;

	return &nullImuCalibration;
}
