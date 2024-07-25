#pragma once

#include <stdint.h>

typedef enum _FEMMode
{
	FEM_MODE_OFF,
	FEM_MODE_TX,
	FEM_MODE_RX,
} FEMMode;

typedef struct _FEMInterface
{
	int16_t (*setModeFunc)(void* pInterfaceData, FEMMode mode);

	void* pInterfaceData;
} FEMInterface;

int16_t FEMInterfaceSetMode(FEMInterface* pFem, FEMMode mode);
