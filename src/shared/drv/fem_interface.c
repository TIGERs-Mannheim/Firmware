#include "fem_interface.h"

int16_t FEMInterfaceSetMode(FEMInterface* pFem, FEMMode mode)
{
	return (*pFem->setModeFunc)(pFem->pInterfaceData, mode);
}
