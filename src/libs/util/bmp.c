/*
 * bmp.c
 *
 *  Created on: 08.06.2020
 *      Author: AndreR
 */

#include "bmp.h"

uint32_t BMPWriteHeaders(FIL* pFile, const BMPFileHeader* pFileHeader, const BMPInfoHeader* pInfoHeader)
{
	UINT bytesWritten;
	uint32_t totalBytes = 0;

	f_write(pFile, pFileHeader, sizeof(BMPFileHeader), &bytesWritten);
	totalBytes += bytesWritten;

	f_write(pFile, pInfoHeader, sizeof(BMPInfoHeader), &bytesWritten);
	totalBytes += bytesWritten;

	return totalBytes;
}
