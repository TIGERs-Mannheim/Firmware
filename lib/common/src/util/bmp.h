/*
 * bmp.h
 *
 *  Created on: 08.06.2020
 *      Author: AndreR
 */

#pragma once

#include <stdint.h>
#include "fatfs/ff.h"

#define BMP_BF_TYPE 0x4D42 // "BM" string

#define BMP_BI_RGB			0
#define BMP_BI_RLE8			1
#define BMP_BI_RLE4			2
#define BMP_BI_BITFIELDS	3

typedef struct __attribute__((packed)) _BMPFileHeader
{
	uint16_t bfType;
	uint32_t bfSize;
	uint32_t bfReserved;
	uint32_t bfOffBits;
} BMPFileHeader;

typedef struct __attribute__((packed)) _BMPInfoHeader
{
	uint32_t biSize;
	int32_t biWidth;
	int32_t biHeight;
	uint16_t biPlanes;
	uint16_t biBitCount;
	uint32_t biCompression;
	uint32_t biSizeImage;
	int32_t biXPelsPerMeter;
	int32_t biYPelsPerMeter;
	uint32_t biClrUsed;
	uint32_t biClrImportant;
} BMPInfoHeader;

uint32_t BMPWriteHeaders(FIL* pFile, const BMPFileHeader* pFileHeader, const BMPInfoHeader* pInfoHeader);
