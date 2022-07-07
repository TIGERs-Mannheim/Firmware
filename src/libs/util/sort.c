/*
 * sort.c
 *
 *  Created on: 01.02.2014
 *      Author: AndreR
 */

#include "sort.h"

void SortShellU32(uint32_t *a, uint32_t n)
{
	uint32_t h, i, j, k;

	for (h = n; h /= 2;)
	{
		for (i = h; i < n; i++)
		{
			k = a[i];
			for (j = i; j >= h && k < a[j - h]; j -= h)
			{
				a[j] = a[j - h];
			}
			a[j] = k;
		}
	}
}

void SortShellU16(uint16_t *a, uint16_t n)
{
	uint16_t h, i, j, k;

	for (h = n; h /= 2;)
	{
		for (i = h; i < n; i++)
		{
			k = a[i];
			for (j = i; j >= h && k < a[j - h]; j -= h)
			{
				a[j] = a[j - h];
			}
			a[j] = k;
		}
	}
}
